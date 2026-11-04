/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#include "dbcontent/dbcontent.h"
#include "asterixpostprocessjob.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/calculationplan.h"
#include "buffer.h"
#include "compass.h"
#include "files.h"
#include "logger.h"
#include "projectionmanager.h"
#include "stringconv.h"
#include "global.h"
#include "json.hpp"

#include <osgEarth/GeoMath>

#include <QThread>

#include <cctype>
#include <fstream>
#include <random>
#include <set>

using namespace std;
using namespace nlohmann;
using namespace Utils;

boost::mutex ASTERIXPostprocessJob::m3a_map_mutex_;
tbb::concurrent_unordered_map<unsigned int, unsigned int> ASTERIXPostprocessJob::obfuscate_m3a_map_;
boost::mutex ASTERIXPostprocessJob::acad_map_mutex_;
tbb::concurrent_unordered_map<unsigned int, unsigned int> ASTERIXPostprocessJob::obfuscate_acad_map_;
boost::mutex ASTERIXPostprocessJob::acid_map_mutex_;
tbb::concurrent_unordered_map<std::string, std::string> ASTERIXPostprocessJob::obfuscate_acid_map_;

namespace
{
// Conspicuity / reserved Mode 3/A codes (octal display -> decimal storage as
// 12-bit value with 3 bits per octal digit). Pass through obfuscation
// unchanged, and excluded from the random pool for newly-generated values.
//   0     -> 0      (no code)
//   01000 -> 512
//   02000 -> 1024
//   07500 -> 3904   (hijack)
//   07600 -> 3968   (radio failure)
//   07700 -> 4032   (general emergency)
//   07776 -> 4094   (reserved)
//   07777 -> 4095   (reserved / military intercept)
const std::set<unsigned int>& m3aConspicuityCodes()
{
    static const std::set<unsigned int> codes = {
        0, 512, 1024, 3904, 3968, 4032, 4094, 4095
    };
    return codes;
}

std::mt19937& obfuscateRng()
{
    static std::mt19937 rng{std::random_device{}()};
    return rng;
}

static const char* kObfuscationFile = "/tmp/compass_obfuscation.json";
}

ASTERIXPostprocessJob::ASTERIXPostprocessJob(
    map<string, shared_ptr<Buffer>> buffers,
    ASTERIXImportTaskSettings settings,
    std::shared_ptr<const dbContent::CalculationPlan> calc_plan,
    COMPASS& compass)
    : Job("ASTERIXPostprocessJob"),
    buffers_(std::move(buffers)),
    settings_(settings),
    calc_plan_(std::move(calc_plan)),
    compass_(compass)
{
    traced_assert(calc_plan_);
}

ASTERIXPostprocessJob::ASTERIXPostprocessJob(map<string, shared_ptr<Buffer>> buffers,
                                             std::shared_ptr<const dbContent::CalculationPlan> calc_plan,
                                             COMPASS& compass)
    : Job("ASTERIXPostprocessJob"),
    buffers_(std::move(buffers)),
    calc_plan_(std::move(calc_plan)),
    compass_(compass)
{
    traced_assert(calc_plan_);
}

ASTERIXPostprocessJob::~ASTERIXPostprocessJob()
{
    logdbg;
}



void ASTERIXPostprocessJob::run_impl()
{
    logdbg << "start";

    unsigned cnt=0;

    for (auto& buf_it : buffers_)
        cnt += buf_it.second->size();

    logdbg << "num buffers " << buffers_.size() << " size " << cnt;

    started_ = true;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    // the selection variables, "Latitude*" from the decoded item or "Latitude HR", are filled
    // by dbContent::SelectionFiller before the timestamp calculation, the calculations below
    // fill the rows still null

    // the calculated variables with an operator, DS ID first, then the functions using it:
    // the radar plot positions of CAT001 and CAT048, the CAT021 ground speed and track angle
    calc_plan_->run(buffers_);

    checkARTASHashes();
    doRadarPlotPositionCalculations();
    doXYPositionCalculations();

    if (settings_.filter_tod_active_  || settings_.filter_position_rec_active_ 
        || settings_.filter_position_circ_active_ || settings_.filter_modec_active_)
        doFilters();

    if (settings_.obfuscate_secondary_info_)
        doObfuscate();

    auto t_diff = boost::posix_time::microsec_clock::local_time() - start_time;

    unsigned int num_processed = 0;

    for (auto& buf_it : buffers_)
    {
        if (buf_it.second && buf_it.second->size())
            num_processed += buf_it.second->size();
    }

    float num_secs =  t_diff.total_milliseconds() ? t_diff.total_milliseconds() / 1000.0 : 10E-6;

    logdbg << "done: took "
           << String::timeStringFromDouble(num_secs, true)
           << " full " << String::timeStringFromDouble(num_secs, true)
           << " " << ((float) num_processed) / num_secs << " rec/s";

    logdbg << "done";
    done_ = true;
}



void ASTERIXPostprocessJob::checkARTASHashes()
{
    DBContentManager& dbcont_man = compass_.dbContentManager();

    for (auto& buf_it : buffers_)
    {
        const string& dbcontent_name = buf_it.first;

        if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_artas_hash_))
            continue;

        shared_ptr<Buffer> buffer = buf_it.second;
        unsigned int buffer_size = buffer->size();

        if (!buffer_size)
            continue;

        dbContent::Variable& hash_var =
            dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_artas_hash_);

        if (!buffer->has<string>(hash_var.name()))
        {
            logerr << dbcontent_name
                   << " has no ARTAS hash, " << buffer_size << " target reports affected";
            continue;
        }

        NullableVector<string>& hash_vec = buffer->get<string>(hash_var.name());

        unsigned int null_cnt = 0;

        for (unsigned int cnt = 0; cnt < buffer_size; ++cnt)
        {
            if (hash_vec.isNull(cnt))
                ++null_cnt;
        }

        if (null_cnt > 0)
            logerr << dbcontent_name
                   << " " << null_cnt << " of " << buffer_size
                   << " target reports have null ARTAS hash";
    }
}

void ASTERIXPostprocessJob::doRadarPlotPositionCalculations()
{
    // radar calculations of the contents without the radar_projection function in the plan,
    // CAT010: fills the "Latitude*" rows the selection left null, before the x/y projection

    map<string, shared_ptr<Buffer>> remaining;

    for (auto& buf_it : buffers_)
        if (!calc_plan_->runsFunction(buf_it.first, dbContent::CalculationPlan::FunctionRadarProjection))
            remaining.insert(buf_it);

    compass_.projectionManager().doRadarPlotPositionCalculations(remaining);
}

void ASTERIXPostprocessJob::doXYPositionCalculations()
{
    logdbg;

    // tracked data sources with only x/y coordinates
    compass_.projectionManager().doXYPositionCalculations(buffers_);
}

void ASTERIXPostprocessJob::doFilters()
{
    string dbcontent_name;

    DBContentManager& dbcont_man = compass_.dbContentManager();

    // do time based filtering first
    if (settings_.filter_tod_active_)
    {
        string tod_var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_time_of_day_));

            dbContent::Variable& tod_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_time_of_day_);

            tod_var_name = tod_var.name();

            traced_assert(buffer->has<float>(tod_var_name));

            NullableVector<float>& tod_vec = buffer->get<float>(tod_var_name);

            std::vector<unsigned int> to_be_removed;

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                    if (tod_vec.isNull(cnt)
                        || (tod_vec.get(cnt) < settings_.filter_tod_min_ || tod_vec.get(cnt) > settings_.filter_tod_max_))
                    {
                        to_be_removed.push_back(cnt);
                        continue;
                    }
            }

            buffer->removeIndexes(to_be_removed);
        }
    }

    // others
    if (settings_.filter_position_rec_active_ || settings_.filter_position_circ_active_ || settings_.filter_modec_active_)
    {
        string lat_var_name;
        string lon_var_name;
        string mc_var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_)
                || !dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_)
                || !dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
                continue;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_));
            traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_));
            traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_));

            dbContent::Variable& lat_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_);
            dbContent::Variable& lon_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_);
            dbContent::Variable& mc_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_);

            lat_var_name = lat_var.name();
            lon_var_name = lon_var.name();
            mc_var_name = mc_var.name();

            traced_assert(buffer->has<double>(lat_var_name));
            traced_assert(buffer->has<double>(lon_var_name));
            traced_assert(buffer->has<float>(mc_var_name));

            NullableVector<double>& lat_vec = buffer->get<double>(lat_var_name);
            NullableVector<double>& lon_vec = buffer->get<double>(lon_var_name);
            // the Mode C Meta Variable, CAT062 is the selection "Mode C Code", filled by the
            // SelectionFiller before this job
            NullableVector<float>& mc_vec = buffer->get<float>(mc_var_name);

            std::vector<unsigned int> to_be_removed;

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                if (settings_.filter_position_rec_active_ && !lat_vec.isNull(cnt) &&
                    !lon_vec.isNull(cnt) &&
                    (lat_vec.get(cnt) < settings_.filter_rec_latitude_min_ ||
                     lat_vec.get(cnt) > settings_.filter_rec_latitude_max_ ||
                     lon_vec.get(cnt) < settings_.filter_rec_longitude_min_ ||
                     lon_vec.get(cnt) > settings_.filter_rec_longitude_max_))
                {
                    to_be_removed.push_back(cnt);
                    continue;
                }

                if (settings_.filter_position_circ_active_ && !lat_vec.isNull(cnt) &&
                    !lon_vec.isNull(cnt))
                {
                    double distance_m = osgEarth::GeoMath::distance(
                        osg::DegreesToRadians(settings_.filter_circ_latitude_),
                        osg::DegreesToRadians(settings_.filter_circ_longitude_),
                        osg::DegreesToRadians(lat_vec.get(cnt)),
                        osg::DegreesToRadians(lon_vec.get(cnt)));

                    logdbg << "distance_m_ " << distance_m << " nm " << distance_m / NM2M
                    << " range " << settings_.filter_circ_range_ << " remove " << (distance_m / NM2M > settings_.filter_circ_range_);

                    if (distance_m / NM2M > settings_.filter_circ_range_)
                    {
                        to_be_removed.push_back(cnt);
                        continue;
                    }
                }

                if (settings_.filter_modec_active_)
                {
                    if (!mc_vec.isNull(cnt)
                        && (mc_vec.get(cnt) < settings_.filter_modec_min_ || mc_vec.get(cnt) > settings_.filter_modec_max_))
                    {
                        to_be_removed.push_back(cnt);
                        continue;
                    }
                }
            }

            buffer->removeIndexes(to_be_removed);
        }
    }

    // delete empty ones

    for (auto it = buffers_.cbegin(); it != buffers_.cend() /* not hoisted */; /* no increment */)
    {
        if (!it->second->size())
            buffers_.erase(it++);    // or "it = m.erase(it)" since C++11
        else
            ++it;
    }
}

void ASTERIXPostprocessJob::doObfuscate()
{
    traced_assert(settings_.obfuscate_secondary_info_);

    string dbcontent_name;

    DBContentManager& dbcont_man = compass_.dbContentManager();

    // filter / change mode 3/a codes
    {
        boost::mutex::scoped_lock locker(m3a_map_mutex_);
        string var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_))
                continue;

            dbContent::Variable& var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_);

            var_name = var.name();

            traced_assert(buffer->has<unsigned int>(var_name));

            NullableVector<unsigned int>& var_vec = buffer->get<unsigned int>(var_name);

            std::vector<unsigned int> to_be_removed;

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                if (var_vec.isNull(cnt))
                    continue;

                if ((var_vec.get(cnt) >= 832 && var_vec.get(cnt) <= 895) // 1500 - 1577
                    || (var_vec.get(cnt) >= 2560 && var_vec.get(cnt) <= 3071)) // 5000 - 5777
                {
                    to_be_removed.push_back(cnt);
                    continue;
                }
                else // obfuscate
                {
                    unsigned int tmp = var_vec.get(cnt);
                    obfuscateM3A(tmp);
                    var_vec.set(cnt, tmp);
                }
            }

            buffer->removeIndexes(to_be_removed);
        }
    }

    // change acads
    {
        boost::mutex::scoped_lock locker(acad_map_mutex_);
        string var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_))
                continue;

            dbContent::Variable& var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_);

            var_name = var.name();

            traced_assert(buffer->has<unsigned int>(var_name));

            NullableVector<unsigned int>& var_vec = buffer->get<unsigned int>(var_name);

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                if (var_vec.isNull(cnt))
                    continue;

                // obfuscate
                {
                    unsigned int tmp = var_vec.get(cnt);
                    obfuscateACAD(tmp);
                    var_vec.set(cnt, tmp);
                }
            }
        }
    }

    // change acids
    {
        boost::mutex::scoped_lock locker(acid_map_mutex_);

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
                continue;

            dbContent::Variable& var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_);

            // the identification columns: the Meta variable and every decoded candidate of a
            // string selection over it, "Aircraft Identification*" of CAT062 and its candidates
            std::set<std::string> var_names;

            var_names.insert(var.name());

            for (const auto& var_it : dbcont_man.dbContent(dbcontent_name).variables())
            {
                const dbContent::Variable& sel_var = *var_it.second;

                if (!sel_var.isSelection() || sel_var.dataType() != PropertyDataType::STRING)
                    continue;

                bool over_acid = sel_var.name() == var.name();

                for (const auto& candidate : sel_var.selection())
                    if (candidate.is_object() && candidate.contains("variable")
                        && candidate.at("variable").is_string() && candidate.at("variable") == var.name())
                        over_acid = true;

                if (!over_acid)
                    continue;

                for (const auto& candidate : sel_var.selection())
                    if (candidate.is_object() && candidate.contains("variable")
                        && candidate.at("variable").is_string())
                        var_names.insert(candidate.at("variable").get<std::string>());
            }

            for (const auto& name : var_names)
            {
                if (!buffer->has<string>(name))
                    continue;

                NullableVector<string>& var_vec = buffer->get<string>(name);

                for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
                {
                    if (var_vec.isNull(cnt))
                        continue;

                    // obfuscate
                    {
                        std::string tmp = var_vec.get(cnt);
                        obfuscateACID(tmp);
                        var_vec.set(cnt, tmp);
                    }
                }
            }
        }
    }
}

void ASTERIXPostprocessJob::obfuscateM3A (unsigned int& value)
{
    // Conspicuity codes pass through unchanged - their operational meaning is
    // part of the data, not the privacy concern.
    const auto& conspicuity = m3aConspicuityCodes();
    if (conspicuity.count(value))
    {
        obfuscate_m3a_map_[value] = value;
        return;
    }

    if (auto it = obfuscate_m3a_map_.find(value); it != obfuscate_m3a_map_.end())
    {
        value = it->second;
        return;
    }

    // Collect already-used target values to avoid collisions.
    std::set<unsigned int> used;
    for (const auto& kv : obfuscate_m3a_map_)
        used.insert(kv.second);

    std::uniform_int_distribution<unsigned int> dist(0, 4095);
    auto& rng = obfuscateRng();

    unsigned int candidate = 0;
    for (int attempts = 0; attempts < 10000; ++attempts)
    {
        candidate = dist(rng);
        if (!conspicuity.count(candidate) && !used.count(candidate))
            break;
    }

    obfuscate_m3a_map_[value] = candidate;
    value = candidate;
}

void ASTERIXPostprocessJob::obfuscateACAD (unsigned int& value)
{
    if (auto it = obfuscate_acad_map_.find(value); it != obfuscate_acad_map_.end())
    {
        value = it->second;
        return;
    }

    std::set<unsigned int> used;
    for (const auto& kv : obfuscate_acad_map_)
        used.insert(kv.second);

    // 24-bit ICAO aircraft address: 0x000001 .. 0xFFFFFF (0 is reserved).
    std::uniform_int_distribution<unsigned int> dist(1, 0xFFFFFF);
    auto& rng = obfuscateRng();

    unsigned int candidate = 0;
    for (int attempts = 0; attempts < 10000; ++attempts)
    {
        candidate = dist(rng);
        if (!used.count(candidate))
            break;
    }

    obfuscate_acad_map_[value] = candidate;
    value = candidate;
}

// Pool of fictional starship names from Star Trek, Star Wars and The Expanse.
// Stored as full uppercase strings; the obfuscator slices each to the
// letter-count of the callsign being replaced. Long names (>= 8 chars)
// guarantee coverage for any plausible ACID letter prefix.
static const std::vector<std::string> ship_names = {
    // Star Trek
    "ENTERPRISE", "VOYAGER", "DEFIANT", "DISCOVERY", "RELIANT", "EXCELSIOR",
    "EQUINOX", "INTREPID", "TITAN", "ODYSSEY", "CONSTELLATION", "STARGAZER",
    "GRISSOM", "SARATOGA", "COCHRANE", "PICARD", "JANEWAY", "SHENZHOU",
    "YORKTOWN", "BURAN", "THUNDERCHILD", "FRANKLIN", "VENGEANCE", "SHRAN",
    "LEXINGTON", "POTEMKIN", "ENDEAVOUR", "COLUMBIA", "KELVIN", "ANTARES",
    "DAEDALUS", "NEBULA", "AKIRA", "PROMETHEUS", "NORWAY", "STEAMRUNNER",
    "EXETER", "MIRANDA", "SOVEREIGN", "AVENGER", "FARRAGUT", "BOZEMAN",
    "CAIRO", "EXCALIBUR", "GALAXY", "HATHAWAY", "MAGELLAN", "YAMAGUCHI",
    "BLACKWELL", "CHALLENGER", "CHARLESTON", "CHEROKEE", "CONCORD",
    "COPERNICUS", "DERBYSHIRE", "DRAKE", "EDISON", "GAGARIN", "GRIFFIN",
    "HANSEN", "MARYLAND", "OBERTH", "OLIVER", "ORION", "PEGASUS",
    "SAGAN", "NIAGARA", "NIMITZ", "GETTYSBURG", "KONGO", "SAREK",
    // Star Wars
    "FALCON", "TANTIVE", "RAZORCREST", "GHOST", "PHANTOM", "EXECUTOR",
    "ECLIPSE", "INTERCEPTOR", "MALEVOLENCE", "INDEPENDENCE", "LIBERTY",
    "PROFUNDITY", "RADDUS", "SUPREMACY", "FINALIZER", "CHIMAERA",
    "ARMAGEDDON", "DEVASTATOR", "INVINCIBLE", "ACCUSER", "RAVAGER",
    "RESOLUTE", "VENATOR", "ACCLAMATOR", "STARDESTROYER", "INVISIBLE",
    // The Expanse
    "ROCINANTE", "TACHI", "DONNAGER", "NAUVOO", "TYNAN", "AGATHAKING",
    "BEHEMOTH", "CONTORTA", "PELLA", "SCOPULI", "BARBAPICCOLA", "RAWESIDE",
    "HYGIEA", "ARBOGHAST", "RAZORBACK", "CHETZEMOKA", "CANTERBURY", "TYCHO",
    "DEWALT", "MAELSTROM", "MEDINA", "MORRIGAN", "BLUEFALCON", "OKIMBO",
    "SCIROCCO", "GUANSHIYIN", "SEUNGUN"
};

namespace
{
inline bool isAsciiLetter(char c) { return std::isalpha(static_cast<unsigned char>(c)); }
inline bool isAsciiDigit (char c) { return std::isdigit(static_cast<unsigned char>(c)); }

// Strip trailing whitespace; uppercase ASCII. Used to canonicalise the
// lookup key so that "XY100", "XY100   " and "xy100" all collapse to one
// entry.
std::string normaliseAcid(const std::string& s)
{
    auto end = s.find_last_not_of(" \t\r\n");
    if (end == std::string::npos)
        return std::string();
    std::string out = s.substr(0, end + 1);
    for (auto& c : out)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

// One contiguous run of input characters of the same class (letter / digit
// / other). Captures the original length and characters; "other" runs are
// passed through unchanged.
struct AcidRun
{
    enum class Kind { Letter, Digit, Other };
    Kind        kind;
    std::string text; // original characters, only meaningful for Other
};

std::vector<AcidRun> splitIntoRuns(const std::string& s)
{
    std::vector<AcidRun> runs;
    std::size_t i = 0;
    while (i < s.size())
    {
        AcidRun::Kind kind = isAsciiLetter(s[i]) ? AcidRun::Kind::Letter
                          : isAsciiDigit (s[i]) ? AcidRun::Kind::Digit
                                               : AcidRun::Kind::Other;
        std::size_t j = i;
        while (j < s.size())
        {
            AcidRun::Kind k = isAsciiLetter(s[j]) ? AcidRun::Kind::Letter
                            : isAsciiDigit (s[j]) ? AcidRun::Kind::Digit
                                                 : AcidRun::Kind::Other;
            if (k != kind) break;
            ++j;
        }
        AcidRun r;
        r.kind = kind;
        r.text = s.substr(i, j - i);
        runs.push_back(std::move(r));
        i = j;
    }
    return runs;
}

// Leading slice of a ship name at index `ship_idx` (wrapping). If the ship
// is shorter than `len`, pad with random A-Z. Subsequent letter runs in
// the same callsign should pass a different `ship_idx` (caller advances)
// so the output reads as multiple ship prefixes rather than one repeated.
std::string letterRunLeading(std::size_t ship_idx, std::size_t len, std::mt19937& rng)
{
    if (len == 0)
        return std::string();
    const std::string& name = ship_names[ship_idx % ship_names.size()];
    if (name.size() >= len)
        return name.substr(0, len);
    std::uniform_int_distribution<int> letter_pick(0, 25);
    std::string out = name;
    while (out.size() < len)
        out += static_cast<char>('A' + letter_pick(rng));
    return out;
}

// 10^n with overflow guard - clamped so callsigns with absurdly long digit
// runs don't blow up the integer space we use for the increment counter.
std::uint64_t pow10Clamped(std::size_t n)
{
    std::uint64_t v = 1;
    for (std::size_t i = 0; i < n && i < 18; ++i)
        v *= 10;
    return v;
}

// Total number of digit characters across all digit runs - the digit slots
// are filled from a single zero-padded counter, so an increment by 1
// monotonically walks the joint digit space.
std::size_t totalDigitSlots(const std::vector<AcidRun>& runs)
{
    std::size_t n = 0;
    for (const auto& r : runs)
        if (r.kind == AcidRun::Kind::Digit)
            n += r.text.size();
    return n;
}

// Materialise a candidate: letter runs use leading slices of ship_names
// starting at `ship_idx` (each successive letter run advances by one);
// digit runs are filled from `digit_value` rendered as a zero-padded
// string of length `totalDigitSlots(runs)`, split across the digit runs.
std::string generateLeadingAcid(const std::vector<AcidRun>& runs,
                                std::size_t ship_idx,
                                std::uint64_t digit_value,
                                std::mt19937& rng)
{
    const std::size_t total_digits = totalDigitSlots(runs);
    std::string digit_str;
    if (total_digits > 0)
    {
        const std::uint64_t mod = pow10Clamped(total_digits);
        if (mod > 0)
            digit_value %= mod;
        digit_str = std::to_string(digit_value);
        if (digit_str.size() < total_digits)
            digit_str = std::string(total_digits - digit_str.size(), '0') + digit_str;
        else if (digit_str.size() > total_digits)
            digit_str = digit_str.substr(digit_str.size() - total_digits);
    }

    std::string out;
    out.reserve(8);
    std::size_t digit_cursor = 0;
    std::size_t letter_run_idx = 0;
    for (const auto& r : runs)
    {
        switch (r.kind)
        {
        case AcidRun::Kind::Letter:
            out += letterRunLeading(ship_idx + letter_run_idx, r.text.size(), rng);
            ++letter_run_idx;
            break;
        case AcidRun::Kind::Digit:
            out.append(digit_str, digit_cursor, r.text.size());
            digit_cursor += r.text.size();
            break;
        case AcidRun::Kind::Other:
            out += r.text;
            break;
        }
    }
    return out;
}

// Pad `s` with trailing spaces up to `target_len`. If `s` is already that
// long or longer, returned unchanged.
std::string padTo(const std::string& s, std::size_t target_len)
{
    if (s.size() >= target_len)
        return s;
    return s + std::string(target_len - s.size(), ' ');
}
}

void ASTERIXPostprocessJob::obfuscateACID (std::string& value)
{
    const std::size_t original_len = value.size();
    const std::string key = normaliseAcid(value);

    // Empty / all-whitespace input: nothing meaningful to obfuscate.
    if (key.empty())
    {
        value = std::string(original_len, ' ');
        return;
    }

    if (auto it = obfuscate_acid_map_.find(key); it != obfuscate_acid_map_.end())
    {
        value = padTo(it->second, original_len);
        return;
    }

    const auto runs = splitIntoRuns(key);
    auto& rng = obfuscateRng();

    const std::size_t total_digits = totalDigitSlots(runs);
    const std::uint64_t digit_mod  = pow10Clamped(total_digits);
    const std::size_t pool_size    = ship_names.size();

    // Initial picks. The ship_idx selects a starting ship for the first
    // letter run; on collision we keep the ship_idx and increment the
    // composite digit value, so the readable "ship prefix" stays stable.
    // Only when the digit space is exhausted do we step to the next ship.
    std::uniform_int_distribution<std::size_t> ship_dist(0, pool_size - 1);
    std::size_t ship_idx = ship_dist(rng);
    std::uint64_t digit_value = total_digits > 0
        ? (std::uniform_int_distribution<std::uint64_t>(0, digit_mod - 1))(rng)
        : 0;

    std::string candidate;
    std::uint64_t digit_attempts = 0;
    std::size_t  ship_attempts   = 0;
    const std::uint64_t digit_budget = total_digits > 0 ? digit_mod : 1;

    while (ship_attempts < pool_size)
    {
        candidate = generateLeadingAcid(runs, ship_idx, digit_value, rng);

        bool collision = (candidate == key);
        if (!collision)
        {
            for (const auto& kv : obfuscate_acid_map_)
                if (kv.second == candidate) { collision = true; break; }
        }
        if (!collision)
            break;

        ++digit_attempts;
        if (total_digits > 0 && digit_attempts < digit_budget)
        {
            digit_value = (digit_value + 1) % digit_mod;
            continue;
        }
        // Digit space exhausted (or none) - advance to the next ship and
        // pick a fresh starting digit value.
        ++ship_attempts;
        ship_idx = (ship_idx + 1) % pool_size;
        digit_attempts = 0;
        if (total_digits > 0)
            digit_value = (std::uniform_int_distribution<std::uint64_t>(0, digit_mod - 1))(rng);
    }

    if (ship_attempts >= pool_size)
    {
        // Pathological saturation - extremely unlikely. Fall back to a
        // random-letters / random-digits rendering, accepting that it may
        // not be a recognisable ship prefix.
        std::uniform_int_distribution<int> letter_pick(0, 25);
        std::uniform_int_distribution<int> digit_pick(0, 9);
        std::string fallback;
        fallback.reserve(key.size());
        for (char c : key)
        {
            if (isAsciiLetter(c))      fallback += static_cast<char>('A' + letter_pick(rng));
            else if (isAsciiDigit(c))  fallback += static_cast<char>('0' + digit_pick(rng));
            else                       fallback += c;
        }
        candidate = std::move(fallback);
    }

    obfuscate_acid_map_[key] = candidate;
    value = padTo(candidate, original_len);
}

void ASTERIXPostprocessJob::loadObfuscationMaps()
{
    static bool loaded = false;
    if (loaded)
        return;

    if (!Utils::Files::fileExists(kObfuscationFile))
    {
        loaded = true;
        return;
    }

    nlohmann::json j;
    try
    {
        std::ifstream in(kObfuscationFile);
        in >> j;
    }
    catch (const std::exception& e)
    {
        logwrn << "could not parse " << kObfuscationFile << ": " << e.what();
        return; // do not flip 'loaded'; allow a retry next time
    }

    if (!j.is_object())
    {
        logwrn << kObfuscationFile << " is not a JSON object, ignoring";
        loaded = true;
        return;
    }

    if (j.contains("m3a") && j.at("m3a").is_object())
    {
        boost::mutex::scoped_lock locker(m3a_map_mutex_);
        for (auto& kv : j.at("m3a").items())
        {
            try
            {
                obfuscate_m3a_map_[static_cast<unsigned int>(std::stoul(kv.key()))]
                    = kv.value().get<unsigned int>();
            }
            catch (...) { /* skip malformed entry */ }
        }
    }
    if (j.contains("acad") && j.at("acad").is_object())
    {
        boost::mutex::scoped_lock locker(acad_map_mutex_);
        for (auto& kv : j.at("acad").items())
        {
            try
            {
                obfuscate_acad_map_[static_cast<unsigned int>(std::stoul(kv.key()))]
                    = kv.value().get<unsigned int>();
            }
            catch (...) {}
        }
    }
    if (j.contains("acid") && j.at("acid").is_object())
    {
        boost::mutex::scoped_lock locker(acid_map_mutex_);
        for (auto& kv : j.at("acid").items())
        {
            try
            {
                obfuscate_acid_map_[kv.key()] = kv.value().get<std::string>();
            }
            catch (...) {}
        }
    }

    loginf << "loaded obfuscation maps from " << kObfuscationFile
           << ": m3a " << obfuscate_m3a_map_.size()
           << " acad " << obfuscate_acad_map_.size()
           << " acid " << obfuscate_acid_map_.size();
    loaded = true;
}

void ASTERIXPostprocessJob::saveObfuscationMaps()
{
    nlohmann::json j;
    j["m3a"]  = nlohmann::json::object();
    j["acad"] = nlohmann::json::object();
    j["acid"] = nlohmann::json::object();
    {
        boost::mutex::scoped_lock locker(m3a_map_mutex_);
        for (const auto& kv : obfuscate_m3a_map_)
            j["m3a"][std::to_string(kv.first)] = kv.second;
    }
    {
        boost::mutex::scoped_lock locker(acad_map_mutex_);
        for (const auto& kv : obfuscate_acad_map_)
            j["acad"][std::to_string(kv.first)] = kv.second;
    }
    {
        boost::mutex::scoped_lock locker(acid_map_mutex_);
        for (const auto& kv : obfuscate_acid_map_)
            j["acid"][kv.first] = kv.second;
    }
    try
    {
        std::ofstream out(kObfuscationFile);
        out << j.dump(2);
    }
    catch (const std::exception& e)
    {
        logwrn << "could not write " << kObfuscationFile << ": " << e.what();
    }
}

