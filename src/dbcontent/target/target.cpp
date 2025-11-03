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

#include "target.h"
#include <exception>
#include "stringconv.h"
#include "timeconv.h"
#include "traced_assert.h"

using namespace std;
using namespace Utils;

namespace dbContent {

const std::string Target::KEY_EVAL                       = "eval";
const std::string Target::KEY_EVAL_USE                   = "use";
const std::string Target::KEY_EVAL_EXCLUDED_TIME_WINDOWS = "excluded_tw";
const std::string Target::KEY_EVAL_EXCLUDED_REQUIREMENTS = "excluded_req";
const std::string Target::KEY_COMMENT                    = "comment";
const std::string Target::KEY_TIME_BEGIN                 = "time_begin";
const std::string Target::KEY_TIME_END                   = "time_end";
const std::string Target::KEY_ACAD                       = "aircraft_addresses";
const std::string Target::KEY_ACID                       = "aircraft_identifications";
const std::string Target::KEY_MODE_3A                    = "mode_3a_codes";
const std::string Target::KEY_MODE_C_MIN                 = "mode_c_min";
const std::string Target::KEY_MODE_C_MAX                 = "mode_c_max";
const std::string Target::KEY_COUNTS                     = "dbcontent_counts";
const std::string Target::KEY_LATITUDE_MIN               = "latitude_min";
const std::string Target::KEY_LATITUDE_MAX               = "latitude_max";
const std::string Target::KEY_LONGITUDE_MIN              = "longitude_min";
const std::string Target::KEY_LONGITUDE_MAX              = "longitude_max";
const std::string Target::KEY_ECAT                       = "emitter_category";
const std::string Target::KEY_ADSB_INFO                  = "adsb_info";
const std::string Target::KEY_ADSB_COUNT                 = "count";
const std::string Target::KEY_ADSB_MOPS                  = "mops";

const Property     Target::DBColumnID     = Property("utn" , PropertyDataType::UINT);
const Property     Target::DBColumnInfo   = Property("json", PropertyDataType::JSON);
const PropertyList Target::DBPropertyList = PropertyList({ Target::DBColumnID,
                                                           Target::DBColumnInfo });

Target::Target(unsigned int utn, nlohmann::json info)
    : utn_(utn), info_(info)
{
    if (!info_.contains(KEY_EVAL) || !info_.at(KEY_EVAL).contains(KEY_EVAL_USE))
        info_[KEY_EVAL][KEY_EVAL_USE] = true;

    use_in_eval_ = info_.at(KEY_EVAL).at(KEY_EVAL_USE);

    if (info_.at(KEY_EVAL).contains(KEY_EVAL_EXCLUDED_TIME_WINDOWS))
        excluded_time_windows_.setFrom(info_.at(KEY_EVAL).at(KEY_EVAL_EXCLUDED_TIME_WINDOWS));

    if (info_.at(KEY_EVAL).contains(KEY_EVAL_EXCLUDED_REQUIREMENTS))
        excluded_requirements_ = info_.at(KEY_EVAL).at(KEY_EVAL_EXCLUDED_REQUIREMENTS).get<std::set<std::string>>();
}

bool Target::useInEval() const
{
    return use_in_eval_;
}

void Target::useInEval(bool value)
{
    use_in_eval_ = value;

    info_[KEY_EVAL][KEY_EVAL_USE] = use_in_eval_;
}

std::string Target::comment() const
{
    if (!info_.contains(KEY_COMMENT))
        return "";

    return info_.at(KEY_COMMENT);
}

void Target::comment (const std::string& value)
{
    info_[KEY_COMMENT] = value;
}

void Target::timeBegin(boost::posix_time::ptime value)
{
    info_[KEY_TIME_BEGIN] = Time::toString(value);

    time_duration_str_ = ""; // clear to force update
}

boost::posix_time::ptime Target::timeBegin() const
{
    if (!info_.contains(KEY_TIME_BEGIN))
        return {};

    return Time::fromString(info_.at(KEY_TIME_BEGIN));
}

std::string Target::timeBeginStr() const
{
    if (!info_.contains(KEY_TIME_BEGIN))
        return {};

    return info_.at(KEY_TIME_BEGIN);
}

void Target::timeEnd(boost::posix_time::ptime value)
{
    info_[KEY_TIME_END] = Time::toString(value);

    time_duration_str_ = ""; // clear to force update
}

boost::posix_time::ptime Target::timeEnd() const
{
    if (!info_.contains(KEY_TIME_END))
        return {};

    return Time::fromString(info_.at(KEY_TIME_END));
}

std::string Target::timeEndStr() const
{
    if (!info_.contains(KEY_TIME_END))
        return {};

    return info_.at(KEY_TIME_END);
}

boost::posix_time::time_duration Target::timeDuration() const
{
    return timeEnd() - timeBegin();
}

std::string Target::timeDurationStr() const
{
    if (!time_duration_str_.size())
        time_duration_str_ = Time::toString(timeDuration(), 1);

    return time_duration_str_;
}

void Target::aircraftIdentifications(const std::set<std::string>& ids)
{
    std::set<std::string> trimmed_id;

    for (auto& id : ids)
        trimmed_id.insert(String::trim(id));

    info_[KEY_ACID] = trimmed_id;
}

std::set<std::string> Target::aircraftIdentifications() const
{
    if (!info_.contains(KEY_ACID))
        return {};

    return info_.at(KEY_ACID).get<set<string>>();
}

std::string Target::aircraftIdentificationsStr() const
{
    std::ostringstream out;

    unsigned int cnt=0;
    for (const auto& it : aircraftIdentifications())
    {
        if (cnt != 0)
            out << ", ";

        out << it;
        ++cnt;
    }

    return out.str().c_str();
}

std::set<unsigned int> Target::aircraftAddresses() const
{
    if (!info_.contains(KEY_ACAD))
        return {};

    return info_.at(KEY_ACAD).get<std::set<unsigned int>>();
}

void Target::aircraftAddresses(const std::set<unsigned int>& tas)
{
    info_[KEY_ACAD] = tas;
}

std::string Target::aircraftAddressesStr() const
{
    std::ostringstream out;

    unsigned int cnt=0;
    for (const auto it : aircraftAddresses())
    {
        if (cnt != 0)
            out << ", ";

        out << String::hexStringFromInt(it, 6, '0');
        ++cnt;
    }

    return out.str().c_str();
}

std::set<unsigned int> Target::modeACodes() const
{
    if (!info_.contains(KEY_MODE_3A))
        return {};

    return info_.at(KEY_MODE_3A).get<std::set<unsigned int>>();
}
void Target::modeACodes(const std::set<unsigned int>& mas)
{
    info_[KEY_MODE_3A] = mas;
}

std::string Target::modeACodesStr() const
{
    std::ostringstream out;

    unsigned int cnt=0;
    for (const auto it : modeACodes())
    {
        if (cnt != 0)
            out << ", ";

        out << String::octStringFromInt(it, 4, '0');
        ++cnt;
    }

    return out.str().c_str();
}

bool Target::hasModeC() const
{
    return info_.contains(KEY_MODE_C_MIN) && info_.contains(KEY_MODE_C_MAX);
}

void Target::modeCMinMax(float min, float max)
{
    info_[KEY_MODE_C_MIN] = min;
    info_[KEY_MODE_C_MAX] = max;

    if (max > 1000)
    {
        if (targetCategory() == Category::Unknown)
            targetCategory(Category::AnyAircraft);
    }
}

float Target::modeCMin() const
{
    traced_assert(info_.contains(KEY_MODE_C_MIN));
    return info_.at(KEY_MODE_C_MIN);
}

std::string Target::modeCMinStr() const
{
    traced_assert(info_.contains(KEY_MODE_C_MIN));
    return to_string(info_.at(KEY_MODE_C_MIN));
}

float Target::modeCMax() const
{
    traced_assert(info_.contains(KEY_MODE_C_MAX));
    return info_.at(KEY_MODE_C_MAX);
}

std::string Target::modeCMaxStr() const
{
    traced_assert(info_.contains(KEY_MODE_C_MAX));
    return to_string(info_.at(KEY_MODE_C_MAX));
}

bool Target::isPrimaryOnly () const
{
    return !aircraftAddresses().size() && !aircraftIdentifications().size()
           && !modeACodes().size() && !hasModeC();
}

bool Target::isModeACOnly () const
{
    return !aircraftAddresses().size() && !aircraftIdentifications().size()
           && (modeACodes().size() || hasModeC());
}

unsigned int Target::numUpdates () const
{
    unsigned int cnt = 0;

    if (info_.contains(KEY_COUNTS))
    {
        for (auto& cnt_it : info_.at(KEY_COUNTS).get<std::map<std::string, unsigned int>>())
            cnt += cnt_it.second;
    }

    return cnt;
}

unsigned int Target::dbContentCount(const std::string& dbcontent_name) const
{
    if (info_.contains(KEY_COUNTS) && info_.at(KEY_COUNTS).contains(dbcontent_name))
        return info_.at(KEY_COUNTS).at(dbcontent_name);
    else
        return 0;
}

void Target::dbContentCount(const std::string& dbcontent_name, unsigned int value)
{
    info_[KEY_COUNTS][dbcontent_name] = value;
}

// void Target::clearDBContentCount(const std::string& dbcontent_name)
// {
//     if (info_[KEY_COUNTS].contains(dbcontent_name))
//         info_[KEY_COUNTS].erase(dbcontent_name);
// }

bool Target::hasPositionBounds() const
{
    return info_.count(KEY_LATITUDE_MIN) && info_.count(KEY_LATITUDE_MAX)
           && info_.count(KEY_LONGITUDE_MIN) && info_.count(KEY_LONGITUDE_MAX);
}

void Target::setPositionBounds (double latitude_min, double latitude_max, double longitude_min, double longitude_max)
{
    traced_assert(latitude_min <= latitude_max);
    traced_assert(latitude_min <= latitude_max);

    info_[KEY_LATITUDE_MIN] = latitude_min;
    info_[KEY_LATITUDE_MAX] = latitude_max;
    info_[KEY_LONGITUDE_MIN] = longitude_min;
    info_[KEY_LONGITUDE_MAX] = longitude_max;
}

double Target::latitudeMin() const
{
    traced_assert(info_.count(KEY_LATITUDE_MIN));
    return info_.at(KEY_LATITUDE_MIN);
}
double Target::latitudeMax() const
{
    traced_assert(info_.count(KEY_LATITUDE_MAX));
    return info_.at(KEY_LATITUDE_MAX);
}
double Target::longitudeMin() const
{
    traced_assert(info_.count(KEY_LONGITUDE_MIN));
    return info_.at(KEY_LONGITUDE_MIN);
}
double Target::longitudeMax() const
{
    traced_assert(info_.count(KEY_LONGITUDE_MAX));
    return info_.at(KEY_LONGITUDE_MAX);
}

void Target::adsbCount(unsigned int count)
{
    info_[KEY_ADSB_INFO][KEY_ADSB_COUNT] = count;
}

unsigned int Target::adsbCount() const
{
    logdbg;

    unsigned int count = 0;

    if (info_.count(KEY_ADSB_INFO) && info_.at(KEY_ADSB_INFO).count(KEY_ADSB_COUNT))
    {
        traced_assert(info_.at(KEY_ADSB_INFO).at(KEY_ADSB_COUNT).is_number());
        count = info_.at(KEY_ADSB_INFO).at(KEY_ADSB_COUNT);
    }

    return count;
}

void Target::adsbMOPSCount(std::map<std::string, unsigned int> adsb_mops_count)
{
    logdbg;

    info_[KEY_ADSB_INFO][KEY_ADSB_MOPS] = adsb_mops_count;
}


bool Target::hasADSBMOPS() const
{
    return info_.count(KEY_ADSB_INFO) && info_.at(KEY_ADSB_INFO).count(KEY_ADSB_MOPS);
}

std::set<unsigned int> Target::adsbMopsList() const
{
    logdbg;

    std::set<unsigned int> ret;

    if (hasADSBMOPS())
    {
        for (const auto& mops_it : info_.at(KEY_ADSB_INFO).at(KEY_ADSB_MOPS).get<std::map<std::string, nlohmann::json>>())
        {
            if (mops_it.first == "NULL") // set in ReconstructorTarget::addTargetReportInternal
                continue;

            logdbg << "key '" << mops_it.first << "' value '" <<  mops_it.second.dump(-1) << "'";
            ret.insert(std::stoul(mops_it.first));
        }
    }

    return ret;
}

std::string Target::adsbMopsStr() const
{
    logdbg;

    traced_assert(hasADSBMOPS());

    unsigned int count = 0;

    std::ostringstream out;

    if (hasADSBMOPS())
    {
        for (const auto& mops_it : info_.at(KEY_ADSB_INFO).at(KEY_ADSB_MOPS).get<
                                   std::map<std::string, nlohmann::json>>())
        {
            count = mops_it.second;

            if (out.str().size())
                out << "\n";

            out << mops_it.first << " (" << count << ")";
        }
    }

    return out.str();
}

void Target::targetCategory(TargetBase::Category category)
{
    info_[KEY_ECAT] = static_cast<unsigned int>(category);
}

TargetBase::Category Target::targetCategory() const
{
    if (!info_.contains(KEY_ECAT) || !info_[KEY_ECAT].is_number_unsigned()) {
        return Category::Unknown;
    }
    return fromECAT(info_[KEY_ECAT].get<unsigned int>());
}

const Utils::TimeWindowCollection& Target::evalExcludedTimeWindows() const
{
    return excluded_time_windows_;
}

void Target::evalExcludedTimeWindows(const Utils::TimeWindowCollection& collection)
{
    excluded_time_windows_ = collection;
    info_[KEY_EVAL][KEY_EVAL_EXCLUDED_TIME_WINDOWS] = excluded_time_windows_.asJSON();
}

void Target::clearEvalExcludedTimeWindows()
{
    excluded_time_windows_.clear();

    if (info_[KEY_EVAL].contains(KEY_EVAL_EXCLUDED_TIME_WINDOWS))
        info_[KEY_EVAL].erase(KEY_EVAL_EXCLUDED_TIME_WINDOWS);
}

const std::set<std::string>& Target::evalExcludedRequirements() const
{
    return excluded_requirements_;
}

void Target::evalExcludedRequirements(const std::set<std::string>& excl_req)
{
    excluded_requirements_ = excl_req;
    info_[KEY_EVAL][KEY_EVAL_EXCLUDED_REQUIREMENTS] = excluded_requirements_;
}

void Target::clearEvalExcludedRequirements()
{
    excluded_requirements_.clear();

    if (info_[KEY_EVAL].contains(KEY_EVAL_EXCLUDED_REQUIREMENTS))
        info_[KEY_EVAL].erase(KEY_EVAL_EXCLUDED_REQUIREMENTS);
}

}



