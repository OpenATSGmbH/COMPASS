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

#include "catch.hpp"

#include "requirement/detection/detection_walk.h"
#include "results/base/detailcode.h"
#include "timeperiod.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::DetectionWalk;

using boost::posix_time::ptime;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;

namespace
{

ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }

ptime at(double t_s) { return t0() + milliseconds((long)std::llround(t_s * 1000.0)); }

enum class TstFlag { Ok, NoRefPos, Outside };

/**
 * One test case: reference times with inside state, test reports with flags, parameters and
 * expected sums. Times are seconds from t0(). tst_starts marks the test reports that start a track.
 */
struct WalkCase
{
    std::string                             name;
    Params                                  params;
    double                                  max_ref_time_diff_s = 5.0;
    std::vector<std::pair<double, bool>>    ref;
    std::vector<std::pair<double, TstFlag>> tst;
    std::vector<bool>                       tst_starts;
    double                                  expected_sum_expected = 0.0;
    double                                  expected_sum_missed   = 0.0;
    unsigned int                            expected_num_initiations = 0;
};

TstFlag flagFromString(const std::string& s)
{
    if (s == "ok")
        return TstFlag::Ok;
    if (s == "no_ref_pos")
        return TstFlag::NoRefPos;
    if (s == "outside")
        return TstFlag::Outside;

    throw std::runtime_error("unknown test report flag '" + s + "'");
}

WalkCase caseFromJSON(const nlohmann::json& j, const std::string& name)
{
    WalkCase c;
    c.name = name;

    const auto& p = j.at("params");
    auto& mp = c.params.miss_params;

    mp.update_interval_s = p.at("update_interval_s").get<float>();

    if (p.contains("miss_tolerance_s"))
    {
        mp.use_miss_tolerance = true;
        mp.miss_tolerance_s   = p.at("miss_tolerance_s").get<float>();
    }
    if (p.contains("min_gap_length_s"))
    {
        mp.use_min_gap_length = true;
        mp.min_gap_length_s   = p.at("min_gap_length_s").get<float>();
    }
    if (p.contains("max_gap_length_s"))
    {
        mp.use_max_gap_length = true;
        mp.max_gap_length_s   = p.at("max_gap_length_s").get<float>();
    }

    c.params.use_gap_count  = p.value("use_gap_count", false);
    c.params.use_time_ratio = p.value("use_time_ratio", false);
    c.max_ref_time_diff_s   = p.value("max_ref_time_diff_s", 5.0);

    if (p.contains("trit_s"))
    {
        c.params.use_trit = true;
        c.params.trit_s   = p.at("trit_s").get<float>();
    }

    // reference: explicit [t, inside] entries and/or series [t_from, t_to, step, inside]
    if (j.contains("ref"))
        for (const auto& r : j.at("ref"))
            c.ref.emplace_back(r.at(0).get<double>(), r.at(1).get<bool>());

    if (j.contains("ref_series"))
    {
        for (const auto& s : j.at("ref_series"))
        {
            double t_from = s.at(0).get<double>();
            double t_to   = s.at(1).get<double>();
            double step   = s.at(2).get<double>();
            bool   inside = s.at(3).get<bool>();

            for (double t = t_from; t <= t_to + 1e-9; t += step)
                c.ref.emplace_back(t, inside);
        }
    }

    std::sort(c.ref.begin(), c.ref.end(), [] (const auto& a, const auto& b) { return a.first < b.first; });

    // test reports: [t, flag] or [t, flag, "start"] for a report that starts a track
    for (const auto& t : j.at("tst"))
    {
        c.tst.emplace_back(t.at(0).get<double>(), flagFromString(t.at(1).get<std::string>()));

        if (t.size() > 2 && t.at(2).get<std::string>() != "start")
            throw std::runtime_error("unknown test report track flag '" + t.at(2).get<std::string>() + "'");

        c.tst_starts.push_back(t.size() > 2);
    }

    c.expected_sum_expected    = j.at("expected").at("sum_expected").get<double>();
    c.expected_sum_missed      = j.at("expected").at("sum_missed").get<double>();
    c.expected_num_initiations = j.at("expected").value("num_initiations", 0u);

    return c;
}

/**
 * Runs the walk as Detection::evaluate() does: reference periods, then the test reports.
 */
Result run(const WalkCase& c)
{
    std::vector<std::pair<ptime, bool>> ref_inside;
    for (const auto& r : c.ref)
        ref_inside.emplace_back(at(r.first), r.second);

    auto ref_periods = referencePeriods(ref_inside, milliseconds((long)std::llround(c.max_ref_time_diff_s * 1000.0)));

    std::vector<ptime> tst_times;
    for (const auto& t : c.tst)
        tst_times.push_back(at(t.first));

    auto has_ref_pos   = [ & ] (size_t i) { return c.tst.at(i).second != TstFlag::NoRefPos; };
    auto inside_sector = [ & ] (size_t i) { return c.tst.at(i).second != TstFlag::Outside; };
    auto starts_track  = [ & ] (size_t i) { return i < c.tst_starts.size() && c.tst_starts.at(i); };
    auto gap_ui        = [ & ] (const ptime&, const ptime&) { return c.params.miss_params.update_interval_s; };

    return walk(ref_periods, tst_times, has_ref_pos, inside_sector, starts_track, gap_ui, c.params);
}

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/detection/unit_tests/detection_cases/";
}

} // namespace

TEST_CASE("DetectionWalk::referencePeriods", "[eval][detection_walk]")
{
    auto ms = [] (double s) { return milliseconds((long)std::llround(s * 1000.0)); };

    SECTION("contiguous inside times give one period")
    {
        std::vector<std::pair<ptime, bool>> ref;
        for (int t = 0; t <= 80; ++t)
            ref.emplace_back(at(t), true);

        auto periods = referencePeriods(ref, ms(5.0));
        REQUIRE(periods.size() == 1);
        CHECK(periods.period(0).begin() == at(0));
        CHECK(periods.period(0).end()   == at(80));
    }

    SECTION("a reference gap larger than the maximum time difference splits")
    {
        std::vector<std::pair<ptime, bool>> ref = { { at(0), true }, { at(4), true }, { at(10), true }, { at(14), true } };

        auto periods = referencePeriods(ref, ms(5.0));
        REQUIRE(periods.size() == 2);
        CHECK(periods.period(0).end()   == at(4));
        CHECK(periods.period(1).begin() == at(10));
    }

    SECTION("outside times split, periods shorter than 1 s are removed")
    {
        std::vector<std::pair<ptime, bool>> ref = { { at(0), true }, { at(0.5), true }, { at(1), false },
                                                    { at(2), true }, { at(4), true } };

        auto periods = referencePeriods(ref, ms(5.0));
        REQUIRE(periods.size() == 1);
        CHECK(periods.period(0).begin() == at(2));
        CHECK(periods.period(0).end()   == at(4));
    }
}

TEST_CASE("DetectionWalk::trackStarts", "[eval][detection_walk]")
{
    typedef boost::optional<bool>         OptBool;
    typedef boost::optional<unsigned int> OptTN;

    SECTION("Track Begin decides when known")
    {
        auto starts = trackStarts({ { OptBool(true), OptBool(true), OptTN(7) },
                                    { OptBool(false), OptBool(false), OptTN(8) } });
        CHECK(starts == std::vector<bool>{ true, false });
    }

    SECTION("without Track Begin a tentative report starts a track")
    {
        auto starts = trackStarts({ { {}, OptBool(false), OptTN(7) },
                                    { {}, OptBool(false), OptTN(7) },
                                    { {}, OptBool(true), OptTN(7) } });
        CHECK(starts == std::vector<bool>{ true, true, false });
    }

    SECTION("without Track Begin and Track Confirmed the first report of a track number starts a track")
    {
        auto starts = trackStarts({ { {}, {}, OptTN(7) },
                                    { {}, {}, OptTN(7) },
                                    { {}, OptBool(true), OptTN(9) },
                                    { {}, {}, OptTN(9) } });
        CHECK(starts == std::vector<bool>{ true, false, true, false });
    }

    SECTION("a track number seen in a report with Track Begin is not new later")
    {
        auto starts = trackStarts({ { OptBool(false), {}, OptTN(7) },
                                    { {}, {}, OptTN(7) } });
        CHECK(starts == std::vector<bool>{ false, false });
    }

    SECTION("nothing known is no track start")
    {
        auto starts = trackStarts({ { {}, {}, {} }, { {}, OptBool(true), {} } });
        CHECK(starts == std::vector<bool>{ false, false });
    }
}

TEST_CASE("DetectionWalk::walk steps", "[eval][detection_walk]")
{
    WalkCase c;
    c.params.miss_params.update_interval_s = 8.0f;

    for (int t = 0; t <= 80; ++t)
        c.ref.emplace_back(t, true);

    c.tst = { { 0, TstFlag::Ok }, { 8, TstFlag::Ok }, { 16, TstFlag::Ok }, { 40, TstFlag::Ok }, { 48, TstFlag::Ok } };

    auto result = run(c);

    // 0 first in period, gaps 8, 8, 24 (miss), 8, period end 48 -> 80 (32 s, miss)
    REQUIRE(result.steps.size() == 6);
    CHECK(result.steps[ 0 ].type == Step::Type::FirstInPeriod);
    CHECK(result.steps[ 3 ].type == Step::Type::Gap);
    CHECK(result.steps[ 3 ].miss);
    CHECK(result.steps[ 3 ].missed == Approx(3.0));
    CHECK(result.steps[ 3 ].sum_missed == Approx(3.0));
    CHECK(result.steps[ 5 ].type == Step::Type::PeriodEnd);
    CHECK(result.steps[ 5 ].tst_index == -1);
    CHECK(result.steps[ 5 ].t0_is_tst);
    CHECK(result.steps[ 5 ].missed == Approx(4.0));

    CHECK(result.sum_expected == Approx(10.0));
    CHECK(result.sum_missed   == Approx(7.0));

    // the reason bit of the misses, none for the other steps
    namespace Gap = EvaluationRequirementResult::DetailCode::Gap;

    CHECK(result.steps[ 0 ].code == 0);
    CHECK(result.steps[ 1 ].code == 0);
    CHECK(result.steps[ 3 ].code == Gap::MissInPeriod);
    CHECK(result.steps[ 5 ].code == Gap::MissAtPeriodEnd);
}

TEST_CASE("DetectionWalk::walk reason bits of the misses", "[eval][detection_walk]")
{
    namespace Gap = EvaluationRequirementResult::DetailCode::Gap;

    WalkCase c;
    c.params.miss_params.update_interval_s = 8.0f;

    for (int t = 0; t <= 80; ++t)
        c.ref.emplace_back(t, true);

    auto codeOf = [] (const Result& result, Step::Type type)
    {
        auto it = std::find_if(result.steps.begin(), result.steps.end(),
                               [ & ] (const Step& s) { return s.type == type; });
        REQUIRE(it != result.steps.end());
        return it->code;
    };

    SECTION("miss at the period begin")
    {
        c.tst = { { 24, TstFlag::Ok }, { 30, TstFlag::Ok } };
        auto result = run(c);

        CHECK(codeOf(result, Step::Type::FirstInPeriod) == Gap::MissAtPeriodBegin);
        CHECK(codeOf(result, Step::Type::Gap) == 0);
        CHECK(codeOf(result, Step::Type::PeriodEnd) == Gap::MissAtPeriodEnd);
    }

    SECTION("no test data")
    {
        auto result = run(c);

        CHECK(codeOf(result, Step::Type::PeriodWithoutTestData) == Gap::PeriodWithoutValidUpdate);
    }

    SECTION("test reports only outside the sector")
    {
        c.tst = { { 20, TstFlag::Outside } };
        auto result = run(c);

        CHECK(codeOf(result, Step::Type::OutsideSector) == 0);
        CHECK(codeOf(result, Step::Type::PeriodEnd) == Gap::PeriodWithoutValidUpdate);
    }
}

TEST_CASE("DetectionWalk cases from JSON", "[eval][detection_walk]")
{
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(casesDir()))
        if (entry.path().extension() == ".json")
            files.push_back(entry.path());

    std::sort(files.begin(), files.end());

    REQUIRE(!files.empty());

    for (const auto& file : files)
    {
        std::ifstream in(file);
        nlohmann::json j = nlohmann::json::parse(in);

        auto c = caseFromJSON(j, file.filename().string());

        INFO("case " << c.name << ": " << j.value("description", ""));

        auto result = run(c);

        CHECK(result.sum_expected    == Approx(c.expected_sum_expected));
        CHECK(result.sum_missed      == Approx(c.expected_sum_missed));
        CHECK(result.num_initiations == c.expected_num_initiations);
    }
}
