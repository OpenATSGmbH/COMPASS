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

#include "requirement/base/intervalhelpers.h"
#include "timeperiod.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::IntervalHelpers;

using boost::posix_time::ptime;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;

typedef dbContent::TargetReport::DataID DataID;

namespace
{

ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }

ptime at(double t_s) { return t0() + milliseconds((long)std::llround(t_s * 1000.0)); }

/**
 * One test case: inside reference periods, test updates with their validity, parameters and
 * expected counts. Times are seconds from t0().
 */
struct IntervalCase
{
    std::string                                         name;
    MissParams                                          params;
    bool                                                exclude_detection_misses = false;
    std::vector<std::pair<double, double>>              periods;
    std::vector<std::pair<double, IntervalValidity::Value>> updates;
    Counts                                              expected;
};

IntervalValidity::Value validityFromString(const std::string& s)
{
    if (s == "valid")
        return IntervalValidity::Value::Valid;
    if (s == "invalid")
        return IntervalValidity::Value::Invalid;
    if (s == "no_ref")
        return IntervalValidity::Value::RefDataMissing;

    throw std::runtime_error("unknown validity '" + s + "'");
}

IntervalCase caseFromJSON(const nlohmann::json& j, const std::string& name)
{
    IntervalCase c;
    c.name = name;

    const auto& p = j.at("params");
    c.params.update_interval_s = p.at("update_interval_s").get<float>();

    if (p.contains("min_gap_length_s"))
        c.params.min_gap_length_s = p.at("min_gap_length_s").get<float>();
    if (p.contains("max_gap_length_s"))
        c.params.max_gap_length_s = p.at("max_gap_length_s").get<float>();
    if (p.contains("miss_tolerance_s"))
        c.params.miss_tolerance_s = p.at("miss_tolerance_s").get<float>();

    c.exclude_detection_misses = p.value("exclude_detection_misses", false);

    for (const auto& period : j.at("periods"))
        c.periods.emplace_back(period.at(0).get<double>(), period.at(1).get<double>());

    for (const auto& update : j.at("updates"))
        c.updates.emplace_back(update.at(0).get<double>(), validityFromString(update.at(1).get<std::string>()));

    c.expected.sum_uis = j.at("expected").at("sum_uis").get<uint32_t>();
    c.expected.misses  = j.at("expected").at("misses").get<uint32_t>();

    return c;
}

/**
 * Runs the helpers as IntervalBase::evaluate() does: periods, test updates, events, counts.
 */
Counts run(const IntervalCase& c, std::vector<Event>* events_out = nullptr)
{
    TimePeriodCollection periods;
    for (const auto& p : c.periods)
        periods.add(TimePeriod(at(p.first), at(p.second)));

    std::vector<DataID> tst_updates;
    std::map<unsigned int, IntervalValidity::Value> validities;

    for (unsigned int i = 0; i < c.updates.size(); ++i)
    {
        tst_updates.emplace_back(at(c.updates[ i ].first), dbContent::TargetReport::Index(i, i));
        validities[ i ] = c.updates[ i ].second;
    }

    addTestUpdates(periods, tst_updates);

    auto validity = [ & ] (const DataID& data_id)
    {
        IntervalValidity v;
        v.value = validities.at(data_id.index().idx_external);
        return v;
    };

    auto events = periodEvents(periods, validity, c.params, false);

    if (events_out)
        *events_out = events;

    return counts(periods, events, c.params, c.exclude_detection_misses);
}

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/base/unit_tests/interval_cases/";
}

} // namespace

TEST_CASE("IntervalHelpers::numMisses", "[eval][interval_helpers]")
{
    MissParams p;
    p.update_interval_s = 8.0f;

    CHECK(numMisses(0.0, p)  == 0);
    CHECK(numMisses(8.0, p)  == 0); // gap equal to the update interval is no miss
    CHECK(numMisses(8.5, p)  == 1);
    CHECK(numMisses(16.0, p) == 2);
    CHECK(numMisses(24.0, p) == 3);

    p.miss_tolerance_s = 1.0f;
    CHECK(numMisses(9.0, p)  == 0); // tolerance is subtracted first
    CHECK(numMisses(9.5, p)  == 1);
    CHECK(numMisses(17.0, p) == 2);

    p.miss_tolerance_s.reset();
    p.min_gap_length_s = 20.0f;
    CHECK(numMisses(16.0, p) == 0); // shorter than the minimum gap length
    CHECK(numMisses(24.0, p) == 3);

    p.min_gap_length_s.reset();
    p.max_gap_length_s = 20.0f;
    CHECK(numMisses(16.0, p) == 2);
    CHECK(numMisses(24.0, p) == 0); // longer than the maximum gap length
}

TEST_CASE("IntervalHelpers::periodEvents marks a period without valid update", "[eval][interval_helpers]")
{
    IntervalCase c;
    c.params.update_interval_s = 8.0f;
    c.periods = { { 0.0, 80.0 } };

    for (double t = 0.0; t <= 80.0; t += 8.0)
        c.updates.emplace_back(t, IntervalValidity::Value::Invalid);

    std::vector<Event> events;
    auto result = run(c, &events);

    CHECK(result.sum_uis == 10);
    CHECK(result.misses == 10);

    auto it = std::find_if(events.begin(), events.end(),
                           [] (const Event& e) { return e.type == Event::TypeEmptyPeriod; });
    REQUIRE(it != events.end());
    CHECK(it->misses == 10);
    REQUIRE(it->period.has_value());
    CHECK(it->period.value() == 0);

    CHECK(std::count_if(events.begin(), events.end(),
                        [] (const Event& e) { return e.type == Event::TypeInvalid; }) == 11);
}

TEST_CASE("IntervalHelpers::periodEvents gives inside periods their index", "[eval][interval_helpers]")
{
    IntervalCase c;
    c.params.update_interval_s = 8.0f;
    c.periods = { { 0.0, 40.0 }, { 100.0, 140.0 } };
    c.updates = { { 20.0, IntervalValidity::Value::Valid },
                  { 70.0, IntervalValidity::Value::Valid },
                  { 120.0, IntervalValidity::Value::Valid } };

    std::vector<Event> events;
    run(c, &events);

    for (const auto& e : events)
    {
        if (e.type == Event::TypeNoReference)
        {
            CHECK(!e.period.has_value()); // update outside the inside periods
        }
        else if (e.type == Event::TypeValidFirst)
        {
            REQUIRE(e.period.has_value());
            CHECK(e.period.value() == (e.data_id.timestamp() < at(50.0) ? 0u : 1u));
        }
    }
}

TEST_CASE("IntervalHelpers cases from JSON", "[eval][interval_helpers]")
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

        CHECK(result.sum_uis == c.expected.sum_uis);
        CHECK(result.misses  == c.expected.misses);
    }
}
