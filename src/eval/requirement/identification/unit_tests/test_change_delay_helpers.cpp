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

#include "requirement/identification/change_delay_helpers.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement::ChangeDelayHelpers;

using boost::posix_time::ptime;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;

namespace
{

ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }

ptime at(double t_s) { return t0() + milliseconds((long)std::llround(t_s * 1000.0)); }

/// values from explicit [t, value|null] entries and series [t_from, t_to, step, value|null], sorted by time
std::vector<TimedValue> valuesFromJSON(const nlohmann::json& j, const std::string& key)
{
    std::vector<std::pair<double, boost::optional<std::string>>> values;

    auto value = [] (const nlohmann::json& v) -> boost::optional<std::string>
    {
        if (v.is_null())
            return {};
        return v.get<std::string>();
    };

    if (j.contains(key))
        for (const auto& e : j.at(key))
            values.emplace_back(e.at(0).get<double>(), value(e.at(1)));

    if (j.contains(key + "_series"))
    {
        for (const auto& s : j.at(key + "_series"))
        {
            double t_from = s.at(0).get<double>();
            double t_to   = s.at(1).get<double>();
            double step   = s.at(2).get<double>();

            for (double t = t_from; t <= t_to + 1e-9; t += step)
                values.emplace_back(t, value(s.at(3)));
        }
    }

    std::stable_sort(values.begin(), values.end(), [] (const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<TimedValue> timed_values;
    for (const auto& v : values)
        timed_values.emplace_back(at(v.first), v.second);

    return timed_values;
}

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/identification/unit_tests/change_delay_cases/";
}

} // namespace

TEST_CASE("ChangeDelayHelpers::normalizedACID", "[eval][change_delay_helpers]")
{
    CHECK(normalizedACID("DLH123  ").value() == "DLH123");
    CHECK(normalizedACID("DLH123").value() == "DLH123");
    CHECK(!normalizedACID("        ").has_value());
    CHECK(!normalizedACID("").has_value());
}

TEST_CASE("ChangeDelayHelpers::changeEvents keeps old and new value", "[eval][change_delay_helpers]")
{
    std::vector<TimedValue> ref;
    for (int t = 0; t <= 100; ++t)
        ref.emplace_back(at(t), std::string(t < 50 ? "1000" : "4567"));

    auto changes = changeEvents(ref, 10.0f);

    REQUIRE(changes.size() == 1);
    CHECK(changes[ 0 ].ref_index == 50);
    CHECK(changes[ 0 ].old_value == "1000");
    CHECK(changes[ 0 ].new_value == "4567");
}

TEST_CASE("ChangeDelayHelpers cases from JSON", "[eval][change_delay_helpers]")
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

        INFO("case " << file.filename().string() << ": " << j.value("description", ""));

        float max_delay_s       = j.at("params").at("max_delay_s").get<float>();
        float min_stable_time_s = j.at("params").at("min_stable_time_s").get<float>();

        auto ref = valuesFromJSON(j, "ref");
        auto tst = valuesFromJSON(j, "tst");

        auto changes = changeEvents(ref, min_stable_time_s);

        unsigned int passed = 0, failed = 0, never_reported = 0, not_assessable = 0;
        std::vector<double> delays;

        for (const auto& change : changes)
        {
            auto a = assessEvent(ref[ change.ref_index ].first, change.new_value, tst, max_delay_s);

            switch (a.result)
            {
                case Assessment::Result::Passed:        ++passed;         break;
                case Assessment::Result::Failed:        ++failed;         break;
                case Assessment::Result::NeverReported: ++never_reported; break;
                case Assessment::Result::NotAssessable: ++not_assessable; break;
            }

            if (a.delay_s.has_value())
                delays.push_back(a.delay_s.value());
        }

        const auto& e = j.at("expected");

        CHECK(changes.size()  == e.at("events").get<size_t>());
        CHECK(passed          == e.at("passed").get<unsigned int>());
        CHECK(failed          == e.at("failed").get<unsigned int>());
        CHECK(never_reported  == e.at("never_reported").get<unsigned int>());
        CHECK(not_assessable  == e.at("not_assessable").get<unsigned int>());

        auto expected_delays = e.at("delays").get<std::vector<double>>();
        REQUIRE(delays.size() == expected_delays.size());
        for (size_t i = 0; i < delays.size(); ++i)
            CHECK(delays[ i ] == Approx(expected_delays[ i ]));
    }
}
