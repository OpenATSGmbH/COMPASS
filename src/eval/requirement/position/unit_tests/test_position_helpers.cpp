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

#include "requirement/position/positionhelpers.h"
#include "requirement/base/comparisontype.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::PositionHelpers;

using boost::posix_time::ptime;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;

namespace
{

ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }

ptime at(double t_s) { return t0() + milliseconds((long)std::llround(t_s * 1000.0)); }

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/position/unit_tests/averaging_cases/";
}

} // namespace

TEST_CASE("compareValue", "[eval][position_helpers]")
{
    CHECK(compareValue(1.0, 2.0, COMPARISON_TYPE::LESS_THAN));
    CHECK(!compareValue(2.0, 2.0, COMPARISON_TYPE::LESS_THAN));
    CHECK(compareValue(2.0, 2.0, COMPARISON_TYPE::LESS_THAN_OR_EQUAL));
    CHECK(!compareValue(2.1, 2.0, COMPARISON_TYPE::LESS_THAN_OR_EQUAL));
    CHECK(compareValue(2.1, 2.0, COMPARISON_TYPE::GREATER_THAN));
    CHECK(!compareValue(2.0, 2.0, COMPARISON_TYPE::GREATER_THAN));
    CHECK(compareValue(2.0, 2.0, COMPARISON_TYPE::GREATER_THAN_OR_EQUAL));
    CHECK(!compareValue(1.9, 2.0, COMPARISON_TYPE::GREATER_THAN_OR_EQUAL));
}

TEST_CASE("PositionHelpers::averagingWindows cases from JSON", "[eval][position_helpers]")
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

        std::vector<ErrorSample> samples;
        for (const auto& s : j.at("samples"))
            samples.push_back({ at(s.at(0).get<double>()), s.at(1).get<double>(), s.at(2).get<double>() });

        auto windows = averagingWindows(samples, j.at("window_s").get<double>());

        const auto& expected = j.at("expected_windows");
        REQUIRE(windows.size() == expected.size());

        for (size_t i = 0; i < windows.size(); ++i)
        {
            INFO("window " << i);
            CHECK(windows[ i ].end - windows[ i ].first == expected[ i ].at("num").get<size_t>());
            CHECK(windows[ i ].mean_distance == Approx(expected[ i ].at("mean_distance").get<double>()));
            CHECK(windows[ i ].span_s == Approx(expected[ i ].at("span_s").get<double>()));
        }
    }
}
