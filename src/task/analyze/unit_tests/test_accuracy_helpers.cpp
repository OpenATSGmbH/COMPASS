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

#include "analyze/accuracy_helpers.h"

#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>

using namespace analysis::AccuracyHelpers;

namespace
{

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/task/analyze/unit_tests/accuracy_cases/";
}

/// null in the cases is a NaN input
double valueOrNaN(const nlohmann::json& j)
{
    return j.is_null() ? std::numeric_limits<double>::quiet_NaN() : j.get<double>();
}

/// null in the cases is no value
std::optional<double> optionalValue(const nlohmann::json& j)
{
    return j.is_null() ? std::nullopt : std::optional<double>(j.get<double>());
}

void checkOptional(const std::optional<double>& value, const nlohmann::json& expected)
{
    if (expected.is_null())
    {
        CHECK(!value.has_value());
        return;
    }

    REQUIRE(value.has_value());
    CHECK(*value == Approx(expected.get<double>()));
}

void checkCase(const nlohmann::json& j)
{
    const std::string type = j.at("type").get<std::string>();

    for (const auto& c : j.at("checks"))
    {
        INFO("check " << c.dump());

        if (type == "reported")
            checkOptional(reportedStdDev(valueOrNaN(c.at(0)), valueOrNaN(c.at(1))), c.at(2));
        else if (type == "reference")
            checkOptional(referenceStdDev(valueOrNaN(c.at(0)), valueOrNaN(c.at(1))), c.at(2));
        else if (type == "gate")
            CHECK(refGated(c.at("factor").get<double>(), optionalValue(c.at("ref")), optionalValue(c.at("claimed")))
                  == c.at("gated").get<bool>());
        else if (type == "ratio")
            checkOptional(consistencyRatio(c.at(0).get<double>(), optionalValue(c.at(1))), c.at(2));
        else if (type == "stats")
        {
            auto stats = valueStats(c.at("values").get<std::vector<double>>());
            CHECK(stats.num == c.at("num").get<std::size_t>());
            CHECK(stats.median == Approx(c.at("median").get<double>()));
            CHECK(stats.mean == Approx(c.at("mean").get<double>()));
            CHECK(stats.p95 == Approx(c.at("p95").get<double>()));
        }
        else
            FAIL("unknown case type " << type);
    }
}

} // namespace

TEST_CASE("AccuracyHelpers cases from JSON", "[analyze][accuracy_helpers]")
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

        checkCase(j);
    }
}
