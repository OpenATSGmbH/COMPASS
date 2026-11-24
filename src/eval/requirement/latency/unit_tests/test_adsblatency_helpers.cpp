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

#include "requirement/latency/adsblatency_helpers.h"

#include "json.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::ADSBLatencyHelpers;

namespace
{

std::string outcomeString(Outcome o)
{
    switch (o)
    {
        case Outcome::NoTimeData: return "no_time_data";
        case Outcome::Passed:     return "passed";
        case Outcome::Negative:   return "negative";
        case Outcome::TooHigh:    return "too_high";
    }
    return "";
}

boost::optional<float> optionalTime(const nlohmann::json& j)
{
    if (j.is_null())
        return {};

    return j.get<float>();
}

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/latency/unit_tests/adsblatency_cases/";
}

} // namespace

TEST_CASE("ADSBLatencyHelpers cases from JSON", "[eval][adsblatency_helpers]")
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

        const float max_value_s = j.at("max_value_s").get<float>();

        unsigned int num_passed {0}, num_failed {0}, num_no_time_data {0};

        for (const auto& r : j.at("reports"))
        {
            INFO("report " << r.dump());

            auto c = check(optionalTime(r.at("tomr")), optionalTime(r.at("tort")), max_value_s);

            CHECK(outcomeString(c.outcome) == r.at("outcome").get<std::string>());

            if (r.contains("latency"))
                CHECK(c.latency == Approx(r.at("latency").get<double>()).margin(1e-4));

            if (c.outcome == Outcome::Passed)
                ++num_passed;
            else if (c.outcome == Outcome::NoTimeData)
                ++num_no_time_data;
            else
                ++num_failed;
        }

        const auto& e = j.at("expected");

        CHECK(num_passed == e.at("passed").get<unsigned int>());
        CHECK(num_failed == e.at("failed").get<unsigned int>());
        CHECK(num_no_time_data == e.at("no_time_data").get<unsigned int>());

        // probability as in SingleADSBLatency: passed / (passed + failed)
        REQUIRE(num_passed + num_failed > 0);
        CHECK((double)num_passed / (double)(num_passed + num_failed) == Approx(e.at("probability").get<double>()));
    }
}
