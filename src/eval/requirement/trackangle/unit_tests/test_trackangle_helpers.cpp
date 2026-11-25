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

#include "requirement/trackangle/trackangle_helpers.h"

#include "json.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::TrackAngleHelpers;

namespace
{

COMPARISON_TYPE checkTypeFromString(const std::string& s)
{
    if (s == "<")  return COMPARISON_TYPE::LESS_THAN;
    if (s == "<=") return COMPARISON_TYPE::LESS_THAN_OR_EQUAL;
    if (s == ">")  return COMPARISON_TYPE::GREATER_THAN;
    if (s == ">=") return COMPARISON_TYPE::GREATER_THAN_OR_EQUAL;

    throw std::runtime_error("unknown check type '" + s + "'");
}

std::string outcomeString(Outcome o)
{
    switch (o)
    {
        case Outcome::RefSpeedTooLow: return "ref_speed_low";
        case Outcome::NoTstValue:     return "no_tst";
        case Outcome::Passed:         return "passed";
        case Outcome::Failed:         return "failed";
    }
    return "";
}

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/trackangle/unit_tests/trackangle_cases/";
}

} // namespace

TEST_CASE("TrackAngleHelpers cases from JSON", "[eval][trackangle_helpers]")
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

        const auto& jp = j.at("params");

        Params p;
        p.threshold         = jp.at("threshold").get<float>();
        p.use_minimum_speed = jp.at("use_minimum_speed").get<bool>();
        p.minimum_speed     = jp.at("minimum_speed").get<float>();
        p.check_type        = checkTypeFromString(jp.at("check_type").get<std::string>());

        unsigned int num_passed {0}, num_failed {0}, num_no_tst {0}, num_ref_speed_low {0};

        for (const auto& r : j.at("reports"))
        {
            INFO("report " << r.dump());

            boost::optional<double> tst;
            if (!r.at("tst").is_null())
                tst = r.at("tst").get<double>();

            auto c = check(r.at("ref_speed").get<double>(), r.at("ref").get<double>(), tst, p);

            CHECK(outcomeString(c.outcome) == r.at("outcome").get<std::string>());

            if (r.contains("diff"))
                CHECK(c.diff == Approx(r.at("diff").get<double>()).margin(1e-4));

            switch (c.outcome)
            {
                case Outcome::RefSpeedTooLow: ++num_ref_speed_low; break;
                case Outcome::NoTstValue:     ++num_no_tst;        break;
                case Outcome::Passed:         ++num_passed;        break;
                case Outcome::Failed:         ++num_failed;        break;
            }
        }

        const auto& e = j.at("expected");

        CHECK(num_passed        == e.at("passed").get<unsigned int>());
        CHECK(num_failed        == e.at("failed").get<unsigned int>());
        CHECK(num_no_tst        == e.at("no_tst").get<unsigned int>());
        CHECK(num_ref_speed_low == e.at("ref_speed_low").get<unsigned int>());

        // probability as in SingleTrackAngle: passed / (passed + failed)
        REQUIRE(num_passed + num_failed > 0);
        CHECK((double)num_passed / (double)(num_passed + num_failed) == Approx(e.at("probability").get<double>()));
    }
}

TEST_CASE("TrackAngleHelpers detail code of the outcomes", "[eval][trackangle_helpers]")
{
    Params p;
    p.threshold         = 10.0f;
    p.use_minimum_speed = true;
    p.minimum_speed     = 5.0f;

    CHECK(check(2.0, 90.0, 90.0, p).code == RefSpeedTooLow);
    CHECK(check(2.0, 90.0, {}, p).code == RefSpeedTooLow); // the speed is checked first
    CHECK(check(50.0, 90.0, {}, p).code == NoTstTrackAngle);
    CHECK(check(50.0, 90.0, 95.0, p).code == 0);
    CHECK(check(50.0, 90.0, 130.0, p).code == 0); // failed: the failed check says it
}
