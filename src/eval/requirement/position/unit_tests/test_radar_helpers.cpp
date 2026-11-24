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

#include "requirement/position/radar_helpers.h"

#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::RadarHelpers;

namespace
{

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/position/unit_tests/radar_cases/";
}

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

void checkReports(const nlohmann::json& j)
{
    const std::string type = j.at("type").get<std::string>();
    const double threshold = j.at("threshold").get<double>();

    unsigned int num_passed {0}, num_failed {0};

    for (const auto& r : j.at("reports"))
    {
        INFO("report " << r.dump());

        double difference = type == "azimuth"
            ? azimuthDifferenceDeg(deg2rad(r.at("ref_deg").get<double>()), deg2rad(r.at("tst_deg").get<double>()))
            : r.at("ref_m").get<double>() - r.at("tst_m").get<double>();

        CHECK(difference == Approx(r.at("difference").get<double>()).margin(1e-9));

        bool ok = passed(difference, threshold);

        CHECK(ok == r.at("passed").get<bool>());

        if (ok)
            ++num_passed;
        else
            ++num_failed;
    }

    const auto& e = j.at("expected");

    CHECK(num_passed == e.at("passed").get<unsigned int>());
    CHECK(num_failed == e.at("failed").get<unsigned int>());

    // probability as in the Radar Azimuth and Range results: passed / (passed + failed)
    REQUIRE(num_passed + num_failed > 0);
    CHECK((double)num_passed / (double)(num_passed + num_failed) == Approx(e.at("probability").get<double>()));
}

void checkFit(const nlohmann::json& j)
{
    auto fit = linearFit(j.at("tst").get<std::vector<double>>(), j.at("ref").get<std::vector<double>>());

    const auto& e = j.at("expected");

    CHECK(fit.gain == Approx(e.at("gain").get<double>()).margin(1e-9));
    CHECK(fit.bias == Approx(e.at("bias").get<double>()).margin(1e-6));
}

} // namespace

TEST_CASE("RadarHelpers cases from JSON", "[eval][radar_helpers]")
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

        if (j.at("type").get<std::string>() == "fit")
            checkFit(j);
        else
            checkReports(j);
    }
}
