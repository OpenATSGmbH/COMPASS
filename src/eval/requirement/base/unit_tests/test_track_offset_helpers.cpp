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

#include "requirement/base/trackoffsethelpers.h"

#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::TrackOffsetHelpers;

namespace
{

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/base/unit_tests/track_offset_cases/";
}

} // namespace

TEST_CASE("TrackOffsetHelpers cases from JSON", "[eval][track_offset_helpers]")
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

        for (const auto& o : j.at("offsets"))
        {
            INFO("offset " << o.dump());

            auto offset = trackOffset(o.at("distance").get<double>(),
                                      o.at("offset_angle_deg").get<double>() * M_PI / 180.0,
                                      o.at("track_angle_deg").get<double>());

            CHECK(offset.along  == Approx(o.at("along").get<double>()).margin(1e-9));
            CHECK(offset.across == Approx(o.at("across").get<double>()).margin(1e-9));
        }
    }
}
