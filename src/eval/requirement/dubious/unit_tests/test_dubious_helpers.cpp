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

#include "requirement/dubious/dubious_helpers.h"

#include "json.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::DubiousHelpers;

namespace
{

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/dubious/unit_tests/dubious_cases/";
}

boost::optional<double> optionalValue(const nlohmann::json& j, const std::string& key)
{
    if (!j.contains(key))
        return {};

    return j.at(key).get<double>();
}

void checkFlag(const boost::optional<double>& flag, const nlohmann::json& expected, const std::string& key)
{
    INFO("flag " << key);

    CHECK(flag.has_value() == expected.contains(key));

    if (flag.has_value() && expected.contains(key))
        CHECK(*flag == Approx(expected.at(key).get<double>()).margin(1e-4));
}

void checkUpdatesCase(const nlohmann::json& j)
{
    const auto& jp = j.at("params");

    Params p;
    p.use_max_groundspeed    = jp.at("use_max_groundspeed").get<bool>();
    p.max_groundspeed_kts    = jp.at("max_groundspeed_kts").get<double>();
    p.check_calculated_speed = jp.at("check_calculated_speed").get<bool>();
    p.use_max_acceleration   = jp.at("use_max_acceleration").get<bool>();
    p.max_acceleration       = jp.at("max_acceleration").get<double>();
    p.use_max_turnrate       = jp.at("use_max_turnrate").get<bool>();
    p.max_turnrate           = jp.at("max_turnrate").get<double>();
    p.use_max_rocd           = jp.at("use_max_rocd").get<bool>();
    p.max_rocd               = jp.at("max_rocd").get<double>();
    p.min_comparison_time    = jp.at("min_comparison_time").get<double>();
    p.max_comparison_time    = jp.at("max_comparison_time").get<double>();

    std::vector<Update> updates;

    for (const auto& ju : j.at("updates"))
    {
        Update u;
        u.time_s                 = ju.at("time").get<double>();
        u.speed_ms               = optionalValue(ju, "speed");
        u.track_angle_deg        = optionalValue(ju, "track");
        u.mode_c_ft              = optionalValue(ju, "mode_c");
        u.distance_to_previous_m = ju.value("distance", 0.0);
        updates.push_back(u);
    }

    auto flags = checkUpdates(updates, p);

    REQUIRE(flags.size() == updates.size());

    for (size_t cnt = 0; cnt < flags.size(); ++cnt)
    {
        INFO("update " << cnt);

        const auto& e = j.at("updates").at(cnt).at("expected");

        checkFlag(flags[cnt].reported_speed_ms, e, "reported_speed_ms");
        checkFlag(flags[cnt].calculated_speed_kts, e, "calculated_speed_kts");
        checkFlag(flags[cnt].acceleration, e, "acceleration");
        checkFlag(flags[cnt].turnrate, e, "turnrate");
        checkFlag(flags[cnt].rocd, e, "rocd");
    }
}

} // namespace

TEST_CASE("DubiousHelpers cases from JSON", "[eval][dubious_helpers]")
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

        if (j.contains("updates"))
            checkUpdatesCase(j);

        if (j.contains("ratios"))
        {
            for (const auto& r : j.at("ratios"))
            {
                INFO("ratio " << r.dump());

                CHECK(isDubious(r.at("num_dubious").get<unsigned int>(), r.at("num_updates").get<unsigned int>(),
                                r.at("dubious_prob").get<float>()) == r.at("is_dubious").get<bool>());
            }
        }
    }
}
