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

#include "results/base/valueaccumulator.h"

#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace
{

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/results/base/unit_tests/accumulator_cases/";
}

void checkStats(const ValueAccumulator& acc, const nlohmann::json& e)
{
    CHECK(acc.numValues() == e.at("num").get<size_t>());
    CHECK(acc.min()    == Approx(e.at("min").get<double>()));
    CHECK(acc.max()    == Approx(e.at("max").get<double>()));
    CHECK(acc.mean()   == Approx(e.at("mean").get<double>()));
    CHECK(acc.rms()    == Approx(e.at("rms").get<double>()));
    CHECK(acc.var()    == Approx(e.at("var").get<double>()).margin(1e-9));
    CHECK(acc.stddev() == Approx(e.at("stddev").get<double>()).margin(1e-6));
    CHECK(!std::isnan(acc.stddev()));
}

} // namespace

TEST_CASE("ValueAccumulator cases from JSON", "[eval][value_accumulator]")
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

        const auto& groups = j.at("groups");

        // each group accumulated on its own, then joined into an empty accumulator
        // (as the sector sums join the single target results)
        ValueAccumulator joined;

        for (const auto& group : groups)
        {
            ValueAccumulator single;
            single.accumulate(group.get<std::vector<double>>(), true);

            joined.join(single, false);
        }

        joined.updateStats();

        {
            INFO("joined");
            checkStats(joined, j.at("expected"));
        }

        // all values accumulated at once give the same statistics
        std::vector<double> all;
        for (const auto& group : groups)
        {
            auto values = group.get<std::vector<double>>();
            all.insert(all.end(), values.begin(), values.end());
        }

        ValueAccumulator direct;
        direct.accumulate(all, true);

        {
            INFO("direct");
            checkStats(direct, j.at("expected"));
        }
    }
}
