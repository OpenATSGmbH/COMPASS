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

#include "analyze/coverage_summary_helpers.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace analysis::CoverageSummaryHelpers;

namespace
{

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/task/analyze/unit_tests/coverage_summary_cases/";
}

boost::posix_time::ptime timeAt(double seconds)
{
    static const auto t0 = boost::posix_time::time_from_string("2026-01-01 00:00:00.000");
    return t0 + boost::posix_time::microseconds(static_cast<long>(seconds * 1e6));
}

void checkPD(const nlohmann::json& j)
{
    for (const auto& c : j.at("checks"))
    {
        INFO("check " << c.dump());

        const double eui = c.at("eui").get<double>();
        const double mui = c.at("mui").get<double>();

        CHECK(pd(eui, mui) == Approx(c.at("pd").get<double>()));

        auto cell_pd = cellPD(eui, mui);

        if (c.at("cell_pd").is_null())
            CHECK(!cell_pd.has_value());
        else
        {
            REQUIRE(cell_pd.has_value());
            CHECK(*cell_pd == Approx(c.at("cell_pd").get<double>()));
        }
    }
}

void checkSummary(const nlohmann::json& j)
{
    std::vector<CellCounts> cells;
    for (const auto& c : j.at("cells"))
        cells.push_back({ c.at(0).get<double>(), c.at(1).get<double>() });

    auto s = summarizePD(cells);
    const auto& e = j.at("expected");

    CHECK(s.total_eui == Approx(e.at("total_eui").get<double>()));
    CHECK(s.total_mui == Approx(e.at("total_mui").get<double>()));
    CHECK(s.overall_pd == Approx(e.at("overall_pd").get<double>()));
    CHECK(s.cells_with_eui == e.at("cells_with_eui").get<std::size_t>());
    CHECK(s.median_cell_pd == Approx(e.at("median_cell_pd").get<double>()));
    CHECK(s.p5_cell_pd == Approx(e.at("p5_cell_pd").get<double>()));
    CHECK(s.has_worst_cell == e.at("has_worst_cell").get<bool>());
    CHECK(s.worst_cell_pd == Approx(e.at("worst_cell_pd").get<double>()));
}

void checkIntervals(const nlohmann::json& j)
{
    std::vector<boost::posix_time::ptime> times;
    for (double t : j.at("times_s").get<std::vector<double>>())
        times.push_back(timeAt(t));

    auto intervals = j.at("existing").get<std::vector<double>>();

    appendUpdateIntervals(times, intervals, j.at("max_interval_s").get<double>());

    auto expected = j.at("expected").get<std::vector<double>>();

    REQUIRE(intervals.size() == expected.size());
    for (std::size_t cnt = 0; cnt < expected.size(); ++cnt)
        CHECK(intervals[cnt] == Approx(expected[cnt]));
}

void checkStats(const nlohmann::json& j)
{
    auto s = intervalStats(j.at("intervals").get<std::vector<double>>(), j.at("hist_max_s").get<double>(),
                           j.at("num_bins").get<int>(), j.at("fallback_max_s").get<double>());
    const auto& e = j.at("expected");

    CHECK(s.num == e.at("num").get<std::size_t>());
    CHECK(s.median_s == Approx(e.at("median_s").get<double>()));
    CHECK(s.p10_s == Approx(e.at("p10_s").get<double>()));
    CHECK(s.p90_s == Approx(e.at("p90_s").get<double>()));
    CHECK(s.mean_s == Approx(e.at("mean_s").get<double>()));
    CHECK(s.hist_max_s == Approx(e.at("hist_max_s").get<double>()));
    CHECK(s.hist_bin_width_s == Approx(e.at("hist_bin_width_s").get<double>()));
    CHECK(s.hist_bins == e.at("hist_bins").get<std::vector<std::uint32_t>>());
    CHECK(s.hist_overflow == e.at("hist_overflow").get<std::uint32_t>());
}

} // namespace

TEST_CASE("CoverageSummaryHelpers cases from JSON", "[analyze][coverage_summary_helpers]")
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

        const std::string type = j.at("type").get<std::string>();

        if (type == "pd")
            checkPD(j);
        else if (type == "summary")
            checkSummary(j);
        else if (type == "intervals")
            checkIntervals(j);
        else if (type == "stats")
            checkStats(j);
        else
            FAIL("unknown case type " << type);
    }
}

TEST_CASE("CoverageSummaryHelpers empty input", "[analyze][coverage_summary_helpers]")
{
    auto s = intervalStats({}, 0.0, 10, 15.0);

    CHECK(s.num == 0);
    CHECK(s.hist_bins.empty());

    auto summary = summarizePD({});

    CHECK(summary.cells_with_eui == 0);
    CHECK(summary.overall_pd == 0.0);
    CHECK(!summary.has_worst_cell);
}
