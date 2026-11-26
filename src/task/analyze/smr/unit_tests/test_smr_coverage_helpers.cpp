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

#include "analyze/smr/smr_coverage_helpers.h"

#include "json.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace analysis::SMRCoverageHelpers;

namespace
{

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/task/analyze/smr/unit_tests/smr_coverage_cases/";
}

/// bins as [[[r_min, r_max, count], ...], ...]
std::vector<CoverageBin> binsFrom(const nlohmann::json& j)
{
    std::vector<CoverageBin> bins;
    for (const auto& jb : j)
    {
        CoverageBin bin;
        for (const auto& js : jb)
        {
            CoverageSegment seg;
            seg.r_min = js.at(0).get<double>();
            seg.r_max = js.at(1).get<double>();
            seg.count = js.at(2).get<std::uint64_t>();
            bin.segments.push_back(seg);
            bin.count += seg.count;
        }
        bins.push_back(bin);
    }
    return bins;
}

CoverageRun runFrom(const nlohmann::json& j)
{
    CoverageRun run;
    run.cut     = j.at("cut").get<std::size_t>();
    run.begin_j = j.at("begin_j").get<std::size_t>();
    run.reports = j.at("reports").get<std::uint64_t>();
    for (const auto& e : j.at("band"))
        run.band.emplace_back(e.at(0).get<double>(), e.at(1).get<double>());
    return run;
}

void checkSegments(const std::vector<CoverageSegment>& segments, const nlohmann::json& expected)
{
    REQUIRE(segments.size() == expected.size());
    for (std::size_t i = 0; i < segments.size(); ++i)
    {
        CHECK(segments[i].r_min == Approx(expected[i].at(0).get<double>()));
        CHECK(segments[i].r_max == Approx(expected[i].at(1).get<double>()));
        CHECK(segments[i].count == expected[i].at(2).get<std::uint64_t>());
    }
}

void checkRuns(const std::vector<CoverageRun>& runs, const nlohmann::json& expected)
{
    REQUIRE(runs.size() == expected.size());
    for (std::size_t i = 0; i < runs.size(); ++i)
    {
        INFO("run " << i);
        auto e = runFrom(expected[i]);
        CHECK(runs[i].cut == e.cut);
        CHECK(runs[i].begin_j == e.begin_j);
        CHECK(runs[i].reports == e.reports);
        CHECK(runs[i].band == e.band);
    }
}

void checkCase(const nlohmann::json& j)
{
    const std::string type = j.at("type").get<std::string>();

    if (type == "polar")
    {
        for (const auto& c : j.at("checks"))
        {
            INFO("check " << c.dump());
            auto polar = polarOf(c.at("dx").get<double>(), c.at("dy").get<double>());
            if (c.at("range").is_null())
            {
                CHECK(!polar.has_value());
                continue;
            }
            REQUIRE(polar.has_value());
            CHECK(polar->range_m == Approx(c.at("range").get<double>()));
            CHECK(polar->azimuth_deg == Approx(c.at("azimuth").get<double>()).margin(1e-9));
        }
    }
    else if (type == "bins")
    {
        for (const auto& c : j.at("checks"))
        {
            const double bin_deg = c.at("bin_deg").get<double>();
            const std::size_t num_bins = numAzimuthBins(bin_deg);
            CHECK(num_bins == c.at("num_bins").get<std::size_t>());
            for (const auto& a : c.at("azimuths"))
                CHECK(azimuthBin(a.at(0).get<double>(), bin_deg, num_bins) == a.at(1).get<std::size_t>());
        }
    }
    else if (type == "split")
    {
        const auto ranges = j.at("ranges").get<std::vector<double>>();
        for (const auto& c : j.at("checks"))
        {
            INFO("gap_m " << c.at("gap_m").get<double>());
            checkSegments(splitSegments(ranges, c.at("gap_m").get<double>(),
                                        c.at("min_segment_reports").get<std::size_t>()),
                          c.at("segments"));
        }
    }
    else if (type == "close")
    {
        auto bins = binsFrom(j.at("bins"));
        closeSingleGaps(bins, j.at("passes").get<unsigned int>());
        const auto& e = j.at("expected");
        REQUIRE(bins.size() == e.size());
        for (std::size_t b = 0; b < bins.size(); ++b)
        {
            INFO("bin " << b);
            checkSegments(bins[b].segments, e[b]);
        }
    }
    else if (type == "chain")
    {
        auto bins = binsFrom(j.at("bins"));
        for (const auto& c : j.at("checks"))
        {
            INFO("link_m " << c.at("link_m").get<double>() << " bridge_bins " << c.at("bridge_bins").get<std::size_t>());
            checkRuns(chainRuns(bins, c.at("link_m").get<double>(), c.at("bridge_bins").get<std::size_t>()),
                      c.at("runs"));
        }
    }
    else if (type == "extent")
    {
        for (const auto& c : j.at("checks"))
        {
            auto extent = runExtent(runFrom(c.at("run")));
            CHECK(extent.r_min == Approx(c.at("r_min").get<double>()));
            CHECK(extent.r_max == Approx(c.at("r_max").get<double>()));
            CHECK(extent.density == Approx(c.at("density").get<double>()));
        }
    }
    else if (type == "widen")
    {
        std::vector<CoverageRun> runs { runFrom(j.at("run")) };
        widenRuns(runs, j.at("smooth_bins").get<int>());
        std::vector<std::pair<double, double>> expected;
        for (const auto& e : j.at("expected"))
            expected.emplace_back(e.at(0).get<double>(), e.at(1).get<double>());
        CHECK(runs.at(0).band == expected);
    }
    else if (type == "merge")
    {
        for (const auto& c : j.at("checks"))
        {
            std::vector<CoverageRun> runs;
            for (const auto& r : j.at("runs"))
                runs.push_back(runFrom(r));
            mergeOverlappingRuns(runs, c.at("tol_m").get<double>());
            checkRuns(runs, c.at("runs"));
        }
    }
    else if (type == "area")
    {
        for (const auto& c : j.at("checks"))
            CHECK(runArea(runFrom(c.at("run")), j.at("bin_deg").get<double>()) == Approx(c.at("area").get<double>()));
    }
    else if (type == "estimate")
    {
        const auto& p = j.at("params");
        CoverageParams params;
        params.bin_deg                  = p.at("bin_deg").get<double>();
        params.min_reports              = p.at("min_reports").get<std::size_t>();
        params.gap_m                    = p.at("gap_m").get<double>();
        params.min_segment_reports      = p.at("min_segment_reports").get<std::size_t>();
        params.smooth_bins              = p.at("smooth_bins").get<unsigned int>();
        params.min_run_reports_per_100m = p.at("min_run_reports_per_100m").get<double>();
        params.sector_link_m            = p.at("sector_link_m").get<double>();
        params.sector_bridge_bins       = p.at("sector_bridge_bins").get<std::size_t>();
        params.sector_smooth_bins       = p.at("sector_smooth_bins").get<int>();
        params.sector_simplify_m        = p.at("sector_simplify_m").get<double>();
        params.sector_min_area_m2       = p.at("sector_min_area_m2").get<double>();

        auto est = estimateCoverage(j.at("ranges_per_bin").get<std::vector<std::vector<double>>>(), params);
        const auto& e = j.at("expected");

        REQUIRE(est.bins.size() == e.at("bins").size());
        for (std::size_t b = 0; b < est.bins.size(); ++b)
        {
            INFO("bin " << b);
            CHECK(est.bins[b].count == e.at("bin_counts")[b].get<std::uint64_t>());
            checkSegments(est.bins[b].segments, e.at("bins")[b]);
        }

        REQUIRE(est.run_checks.size() == e.at("run_checks").size());
        for (std::size_t i = 0; i < est.run_checks.size(); ++i)
        {
            const auto& rc = est.run_checks[i];
            const auto& ec = e.at("run_checks")[i];
            INFO("run check " << i);
            CHECK(rc.azimuth_from_deg == Approx(ec.at("azimuth_from_deg").get<double>()));
            CHECK(rc.azimuth_to_deg == Approx(ec.at("azimuth_to_deg").get<double>()));
            CHECK(rc.num_bins == ec.at("num_bins").get<std::size_t>());
            CHECK(rc.reports == ec.at("reports").get<std::uint64_t>());
            CHECK(rc.band_m == Approx(ec.at("band_m").get<double>()));
            CHECK(rc.density == Approx(ec.at("density").get<double>()));
            CHECK(rc.cut == ec.at("cut").get<bool>());
        }

        checkRuns(est.sector_runs, e.at("sector_runs"));
        CHECK(est.slivers.size() == e.at("slivers").size());
    }
    else
    {
        FAIL("unknown case type " << type);
    }
}

} // namespace

TEST_CASE("SMRCoverageHelpers cases from JSON", "[analyze][smr_coverage_helpers]")
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
