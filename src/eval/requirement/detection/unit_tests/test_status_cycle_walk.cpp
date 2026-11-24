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

#include "requirement/detection/detection_walk.h"
#include "timeperiod.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::DetectionWalk;

using boost::posix_time::ptime;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;

namespace
{

ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }

ptime at(double t_s) { return t0() + milliseconds((long)std::llround(t_s * 1000.0)); }

double seconds(const ptime& t) { return (t - t0()).total_milliseconds() / 1000.0; }

/// explicit times [t, ...] and/or series [t_from, t_to, step]
std::vector<ptime> timesFromJSON(const nlohmann::json& j, const std::string& key, const std::string& series_key)
{
    std::vector<double> times_s;

    if (j.contains(key))
        for (const auto& t : j.at(key))
            times_s.push_back(t.get<double>());

    if (j.contains(series_key))
        for (const auto& s : j.at(series_key))
            for (double t = s.at(0).get<double>(); t <= s.at(1).get<double>() + 1e-9; t += s.at(2).get<double>())
                times_s.push_back(t);

    std::sort(times_s.begin(), times_s.end());

    std::vector<ptime> times;
    for (double t : times_s)
        times.push_back(at(t));

    return times;
}

std::string casesDir()
{
    // the source path of the build, or COMPASS_SOURCE_DIR (e.g. tests of a cross build)
    const char* source_dir_env = std::getenv("COMPASS_SOURCE_DIR");
    return std::string(source_dir_env ? source_dir_env : COMPASS_SOURCE_DIR)
        + "/src/eval/requirement/detection/unit_tests/status_cycle_cases/";
}

} // namespace

TEST_CASE("DetectionWalk::cyclesPerPeriod", "[eval][status_cycle_walk]")
{
    auto cycles = [] (double step_s, unsigned int num)
    {
        std::vector<ptime> c;
        for (unsigned int i = 0; i < num; ++i)
            c.push_back(at(i * step_s));
        return c;
    };

    CHECK(cyclesPerPeriod(cycles(1.0, 10), 1.0f) == 1);
    CHECK(cyclesPerPeriod(cycles(1.0, 10), 2.0f) == 2);
    CHECK(cyclesPerPeriod(cycles(1.0, 10), 5.0f) == 5);
    CHECK(cyclesPerPeriod(cycles(2.0, 10), 5.0f) == 3);  // 2.5 rounded away from zero
    CHECK(cyclesPerPeriod(cycles(10.0, 10), 1.0f) == 1); // never below 1 cycle
    CHECK(cyclesPerPeriod(cycles(4.0, 2), 8.0f) == 2);   // 2 cycles give 1 length

    // the median ignores single outliers (one cycle gap of 30 s)
    std::vector<ptime> with_gap { at(0), at(1), at(2), at(32), at(33) };
    CHECK(cyclesPerPeriod(with_gap, 2.0f) == 2);
}

TEST_CASE("DetectionWalk::cycleStats", "[eval][status_cycle_walk]")
{
    std::vector<ptime> regular;
    for (unsigned int i = 0; i <= 10; ++i)
        regular.push_back(at(i));

    auto s = cycleStats(regular, 1.0f);
    CHECK(s.median_cycle_s == Approx(1.0));
    CHECK(s.cycles_per_period == 1);
    CHECK(s.period_s == Approx(1.0));
    CHECK(s.num_outages == 0);

    // update interval 2 s on 1 s cycles: periods of 2 cycles
    s = cycleStats(regular, 2.0f);
    CHECK(s.cycles_per_period == 2);
    CHECK(s.period_s == Approx(2.0));

    // cycles every 10 s and update interval 1 s: periods of 10 s
    std::vector<ptime> slow { at(0), at(10), at(20), at(30) };
    s = cycleStats(slow, 1.0f);
    CHECK(s.cycles_per_period == 1);
    CHECK(s.period_s == Approx(10.0));

    // the cycles of 07_cycle_outage: 0.9 s and 1.1 s are no outage (below 1.5 median cycles), 12 s is one
    std::vector<ptime> outage { at(0), at(1), at(2), at(3), at(3.9), at(5), at(6), at(7), at(8), at(20), at(21) };
    s = cycleStats(outage, 2.0f);
    CHECK(s.median_cycle_s == Approx(1.0));
    CHECK(s.num_outages == 1);
    CHECK(s.longest_outage_s == Approx(12.0));

    // the threshold: 1.5 median cycles is no outage, above it is
    std::vector<ptime> at_threshold { at(0), at(1), at(2), at(3.5), at(4.5), at(6.1), at(7.1) };
    s = cycleStats(at_threshold, 1.0f);
    CHECK(s.num_outages == 1);
    CHECK(s.longest_outage_s == Approx(1.6));
}

TEST_CASE("DetectionWalk::statusCycleWalk cases from JSON", "[eval][status_cycle_walk]")
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

        auto cycles = timesFromJSON(j, "cycles", "cycle_series");

        TimePeriodCollection ref_periods;
        for (const auto& p : j.at("ref_periods"))
            ref_periods.add({ at(p.at(0).get<double>()), at(p.at(1).get<double>()) });

        // test reports [t, "ok" | "excluded"] and/or series [t_from, t_to, step] of "ok" reports
        std::vector<std::pair<double, bool>> tst_s;

        if (j.contains("tst"))
            for (const auto& t : j.at("tst"))
                tst_s.emplace_back(t.at(0).get<double>(), t.at(1).get<std::string>() == "ok");

        if (j.contains("tst_series"))
            for (const auto& s : j.at("tst_series"))
                for (double t = s.at(0).get<double>(); t <= s.at(1).get<double>() + 1e-9; t += s.at(2).get<double>())
                    tst_s.emplace_back(t, true);

        std::sort(tst_s.begin(), tst_s.end());

        std::vector<ptime> tst_times;
        std::vector<bool>  tst_detects;
        for (const auto& t : tst_s)
        {
            tst_times.push_back(at(t.first));
            tst_detects.push_back(t.second);
        }

        const auto& e = j.at("expected");

        unsigned int cycles_per_period = cyclesPerPeriod(cycles, j.at("update_interval_s").get<float>());
        CHECK(cycles_per_period == e.at("cycles_per_period").get<unsigned int>());

        auto result = statusCycleWalk(ref_periods, cycles, tst_times,
                                      [ & ] (size_t i) { return (bool)tst_detects.at(i); }, cycles_per_period);

        CHECK(result.sum_expected == Approx(e.at("sum_expected").get<double>()));
        CHECK(result.sum_missed   == Approx(e.at("sum_missed").get<double>()));

        const auto& misses = e.at("misses");
        REQUIRE(result.misses.size() == misses.size());

        for (size_t i = 0; i < misses.size(); ++i)
        {
            INFO("miss " << i);
            CHECK(seconds(result.misses[ i ].begin) == Approx(misses[ i ].at(0).get<double>()));
            CHECK(seconds(result.misses[ i ].end)   == Approx(misses[ i ].at(1).get<double>()));
            CHECK(result.misses[ i ].sum_missed     == Approx((double)(i + 1)));
        }
    }
}
