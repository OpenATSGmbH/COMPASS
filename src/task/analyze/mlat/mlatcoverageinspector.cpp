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

#include "mlatcoverageinspector.h"
#include "analysedatasourcetask.h"
#include "analysisdataset.h"
#include "targetreport3dgrid.h"

#include "compass.h"
#include "logger.h"
#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"
#include "targetposition.h"
#include "dbcontent/target/targetreportchain.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using dbContent::TargetReport::Chain;

MLATCoverageInspectorSettings::MLATCoverageInspectorSettings(nlohmann::json& config_json,
                                                             Configurable* parent)
    : InspectorSettingsBase(config_json, parent)
{
    registerParameter("pd_method_int",   &pd_method_int_,   pd_method_int_);
    registerParameter("update_interval_s", &update_interval_s_, update_interval_s_);

    registerParameter("use_miss_tolerance", &use_miss_tolerance_, use_miss_tolerance_);
    registerParameter("miss_tolerance_s",   &miss_tolerance_s_,   miss_tolerance_s_);

    registerParameter("pd_acceptable_above",   &pd_acceptable_above_,   pd_acceptable_above_);
    registerParameter("pd_unacceptable_below", &pd_unacceptable_below_, pd_unacceptable_below_);
}

MLATCoverageInspector::MLATCoverageInspector(AnalyseDataSourceTask& task,
                                             MLATCoverageInspectorSettings& settings)
    : DataSourceInspectorBase(task, settings)
{
}

std::set<std::string> MLATCoverageInspector::testDBContentNames() const
{
    return {"CAT020", "CAT010"};
}

bool MLATCoverageInspector::prerequisitesMet(std::string& reason_out) const
{
    reason_out.clear();

    if (task_.selectedDataSourceIDs().empty())
    {
        reason_out = "No data source selected.";
        return false;
    }
    return true;
}

namespace
{
using Settings = MLATCoverageInspectorSettings;

bool gapIsMiss(double gap_s, const Settings& s)
{
    double adj = gap_s;
    if (s.use_miss_tolerance_)
        adj -= s.miss_tolerance_s_;
    if (adj <= s.update_interval_s_)
        return false;
    return true;
}

std::uint32_t numMissesIn(double gap_s, const Settings& s)
{
    double adj = gap_s;
    if (s.use_miss_tolerance_)
        adj -= s.miss_tolerance_s_;
    if (adj <= 0.0 || s.update_interval_s_ <= 0.0)
        return 0;
    return static_cast<std::uint32_t>(std::floor(adj / s.update_interval_s_));
}

double partialSeconds(const time_duration& d)
{
    return static_cast<double>(d.total_microseconds()) / 1.0e6;
}

ptime addSeconds(ptime t, double s)
{
    long long us = static_cast<long long>(std::llround(s * 1.0e6));
    return t + boost::posix_time::microseconds(us);
}

double percentile(std::vector<double> v, double p)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    if (p <= 0.0) return v.front();
    if (p >= 1.0) return v.back();
    double idx = p * (v.size() - 1);
    size_t lo = static_cast<size_t>(std::floor(idx));
    size_t hi = static_cast<size_t>(std::ceil(idx));
    double w = idx - lo;
    return v[lo] * (1.0 - w) + v[hi] * w;
}

std::string formatNumber(double v, int prec = 4)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}
}

void MLATCoverageInspector::compute(AnalysisDataset* dataset)
{
    result_ = ComputeResult{};

    if (!dataset)
        return;

    auto& settings = static_cast<MLATCoverageInspectorSettings&>(settings_);

    double ref_lat = dataset->centreLatitudeDeg();
    if (ref_lat == 0.0) ref_lat = 47.0;

    TargetReport3DGrid grid(task_.cellSizeMeters(), task_.cellSizeFeet(), ref_lat);

    time_duration d_max = boost::posix_time::seconds(60);

    unsigned int targets_walked = 0;
    unsigned int targets_no_ref = 0;
    unsigned int targets_no_tst = 0;

    for (auto utn : dataset->utns())
    {
        if (!dataset->hasReferenceChain(utn))
        {
            ++targets_no_ref;
            continue;
        }

        std::set<ptime> tst_timestamps;
        for (const auto& dbc : dataset->testDbContentsPresent())
        {
            if (!dataset->hasTestChain(utn, dbc))
                continue;
            const auto& tst_chain = dataset->testChain(utn, dbc);
            for (const auto& kv : tst_chain.timestampIndexes())
                tst_timestamps.insert(kv.first);
        }
        if (tst_timestamps.empty())
        {
            ++targets_no_tst;
            continue;
        }

        auto& ref_chain = dataset->referenceChain(utn);
        if (!ref_chain.hasData())
            continue;

        ptime t_begin = ref_chain.timeBegin();
        ptime t_end   = ref_chain.timeEnd();
        if (t_begin.is_not_a_date_time() || t_end.is_not_a_date_time() || t_end <= t_begin)
            continue;

        double duration_s = partialSeconds(t_end - t_begin);
        unsigned int n_slots = static_cast<unsigned int>(
            std::floor(duration_s / settings.update_interval_s_));
        if (n_slots == 0)
            continue;

        ++targets_walked;

        for (unsigned int k = 0; k < n_slots; ++k)
        {
            ptime t_slot = addSeconds(t_begin, k * settings.update_interval_s_);
            auto pos = dataset->mappedRefPos(utn, t_slot, d_max);
            if (!pos.has_value())
                continue;
            double alt_ft = pos->has_altitude_ ? static_cast<double>(pos->altitude_) : 0.0;
            grid.addEUI(pos->latitude_, pos->longitude_, alt_ft);
        }

        std::vector<ptime> walk_points;
        walk_points.reserve(tst_timestamps.size() + 2);
        walk_points.push_back(t_begin);
        for (auto ts : tst_timestamps)
            if (ts > t_begin && ts < t_end)
                walk_points.push_back(ts);
        walk_points.push_back(t_end);

        for (std::size_t i = 0; i + 1 < walk_points.size(); ++i)
        {
            ptime gap_start = walk_points[i];
            ptime gap_end   = walk_points[i + 1];
            if (gap_end <= gap_start)
                continue;
            double gap_s = partialSeconds(gap_end - gap_start);

            if (!gapIsMiss(gap_s, settings))
                continue;

            std::uint32_t n_misses = numMissesIn(gap_s, settings);
            for (std::uint32_t m = 0; m < n_misses; ++m)
            {
                ptime t_miss = addSeconds(gap_start, (m + 1) * settings.update_interval_s_);
                if (t_miss >= gap_end)
                    break;
                auto pos = dataset->mappedRefPos(utn, t_miss, d_max);
                if (!pos.has_value())
                    continue;
                double alt_ft = pos->has_altitude_ ? static_cast<double>(pos->altitude_) : 0.0;
                grid.addMUI(pos->latitude_, pos->longitude_, alt_ft);
            }
        }
    }

    auto horizontal = grid.projectHorizontal();
    std::uint64_t total_eui = 0, total_mui = 0;
    double worst_pd = 1.0;
    bool   worst_set = false;
    std::vector<double> per_cell_pd;
    per_cell_pd.reserve(horizontal.size());
    for (const auto& kv : horizontal)
    {
        total_eui += kv.second.num_eui;
        total_mui += kv.second.num_mui;
        if (kv.second.num_eui == 0) continue;
        double pd = (static_cast<double>(kv.second.num_eui)
                     - static_cast<double>(kv.second.num_mui))
                    / static_cast<double>(kv.second.num_eui);
        per_cell_pd.push_back(pd);
        if (kv.second.num_eui >= 5 && (!worst_set || pd < worst_pd))
        {
            worst_pd  = pd;
            worst_set = true;
        }
    }
    double sector_pd = total_eui > 0
                           ? (static_cast<double>(total_eui) - static_cast<double>(total_mui))
                                 / static_cast<double>(total_eui)
                           : 0.0;

    result_.valid              = true;
    result_.targets_walked     = targets_walked;
    result_.targets_no_ref     = targets_no_ref;
    result_.targets_no_tst     = targets_no_tst;
    result_.total_eui          = total_eui;
    result_.total_mui          = total_mui;
    result_.sector_pd          = sector_pd;
    result_.cells_with_eui     = per_cell_pd.size();
    result_.median_per_cell_pd = percentile(per_cell_pd, 0.5);
    result_.p5_per_cell_pd     = percentile(per_cell_pd, 0.05);
    result_.has_worst_cell     = worst_set;
    result_.worst_cell_pd      = worst_pd;
}

void MLATCoverageInspector::writeReport(ResultReport::Section& root)
{
    auto& settings = static_cast<MLATCoverageInspectorSettings&>(settings_);
    auto& section  = root.addSubSection(name());

    auto& recap = section.addTable("Settings", 2, {"Setting", "Value"}, false);
    recap.addRow({"PD calculation method",
                  settings.pdMethod() ==
                          MLATCoverageInspectorSettings::PDMethod::TimeDifference
                      ? std::string("Time Difference")
                      : std::string("Status Period Message Based "
                                    "(time-difference fallback)")});
    {
        std::ostringstream os;
        os << settings.update_interval_s_ << " s";
        recap.addRow({"Nominal update interval (UI)", os.str()});
    }
    recap.addRow({"Use miss tolerance", settings.use_miss_tolerance_ ? "yes" : "no"});
    {
        std::ostringstream os;
        os << settings.miss_tolerance_s_ << " s";
        recap.addRow({"Miss tolerance", os.str()});
    }
    {
        std::ostringstream os;
        os << task_.cellSizeMeters() << " m horizontal / "
           << task_.cellSizeFeet()   << " ft vertical (max "
           << task_.maxCellsPerAxis() << " cells/axis)";
        recap.addRow({"Grid resolution", os.str()});
    }
    {
        std::ostringstream os;
        os << "green >= " << settings.pd_acceptable_above_
           << ", red <= "  << settings.pd_unacceptable_below_
           << ", orange in between";
        recap.addRow({"PD color thresholds", os.str()});
    }

    if (!result_.valid)
    {
        auto& note = section.addText("Note");
        note.addText("No data loaded; coverage analysis skipped.");
        return;
    }

    auto& summary = section.addTable("Summary", 2, {"Property", "Value"}, false);
    summary.addRow({"Targets walked",        std::to_string(result_.targets_walked)});
    summary.addRow({"Targets w/o RefTraj",   std::to_string(result_.targets_no_ref)});
    summary.addRow({"Targets w/o test data", std::to_string(result_.targets_no_tst)});
    summary.addRow({"Total expected slots (#EUI)", std::to_string(result_.total_eui)});
    summary.addRow({"Total missed slots (#MUI)",   std::to_string(result_.total_mui)});
    summary.addRow({"Sector-aggregate PD",         formatNumber(result_.sector_pd)});
    summary.addRow({"Cells with EUI",              std::to_string(result_.cells_with_eui)});
    summary.addRow({"Median per-cell PD",          formatNumber(result_.median_per_cell_pd)});
    summary.addRow({"P5 per-cell PD",              formatNumber(result_.p5_per_cell_pd)});
    if (result_.has_worst_cell)
        summary.addRow({"Worst cell PD (>=5 EUI)", formatNumber(result_.worst_cell_pd)});

    loginf << "MLATCoverageInspector: walked " << result_.targets_walked << " target(s), "
           << result_.total_eui << " EUI, " << result_.total_mui
           << " MUI, sector PD " << result_.sector_pd;
}
