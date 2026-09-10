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

#pragma once

#include <set>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

namespace ResultReport
{
class Section;
}

class AnalyzeDataSourceTask;
class AnalysisDataset;
class InspectorSettingsBase;
class ReportTableWriter;
class ReportTableRows;
struct ReportTableDefinition;

/**
 * One row of an accuracy report table: a test report with its offset to the reference.
 */
struct AccuracyRow
{
    unsigned long            rec_num = 0;
    unsigned int             utn     = 0;
    boost::posix_time::ptime timestamp;

    double tst_lat    = 0.0;
    double tst_lon    = 0.0;
    double ref_lat    = 0.0;
    double ref_lon    = 0.0;
    double distance_m = 0.0;
    bool   gated      = false;   // reference not accurate enough, the distance is not assessed

    boost::optional<double> reported_stddev_m;
    boost::optional<double> consistency_ratio;
    boost::optional<double> dt_prev_s;           // time to the previous test report of the target

    std::pair<boost::optional<unsigned long>, boost::optional<unsigned long>> ref_rec_nums;
};

/**
 * One gap of a coverage inspector: a span of the reference period without a test report, with
 * at least one missed update inside.
 */
struct GapRow
{
    unsigned int             utn = 0;
    boost::posix_time::ptime begin;
    boost::posix_time::ptime end;
    unsigned int             num_missed = 0;
};

class DataSourceInspectorBase
{
public:
    DataSourceInspectorBase(AnalyzeDataSourceTask& task,
                            InspectorSettingsBase& settings);
    virtual ~DataSourceInspectorBase() = default;

    /// Identifying class name (matches the class name registered in Configurable JSON).
    virtual std::string className() const = 0;

    /// Human-readable inspector name; used as the section heading.
    virtual std::string name() const = 0;

    /// DSType this inspector applies to (e.g. "MLAT").
    virtual std::string dsType() const = 0;

    /// Optional one-line description shown in the dialog tooltip.
    virtual std::string description() const { return {}; }

    /// True if the inspector requires the Professional license.
    virtual bool requiresProfessionalLicense() const { return false; }

    /// True if at least one selected data source must be CAT020 (Feature 4 / 5).
    virtual bool requiresCAT020Source() const { return false; }

    /// True if reconstruction must have been run (RefTraj available). Default true:
    /// almost every inspector compares against the Reference Trajectory.
    virtual bool requiresReferenceTrajectory() const { return true; }

    /// True if the inspector consumes a loaded `AnalysisDataset` (test buffers +
    /// per-UTN ref/test chains). Inspectors that operate on metadata only (e.g.
    /// the data-item inspector) can return false to skip the load.
    virtual bool requiresLoadedDataset() const { return true; }

    /// Test dbcontent names this inspector needs loaded (e.g. {"CAT020", "CAT010"}).
    /// Only consulted when `requiresLoadedDataset()` is true. Empty by default.
    virtual std::set<std::string> testDBContentNames() const { return {}; }

    /// Returns true if all prerequisites for this inspector are met, given the
    /// current task state. On false, `reason_out` is filled with a human-readable
    /// explanation suitable for a tooltip.
    virtual bool prerequisitesMet(std::string& reason_out) const;

    /// Heavy data-side computation: the per-target / per-cell walk, statistics,
    /// 3D-grid population. The task may invoke this from a worker thread so it
    /// MUST NOT touch any Qt GUI or any QObject in the report tree. Results
    /// must be stored on the inspector instance so that `writeReport()` can
    /// emit them on the main thread afterwards.
    /// `dataset` is non-null when `requiresLoadedDataset()` is true and the
    /// load succeeded. Default: no-op.
    virtual void compute(AnalysisDataset* dataset) { (void)dataset; }

    /// Write this inspector's section into `root`. Always invoked on the main
    /// thread, after `compute()` returns.
    virtual void writeReport(ResultReport::Section& root) = 0;

    AnalyzeDataSourceTask&  task()     const { return task_; }
    InspectorSettingsBase&  settings() const { return settings_; }

    /// Writer of the report tables of the current run, usable from `compute()`.
    ReportTableWriter& tableWriter() const;

    /// Table key derived from the class name, "MLATAccuracyInspector" gives
    /// "mlat_accuracy", plus an optional suffix such as "_gaps".
    std::string tableKey(const std::string& suffix = "") const;

    /// Record table definition with the common columns, named after the
    /// inspector, with an optional key and name suffix.
    ReportTableDefinition recordTableDefinition(const std::vector<std::string>& host_dbcontents,
                                                const std::string& key_suffix = "",
                                                const std::string& name_suffix = "") const;

    /// Record table definition of a position accuracy inspector, see AccuracyRow.
    ReportTableDefinition accuracyTableDefinition(const std::vector<std::string>& host_dbcontents) const;

    /// Writes one accuracy row into a table created from accuracyTableDefinition().
    static void writeAccuracyRow(ReportTableRows& rows, const AccuracyRow& row);

    /// Record table definition of the gaps of a coverage inspector, hosted on the reference
    /// content, see GapRow.
    ReportTableDefinition gapTableDefinition(const std::string& key_suffix = "_gaps",
                                             const std::string& name_suffix = " Gaps") const;

    /// Writes one gap row, keyed by the first reference sample inside the gap. Returns false
    /// and writes nothing when the gap holds no reference sample.
    static bool writeGapRow(ReportTableRows& rows, AnalysisDataset& dataset, const GapRow& gap);

    /// Seconds between two timestamps as double.
    static double secondsBetween(const boost::posix_time::ptime& t0,
                                 const boost::posix_time::ptime& t1);

protected:
    AnalyzeDataSourceTask&  task_;
    InspectorSettingsBase&  settings_;
};
