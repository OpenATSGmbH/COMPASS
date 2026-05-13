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

namespace ResultReport
{
class Section;
}

class AnalyseDataSourceTask;
class AnalysisDataset;
class InspectorSettingsBase;

class DataSourceInspectorBase
{
public:
    DataSourceInspectorBase(AnalyseDataSourceTask& task,
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

    AnalyseDataSourceTask&  task()     const { return task_; }
    InspectorSettingsBase&  settings() const { return settings_; }

protected:
    AnalyseDataSourceTask&  task_;
    InspectorSettingsBase&  settings_;
};
