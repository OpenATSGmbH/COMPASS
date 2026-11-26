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

#include "target.h"
#include "timewindow.h"

#include "json_fwd.hpp"

#include "boost/date_time/posix_time/ptime.hpp"

#include <map>
#include <set>
#include <string>

class EvaluationManager;

namespace ResultReport
{
    class Section;
}

/**
 * Snapshot of the data exclusions of the Evaluation: the global time filter (global time
 * window and global exclusion windows) and the per-target constraints (use flag, excluded
 * time windows, excluded requirements). Used by the Evaluation and by the Data Source
 * Analysis, so that both apply the same exclusions.
 */
class EvaluationConstraints
{
public:
    static const std::string ExcludedDataSectionHeading;
    static const std::string ExcludedTimeWindowsTableName;
    static const std::string ExcludedRequirementsTableName;
    static const std::string NoExcludedDataText;

    EvaluationConstraints() = default;

    void clear();
    void update(EvaluationManager& eval_man);

    bool useTarget(unsigned int utn) const;
    bool isTimeStampNotExcluded(const boost::posix_time::ptime& ts) const;
    bool isTimeStampNotExcluded(unsigned int utn, const boost::posix_time::ptime& ts) const;

    bool globalTimeFilterEnabled() const { return global_time_filter_enabled_; }
    void globalTimeFilterEnabled(bool value) { global_time_filter_enabled_ = value; }

    const Utils::TimeWindow& globalTimeWindow() const { return global_time_window_; }
    const Utils::TimeWindowCollection& globalExclusionTimeWindows() const { return global_exclusion_time_windows_; }

    const dbContent::TargetEvalConstraints* targetConstraint(unsigned int utn) const;
    const std::map<unsigned int, dbContent::TargetEvalConstraints>& targetConstraints() const { return target_constraints_; }

    /// number of targets with the use flag off
    size_t numExcludedTargets() const;

    nlohmann::json globalTimeWindowAsJSON() const;
    nlohmann::json globalExclusionTimeWindowsAsJSON() const;
    nlohmann::json targetConstraintsAsJSON() const;

    void globalTimeWindowFromJSON(const nlohmann::json& j);
    void globalExclusionTimeWindowsFromJSON(const nlohmann::json& j);
    void targetConstraintsFromJSON(const nlohmann::json& j);

    /**
     * Adds the excluded data content to the given section: a table of the excluded time
     * windows (global and per target) and, if wanted, a table of the excluded requirements,
     * or a placeholder text when there is nothing to show. If utns is set, the per-target
     * rows are limited to these targets.
     */
    void addReportSection(ResultReport::Section& section,
                          const std::set<unsigned int>* utns,
                          bool with_excluded_requirements) const;

private:
    bool                                                     global_time_filter_enabled_ = false;
    Utils::TimeWindow                                        global_time_window_;
    Utils::TimeWindowCollection                              global_exclusion_time_windows_;
    std::map<unsigned int, dbContent::TargetEvalConstraints> target_constraints_;
};
