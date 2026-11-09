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

#include "targetmodel.h"

#include <set>
#include <string>

class QWidget;

namespace dbContent
{
class VariableSet;
}

namespace Utils
{
class TimeWindowCollection;
}

/**
 * The evaluation functions that code below the evaluation layer uses, via COMPASS::evalProvider().
 * Implemented by EvaluationManager.
 * - read set variables, timestamp and target filter (DBContentManager, TargetModel)
 * - the evaluation dialogs of the target list
 */
class IEvalProvider
{
public:
    virtual ~IEvalProvider() = default;

    virtual bool needsAdditionalVariables() const = 0;
    virtual void addVariables(const std::string& dbcontent_name, dbContent::VariableSet& read_set) = 0;

    virtual bool useTimestampFilter() const = 0;
    virtual void applyTargetFilter(dbContent::TargetCache& target_data) = 0;

    virtual bool hasCurrentStandard() const = 0;
    virtual std::set<std::string> currentRequirementNames() const = 0; // requirements of the current standard

    // dialogs, return true if accepted (resp. changed)
    virtual void editTargetFilter(dbContent::TargetModel& model, QWidget* parent) = 0;
    virtual bool editTimestampConditions(QWidget* parent) = 0; // saves the conditions if changed
    virtual bool editExcludedTimeWindows(const std::string& utns,
                                         Utils::TimeWindowCollection& time_windows,
                                         QWidget* parent) = 0;
    virtual bool editExcludedRequirements(const std::string& utns,
                                          std::set<std::string>& selected_requirements,
                                          const std::set<std::string>& all_requirements,
                                          std::string& comment,
                                          QWidget* parent) = 0;
};
