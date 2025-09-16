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

#include "eval/requirement/base/probabilitybaseconfig.h"
#include "eval/requirement/base/positionbaseconfigwidget.h"
#include "evaluationstandardtreeitem.h"

class Group;
class EvaluationStandard;

class QWidget;
class QFormLayout;
class EvaluationCalculator;

namespace ResultReport
{
class Report;
}

namespace EvaluationRequirement
{

class PositionBase;

/**
 */
class PositionBaseConfig : public ProbabilityBaseConfig
{
    Q_OBJECT
  public:
    PositionBaseConfig(const std::string& class_id, const std::string& instance_id, Group& group,
                       EvaluationStandard& standard, EvaluationCalculator& calculator);
    virtual ~PositionBaseConfig();

    virtual void addToReport(std::shared_ptr<ResultReport::Report> report);

    float refMinimumAccuracy() const { return ref_min_accuracy_; }
    void refMinimumAccuracy(float val) { ref_min_accuracy_ = val; }

  protected:
    float ref_min_accuracy_{10};
};

}  // namespace EvaluationRequirement
