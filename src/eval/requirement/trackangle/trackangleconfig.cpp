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

#include "eval/requirement/trackangle/trackangleconfig.h"
#include "eval/requirement/trackangle/trackangleconfigwidget.h"
#include "eval/requirement/trackangle/trackangle.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

using namespace std;

namespace EvaluationRequirement
{
TrackAngleConfig::TrackAngleConfig(
        const std::string& class_id, const std::string& instance_id,
        Group& group, EvaluationStandard& standard, EvaluationCalculator& calculator)
    : ProbabilityBaseConfig(class_id, instance_id, group, standard, calculator)
{
    registerParameter("threshold", &threshold_, 15.0f);

    registerParameter("use_minimum_speed", &use_minimum_speed_, true);
    registerParameter("minimum_speed", &minimum_speed_, 3.0f);

    registerParameter("threshold_value_check_type", (unsigned int*)&threshold_value_check_type_,
                      (unsigned int) COMPARISON_TYPE::LESS_THAN_OR_EQUAL);

    registerParameter("failed_values_of_interest", &failed_values_of_interest_, true);
}

TrackAngleConfig::~TrackAngleConfig()
{

}

std::shared_ptr<Base> TrackAngleConfig::createRequirement()
{
    shared_ptr<TrackAngle> req = make_shared<TrackAngle>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_,
                threshold_, use_minimum_speed_, minimum_speed_,
                threshold_value_check_type_, failed_values_of_interest_);

    return req;
}

COMPARISON_TYPE TrackAngleConfig::thresholdValueCheckType() const
{
    return threshold_value_check_type_;
}

void TrackAngleConfig::thresholdValueCheckType(const COMPARISON_TYPE &type)
{
    threshold_value_check_type_ = type;
}

bool TrackAngleConfig::failedValuesOfInterest() const
{
    return failed_values_of_interest_;
}

void TrackAngleConfig::failedValuesOfInterest(bool value)
{
    failed_values_of_interest_ = value;
}

float TrackAngleConfig::threshold() const
{
    return threshold_;
}

void TrackAngleConfig::threshold(float value)
{
    threshold_ = value;
}

BaseConfigWidget* TrackAngleConfig::createWidget()
{
    return new TrackAngleConfigWidget(*this);
}

void TrackAngleConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    //auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    //   auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    //    table.addRow({"Name", "Requirement name", name_});
    //    table.addRow({"Short Name", "Requirement short name", short_name_});
    //    table.addRow({"Comment", "", comment_});

    // prob & check type added in subclass
}

bool TrackAngleConfig::useMinimumSpeed() const
{
    return use_minimum_speed_;
}

void TrackAngleConfig::useMinimumSpeed(bool value)
{
    use_minimum_speed_ = value;
}

float TrackAngleConfig::minimumSpeed() const
{
    return minimum_speed_;
}

void TrackAngleConfig::minimumSpeed(float value)
{
    minimum_speed_ = value;
}
}
