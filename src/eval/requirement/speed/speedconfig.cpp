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

#include "eval/requirement/speed/speedconfig.h"
#include "eval/requirement/speed/speedconfigwidget.h"
#include "eval/requirement/speed/speed.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

using namespace std;


namespace EvaluationRequirement
{
SpeedConfig::SpeedConfig(
        const std::string& class_id, const std::string& instance_id,
        Group& group, EvaluationStandard& standard, EvaluationCalculator& calculator)
    : ProbabilityBaseConfig(class_id, instance_id, group, standard, calculator)
{
    registerParameter("threshold_value", &threshold_value_, 50.0f);
    registerParameter("use_percent_if_higher", &use_percent_if_higher_, false);
    registerParameter("threshold_percent", &threshold_percent_, 10.0f);

    registerParameter("threshold_value_check_type", (unsigned int*)&threshold_value_check_type_,
                      (unsigned int) COMPARISON_TYPE::LESS_THAN_OR_EQUAL);

    registerParameter("failed_values_of_interest", &failed_values_of_interest_, true);
}

SpeedConfig::~SpeedConfig()
{

}

std::shared_ptr<Base> SpeedConfig::createRequirement()
{
    shared_ptr<Speed> req = make_shared<Speed>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_,
                threshold_value_, use_percent_if_higher_, threshold_percent_,
                threshold_value_check_type_, failed_values_of_interest_);

    return req;
}

float SpeedConfig::thresholdValue() const
{
    return threshold_value_;
}

void SpeedConfig::thresholdValue(float value)
{
    threshold_value_ = value;
}

COMPARISON_TYPE SpeedConfig::thresholdValueCheckType() const
{
    return threshold_value_check_type_;
}

void SpeedConfig::thresholdValueCheckType(const COMPARISON_TYPE &type)
{
    threshold_value_check_type_ = type;
}

bool SpeedConfig::failedValuesOfInterest() const
{
    return failed_values_of_interest_;
}

void SpeedConfig::failedValuesOfInterest(bool value)
{
    failed_values_of_interest_ = value;
}

bool SpeedConfig::usePercentIfHigher() const
{
    return use_percent_if_higher_;
}

void SpeedConfig::usePercentIfHigher(bool value)
{
    use_percent_if_higher_ = value;
}

float SpeedConfig::thresholdPercent() const
{
    return threshold_percent_;
}

void SpeedConfig::thresholdPercent(float value)
{
    threshold_percent_ = value;
}

BaseConfigWidget* SpeedConfig::createWidget()
{
    return new SpeedConfigWidget(*this);
}

void SpeedConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    //auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    //   auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    //    table.addRow({"Name", "Requirement name", name_});
    //    table.addRow({"Short Name", "Requirement short name", short_name_});
    //    table.addRow({"Comment", "", comment_});

    // prob & check type added in subclass
}
}
