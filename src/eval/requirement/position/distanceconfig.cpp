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

#include "eval/requirement/position/distanceconfig.h"
#include "eval/requirement/position/distanceconfigwidget.h"
#include "eval/requirement/position/distance.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/probabilitybase.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include "stringconv.h"

using namespace Utils;
using namespace std;

namespace EvaluationRequirement
{
PositionDistanceConfig::PositionDistanceConfig(
        nlohmann::json& config,
        Group* parent)
    : PositionBaseProbConfig(config, parent)
{
    registerParameter("threshold_value", &threshold_value_, 50.0f);
    registerParameter("threshold_value_check_type", (unsigned int*)&threshold_value_check_type_,
                      (unsigned int) COMPARISON_TYPE::LESS_THAN_OR_EQUAL);
    registerParameter("failed_values_of_interest", &failed_values_of_interest_, true);

    registerParameter("use_averaging", &use_averaging_, false);
    registerParameter("averaging_window_s", &averaging_window_s_, 5.0f);
}

PositionDistanceConfig::~PositionDistanceConfig()
{
}

std::shared_ptr<Base> PositionDistanceConfig::createRequirement()
{
    shared_ptr<PositionDistance> req = make_shared<PositionDistance>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, ref_min_accuracy_, calculator_,
                threshold_value_, threshold_value_check_type_, failed_values_of_interest_,
                use_averaging_, averaging_window_s_);

    return req;
}

float PositionDistanceConfig::thresholdValue() const
{
    return threshold_value_;
}

void PositionDistanceConfig::thresholdValue(float value)
{
    threshold_value_ = value;
}

COMPARISON_TYPE PositionDistanceConfig::thresholdValueCheckType() const
{
    return threshold_value_check_type_;
}

void PositionDistanceConfig::thresholdValueCheckType(const COMPARISON_TYPE &type)
{
    threshold_value_check_type_ = type;
}

bool PositionDistanceConfig::failedValuesOfInterest() const
{
    return failed_values_of_interest_;
}

void PositionDistanceConfig::failedValuesOfInterest(bool value)
{
    failed_values_of_interest_ = value;
}

bool PositionDistanceConfig::useAveraging() const
{
    return use_averaging_;
}

void PositionDistanceConfig::useAveraging(bool value)
{
    use_averaging_ = value;
}

float PositionDistanceConfig::averagingWindow() const
{
    return averaging_window_s_;
}

void PositionDistanceConfig::averagingWindow(float value)
{
    averaging_window_s_ = value;
}

BaseConfigWidget* PositionDistanceConfig::createWidget()
{
    return new PositionDistanceConfigWidget(*this);
}

void PositionDistanceConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]", "Probability of correct/false position",
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Threshold Value [m]",
                  "Minimum/Maximum allowed distance from test target report to reference",
                  threshold_value_});

    table.addRow({"Threshold Value Check Type",
                  "Distance comparison operator with the given threshold",
                  comparisonTypeString(threshold_value_check_type_)});

    table.addRow({"Failed Values are of Interest",
                  "If the distances of interest are the ones not passing the check",
                  String::boolToString(failed_values_of_interest_)});

    table.addRow({"Use Averaged Position", "One comparison per averaging window, using the mean"
                  " position error of the test reports in that window, instead of one comparison"
                  " per test report",
                  String::boolToString(use_averaging_)});
    table.addRow({"Averaging Window [s]", "Length of one averaging window",
                  averaging_window_s_});
}
}
