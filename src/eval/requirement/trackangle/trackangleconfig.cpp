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

#include "requirement/trackangle/trackangleconfig.h"
#include "requirement/trackangle/trackangleconfigwidget.h"
#include "requirement/trackangle/trackangle.h"
#include "requirement/group.h"
#include "requirement/base/base.h"

#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontenttable.h"

#include "stringconv.h"
#include "number.h"

using namespace std;
using namespace Utils;

namespace EvaluationRequirement
{
TrackAngleConfig::TrackAngleConfig(
        nlohmann::json& config,
        Group* parent)
    : ProbabilityBaseConfig(config, parent)
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
    BaseConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Probability [%]", "Probability of acceptable track angle",
                  probabilityPercent(prob_)});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});
    table.addRow({"Track Angle Offset Value [deg]",
                  "Maximum allowed track angle difference between the test and the reference",
                  Number::floatToDouble(threshold_)});
    table.addRow({"Use Minimum Speed",
                  "If test target reports with a reference speed below the minimum speed are not counted",
                  String::boolToString(use_minimum_speed_)});
    table.addRow({"Minimum Speed [m/s]", "Minimum reference speed", Number::floatToDouble(minimum_speed_)});
    table.addRow({"Threshold Value Check Type",
                  "Track angle difference comparison operator with the given threshold",
                  comparisonTypeString(threshold_value_check_type_)});
    table.addRow({"Failed Values are of Interest",
                  "If the track angle differences of interest are the ones not passing the check",
                  String::boolToString(failed_values_of_interest_)});
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
