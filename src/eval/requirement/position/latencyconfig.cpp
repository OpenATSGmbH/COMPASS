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

#include "eval/requirement/position/latencyconfig.h"
#include "eval/requirement/position/latencyconfigwidget.h"
#include "eval/requirement/position/latency.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/positionbase.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

using namespace std;


namespace EvaluationRequirement
{
PositionLatencyConfig::PositionLatencyConfig(
        const std::string& class_id, const std::string& instance_id,
        Group& group, EvaluationStandard& standard, EvaluationCalculator& calculator)
    : PositionBaseConfig(class_id, instance_id, group, standard, calculator)
{
    registerParameter("max_abs_value", &max_abs_value_, 0.050f);
}

PositionLatencyConfig::~PositionLatencyConfig()
{
}

std::shared_ptr<Base> PositionLatencyConfig::createRequirement()
{
    shared_ptr<PositionLatency> req = make_shared<PositionLatency>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, 
                ref_min_accuracy_, calculator_, max_abs_value_);

    return req;
}

float PositionLatencyConfig::maxAbsValue() const
{
    return max_abs_value_;
}

void PositionLatencyConfig::maxAbsValue(float value)
{
    max_abs_value_ = value;
}

BaseConfigWidget* PositionLatencyConfig::createWidget()
{
    return new PositionLatencyConfigWidget(*this);
}

void PositionLatencyConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]", "Probability of acceptable position latency",
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Maximum Absolute Value [HH:MM:SS.SSS]",
                  "Maximum absolute latency",
                  roundf(prob_ * 10000.0) / 100.0});
}
}
