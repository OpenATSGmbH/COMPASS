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

#include "eval/requirement/latency/adsblatencyconfig.h"
#include "eval/requirement/latency/adsblatencyconfigwidget.h"
#include "eval/requirement/latency/adsblatency.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include "comparisontype.h"

using namespace std;

namespace EvaluationRequirement
{

ADSBLatencyConfig::ADSBLatencyConfig(
        nlohmann::json& config,
        Group* parent)
    : ProbabilityBaseConfig(config, parent)
{
    registerParameter("max_value_s", &max_value_s_, 0.6f);
}

ADSBLatencyConfig::~ADSBLatencyConfig()
{
}

std::shared_ptr<Base> ADSBLatencyConfig::createRequirement()
{
    shared_ptr<ADSBLatency> req = make_shared<ADSBLatency>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_,
                max_value_s_);

    return req;
}

float ADSBLatencyConfig::maxValue() const
{
    return max_value_s_;
}

void ADSBLatencyConfig::maxValue(float value)
{
    max_value_s_ = value;
}

BaseConfigWidget* ADSBLatencyConfig::createWidget()
{
    return new ADSBLatencyConfigWidget(*this);
}

void ADSBLatencyConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]", "Probability of acceptable latency",
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Maximum Value [s]",
                  "Maximum acceptable latency (Time of Report Transmission minus"
                  " Time of Message Reception for Position)",
                  max_value_s_});
}

}
