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

#include "eval/requirement/mode_c/falseconfig.h"
#include "eval/requirement/mode_c/false.h"
#include "eval/requirement/mode_c/modecfalseconfigwidget.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

using namespace std;

namespace EvaluationRequirement
{

ModeCFalseConfig::ModeCFalseConfig(
        nlohmann::json& config,
        Group* parent)
    : ProbabilityBaseConfig(config, parent)
{
    registerParameter("maximum_probability_false", &maximum_probability_false_, 0.01f);

    registerParameter("max_difference", &max_difference_, 100.0f);
}

std::shared_ptr<Base> ModeCFalseConfig::createRequirement()
{
    shared_ptr<ModeCFalse> req = make_shared<ModeCFalse>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_,
                maximum_probability_false_, max_difference_);

    return req;
}

float ModeCFalseConfig::maximumProbabilityFalse() const
{
    return maximum_probability_false_;
}

void ModeCFalseConfig::maximumProbabilityFalse(float value)
{
    maximum_probability_false_ = value;
}

float ModeCFalseConfig::maxDifference() const
{
    return max_difference_;
}

void ModeCFalseConfig::maxDifference(float value)
{
    max_difference_ = value;
}

BaseConfigWidget* ModeCFalseConfig::createWidget()
{
    return new ModeCFalseConfigWidget(*this);
}

void ModeCFalseConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]", "Probability of false Mode C code",
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});
    table.addRow({"Maximum Difference [ft]", "Maximum altitude difference between the test and the reference",
                  max_difference_});
}

}
