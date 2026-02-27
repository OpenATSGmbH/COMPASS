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

#include "eval/requirement/mode_c/correctconfig.h"
#include "eval/requirement/mode_c/correct.h"
#include "eval/requirement/mode_c/modeccorrectconfigwidget.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

using namespace std;

namespace EvaluationRequirement
{

ModeCCorrectConfig::ModeCCorrectConfig(
        nlohmann::json& config,
        Group* parent)
    : ProbabilityBaseConfig(config, parent)
{
    registerParameter("max_distance_ft", &max_distance_ft_, 300.0f);

}

std::shared_ptr<Base> ModeCCorrectConfig::createRequirement()
{
    shared_ptr<ModeCCorrect> req = make_shared<ModeCCorrect>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_,
                max_distance_ft_);

    return req;
}

float ModeCCorrectConfig::maxDistanceFt() const
{
    return max_distance_ft_;
}

void ModeCCorrectConfig::maxDistanceFt(float value)
{
    max_distance_ft_ = value;
}

BaseConfigWidget* ModeCCorrectConfig::createWidget()
{
    return new ModeCCorrectConfigWidget(*this);
}

void ModeCCorrectConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]", "Probability of correct Mode C",
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Max Distance",
                  "If checked, value must match "
                  "the reference with the given maximum distance",
                 to_string(max_distance_ft_)});
}

}
