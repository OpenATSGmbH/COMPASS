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

#include "eval/requirement/identification/falseconfig.h"
#include "eval/requirement/identification/identificationfalseconfigwidget.h"
#include "eval/requirement/identification/false.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include "stringconv.h"

using namespace Utils;
using namespace std;

namespace EvaluationRequirement
{

IdentificationFalseConfig::IdentificationFalseConfig(
        nlohmann::json& config,
        Group* parent)
    : ProbabilityBaseConfig(config, parent)
{
    registerParameter("require_all_false", &require_all_false_, true);

    registerParameter("use_mode_a", &use_mode_a_, true);
    registerParameter("use_ms_ta", &use_ms_ta_, true);
    registerParameter("use_ms_ti", &use_ms_ti_, true);
}

std::shared_ptr<Base> IdentificationFalseConfig::createRequirement()
{
    shared_ptr<IdentificationFalse> req = make_shared<IdentificationFalse>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_,
                require_all_false_, use_mode_a_, use_ms_ta_, use_ms_ti_);

    return req;
}

BaseConfigWidget* IdentificationFalseConfig::createWidget()
{
    return new IdentificationFalseConfigWidget(*this);
}

bool IdentificationFalseConfig::requireAllFalse() const
{
    return require_all_false_;
}

void IdentificationFalseConfig::requireAllFalse(bool value)
{
    require_all_false_ = value;
}

bool IdentificationFalseConfig::useModeA() const
{
    return use_mode_a_;
}

void IdentificationFalseConfig::useModeA(bool value)
{
    use_mode_a_ = value;
}

bool IdentificationFalseConfig::useMsTa() const
{
    return use_ms_ta_;
}

void IdentificationFalseConfig::useMsTa(bool value)
{
    use_ms_ta_ = value;
}

bool IdentificationFalseConfig::useMsTi() const
{
    return use_ms_ti_;
}

void IdentificationFalseConfig::useMsTi(bool value)
{
    use_ms_ti_ = value;
}

void IdentificationFalseConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]", "Probability of false identification",
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Require All False",
                  "If checked, all available secondary attributes be different than in "
                  " the reference to count. If not checked, a single wrong secondary attribute is enough.",
                  String::boolToString(require_all_false_)});

    table.addRow({"Use Mode 3/A Code",
                  "If the Mode 3/A code should be checked",
                  String::boolToString(use_mode_a_)});

    table.addRow({"Use Mode S Target Address",
                  "If the Mode S target address should be checked",
                  String::boolToString(use_ms_ta_)});

    table.addRow({"Use Mode S Target Identification",
                  "If the Mode S target identification should be checked",
                  String::boolToString(use_ms_ti_)});
}

}
