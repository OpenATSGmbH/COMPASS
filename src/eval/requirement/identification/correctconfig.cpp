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

#include "eval/requirement/identification/correctconfig.h"
#include "eval/requirement/identification/correct.h"
#include "eval/requirement/identification/identificationcorrectconfigwidget.h"
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

IdentificationCorrectConfig::IdentificationCorrectConfig(
        nlohmann::json& config,
        Group* parent)
    : ProbabilityBaseConfig(config, parent)
{
    registerParameter("require_correctness_of_all", &require_correctness_of_all_, false);

    registerParameter("use_mode_a", &use_mode_a_, true);
    registerParameter("use_ms_ta", &use_ms_ta_, true);
    registerParameter("use_ms_ti", &use_ms_ti_, true);
}

std::shared_ptr<Base> IdentificationCorrectConfig::createRequirement()
{
    shared_ptr<IdentificationCorrect> req = make_shared<IdentificationCorrect>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_,
                require_correctness_of_all_,
                use_mode_a_, use_ms_ta_, use_ms_ti_);

    return req;
}

bool IdentificationCorrectConfig::requireCorrectnessOfAll() const
{
    return require_correctness_of_all_;
}

void IdentificationCorrectConfig::requireCorrectnessOfAll(bool value)
{
    require_correctness_of_all_ = value;
}

bool IdentificationCorrectConfig::useModeA() const
{
    return use_mode_a_;
}

void IdentificationCorrectConfig::useModeA(bool value)
{
    use_mode_a_ = value;
}

bool IdentificationCorrectConfig::useMsTa() const
{
    return use_ms_ta_;
}

void IdentificationCorrectConfig::useMsTa(bool value)
{
    use_ms_ta_ = value;
}

bool IdentificationCorrectConfig::useMsTi() const
{
    return use_ms_ti_;
}

void IdentificationCorrectConfig::useMsTi(bool value)
{
    use_ms_ti_ = value;
}

BaseConfigWidget* IdentificationCorrectConfig::createWidget()
{
    return new IdentificationCorrectConfigWidget(*this);
}

void IdentificationCorrectConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]", "Probability of correct identification",
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Require Correctness of All",
                  "If checked, all available secondary attributes must match "
                  "the reference. If not checked, a single matching secondary attribute is enough.",
                  String::boolToString(require_correctness_of_all_)});

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
