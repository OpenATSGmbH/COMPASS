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

#include "requirement/mode_a/presentconfig.h"
#include "requirement/mode_a/modeapresentconfigwidget.h"
#include "requirement/mode_a/present.h"
#include "requirement/group.h"
#include "requirement/base/base.h"

#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontenttable.h"

using namespace std;

namespace EvaluationRequirement
{

ModeAPresentConfig::ModeAPresentConfig(
        nlohmann::json& config,
        Group* parent)
    : ProbabilityBaseConfig(config, parent)
{
}

std::shared_ptr<Base> ModeAPresentConfig::createRequirement()
{
    shared_ptr<ModeAPresent> req = make_shared<ModeAPresent>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_);

    return req;
}

BaseConfigWidget* ModeAPresentConfig::createWidget()
{
    return new ModeAPresentConfigWidget(*this);
}

void ModeAPresentConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    BaseConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Probability [%]", "Probability of Mode 3/A code present",
                  probabilityPercent(prob_)});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});
}

}
