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


#include "requirement/position/radartimestampconfig.h"
#include "requirement/position/radartimestampconfigwidget.h"
#include "requirement/position/radartimestamp.h"
#include "requirement/group.h"

#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontenttable.h"

using namespace std;

namespace EvaluationRequirement
{

PositionRadarTimeStampConfig::PositionRadarTimeStampConfig(nlohmann::json& config, Group* parent)
    : PositionBaseValueConfig(config, parent)
{
    registerParameter("threshold_value", &threshold_value_, 0.1);
}

PositionRadarTimeStampConfig::~PositionRadarTimeStampConfig()
{
}

std::shared_ptr<Base> PositionRadarTimeStampConfig::createRequirement()
{
    return make_shared<PositionRadarTimeStamp>(
                name_, short_name_, group_.name(), ref_min_accuracy_, calculator_, threshold_value_);
}

double PositionRadarTimeStampConfig::thresholdValue() const
{
    return threshold_value_;
}

void PositionRadarTimeStampConfig::thresholdValue(double value)
{
    threshold_value_ = value;
}

BaseConfigWidget* PositionRadarTimeStampConfig::createWidget()
{
    return new PositionRadarTimeStampConfigWidget(*this);
}

void PositionRadarTimeStampConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    PositionBaseValueConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Maximum Absolute Error [s]",
                  "The absolute time stamp error, estimated from the along-track offsets to the reference, must be smaller",
                  threshold_value_});
}

}
