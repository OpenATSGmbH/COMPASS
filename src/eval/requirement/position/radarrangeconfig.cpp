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

#include "requirement/position/radarrangeconfig.h"
#include "requirement/position/radarrangeconfigwidget.h"
#include "requirement/position/radarrange.h"
#include "requirement/group.h"
#include "requirement/base/positionbase.h"

#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontenttable.h"

using namespace std;

namespace EvaluationRequirement
{
PositionRadarRangeConfig::PositionRadarRangeConfig(
        nlohmann::json& config,
        Group* parent)
    : PositionBaseValueConfig(config, parent)
{
    registerParameter("threshold_value", &threshold_value_, 50.0);
    registerParameter("statistic", &statistic_, (unsigned int)RadarHelpers::Statistic::StdDev);
}

PositionRadarRangeConfig::~PositionRadarRangeConfig()
{
}

std::shared_ptr<Base> PositionRadarRangeConfig::createRequirement()
{
    shared_ptr<PositionRadarRange> req = make_shared<PositionRadarRange>(
                name_, short_name_, group_.name(), ref_min_accuracy_, calculator_, threshold_value_,
                statistic());

    return req;
}

double PositionRadarRangeConfig::thresholdValue() const
{
    return threshold_value_;
}

void PositionRadarRangeConfig::thresholdValue(double value)
{
    threshold_value_ = value;
}

RadarHelpers::Statistic PositionRadarRangeConfig::statistic() const
{
    return statisticInfo().statistic;
}

void PositionRadarRangeConfig::statistic(RadarHelpers::Statistic value)
{
    statistic_ = (unsigned int)value;
}

const RadarHelpers::StatisticInfo& PositionRadarRangeConfig::statisticInfo() const
{
    return RadarHelpers::statisticInfo(RadarHelpers::rangeStatistics(), (RadarHelpers::Statistic)statistic_);
}

BaseConfigWidget* PositionRadarRangeConfig::createWidget()
{
    return new PositionRadarRangeConfigWidget(*this);
}

void PositionRadarRangeConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    PositionBaseValueConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Statistic", "Checked statistic of the differences", statisticInfo().label});
    table.addRow({statisticInfo().threshold_label, statisticInfo().threshold_description, threshold_value_});

}
}
