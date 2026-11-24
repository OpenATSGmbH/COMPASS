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

#include "requirement/position/radarazimuthconfig.h"
#include "requirement/position/radarazimuthconfigwidget.h"
#include "requirement/position/radarazimuth.h"
#include "requirement/group.h"
#include "requirement/base/positionbase.h"

#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontenttable.h"

using namespace std;


namespace EvaluationRequirement
{
PositionRadarAzimuthConfig::PositionRadarAzimuthConfig(
        nlohmann::json& config,
        Group* parent)
    : PositionBaseValueConfig(config, parent)
{
    registerParameter("threshold_value", &threshold_value_, 0.1);
    registerParameter("statistic", &statistic_, (unsigned int)RadarHelpers::Statistic::StdDev);
}

PositionRadarAzimuthConfig::~PositionRadarAzimuthConfig()
{
}

std::shared_ptr<Base> PositionRadarAzimuthConfig::createRequirement()
{
    shared_ptr<PositionRadarAzimuth> req = make_shared<PositionRadarAzimuth>(
                name_, short_name_, group_.name(), ref_min_accuracy_, calculator_, threshold_value_,
                statistic());

    return req;
}

double PositionRadarAzimuthConfig::thresholdValue() const
{
    return threshold_value_;
}

void PositionRadarAzimuthConfig::thresholdValue(double value)
{
    threshold_value_ = value;
}

RadarHelpers::Statistic PositionRadarAzimuthConfig::statistic() const
{
    return statisticInfo().statistic;
}

void PositionRadarAzimuthConfig::statistic(RadarHelpers::Statistic value)
{
    statistic_ = (unsigned int)value;
}

const RadarHelpers::StatisticInfo& PositionRadarAzimuthConfig::statisticInfo() const
{
    return RadarHelpers::statisticInfo(RadarHelpers::azimuthStatistics(), (RadarHelpers::Statistic)statistic_);
}

BaseConfigWidget* PositionRadarAzimuthConfig::createWidget()
{
    return new PositionRadarAzimuthConfigWidget(*this);
}

void PositionRadarAzimuthConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    PositionBaseValueConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Statistic", "Checked statistic of the differences", statisticInfo().label});
    table.addRow({statisticInfo().threshold_label, statisticInfo().threshold_description, threshold_value_});

}
}
