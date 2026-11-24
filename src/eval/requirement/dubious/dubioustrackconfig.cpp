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

#include "dubioustrackconfig.h"
#include "requirement/dubious/dubioustrack.h"
#include "dubioustrackconfigwidget.h"
#include "requirement/group.h"
#include "requirement/base/base.h"

#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontenttable.h"

#include "stringconv.h"
#include "number.h"

using namespace std;
using namespace Utils;


namespace EvaluationRequirement
{

DubiousTrackConfig::DubiousTrackConfig(
        nlohmann::json& config,
        Group* parent)
    : ProbabilityBaseConfig(config, parent)
{
    prob_check_type_ = COMPARISON_TYPE::LESS_THAN_OR_EQUAL;

    registerParameter("minimum_comparison_time", &minimum_comparison_time_, 1.0f);
    registerParameter("maximum_comparison_time", &maximum_comparison_time_, 30.0f);

    registerParameter("mark_primary_only", &mark_primary_only_, true);

    registerParameter("use_min_updates", &use_min_updates_, true);
    registerParameter("min_updates", &min_updates_, 10u);

    registerParameter("use_min_duration", &use_min_duration_, true);
    registerParameter("min_duration", &min_duration_, 30.0f);

    registerParameter("use_max_groundspeed", &use_max_groundspeed_, true);
    registerParameter("max_groundspeed_kts", &max_groundspeed_kts_, 1333.0f);

    registerParameter("use_max_acceleration", &use_max_acceleration_, true);
    registerParameter("max_acceleration", &max_acceleration_, 29.43f);

    registerParameter("use_max_turnrate", &use_max_turnrate_, true);
    registerParameter("max_turnrate", &max_turnrate_, 30.0f);

    registerParameter("use_rocd", &use_max_rocd_, true);
    registerParameter("max_rocd", &max_rocd_, 1000.0f);

    registerParameter("dubious_prob", &dubious_prob_, 0.05f);
}

DubiousTrackConfig::~DubiousTrackConfig()
{
}

bool DubiousTrackConfig::markPrimaryOnly() const
{
    return mark_primary_only_;
}

void DubiousTrackConfig::markPrimaryOnly(bool mark_primary_only)
{
    mark_primary_only_ = mark_primary_only;
}

bool DubiousTrackConfig::useMinUpdates() const
{
    return use_min_updates_;
}

void DubiousTrackConfig::useMinUpdates(bool use_min_updates)
{
    use_min_updates_ = use_min_updates;
}

unsigned int DubiousTrackConfig::minUpdates() const
{
    return min_updates_;
}

void DubiousTrackConfig::minUpdates(unsigned int min_updates)
{
    min_updates_ = min_updates;
}

bool DubiousTrackConfig::useMinDuration() const
{
    return use_min_duration_;
}

void DubiousTrackConfig::useMinDuration(bool use_min_duration)
{
    use_min_duration_ = use_min_duration;
}

float DubiousTrackConfig::minDuration() const
{
    return min_duration_;
}

void DubiousTrackConfig::minDuration(float min_duration)
{
    min_duration_ = min_duration;
}

bool DubiousTrackConfig::useMaxGroundspeed() const
{
    return use_max_groundspeed_;
}

void DubiousTrackConfig::useMaxGroundspeed(bool use_max_groundspeed)
{
    use_max_groundspeed_ = use_max_groundspeed;
}

float DubiousTrackConfig::maxGroundspeedKts() const
{
    return max_groundspeed_kts_;
}

void DubiousTrackConfig::maxGroundspeedKts(float max_groundspeed_kts)
{
    max_groundspeed_kts_ = max_groundspeed_kts;
}

bool DubiousTrackConfig::useMaxAcceleration() const
{
    return use_max_acceleration_;
}

void DubiousTrackConfig::useMaxAcceleration(bool use_max_acceleration)
{
    use_max_acceleration_ = use_max_acceleration;
}

float DubiousTrackConfig::maxAcceleration() const
{
    return max_acceleration_;
}

void DubiousTrackConfig::maxAcceleration(float max_acceleration)
{
    max_acceleration_ = max_acceleration;
}

bool DubiousTrackConfig::useMaxTurnrate() const
{
    return use_max_turnrate_;
}

void DubiousTrackConfig::useMaxTurnrate(bool use_max_turnrate)
{
    use_max_turnrate_ = use_max_turnrate;
}

float DubiousTrackConfig::maxTurnrate() const
{
    return max_turnrate_;
}

void DubiousTrackConfig::maxTurnrate(float max_turnrate)
{
    max_turnrate_ = max_turnrate;
}

bool DubiousTrackConfig::useMaxROCD() const
{
    return use_max_rocd_;
}

void DubiousTrackConfig::useMaxROCD(bool use_rocd)
{
    use_max_rocd_ = use_rocd;
}

float DubiousTrackConfig::maxROCD() const
{
    return max_rocd_;
}

void DubiousTrackConfig::maxROCD(float max_rocd)
{
    max_rocd_ = max_rocd;
}

float DubiousTrackConfig::minimumComparisonTime() const
{
    return minimum_comparison_time_;
}

void DubiousTrackConfig::minimumComparisonTime(float minimum_comparison_time)
{
    minimum_comparison_time_ = minimum_comparison_time;
}

float DubiousTrackConfig::maximumComparisonTime() const
{
    return maximum_comparison_time_;
}

void DubiousTrackConfig::maximumComparisonTime(float maximum_comparison_time)
{
    maximum_comparison_time_ = maximum_comparison_time;
}

float DubiousTrackConfig::dubiousProb() const
{
    return dubious_prob_;
}

void DubiousTrackConfig::dubiousProb(float dubious_prob)
{
    dubious_prob_ = dubious_prob;
}

std::shared_ptr<Base> DubiousTrackConfig::createRequirement()
{
    shared_ptr<DubiousTrack> req = make_shared<DubiousTrack>(
                name_, short_name_, group_.name(),
                minimum_comparison_time_, maximum_comparison_time_,
                mark_primary_only_, use_min_updates_, min_updates_,
                use_min_duration_, min_duration_,
                use_max_groundspeed_, max_groundspeed_kts_,
                use_max_acceleration_, max_acceleration_,
                use_max_turnrate_, max_turnrate_,
                use_max_rocd_, max_rocd_, dubious_prob_,
                prob_, COMPARISON_TYPE::LESS_THAN_OR_EQUAL, calculator_);

    return req;
}

BaseConfigWidget* DubiousTrackConfig::createWidget()
{
    return new DubiousTrackConfigWidget(*this);
}

void DubiousTrackConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    BaseConfig::addToReport(report);

    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.getTable("req_table");

    table.addRow({"Probability [%]", "Probability of dubious updates",
                  probabilityPercent(prob_)});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});
    table.addRow({"Minimum Comparison Time [s]",
                  "Minimum time between two updates for the acceleration, turnrate and ROCD checks",
                  Number::floatToDouble(minimum_comparison_time_)});
    table.addRow({"Maximum Comparison Time [s]",
                  "Maximum time between two updates for the acceleration, turnrate and ROCD checks",
                  Number::floatToDouble(maximum_comparison_time_)});
    table.addRow({"Mark Primary-Only", "If all updates of a primary-only track are dubious",
                  String::boolToString(mark_primary_only_)});
    table.addRow({"Use Minimum Updates", "If all updates of a track with fewer updates are dubious",
                  String::boolToString(use_min_updates_)});
    table.addRow({"Minimum Updates [1]", "", min_updates_});
    table.addRow({"Use Minimum Duration", "If all updates of a shorter track are dubious",
                  String::boolToString(use_min_duration_)});
    table.addRow({"Minimum Duration [s]", "", Number::floatToDouble(min_duration_)});
    table.addRow({"Use Maximum Groundspeed", "If updates with a higher groundspeed are dubious",
                  String::boolToString(use_max_groundspeed_)});
    table.addRow({"Maximum Groundspeed [kts]", "", Number::floatToDouble(max_groundspeed_kts_)});
    table.addRow({"Use Maximum Acceleration", "If updates with a higher acceleration are dubious",
                  String::boolToString(use_max_acceleration_)});
    table.addRow({"Maximum Acceleration [m/s^2]", "", Number::floatToDouble(max_acceleration_)});
    table.addRow({"Use Maximum Turnrate", "If updates with a higher turnrate are dubious",
                  String::boolToString(use_max_turnrate_)});
    table.addRow({"Maximum Turnrate [deg/s]", "", Number::floatToDouble(max_turnrate_)});
    table.addRow({"Use Maximum ROCD", "If updates with a higher rate of climb or descent are dubious",
                  String::boolToString(use_max_rocd_)});
    table.addRow({"Maximum ROCD [ft/s]", "", Number::floatToDouble(max_rocd_)});
    table.addRow({"Dubious Probability [%]",
                  "A track is dubious if the ratio of its dubious updates is higher",
                  probabilityPercent(Number::floatToDouble(dubious_prob_))});
}

}
