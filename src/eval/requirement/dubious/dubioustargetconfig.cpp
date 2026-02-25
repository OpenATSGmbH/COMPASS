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

#include "eval/requirement/dubious/dubioustargetconfig.h"
#include "eval/requirement/dubious/dubioustargetconfigwidget.h"
#include "eval/requirement/dubious/dubioustarget.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

using namespace std;

namespace EvaluationRequirement
{
    
DubiousTargetConfig::DubiousTargetConfig(
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

DubiousTargetConfig::~DubiousTargetConfig()
{
}

bool DubiousTargetConfig::markPrimaryOnly() const
{
    return mark_primary_only_;
}

void DubiousTargetConfig::markPrimaryOnly(bool mark_primary_only)
{
    mark_primary_only_ = mark_primary_only;
}

bool DubiousTargetConfig::useMinUpdates() const
{
    return use_min_updates_;
}

void DubiousTargetConfig::useMinUpdates(bool use_min_updates)
{
    use_min_updates_ = use_min_updates;
}

unsigned int DubiousTargetConfig::minUpdates() const
{
    return min_updates_;
}

void DubiousTargetConfig::minUpdates(unsigned int min_updates)
{
    min_updates_ = min_updates;
}

bool DubiousTargetConfig::useMinDuration() const
{
    return use_min_duration_;
}

void DubiousTargetConfig::useMinDuration(bool use_min_duration)
{
    use_min_duration_ = use_min_duration;
}

float DubiousTargetConfig::minDuration() const
{
    return min_duration_;
}

void DubiousTargetConfig::minDuration(float min_duration)
{
    min_duration_ = min_duration;
}

bool DubiousTargetConfig::useMaxGroundspeed() const
{
    return use_max_groundspeed_;
}

void DubiousTargetConfig::useMaxGroundspeed(bool use_max_groundspeed)
{
    use_max_groundspeed_ = use_max_groundspeed;
}

float DubiousTargetConfig::maxGroundspeedKts() const
{
    return max_groundspeed_kts_;
}

void DubiousTargetConfig::maxGroundspeedKts(float max_groundspeed_kts)
{
    max_groundspeed_kts_ = max_groundspeed_kts;
}

bool DubiousTargetConfig::useMaxAcceleration() const
{
    return use_max_acceleration_;
}

void DubiousTargetConfig::useMaxAcceleration(bool use_max_acceleration)
{
    use_max_acceleration_ = use_max_acceleration;
}

float DubiousTargetConfig::maxAcceleration() const
{
    return max_acceleration_;
}

void DubiousTargetConfig::maxAcceleration(float max_acceleration)
{
    max_acceleration_ = max_acceleration;
}

bool DubiousTargetConfig::useMaxTurnrate() const
{
    return use_max_turnrate_;
}

void DubiousTargetConfig::useMaxTurnrate(bool use_max_turnrate)
{
    use_max_turnrate_ = use_max_turnrate;
}

float DubiousTargetConfig::maxTurnrate() const
{
    return max_turnrate_;
}

void DubiousTargetConfig::maxTurnrate(float max_turnrate)
{
    max_turnrate_ = max_turnrate;
}

bool DubiousTargetConfig::useMaxROCD() const
{
    return use_max_rocd_;
}

void DubiousTargetConfig::useMaxROCD(bool use_rocd)
{
    use_max_rocd_ = use_rocd;
}

float DubiousTargetConfig::maxROCD() const
{
    return max_rocd_;
}

void DubiousTargetConfig::maxROCD(float max_rocd)
{
    max_rocd_ = max_rocd;
}

float DubiousTargetConfig::minimumComparisonTime() const
{
    return minimum_comparison_time_;
}

void DubiousTargetConfig::minimumComparisonTime(float minimum_comparison_time)
{
    minimum_comparison_time_ = minimum_comparison_time;
}

float DubiousTargetConfig::maximumComparisonTime() const
{
    return maximum_comparison_time_;
}

void DubiousTargetConfig::maximumComparisonTime(float maximum_comparison_time)
{
    maximum_comparison_time_ = maximum_comparison_time;
}

float DubiousTargetConfig::dubiousProb() const
{
    return dubious_prob_;
}

void DubiousTargetConfig::dubiousProb(float dubious_prob)
{
    dubious_prob_ = dubious_prob;
}

std::shared_ptr<Base> DubiousTargetConfig::createRequirement()
{
    shared_ptr<DubiousTarget> req = make_shared<DubiousTarget>(
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

BaseConfigWidget* DubiousTargetConfig::createWidget()
{
    return new DubiousTargetConfigWidget(*this);
}

void DubiousTargetConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    //auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    //   auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    //    table.addRow({"Name", "Requirement name", name_});
    //    table.addRow({"Short Name", "Requirement short name", short_name_});
    //    table.addRow({"Comment", "", comment_});

    // prob & check type added in subclass
}

}
