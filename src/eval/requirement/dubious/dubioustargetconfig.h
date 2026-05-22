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

#pragma once

#include "eval/requirement/base/probabilitybaseconfig.h"

class Group;
class EvaluationStandard;

namespace ResultReport
{
    class Report;
}

namespace EvaluationRequirement
{

class DubiousTargetConfig : public ProbabilityBaseConfig
{
public:
    DubiousTargetConfig(nlohmann::json& config,
                        Group* parent);
    virtual ~DubiousTargetConfig();

    std::shared_ptr<Base> createRequirement() override;

    virtual void addToReport (std::shared_ptr<ResultReport::Report> report) override;

    bool markPrimaryOnly() const;
    void markPrimaryOnly(bool mark_primary_only);

    bool useMinUpdates() const;
    void useMinUpdates(bool use_min_updates);

    unsigned int minUpdates() const;
    void minUpdates(unsigned int min_updates);

    bool useMinDuration() const;
    void useMinDuration(bool use_min_duration);

    float minDuration() const;
    void minDuration(float min_duration);

    bool useMaxGroundspeed() const;
    void useMaxGroundspeed(bool use_max_groundspeed);

    float maxGroundspeedKts() const;
    void maxGroundspeedKts(float max_groundspeed_kts);

    bool useMaxAcceleration() const;
    void useMaxAcceleration(bool use_max_acceleration);

    float maxAcceleration() const;
    void maxAcceleration(float max_acceleration);

    bool useMaxTurnrate() const;
    void useMaxTurnrate(bool use_max_turnrate);

    float maxTurnrate() const;
    void maxTurnrate(float max_turnrate);

    bool useMaxROCD() const;
    void useMaxROCD(bool use_rocd);

    float maxROCD() const;
    void maxROCD(float max_rocd);

    float minimumComparisonTime() const;
    void minimumComparisonTime(float minimum_comparison_time);

    float maximumComparisonTime() const;
    void maximumComparisonTime(float maximum_comparison_time);

    float dubiousProb() const;
    void dubiousProb(float dubious_prob);


protected:
    float minimum_comparison_time_ {1.0};
    float maximum_comparison_time_ {30.0};

    bool mark_primary_only_ {true};

    bool use_min_updates_ {true};
    unsigned int min_updates_ {10};

    bool use_min_duration_ {true};
    float min_duration_ {30.0};

    bool use_max_groundspeed_ {true};
    float max_groundspeed_kts_ {1333.0};

    bool use_max_acceleration_ {true};
    float max_acceleration_ {29.43}; // m/s^2

    bool use_max_turnrate_ {true};
    float max_turnrate_ {30.0}; // deg/s

    bool use_max_rocd_ {true};
    float max_rocd_ {1000.0}; // ft/s

    float dubious_prob_ {0.05};

    virtual BaseConfigWidget* createWidget() override;
};

}
