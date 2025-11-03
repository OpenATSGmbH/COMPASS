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

#include "eval/requirement/group.h"
#include "evaluationstandard.h"
#include "eval/requirement/detection/detectionconfig.h"
#include "eval/requirement/position/distanceconfig.h"
#include "eval/requirement/position/distancermsconfig.h"
#include "eval/requirement/position/radarrangeconfig.h"
#include "eval/requirement/position/radarazimuthconfig.h"
#include "eval/requirement/position/alongconfig.h"
#include "eval/requirement/position/acrossconfig.h"
#include "eval/requirement/position/latencyconfig.h"
#include "eval/requirement/speed/speedconfig.h"
#include "eval/requirement/trackangle/trackangleconfig.h"
#include "eval/requirement/identification/correctconfig.h"
#include "eval/requirement/identification/falseconfig.h"
#include "eval/requirement/identification/correct_period.h"
#include "eval/requirement/mode_a/presentconfig.h"
#include "eval/requirement/mode_a/falseconfig.h"
#include "eval/requirement/mode_c/falseconfig.h"
#include "eval/requirement/mode_c/presentconfig.h"
#include "eval/requirement/mode_c/correctconfig.h"
#include "eval/requirement/mode_c/correct_period.h"
#include "eval/requirement/extra/dataconfig.h"
#include "eval/requirement/extra/trackconfig.h"
#include "eval/requirement/dubious/dubioustrackconfig.h"
#include "eval/requirement/dubious/dubioustargetconfig.h"
#include "eval/requirement/generic/genericconfig.h"
#include "logger.h"

#include <QInputDialog>
#include <QMessageBox>

#include <algorithm>

using namespace std;

const std::map<std::string, std::string> Group::requirement_type_mapping_
{
    {"EvaluationRequirementExtraDataConfig", "Extra Data"},
    {"EvaluationRequirementExtraTrackConfig", "Extra Track"},
    {"EvaluationRequirementDubiousTargetConfig", "Dubious Target"},
    {"EvaluationRequirementDubiousTrackConfig", "Dubious Track"},
    {"EvaluationRequirementDetectionConfig", "Detection"},
    {"EvaluationRequirementIdentificationCorrectConfig", "Identification Correct"},
    {"EvaluationRequirementIdentificationFalseConfig", "Identification False"},
    {"EvaluationRequirementIdentificationCorrectPeriodConfig", "Identification Correct (Periods)"},
    {"EvaluationRequirementModeAPresentConfig", "Mode 3/A Present"},
    {"EvaluationRequirementModeAFalseConfig", "Mode 3/A False"},
    {"EvaluationRequirementModeCPresentConfig", "Mode C Present"},
    {"EvaluationRequirementModeCCorrectConfig", "Mode C Correct"},
    {"EvaluationRequirementModeCFalseConfig", "Mode C False"},
    {"EvaluationRequirementModeCCorrectPeriodConfig", "Mode C Correct (Periods)"},
    {"EvaluationRequirementPositionDistanceConfig", "Position Distance"},
    {"EvaluationRequirementPositionDistanceRMSConfig", "Position Distance RMS"},
    {"EvaluationRequirementPositionRadarRangeConfig", "Position Radar Range"},
    {"EvaluationRequirementPositionRadarAzimuthConfig", "Position Radar Azimuth"},
    {"EvaluationRequirementPositionAlongConfig", "Position Along"},
    {"EvaluationRequirementPositionAcrossConfig", "Position Across"},
    {"EvaluationRequirementPositionLatencyConfig", "Position Latency"},
    {"EvaluationRequirementSpeedConfig", "Speed"},
    {"EvaluationRequirementTrackAngleConfig", "TrackAngle"},
    {"EvaluationRequirementMoMLongAccConfig", "MoM Longitudinal Acceleration Correct"},
    {"EvaluationRequirementMoMTransAccConfig", "MoM Transversal Acceleration Correct"},
    {"EvaluationRequirementMoMVertRateConfig", "MoM Vertical Rate Correct"},
    {"EvaluationRequirementROCDCorrectConfig", "ROCD Correct"},
    {"EvaluationRequirementAccelerationCorrectConfig", "Acceleration Correct"},
    {"EvaluationRequirementCoastingCorrectConfig", "Track Coasting Correct"}
};

Group::Group(const std::string& class_id, const std::string& instance_id,
                                                       EvaluationStandard& standard, EvaluationCalculator& calculator)
    : Configurable(class_id, instance_id, &standard), EvaluationStandardTreeItem(&standard), standard_(standard),
      calculator_(calculator)
{
    registerParameter("name", &name_, std::string());
    registerParameter("use", &use_, true);

    traced_assert(name_.size());

    createSubConfigurables();
}

Group::~Group()
{
}

void Group::use(bool ok)
{
    use_ = ok;
}

bool Group::used() const
{
    return use_;
}

bool Group::checkable() const
{
    return true;
}

void Group::generateSubConfigurable(const std::string& class_id,
                                                         const std::string& instance_id)
{
    if (class_id == "EvaluationRequirementExtraDataConfig")
    {
        EvaluationRequirement::ExtraDataConfig* config =
                new EvaluationRequirement::ExtraDataConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementExtraTrackConfig")
    {
        EvaluationRequirement::ExtraTrackConfig* config =
                new EvaluationRequirement::ExtraTrackConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementDubiousTrackConfig")
    {
        EvaluationRequirement::DubiousTrackConfig* config =
                new EvaluationRequirement::DubiousTrackConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementDubiousTargetConfig")
    {
        EvaluationRequirement::DubiousTargetConfig* config =
                new EvaluationRequirement::DubiousTargetConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementDetectionConfig")
    {
        EvaluationRequirement::DetectionConfig* config =
                new EvaluationRequirement::DetectionConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementPositionDistanceConfig")
    {
        EvaluationRequirement::PositionDistanceConfig* config =
                new EvaluationRequirement::PositionDistanceConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementPositionDistanceRMSConfig")
    {
        EvaluationRequirement::PositionDistanceRMSConfig* config =
                new EvaluationRequirement::PositionDistanceRMSConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementPositionRadarRangeConfig")
    {
        EvaluationRequirement::PositionRadarRangeConfig* config =
                new EvaluationRequirement::PositionRadarRangeConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementPositionRadarAzimuthConfig")
    {
        EvaluationRequirement::PositionRadarAzimuthConfig* config =
                new EvaluationRequirement::PositionRadarAzimuthConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementPositionAlongConfig")
    {
        EvaluationRequirement::PositionAlongConfig* config =
                new EvaluationRequirement::PositionAlongConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementPositionAcrossConfig")
    {
        EvaluationRequirement::PositionAcrossConfig* config =
                new EvaluationRequirement::PositionAcrossConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementPositionLatencyConfig")
    {
        EvaluationRequirement::PositionLatencyConfig* config =
                new EvaluationRequirement::PositionLatencyConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementSpeedConfig")
    {
        EvaluationRequirement::SpeedConfig* config =
                new EvaluationRequirement::SpeedConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementTrackAngleConfig")
    {
        EvaluationRequirement::TrackAngleConfig* config =
                new EvaluationRequirement::TrackAngleConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementIdentificationCorrectConfig")
    {
        EvaluationRequirement::IdentificationCorrectConfig* config =
                new EvaluationRequirement::IdentificationCorrectConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementIdentificationFalseConfig")
    {
        EvaluationRequirement::IdentificationFalseConfig* config =
                new EvaluationRequirement::IdentificationFalseConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementIdentificationCorrectPeriodConfig")
    {
        EvaluationRequirement::IdentificationCorrectPeriodConfig* config =
                new EvaluationRequirement::IdentificationCorrectPeriodConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementModeAPresentConfig")
    {
        EvaluationRequirement::ModeAPresentConfig* config =
                new EvaluationRequirement::ModeAPresentConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementModeAFalseConfig")
    {
        EvaluationRequirement::ModeAFalseConfig* config =
                new EvaluationRequirement::ModeAFalseConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementModeCPresentConfig")
    {
        EvaluationRequirement::ModeCPresentConfig* config =
                new EvaluationRequirement::ModeCPresentConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementModeCCorrectConfig")
    {
        EvaluationRequirement::ModeCCorrectConfig* config =
                new EvaluationRequirement::ModeCCorrectConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementModeCFalseConfig")
    {
        EvaluationRequirement::ModeCFalseConfig* config =
                new EvaluationRequirement::ModeCFalseConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementModeCCorrectPeriodConfig")
    {
        EvaluationRequirement::ModeCCorrectPeriodConfig* config =
                new EvaluationRequirement::ModeCCorrectPeriodConfig(
                    class_id, instance_id, *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementMoMLongAccConfig")
    {
        EvaluationRequirement::GenericIntegerConfig* config = new EvaluationRequirement::GenericIntegerConfig(
                class_id, instance_id, "MomLongAccCorrect", *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementMoMTransAccConfig")
    {
        EvaluationRequirement::GenericIntegerConfig* config = new EvaluationRequirement::GenericIntegerConfig(
            class_id, instance_id, "MomTransAccCorrect", *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementMoMVertRateConfig")
    {
        EvaluationRequirement::GenericIntegerConfig* config = new EvaluationRequirement::GenericIntegerConfig(
            class_id, instance_id, "MomVertRateCorrect", *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementCoastingCorrectConfig")
    {
        EvaluationRequirement::GenericIntegerConfig* config = new EvaluationRequirement::GenericIntegerConfig(
            class_id, instance_id, "CoastingCorrect", *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementROCDCorrectConfig")
    {
        EvaluationRequirement::GenericDoubleConfig* config = new EvaluationRequirement::GenericDoubleConfig(
            class_id, instance_id, "ROCDCorrect", *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_id == "EvaluationRequirementAccelerationCorrectConfig")
    {
        EvaluationRequirement::GenericDoubleConfig* config = new EvaluationRequirement::GenericDoubleConfig(
            class_id, instance_id, "AccelerationCorrect", *this, standard_, calculator_);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else
        throw std::runtime_error("EvaluationRequirementGroup: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

std::string Group::name() const
{
    return name_;
}


bool Group::hasRequirementConfig (const std::string& name)
{
    auto iter = std::find_if(configs_.begin(), configs_.end(),
       [&name](const unique_ptr<EvaluationRequirement::BaseConfig>& x) { return x->name() == name;});

    return iter != configs_.end();
}

void Group::addRequirementConfig (const std::string& class_id, const std::string& name, const std::string& short_name)
{
    loginf << "class_id " << class_id << " name " << name
           << " short_name " << short_name;

    traced_assert(!hasRequirementConfig(name));

    std::string instance = class_id + name + "0";

    auto config = Configuration::create(class_id, instance);
    config->addParameter<std::string>("name", name);
    config->addParameter<std::string>("short_name", short_name);

    generateSubConfigurableFromConfig(std::move(config));

    sortConfigs();

    traced_assert(hasRequirementConfig(name));

    emit configsChangedSignal();
}

EvaluationRequirement::BaseConfig& Group::requirementConfig (const std::string& name)
{
    traced_assert(hasRequirementConfig(name));

    auto iter = std::find_if(configs_.begin(), configs_.end(),
                             [&name](const unique_ptr<EvaluationRequirement::BaseConfig>& x) { return x->name() == name;});

    return **iter;
}

void Group::removeRequirementConfig (const std::string& name)
{
    traced_assert(hasRequirementConfig(name));

    auto iter = std::find_if(configs_.begin(), configs_.end(),
                             [&name](const unique_ptr<EvaluationRequirement::BaseConfig>& x) { return x->name() == name;});

    traced_assert(iter != configs_.end());

    configs_.erase(iter);

    emit configsChangedSignal();
}

void Group::checkSubConfigurables()
{

}

EvaluationStandardTreeItem* Group::child(int row)
{
    if (row < 0 || row >= static_cast<int>(configs_.size()))
        return nullptr;

    return configs_.at(row).get();
}

int Group::childCount() const
{
    return configs_.size();
}

int Group::columnCount() const
{
    return 1;
}

QVariant Group::data(int column) const
{
    traced_assert(column == 0);

    return name_.c_str();
}

int Group::row() const
{
    return 0;
}

const std::vector<std::unique_ptr<EvaluationRequirement::BaseConfig>>& Group::configs() const
{
    return configs_;
}

void Group::sortConfigs()
{
    sort(configs_.begin(), configs_.end(),
         [](const unique_ptr<EvaluationRequirement::BaseConfig>&a, const unique_ptr<EvaluationRequirement::BaseConfig>& b) -> bool
    {
        return a->name() > b->name();
    });
}

unsigned int Group::numUsedRequirements() const
{
    unsigned int n = 0;

    for (const auto& c : configs_)
        if (c->used())
            ++n;

    return n;
}

void Group::useAll()
{
    for (auto& c : configs_)
        c->use(true);
}

void Group::useNone()
{
    for (auto& c : configs_)
        c->use(false);
}
