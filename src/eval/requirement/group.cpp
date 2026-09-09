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
#include "evaluationcalculator.h"
#include "eval/requirement/detection/detectionconfig.h"
#include "eval/requirement/position/distanceconfig.h"
#include "eval/requirement/position/distancermsconfig.h"
#include "eval/requirement/position/radarrangeconfig.h"
#include "eval/requirement/position/radarazimuthconfig.h"
#include "eval/requirement/position/alongconfig.h"
#include "eval/requirement/position/acrossconfig.h"
#include "eval/requirement/latency/latencyconfig.h"
#include "eval/requirement/latency/adsblatencyconfig.h"
#include "eval/requirement/speed/speedconfig.h"
#include "eval/requirement/trackangle/trackangleconfig.h"
#include "eval/requirement/identification/correctconfig.h"
#include "eval/requirement/identification/falseconfig.h"
#include "eval/requirement/identification/correct_period.h"
#include "eval/requirement/identification/change_delay.h"
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
    {"EvaluationRequirementIdentificationChangeDelayConfig", "Identification Change Delay"},
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
    {"EvaluationRequirementADSBLatencyConfig", "ADS-B Latency"},
    {"EvaluationRequirementSpeedConfig", "Speed"},
    {"EvaluationRequirementTrackAngleConfig", "TrackAngle"},
    {"EvaluationRequirementMoMLongAccConfig", "MoM Longitudinal Acceleration Correct"},
    {"EvaluationRequirementMoMTransAccConfig", "MoM Transversal Acceleration Correct"},
    {"EvaluationRequirementMoMVertRateConfig", "MoM Vertical Rate Correct"},
    {"EvaluationRequirementROCDCorrectConfig", "ROCD Correct"},
    {"EvaluationRequirementAccelerationCorrectConfig", "Acceleration Correct"},
    {"EvaluationRequirementCoastingCorrectConfig", "Track Coasting Correct"}
};

Group::Group(nlohmann::json& config,
             EvaluationStandard* parent)
    : Configurable(config, parent),
      EvaluationStandardTreeItem(parent),
      standard_(*parent),
      calculator_(*standard_.parentConfigurable())
{
    registerParameter("name", &name_, std::string());
    registerParameter("use", &use_, true);

    traced_assert(name_.size());

    createSubConfigurables();
}

EvaluationStandard* Group::parentConfigurable() const
{
    return static_cast<EvaluationStandard*>(Configurable::parentConfigurable());
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

void Group::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);

    if (class_name == "EvaluationRequirementExtraDataConfig")
    {
        EvaluationRequirement::ExtraDataConfig* config =
                new EvaluationRequirement::ExtraDataConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementExtraTrackConfig")
    {
        EvaluationRequirement::ExtraTrackConfig* config =
                new EvaluationRequirement::ExtraTrackConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementDubiousTrackConfig")
    {
        EvaluationRequirement::DubiousTrackConfig* config =
                new EvaluationRequirement::DubiousTrackConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementDubiousTargetConfig")
    {
        EvaluationRequirement::DubiousTargetConfig* config =
                new EvaluationRequirement::DubiousTargetConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementDetectionConfig")
    {
        EvaluationRequirement::DetectionConfig* config =
                new EvaluationRequirement::DetectionConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementPositionDistanceConfig")
    {
        EvaluationRequirement::PositionDistanceConfig* config =
                new EvaluationRequirement::PositionDistanceConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementPositionDistanceRMSConfig")
    {
        EvaluationRequirement::PositionDistanceRMSConfig* config =
                new EvaluationRequirement::PositionDistanceRMSConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementPositionRadarRangeConfig")
    {
        EvaluationRequirement::PositionRadarRangeConfig* config =
                new EvaluationRequirement::PositionRadarRangeConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementPositionRadarAzimuthConfig")
    {
        EvaluationRequirement::PositionRadarAzimuthConfig* config =
                new EvaluationRequirement::PositionRadarAzimuthConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementPositionAlongConfig")
    {
        EvaluationRequirement::PositionAlongConfig* config =
                new EvaluationRequirement::PositionAlongConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementPositionAcrossConfig")
    {
        EvaluationRequirement::PositionAcrossConfig* config =
                new EvaluationRequirement::PositionAcrossConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementPositionLatencyConfig")
    {
        EvaluationRequirement::PositionLatencyConfig* config =
                new EvaluationRequirement::PositionLatencyConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementADSBLatencyConfig")
    {
        EvaluationRequirement::ADSBLatencyConfig* config =
                new EvaluationRequirement::ADSBLatencyConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementSpeedConfig")
    {
        EvaluationRequirement::SpeedConfig* config =
                new EvaluationRequirement::SpeedConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementTrackAngleConfig")
    {
        EvaluationRequirement::TrackAngleConfig* config =
                new EvaluationRequirement::TrackAngleConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementIdentificationCorrectConfig")
    {
        EvaluationRequirement::IdentificationCorrectConfig* config =
                new EvaluationRequirement::IdentificationCorrectConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementIdentificationFalseConfig")
    {
        EvaluationRequirement::IdentificationFalseConfig* config =
                new EvaluationRequirement::IdentificationFalseConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementIdentificationCorrectPeriodConfig")
    {
        EvaluationRequirement::IdentificationCorrectPeriodConfig* config =
                new EvaluationRequirement::IdentificationCorrectPeriodConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementIdentificationChangeDelayConfig")
    {
        EvaluationRequirement::IdentificationChangeDelayConfig* config =
                new EvaluationRequirement::IdentificationChangeDelayConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementModeAPresentConfig")
    {
        EvaluationRequirement::ModeAPresentConfig* config =
                new EvaluationRequirement::ModeAPresentConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementModeAFalseConfig")
    {
        EvaluationRequirement::ModeAFalseConfig* config =
                new EvaluationRequirement::ModeAFalseConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementModeCPresentConfig")
    {
        EvaluationRequirement::ModeCPresentConfig* config =
                new EvaluationRequirement::ModeCPresentConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementModeCCorrectConfig")
    {
        EvaluationRequirement::ModeCCorrectConfig* config =
                new EvaluationRequirement::ModeCCorrectConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementModeCFalseConfig")
    {
        EvaluationRequirement::ModeCFalseConfig* config =
                new EvaluationRequirement::ModeCFalseConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementModeCCorrectPeriodConfig")
    {
        EvaluationRequirement::ModeCCorrectPeriodConfig* config =
                new EvaluationRequirement::ModeCCorrectPeriodConfig(
                    child_json, this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementMoMLongAccConfig")
    {
        EvaluationRequirement::GenericIntegerConfig* config = new EvaluationRequirement::GenericIntegerConfig(
                child_json, "MomLongAccCorrect", this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementMoMTransAccConfig")
    {
        EvaluationRequirement::GenericIntegerConfig* config = new EvaluationRequirement::GenericIntegerConfig(
                child_json, "MomTransAccCorrect", this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementMoMVertRateConfig")
    {
        EvaluationRequirement::GenericIntegerConfig* config = new EvaluationRequirement::GenericIntegerConfig(
                child_json, "MomVertRateCorrect", this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementCoastingCorrectConfig")
    {
        EvaluationRequirement::GenericIntegerConfig* config = new EvaluationRequirement::GenericIntegerConfig(
                child_json, "CoastingCorrect", this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementROCDCorrectConfig")
    {
        EvaluationRequirement::GenericDoubleConfig* config = new EvaluationRequirement::GenericDoubleConfig(
                child_json, "ROCDCorrect", this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else if (class_name == "EvaluationRequirementAccelerationCorrectConfig")
    {
        EvaluationRequirement::GenericDoubleConfig* config = new EvaluationRequirement::GenericDoubleConfig(
                child_json, "AccelerationCorrect", this);
        logdbg << "adding config " << config->name();

        traced_assert(!hasRequirementConfig(config->name()));
        configs_.push_back(std::unique_ptr<EvaluationRequirement::BaseConfig>(config));
    }
    else
        throw std::runtime_error("EvaluationRequirementGroup: generateSubConfigurable: unknown class_name " +
                                 class_name);
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

void Group::addRequirementConfig (const std::string& class_name, const std::string& name, const std::string& short_name)
{
    loginf << "class_name " << class_name << " name " << name
           << " short_name " << short_name;

    traced_assert(!hasRequirementConfig(name));

    std::string instance = class_name + name + "0";

    auto& child_json = addNewSubConfiguration(class_name, instance);
    child_json[Configuration::ParameterSection]["name"] = name;
    child_json[Configuration::ParameterSection]["short_name"] = short_name;

    generateSubConfigurable(child_json);

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

