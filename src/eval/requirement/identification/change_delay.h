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

#include "eval/requirement/base/probabilitybase.h"
#include "eval/requirement/base/probabilitybaseconfig.h"
#include "eval/requirement/base/probabilitybaseconfigwidget.h"

class QLineEdit;
class QComboBox;

namespace EvaluationRequirement
{

/**
 * Identity change delay per EUROCAE ED-129C Section 3.3.1 REC 495 and
 * TABLE 4: the delay of change of Aircraft Identification (ACID or Mode A
 * code) with correct value, measured from the final value set in the
 * reference to the first test report carrying that value. Transitional
 * Mode A codes are ignored: a new reference value only counts as a change
 * event when it stays stable for a configurable minimum time.
 */
class IdentificationChangeDelayConfig : public ProbabilityBaseConfig
{
public:
    enum class IdentificationType
    {
        AircraftID = 0,
        ModeA
    };

    IdentificationChangeDelayConfig(nlohmann::json& config,
                                    Group* parent);
    virtual ~IdentificationChangeDelayConfig() = default;

    IdentificationType identificationType() const { return identification_type_; }
    void identificationType(IdentificationType type) { identification_type_ = type; }

    float maxDelay() const;
    void maxDelay(float value);

    float minStableTime() const;
    void minStableTime(float value);

    std::shared_ptr<Base> createRequirement() override;

    virtual void addToReport (std::shared_ptr<ResultReport::Report> report) override;

protected:
    IdentificationType identification_type_ = IdentificationType::AircraftID;

    float max_delay_s_       {15.0f};
    float min_stable_time_s_ {10.0f};

    virtual BaseConfigWidget* createWidget() override;
};

/**
*/
class IdentificationChangeDelay : public ProbabilityBase
{
public:
    typedef IdentificationChangeDelayConfig::IdentificationType IdentificationType;

    IdentificationChangeDelay(const std::string& name,
                              const std::string& short_name,
                              const std::string& group_name,
                              double prob,
                              COMPARISON_TYPE prob_check_type,
                              EvaluationCalculator& calculator,
                              IdentificationType identification_type,
                              float max_delay_s,
                              float min_stable_time_s);

    IdentificationType identificationType() const { return identification_type_; }
    float maxDelay() const { return max_delay_s_; }
    float minStableTime() const { return min_stable_time_s_; }

    virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate(
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer) override;

    static std::string probabilityNameShort(IdentificationType identification_type);
    static std::string probabilityName(IdentificationType identification_type);
    static std::string identificationName(IdentificationType identification_type);

    std::string probabilityNameShort() const override final;
    std::string probabilityName() const override final;

protected:
    IdentificationType identification_type_ = IdentificationType::AircraftID;

    float max_delay_s_       {15.0f};
    float min_stable_time_s_ {10.0f};
};

/**
*/
class IdentificationChangeDelayConfigWidget : public ProbabilityBaseConfigWidget
{
    Q_OBJECT

public slots:
    void identificationTypeChangedSlot();
    void maxDelayEditSlot(QString value);
    void minStableTimeEditSlot(QString value);

public:
    IdentificationChangeDelayConfigWidget(IdentificationChangeDelayConfig& cfg);

protected:
    QComboBox* identification_type_combo_ {nullptr};
    QLineEdit* max_delay_edit_            {nullptr};
    QLineEdit* min_stable_time_edit_      {nullptr};

    IdentificationChangeDelayConfig& config();
};

}
