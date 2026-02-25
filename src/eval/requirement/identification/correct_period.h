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

#include "eval/requirement/base/intervalbase.h"
#include "eval/requirement/base/intervalbaseconfig.h"
#include "eval/requirement/base/intervalbaseconfigwidget.h"

class QComboBox;

namespace EvaluationRequirement
{

/**
*/
class IdentificationCorrectPeriodConfig : public IntervalBaseConfig
{
public:
    enum class IdentificationType
    {
        AircraftAddress = 0,
        AircraftID,
        ModeA
    };

    IdentificationCorrectPeriodConfig(nlohmann::json& config,
                                      Group* parent);
    virtual ~IdentificationCorrectPeriodConfig() = default;

    IdentificationType identificationType() const { return identification_type_; }
    void identificationType(IdentificationType type) { identification_type_ = type; }

    std::shared_ptr<Base> createRequirement() override;

protected:
    virtual BaseConfigWidget* createWidget_impl() override;
    virtual std::string probabilityDescription() const override;

    IdentificationType identification_type_ = IdentificationType::AircraftAddress;
};

/**
*/
class IdentificationCorrectPeriod : public IntervalBase
{
public:
    typedef IdentificationCorrectPeriodConfig::IdentificationType IdentificationType;

    IdentificationCorrectPeriod(
            const std::string& name, 
            const std::string& short_name, 
            const std::string& group_name,
            double prob, 
            COMPARISON_TYPE prob_check_type,
            EvaluationCalculator& calculator,
            float update_interval_s, 
            bool  use_miss_tolerance,
            float miss_tolerance_s,
            IdentificationType identification_type);

    static std::string probabilityNameShort(IdentificationType identification_type);
    static std::string probabilityName(IdentificationType identification_type);
    static std::string identificationName(IdentificationType identification_type);

    IdentificationType identificationType() const { return identification_type_; }

    std::string probabilityNameShort() const override final;
    std::string probabilityName() const override final;

protected:
    virtual Validity isValid(const dbContent::TargetReport::DataID& data_id,
                             const EvaluationTargetData& target_data,
                             const SectorLayer& sector_layer,
                             const boost::posix_time::time_duration& max_ref_time_diff) const override;
    virtual std::shared_ptr<EvaluationRequirementResult::Single> createResult(const std::string& result_id,
                                                                              std::shared_ptr<Base> instance, 
                                                                              const EvaluationTargetData& target_data,
                                                                              const SectorLayer& sector_layer, 
                                                                              const std::vector<EvaluationDetail>& details,
                                                                              const TimePeriodCollection& periods,
                                                                              unsigned int sum_uis,
                                                                              unsigned int misses_total) override;
    IdentificationType identification_type_ = IdentificationType::AircraftAddress;
};

/**
*/
class IdentificationCorrectPeriodConfigWidget : public IntervalBaseConfigWidget
{
public:
    IdentificationCorrectPeriodConfigWidget(IdentificationCorrectPeriodConfig& cfg);
    virtual ~IdentificationCorrectPeriodConfigWidget() = default;

protected:
    IdentificationCorrectPeriodConfig& config();

    void identificationTypeChanged();

    QComboBox* identification_type_combo_ = nullptr;
};

}
