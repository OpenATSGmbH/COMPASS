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

class QLineEdit;

namespace ResultReport
{
    class SectionContentTable;
}

namespace EvaluationRequirement
{

/**
*/
class ModeCCorrectPeriod : public IntervalBase
{
public:
    ModeCCorrectPeriod(
            const std::string& name, 
            const std::string& short_name, 
            const std::string& group_name,
            double prob, 
            COMPARISON_TYPE prob_check_type,
            EvaluationCalculator& calculator,
            float update_interval_s, 
            bool  use_miss_tolerance,
            float miss_tolerance_s,
            float max_distance_ft);

    static std::string probabilityNameShortStatic();
    static std::string probabilityNameStatic();

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
    float max_distance_ft_;
};

/**
*/
class ModeCCorrectPeriodConfig : public IntervalBaseConfig
{
public:
    ModeCCorrectPeriodConfig(nlohmann::json& config,
                             Group* parent);
    virtual ~ModeCCorrectPeriodConfig() = default;

    std::shared_ptr<Base> createRequirement() override;

    float maxDistanceFt() const;
    void maxDistanceFt(float value);

protected:
    virtual BaseConfigWidget* createWidget_impl() override;
    virtual void addCustomTableEntries(ResultReport::SectionContentTable& table) const override;
    virtual std::string probabilityDescription() const override;

    float max_distance_ft_ = 300.0f;
};

/**
*/
class ModeCCorrectPeriodConfigWidget : public IntervalBaseConfigWidget
{
public:
    ModeCCorrectPeriodConfigWidget(ModeCCorrectPeriodConfig& cfg);
    virtual ~ModeCCorrectPeriodConfigWidget() = default;

protected:
    ModeCCorrectPeriodConfig& config();

    void distanceChanged(const QString& value);

    QLineEdit* distance_value_edit_ = nullptr;
};

}
