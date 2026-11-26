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

#include "requirement/mode_c/correct_period.h"
#include "requirement/group.h"

#include "results/mode_c/correct_period.h"
#include "results/base/detailcode.h"

#include "report/sectioncontenttable.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "logger.h"

#include <cmath>

#include <QLineEdit>
#include <QCheckBox>
#include <QDoubleValidator>
#include <QFormLayout>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

/********************************************************************************************************
 * ModeCCorrectPeriod
 ********************************************************************************************************/

/**
 */
ModeCCorrectPeriod::ModeCCorrectPeriod(const std::string& name, 
                                       const std::string& short_name, 
                                       const std::string& group_name,
                                       double prob, 
                                       COMPARISON_TYPE prob_check_type, 
                                       EvaluationCalculator& calculator,
                                       float update_interval_s, 
                                       bool  use_miss_tolerance,
                                       float miss_tolerance_s,
                                       float max_distance_ft,
                                       bool  level_flight_only,
                                       float max_level_rocd_ftpm)
:   IntervalBase(name, 
                 short_name, 
                 group_name, 
                 prob, 
                 prob_check_type, 
                 calculator, 
                 update_interval_s, 
                 {}, 
                 {},
                 use_miss_tolerance ? boost::optional<float>(miss_tolerance_s) : boost::optional<float>())
,   max_distance_ft_    (max_distance_ft    )
,   level_flight_only_  (level_flight_only  )
,   max_level_rocd_ftpm_(max_level_rocd_ftpm)
{
}

/**
 * With level flight only, the periods contain only reference updates in level flight
 * (absolute reference ROCD at most the maximum level ROCD), e.g. EUROCAE ED-142 6.4.3:
 * "use segments of trajectories with constant flight level only".
 * A reference update without ROCD is not in level flight.
 */
bool ModeCCorrectPeriod::useReferenceUpdate(const EvaluationTargetData& target_data,
                                            const dbContent::TargetReport::DataID& ref_id) const
{
    if (!level_flight_only_)
        return true;

    auto rocd = target_data.refChain().rocd(ref_id);

    return rocd.has_value() && std::fabs(rocd.value()) <= max_level_rocd_ftpm_;
}

/**
 */
ModeCCorrectPeriod::Validity ModeCCorrectPeriod::isValid(const dbContent::TargetReport::DataID& data_id,
                                                         const EvaluationTargetData& target_data,
                                                         const SectorLayer& sector_layer,
                                                         const boost::posix_time::time_duration& max_ref_time_diff) const
{
    auto cmp_res = compareModeC(data_id, target_data, max_ref_time_diff, max_distance_ft_);

    bool no_ref = cmp_res == ValueComparisonResult::Unknown_NoRefData;
    bool failed = cmp_res == ValueComparisonResult::Unknown_NoTstData || 
                  cmp_res == ValueComparisonResult::Different;

    Validity v;

    v.value   = no_ref ? Validity::Value::RefDataMissing : (failed ? Validity::Value::Invalid : Validity::Value::Valid);
    v.code    = comparisonCode(cmp_res,
                               EvaluationRequirementResult::DetailCode::Value::ModeCDiffers,
                               EvaluationRequirementResult::DetailCode::Value::ModeCMissing);

    return v;
}

/**
 */
std::shared_ptr<EvaluationRequirementResult::Single> ModeCCorrectPeriod::createResult(
                                        const std::string& result_id,
                                        std::shared_ptr<Base> instance, 
                                        const EvaluationTargetData& target_data,
                                        const SectorLayer& sector_layer, 
                                        const std::vector<EvaluationDetail>& details,
                                        const TimePeriodCollection& periods,
                                        unsigned int sum_uis,
                                        unsigned int misses_total)
{
    return std::make_shared<EvaluationRequirementResult::SingleModeCCorrectPeriod>(
                                        "SingleModeCCorrectPeriod", 
                                        result_id, 
                                        instance, 
                                        sector_layer, 
                                        target_data.utn_, 
                                        &target_data, 
                                        calculator_, 
                                        details, 
                                        sum_uis, 
                                        misses_total, 
                                        periods);
}

/**
 */
std::string ModeCCorrectPeriod::probabilityNameShortStatic()
{
    return "PCMCD";
}

/**
 */
std::string ModeCCorrectPeriod::probabilityNameStatic()
{
    return "Probability of Correct Mode C Detection";
}

/**
 */
std::string ModeCCorrectPeriod::probabilityNameShort() const
{
    return ModeCCorrectPeriod::probabilityNameShortStatic();
}

/**
 */
std::string ModeCCorrectPeriod::probabilityName() const
{
    return ModeCCorrectPeriod::probabilityNameStatic();
}

/********************************************************************************************************
 * ModeCCorrectPeriodConfig
 ********************************************************************************************************/

/**
 */
ModeCCorrectPeriodConfig::ModeCCorrectPeriodConfig(nlohmann::json& config,
                                                   Group* parent)
:   IntervalBaseConfig(config, parent)
{
    configure(UseMissTol | UseExclDetMisses);

    registerParameter("max_distance_ft", &max_distance_ft_, 300.0f);
    registerParameter("level_flight_only", &level_flight_only_, false);
    registerParameter("max_level_rocd_ftpm", &max_level_rocd_ftpm_, 300.0f);
}

/**
*/
std::shared_ptr<Base> ModeCCorrectPeriodConfig::createRequirement()
{
    shared_ptr<ModeCCorrectPeriod> req = make_shared<ModeCCorrectPeriod>(
                name_, 
                short_name_, 
                group_.name(), 
                prob_, 
                prob_check_type_, 
                calculator_,
                update_interval_s_,
                use_miss_tolerance_, 
                miss_tolerance_s_,
                max_distance_ft_,
                level_flight_only_,
                max_level_rocd_ftpm_);

    req->excludeDetectionMisses(exclude_detection_misses_);

    return req;
}

/**
*/
float ModeCCorrectPeriodConfig::maxDistanceFt() const
{
    return max_distance_ft_;
}

/**
*/
void ModeCCorrectPeriodConfig::maxDistanceFt(float value)
{
    max_distance_ft_ = value;
}

/**
*/
bool ModeCCorrectPeriodConfig::levelFlightOnly() const
{
    return level_flight_only_;
}

/**
*/
void ModeCCorrectPeriodConfig::levelFlightOnly(bool value)
{
    level_flight_only_ = value;
}

/**
*/
float ModeCCorrectPeriodConfig::maxLevelROCDFtpm() const
{
    return max_level_rocd_ftpm_;
}

/**
*/
void ModeCCorrectPeriodConfig::maxLevelROCDFtpm(float value)
{
    max_level_rocd_ftpm_ = value;
}

/**
*/
BaseConfigWidget* ModeCCorrectPeriodConfig::createWidget_impl()
{
    return new ModeCCorrectPeriodConfigWidget(*this);
}

/**
*/
void ModeCCorrectPeriodConfig::addCustomTableEntries(ResultReport::SectionContentTable& table) const
{
    table.addRow( { "Maximum Difference [ft]", 
                    "Maximum altitude difference between the test and the reference",
                    std::to_string(max_distance_ft_)});
    table.addRow( { "Level Flight Only",
                    "Use only reference updates in level flight",
                    level_flight_only_ ? "true" : "false"});

    if (level_flight_only_)
        table.addRow( { "Maximum Level ROCD [ft/min]",
                        "Maximum absolute reference ROCD of level flight",
                        std::to_string(max_level_rocd_ftpm_)});
}

/**
*/
std::string ModeCCorrectPeriodConfig::probabilityDescription() const
{
    return ModeCCorrectPeriod::probabilityNameStatic();
}

/********************************************************************************************************
 * ModeCCorrectPeriodConfigWidget
 ********************************************************************************************************/

/**
*/
ModeCCorrectPeriodConfigWidget::ModeCCorrectPeriodConfigWidget(ModeCCorrectPeriodConfig& cfg)
:   IntervalBaseConfigWidget(cfg)
{
    distance_value_edit_ = new QLineEdit(QString::number(config().maxDistanceFt()));
    distance_value_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 2, this));
    connect(distance_value_edit_, &QLineEdit::textEdited,
            this, &ModeCCorrectPeriodConfigWidget::distanceChanged);

    form_layout_->addRow("Maximum Offset [ft]", distance_value_edit_);

    level_flight_only_box_ = new QCheckBox();
    level_flight_only_box_->setChecked(config().levelFlightOnly());
    level_flight_only_box_->setToolTip("Use only reference updates in level flight (absolute reference ROCD at most the maximum level ROCD)");
    connect(level_flight_only_box_, &QCheckBox::toggled,
            this, &ModeCCorrectPeriodConfigWidget::levelFlightOnlyChanged);

    form_layout_->addRow("Level Flight Only", level_flight_only_box_);

    max_level_rocd_edit_ = new QLineEdit(QString::number(config().maxLevelROCDFtpm()));
    max_level_rocd_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 2, this));
    max_level_rocd_edit_->setToolTip("Maximum absolute reference ROCD of level flight");
    max_level_rocd_edit_->setEnabled(config().levelFlightOnly());
    connect(max_level_rocd_edit_, &QLineEdit::textEdited,
            this, &ModeCCorrectPeriodConfigWidget::maxLevelROCDChanged);

    form_layout_->addRow("Maximum Level ROCD [ft/min]", max_level_rocd_edit_);

    prob_edit_->setToolTip(QString::fromStdString(ModeCCorrectPeriod::probabilityNameStatic()));
}

/**
*/
ModeCCorrectPeriodConfig& ModeCCorrectPeriodConfigWidget::config()
{
    return static_cast<ModeCCorrectPeriodConfig&>(config_);
}

/**
*/
void ModeCCorrectPeriodConfigWidget::distanceChanged(const QString& value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxDistanceFt(val);
    else
        loginf << "invalid value";
}

/**
*/
void ModeCCorrectPeriodConfigWidget::levelFlightOnlyChanged(bool checked)
{
    config().levelFlightOnly(checked);
    max_level_rocd_edit_->setEnabled(checked);
}

/**
*/
void ModeCCorrectPeriodConfigWidget::maxLevelROCDChanged(const QString& value)
{
    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxLevelROCDFtpm(val);
    else
        loginf << "invalid value " << value.toStdString();
}

} // namespace EvaluationRequirement
