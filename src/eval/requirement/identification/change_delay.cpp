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

#include "eval/requirement/identification/change_delay.h"
#include "eval/requirement/group.h"

#include "eval/results/identification/change_delay.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "comparisontype.h"
#include "stringconv.h"
#include "util/timeconv.h"

#include "logger.h"

#include <QLineEdit>
#include <QComboBox>
#include <QFormLayout>
#include <QDoubleValidator>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

/********************************************************************************************************
 * IdentificationChangeDelay
 ********************************************************************************************************/

/**
*/
IdentificationChangeDelay::IdentificationChangeDelay(const std::string& name,
                                                     const std::string& short_name,
                                                     const std::string& group_name,
                                                     double prob,
                                                     COMPARISON_TYPE prob_check_type,
                                                     EvaluationCalculator& calculator,
                                                     IdentificationType identification_type,
                                                     float max_delay_s,
                                                     float min_stable_time_s)
    : ProbabilityBase      (name, short_name, group_name, prob, prob_check_type, false, calculator, false)
    , identification_type_ (identification_type)
    , max_delay_s_         (max_delay_s)
    , min_stable_time_s_   (min_stable_time_s)
{
}

/**
*/
std::shared_ptr<EvaluationRequirementResult::Single> IdentificationChangeDelay::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " max_delay_s " << max_delay_s_ << " min_stable_time_s " << min_stable_time_s_;

    typedef EvaluationRequirementResult::SingleIdentificationChangeDelay Result;
    typedef EvaluationDetail                                             Detail;
    typedef Result::EvaluationDetails                                    Details;
    Details details;

    unsigned int num_events         {0};
    unsigned int num_not_assessable {0};
    unsigned int num_passed         {0};
    unsigned int num_failed         {0};

    std::vector<double> delays;

    const auto& ref_data = target_data.refChain().timestampIndexes();
    const auto& tst_data = target_data.tstChain().timestampIndexes();

    // value of a reference or test report as display/comparison string,
    // empty optional if not set
    auto getValue = [ & ] (const dbContent::TargetReport::Chain& chain,
                           const dbContent::TargetReport::Chain::DataID& id) -> boost::optional<string>
    {
        if (identification_type_ == IdentificationType::AircraftID)
        {
            auto acid = chain.acid(id);
            if (!acid.has_value())
                return {};

            // remove trailing spaces for comparison
            string value = acid.value();
            value.erase(value.find_last_not_of(' ') + 1);

            if (value.empty())
                return {};

            return value;
        }

        auto mode_a = chain.modeA(id);
        if (!mode_a.has_value())
            return {};

        return String::octStringFromInt(mode_a.value(), 4, '0');
    };

    auto addDetail = [ & ] (const ptime& ts,
                            const dbContent::TargetPosition& pos,
                            const QVariant& value,
                            const QVariant& check_passed,
                            const std::string& comment)
    {
        details.push_back(Detail(ts, pos).setValue(Result::DetailKey::Value, value)
                                         .setValue(Result::DetailKey::CheckPassed, check_passed)
                                         .setValue(Result::DetailKey::NumEvents, num_events)
                                         .setValue(Result::DetailKey::NumNotAssessable, num_not_assessable)
                                         .setValue(Result::DetailKey::NumCheckPassed, num_passed)
                                         .setValue(Result::DetailKey::NumCheckFailed, num_failed)
                                         .generalComment(comment));
    };

    // assess one detected change event: search the first test report at or
    // after the event time carrying the new value
    auto assessEvent = [ & ] (const ptime& event_ts,
                              const dbContent::TargetReport::Chain::DataID& event_id,
                              const string& new_value,
                              const string& old_value)
    {
        // only events inside the sector and not excluded
        if (!target_data.isTimeStampNotExcluded(event_ts)
            || !target_data.refPosInside(sector_layer, event_id))
            return;

        ++num_events;

        auto event_pos = target_data.refChain().pos(event_id);

        boost::optional<double> delay;

        for (auto it = tst_data.lower_bound(event_ts); it != tst_data.end(); ++it)
        {
            auto tst_value = getValue(target_data.tstChain(), *it);

            if (tst_value.has_value() && tst_value.value() == new_value)
            {
                delay = Time::partialSeconds(it->first - event_ts);
                break;
            }
        }

        string change_str = identificationName(identification_type_)
                            + " change '" + old_value + "' -> '" + new_value + "'";

        if (!delay.has_value())
        {
            // no test report with the new value: only a failure if enough
            // test data time remained after the event to expect one
            ptime last_tst_ts = tst_data.empty() ? ptime() : tst_data.rbegin()->first;

            if (tst_data.empty() || Time::partialSeconds(last_tst_ts - event_ts) < max_delay_s_)
            {
                ++num_not_assessable;
                addDetail(event_ts, event_pos, {}, {},
                          change_str + ": not assessable (insufficient test data after event)");
            }
            else
            {
                ++num_failed;
                addDetail(event_ts, event_pos, {}, false,
                          change_str + ": new value never reported");
            }

            return;
        }

        delays.push_back(delay.value());

        bool passed = delay.value() <= max_delay_s_;

        if (passed)
            ++num_passed;
        else
            ++num_failed;

        addDetail(event_ts, event_pos, delay.value(), passed,
                  change_str + (passed ? "" : ": delay not OK"));
    };

    // walk the reference identity values: a new value becomes stable when it
    // holds for min_stable_time_s (transitional Mode A codes are ignored,
    // ED-129C REC 495 NOTE 1); each stable value after the first is a change
    // event with the event time of the first report carrying it
    boost::optional<string> stable_value;

    boost::optional<string> candidate_value;
    ptime candidate_start_ts;
    dbContent::TargetReport::Chain::DataID candidate_start_id;

    for (const auto& ref_id : ref_data)
    {
        auto value = getValue(target_data.refChain(), ref_id);

        if (!value.has_value())
            continue;

        if (stable_value.has_value() && value.value() == stable_value.value())
        {
            // back at the stable value: candidate was transitional
            candidate_value.reset();
            continue;
        }

        if (candidate_value.has_value() && value.value() == candidate_value.value())
        {
            if (Time::partialSeconds(ref_id.first - candidate_start_ts) >= min_stable_time_s_)
            {
                // candidate held long enough: new stable value
                if (stable_value.has_value())
                    assessEvent(candidate_start_ts, candidate_start_id,
                                candidate_value.value(), stable_value.value());

                stable_value = candidate_value;
                candidate_value.reset();
            }

            continue;
        }

        // new candidate value
        candidate_value    = value;
        candidate_start_ts = ref_id.first;
        candidate_start_id = ref_id;
    }

    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " num_events " << num_events << " num_not_assessable " << num_not_assessable
           << " num_passed " << num_passed << " num_failed " << num_failed;

    traced_assert(num_events == num_not_assessable + num_passed + num_failed);

    return make_shared<EvaluationRequirementResult::SingleIdentificationChangeDelay>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_events, num_not_assessable, num_passed, num_failed, delays);
}

/**
*/
std::string IdentificationChangeDelay::probabilityNameShort(IdentificationType identification_type)
{
    switch(identification_type)
    {
        case IdentificationType::AircraftID:
            return "PAACD";
        case IdentificationType::ModeA:
            return "PAMCD";
    }
    return "";
}

/**
*/
std::string IdentificationChangeDelay::probabilityName(IdentificationType identification_type)
{
    return "Probability of Acceptable " + identificationName(identification_type) + " Change Delay";
}

/**
*/
std::string IdentificationChangeDelay::identificationName(IdentificationType identification_type)
{
    switch(identification_type)
    {
        case IdentificationType::AircraftID:
            return "Aircraft ID";
        case IdentificationType::ModeA:
            return "Mode3A";
    }
    return "";
}

/**
*/
std::string IdentificationChangeDelay::probabilityNameShort() const
{
    return IdentificationChangeDelay::probabilityNameShort(identification_type_);
}

/**
*/
std::string IdentificationChangeDelay::probabilityName() const
{
    return IdentificationChangeDelay::probabilityName(identification_type_);
}

/********************************************************************************************************
 * IdentificationChangeDelayConfig
 ********************************************************************************************************/

/**
*/
IdentificationChangeDelayConfig::IdentificationChangeDelayConfig(nlohmann::json& config,
                                                                 Group* parent)
:   ProbabilityBaseConfig(config, parent)
{
    registerParameter("identification_type", reinterpret_cast<int*>(&identification_type_), (int)IdentificationType::AircraftID);

    registerParameter("max_delay_s", &max_delay_s_, 15.0f);
    registerParameter("min_stable_time_s", &min_stable_time_s_, 10.0f);
}

/**
*/
float IdentificationChangeDelayConfig::maxDelay() const
{
    return max_delay_s_;
}

/**
*/
void IdentificationChangeDelayConfig::maxDelay(float value)
{
    max_delay_s_ = value;
}

/**
*/
float IdentificationChangeDelayConfig::minStableTime() const
{
    return min_stable_time_s_;
}

/**
*/
void IdentificationChangeDelayConfig::minStableTime(float value)
{
    min_stable_time_s_ = value;
}

/**
*/
std::shared_ptr<Base> IdentificationChangeDelayConfig::createRequirement()
{
    shared_ptr<IdentificationChangeDelay> req = make_shared<IdentificationChangeDelay>(
                name_,
                short_name_,
                group_.name(),
                prob_,
                prob_check_type_,
                calculator_,
                identification_type_,
                max_delay_s_,
                min_stable_time_s_);
    return req;
}

/**
*/
BaseConfigWidget* IdentificationChangeDelayConfig::createWidget()
{
    return new IdentificationChangeDelayConfigWidget(*this);
}

/**
*/
void IdentificationChangeDelayConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]",
                  IdentificationChangeDelay::probabilityName(identification_type_),
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Identification Type", "Compared identification",
                  IdentificationChangeDelay::identificationName(identification_type_)});

    table.addRow({"Maximum Delay [s]",
                  "Maximum acceptable delay from the final reference value change"
                  " to the first test report with the new value",
                  max_delay_s_});

    table.addRow({"Minimum Stable Time [s]",
                  "Minimum time a new reference value must hold to count as a"
                  " change event (transitional code filter)",
                  min_stable_time_s_});
}

/********************************************************************************************************
 * IdentificationChangeDelayConfigWidget
 ********************************************************************************************************/

/**
*/
IdentificationChangeDelayConfigWidget::IdentificationChangeDelayConfigWidget(IdentificationChangeDelayConfig& cfg)
:   ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip(QString::fromStdString(
        IdentificationChangeDelay::probabilityName(config().identificationType())));

    traced_assert(check_type_box_);

    identification_type_combo_ = new QComboBox;
    identification_type_combo_->addItem("Aircraft ID", QVariant((int)IdentificationChangeDelayConfig::IdentificationType::AircraftID));
    identification_type_combo_->addItem("Mode A", QVariant((int)IdentificationChangeDelayConfig::IdentificationType::ModeA));
    identification_type_combo_->setCurrentIndex((int)config().identificationType());

    connect(identification_type_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &IdentificationChangeDelayConfigWidget::identificationTypeChangedSlot);

    form_layout_->addRow("Identification Type", identification_type_combo_);

    max_delay_edit_ = new QLineEdit(QString::number(config().maxDelay()));
    max_delay_edit_->setValidator(new QDoubleValidator(0.1, 300.0, 2, this));
    max_delay_edit_->setToolTip("Maximum acceptable delay from the final reference value change"
                                " to the first test report with the new value");
    connect(max_delay_edit_, &QLineEdit::textEdited,
            this, &IdentificationChangeDelayConfigWidget::maxDelayEditSlot);

    form_layout_->addRow("Maximum Delay [s]", max_delay_edit_);

    min_stable_time_edit_ = new QLineEdit(QString::number(config().minStableTime()));
    min_stable_time_edit_->setValidator(new QDoubleValidator(0.0, 300.0, 2, this));
    min_stable_time_edit_->setToolTip("Minimum time a new reference value must hold to count as a"
                                      " change event (transitional code filter)");
    connect(min_stable_time_edit_, &QLineEdit::textEdited,
            this, &IdentificationChangeDelayConfigWidget::minStableTimeEditSlot);

    form_layout_->addRow("Minimum Stable Time [s]", min_stable_time_edit_);
}

/**
*/
IdentificationChangeDelayConfig& IdentificationChangeDelayConfigWidget::config()
{
    IdentificationChangeDelayConfig* config = dynamic_cast<IdentificationChangeDelayConfig*>(&config_);
    traced_assert(config);

    return *config;
}

/**
*/
void IdentificationChangeDelayConfigWidget::identificationTypeChangedSlot()
{
    auto data = identification_type_combo_->currentData();
    traced_assert(!data.isNull());

    auto id_type = (IdentificationChangeDelayConfig::IdentificationType)data.toInt();

    loginf << "value " << IdentificationChangeDelay::identificationName(id_type);

    config().identificationType(id_type);

    prob_edit_->setToolTip(QString::fromStdString(
        IdentificationChangeDelay::probabilityName(config().identificationType())));
}

/**
*/
void IdentificationChangeDelayConfigWidget::maxDelayEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxDelay(val);
    else
        loginf << "invalid value";
}

/**
*/
void IdentificationChangeDelayConfigWidget::minStableTimeEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().minStableTime(val);
    else
        loginf << "invalid value";
}

} // namespace EvaluationRequirement
