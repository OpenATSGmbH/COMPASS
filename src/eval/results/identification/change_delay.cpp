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

#include "results/identification/change_delay.h"
#include "results/evaluationdetail.h"
#include "results/base/featuredefinitions.h"

#include "requirement/base/base.h"
#include "requirement/identification/change_delay.h"
#include "requirement/identification/change_delay_helpers.h"

#include "evaluationtargetdata.h"
#include "evaluationmanager.h"

#include "logger.h"
#include "stringconv.h"

#include "traced_assert.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

/**********************************************************************************************
 * IdentificationChangeDelayBase
 **********************************************************************************************/

/**
*/
IdentificationChangeDelayBase::IdentificationChangeDelayBase(unsigned int num_events,
                                                             unsigned int num_not_assessable,
                                                             unsigned int num_passed,
                                                             unsigned int num_failed)
:   num_events_         (num_events)
,   num_not_assessable_ (num_not_assessable)
,   num_passed_         (num_passed)
,   num_failed_         (num_failed)
{
}

/**********************************************************************************************
 * SingleIdentificationChangeDelay
 **********************************************************************************************/

/**
*/
SingleIdentificationChangeDelay::SingleIdentificationChangeDelay(
        const std::string& result_id,
        std::shared_ptr<EvaluationRequirement::Base> requirement,
        const SectorLayer& sector_layer,
        unsigned int utn,
        const EvaluationTargetData* target,
        EvaluationCalculator& calculator,
        const EvaluationDetails& details,
        unsigned int num_events,
        unsigned int num_not_assessable,
        unsigned int num_passed,
        unsigned int num_failed,
        const std::vector<double>& delays)
:   IdentificationChangeDelayBase(num_events, num_not_assessable, num_passed, num_failed)
,   SingleProbabilityBase("SingleIdentificationChangeDelay", result_id, requirement, sector_layer, utn, target, calculator, details)
{
    accumulator_.accumulate(delays, true);

    updateResult();
}

/**
*/
std::shared_ptr<Joined> SingleIdentificationChangeDelay::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedIdentificationChangeDelay>(result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
boost::optional<double> SingleIdentificationChangeDelay::computeResult_impl() const
{
    unsigned int num_checked = num_passed_ + num_failed_;

    if (!num_checked)
        return {};

    return (double)num_passed_ / (double)num_checked;
}

/**
*/
unsigned int SingleIdentificationChangeDelay::numIssues() const
{
    return num_failed_;
}

/**
*/
std::vector<std::string> SingleIdentificationChangeDelay::targetTableHeadersCustom() const
{
    return { "#Events", "CDMin", "CDMax", "CDAvg", "#CDOK", "#CDNOK" };
}

/**
*/
nlohmann::json::array_t SingleIdentificationChangeDelay::targetTableValuesCustom() const
{
    return { num_events_,
             (accumulator_.numValues() ? json(formatValue(accumulator_.min())) : json()),
             (accumulator_.numValues() ? json(formatValue(accumulator_.max())) : json()),
             (accumulator_.numValues() ? json(formatValue(accumulator_.mean())) : json()),
             num_passed_,
             num_failed_ };
}

/**
*/
std::vector<Single::TargetInfo> SingleIdentificationChangeDelay::targetInfos() const
{
    return { { "#Events [1]"  , "Number of identity change events"           , num_events_         },
             { "#NotAssessable [1]", "Number of events without sufficient data", num_not_assessable_ },
             { "CDMin [s]"    , "Minimum of change delay"                    , (accumulator_.numValues() ? json(formatValue(accumulator_.min())) : json())    },
             { "CDMax [s]"    , "Maximum of change delay"                    , (accumulator_.numValues() ? json(formatValue(accumulator_.max())) : json())    },
             { "CDAvg [s]"    , "Average of change delay"                    , (accumulator_.numValues() ? json(formatValue(accumulator_.mean())) : json())   },
             { "CDSDev [s]"   , "Standard Deviation of change delay"         , (accumulator_.numValues() ? json(formatValue(accumulator_.stddev())) : json()) },
             { "#CDOK [1]"    , "Number of events with acceptable delay"     , num_passed_         },
             { "#CDNOK [1]"   , "Number of events with unacceptable delay"   , num_failed_         } };
}

/**
*/
bool SingleIdentificationChangeDelay::detailIsOk(const EvaluationDetail& detail) const
{
    // no comparison (outside, no time data, not assessable): not an issue
    auto check_passed = detail.getValue(DetailKey::CheckPassed);

    return !check_passed.isValid() || check_passed.toBool();
}

/**
*/
void SingleIdentificationChangeDelay::addAnnotationForDetail(nlohmann::json& annotations_json,
                                                             const EvaluationDetail& detail,
                                                             TargetAnnotationType type,
                                                             bool is_ok) const
{
    traced_assert(detail.numPositions() >= 1);

    if (type == TargetAnnotationType::Highlight)
    {
        addAnnotationPos(annotations_json, detail.firstPos(), AnnotationArrayType::TypeHighlight);
    }
    else if (type == TargetAnnotationType::TargetOverview)
    {
        addAnnotationPos(annotations_json, detail.firstPos(),
                         is_ok ? AnnotationArrayType::TypeOk : AnnotationArrayType::TypeError);
    }
}

/**********************************************************************************************
 * JoinedIdentificationChangeDelay
 **********************************************************************************************/

/**
*/
JoinedIdentificationChangeDelay::JoinedIdentificationChangeDelay(
        const std::string& result_id,
        std::shared_ptr<EvaluationRequirement::Base> requirement,
        const SectorLayer& sector_layer,
        EvaluationCalculator& calculator)
:   IdentificationChangeDelayBase()
,   JoinedProbabilityBase("JoinedIdentificationChangeDelay", result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedIdentificationChangeDelay::numIssues() const
{
    return num_failed_;
}

/**
*/
unsigned int JoinedIdentificationChangeDelay::numUpdates() const
{
    return num_passed_ + num_failed_;
}

/**
*/
void JoinedIdentificationChangeDelay::clearResults_impl()
{
    num_events_         = 0;
    num_not_assessable_ = 0;
    num_passed_         = 0;
    num_failed_         = 0;

    accumulator_.reset();
}

/**
*/
void JoinedIdentificationChangeDelay::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleIdentificationChangeDelay> single =
            std::static_pointer_cast<SingleIdentificationChangeDelay>(single_result);

    assert (single->resultUsable());

    num_events_         += single->numEvents();
    num_not_assessable_ += single->numNotAssessable();
    num_passed_         += single->numPassed();
    num_failed_         += single->numFailed();

    accumulator_.join(single->accumulator(), last);
}

/**
*/
boost::optional<double> JoinedIdentificationChangeDelay::computeResult_impl() const
{
    loginf << "start"
           << " num_passed " << num_passed_
           << " num_failed " << num_failed_;

    unsigned int num_checked = num_passed_ + num_failed_;

    if (!num_checked)
        return {};

    return (double)num_passed_ / (double)num_checked;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedIdentificationChangeDelay::sectorInfos() const
{
    return { { "#Events [1]"  , "Number of identity change events"           , num_events_         },
             { "#NotAssessable [1]", "Number of events without sufficient data", num_not_assessable_ },
             { "CDMin [s]"    , "Minimum of change delay"                    , (accumulator_.numValues() ? json(formatValue(accumulator_.min())) : json())    },
             { "CDMax [s]"    , "Maximum of change delay"                    , (accumulator_.numValues() ? json(formatValue(accumulator_.max())) : json())    },
             { "CDAvg [s]"    , "Average of change delay"                    , (accumulator_.numValues() ? json(formatValue(accumulator_.mean())) : json())   },
             { "CDSDev [s]"   , "Standard Deviation of change delay"         , (accumulator_.numValues() ? json(formatValue(accumulator_.stddev())) : json()) },
             { "#CDOK [1]"    , "Number of events with acceptable delay"     , num_passed_         },
             { "#CDNOK [1]"   , "Number of events with unacceptable delay"   , num_failed_         } };
}

/**
 * Overview: where the change delay passed or failed (at the change events), the counts and the distribution
 * of the delays.
 */
FeatureDefinitions JoinedIdentificationChangeDelay::getCustomAnnotationDefinitions() const
{
    FeatureDefinitions defs;

    defs.addDefinition<FeatureDefinitionIssueCountGrid>(requirement()->name(), calculator_, "Issues")
        .addDataSeries(SingleIdentificationChangeDelay::DetailKey::CheckPassed,
                       GridAddDetailMode::AddEvtPosition,
                       false);

    defs.addDefinition<FeatureDefinitionStringCategoryHistogram>(requirement()->name(), calculator_, "Change Delay Count", "Change Delay Count")
        .addDataSeries("", { "#CDNOK", "#CDOK" }, { num_failed_, num_passed_ });
    defs.addDefinition<FeatureDefinitionHistogram<double>>(requirement()->name(), calculator_, "Full distribution", "Full distribution [s]")
        .addDataSeries("", SingleIdentificationChangeDelay::DetailKey::Value);

    return defs;
}

/**
 * From the change event (the detail time, a reference report) to the first test report with the new
 * value. Without a reported value (never reported, not assessable) the bounds only hold the reference
 * report of the change.
 */
boost::optional<std::pair<boost::posix_time::ptime, boost::posix_time::ptime>>
SingleIdentificationChangeDelay::reportTableGapBounds(const EvaluationDetail& detail) const
{
    const auto begin = detail.timestamp();
    const auto delay = detail.getValueAs<double>(DetailKey::Value);

    // the reference report of the change lies in [begin, end), so the end lies after it
    auto end = begin + boost::posix_time::microseconds(1);
    if (delay.has_value() && delay.value() > 0.0)
        end = begin + boost::posix_time::microseconds(static_cast<long long>(delay.value() * 1e6));

    return std::make_pair(begin, end);
}

/**
 */
void SingleIdentificationChangeDelay::addReportTableColumns(ReportTableDefinition& def) const
{
    def.addColumn("delay_s", PropertyDataType::DOUBLE, "Change Delay",
                  "Delay of the identification change against the reference", "Time", "Second");
    def.addColumn("check_passed", PropertyDataType::BOOL, "Check Passed",
                  "Value within the requirement threshold");

    DetailCode::addTypeText(def, EvaluationRequirement::ChangeDelayHelpers::NotAssessable,
                            "Not assessable (insufficient test data after the change)");
    DetailCode::addTypeText(def, EvaluationRequirement::ChangeDelayHelpers::NeverReported,
                            "New value never reported");
}

/**
 */
void SingleIdentificationChangeDelay::fillReportTableRow(ReportTableRows& rows,
                                  const EvaluationDetail& detail,
                                  const EvaluationDetail* parent_detail,
                                  const EvaluationDetail* prev_detail) const
{
    setReportTableValue<double>(rows, "delay_s", detail, DetailKey::Value);
    setReportTableValue<bool>(rows, "check_passed", detail, DetailKey::CheckPassed);
}

/**
 */
void SingleIdentificationChangeDelay::fillDetailFromReportTableRow(EvaluationDetail& detail,
                                  const Buffer& buffer,
                                  unsigned int row,
                                  const EvaluationDetail* prev_detail) const
{
    setDetailValue<double>(detail, DetailKey::Value, buffer, "delay_s", row);
    setDetailValue<bool>(detail, DetailKey::CheckPassed, buffer, "check_passed", row);
}

}
