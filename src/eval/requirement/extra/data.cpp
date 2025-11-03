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

#include "eval/requirement/extra/data.h"
#include "eval/standard/evaluationstandard.h"
#include "eval/results/extra/data.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "logger.h"
#include "timeperiod.h"

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

/**
*/
ExtraData::ExtraData(const std::string& name, 
                     const std::string& short_name, 
                     const std::string& group_name,
                     double prob, 
                     COMPARISON_TYPE prob_check_type, 
                     EvaluationCalculator& calculator,
                     float min_duration, 
                     unsigned int min_num_updates, 
                     bool ignore_primary_only)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator),
      min_duration_(Time::partialSeconds(min_duration)),
      min_num_updates_(min_num_updates), 
      ignore_primary_only_(ignore_primary_only)
{
}

/**
*/
float ExtraData::minDuration() const
{
    return Time::partialSeconds(min_duration_);
}

/**
*/
unsigned int ExtraData::minNumUpdates() const
{
    return min_num_updates_;
}

/**
*/
bool ExtraData::ignorePrimaryOnly() const
{
    return ignore_primary_only_;
}

/**
*/
std::shared_ptr<EvaluationRequirementResult::Single> ExtraData::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " min_duration " << min_duration_ << " min_num_updates " << min_num_updates_
           << " ignore_primary_only " << ignore_primary_only_ << " prob " << threshold();

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    // create ref time periods, irrespective of inside
    TimePeriodCollection ref_periods;
    ref_periods.createFromReference(target_data, sector_layer, max_ref_time_diff);
 
    ptime timestamp;
    bool is_inside;

    ref_periods.removeSmallPeriods(seconds(1));

    bool is_inside_ref_time_period {false};
    bool has_tod{false};
    ptime tod_min, tod_max;
    unsigned int num_ok = 0;
    unsigned int num_extra = 0;
    dbContent::TargetPosition tst_pos;

    typedef EvaluationRequirementResult::SingleExtraData Result;
    typedef EvaluationDetail                             Detail;
    typedef Result::EvaluationDetails                    Details;
    Details details;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    auto addDetail = [ & ] (const ptime& ts,
                            const dbContent::TargetPosition& tst_pos,
                            const QVariant& inside,
                            const QVariant& extra,
                            const QVariant& ref_exists,
                            const std::string& comment)
    {
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::Inside, inside)
                                             .setValue(Result::DetailKey::Extra, extra)
                                             .setValue(Result::DetailKey::RefExists, ref_exists)
                                             .generalComment(comment));
    };

    {
        const auto& tst_data = target_data.tstChain().timestampIndexes();

        for (auto& tst_it : tst_data)
        {
            timestamp = tst_it.first;

            tst_pos = target_data.tstChain().pos(tst_it);

            is_inside_ref_time_period = ref_periods.isInside(timestamp);

            if (is_inside_ref_time_period)
            {
                ++num_ok;
                addDetail(timestamp, tst_pos, true, false, true, "OK"); // inside, extra, ref
                continue;
            }

            // no ref
            is_inside = target_data.isTimeStampNotExcluded(timestamp)
                     && target_data.tstPosInside(sector_layer, tst_it);

            if (is_inside)
            {
                ++num_extra;
                addDetail(timestamp, tst_pos, true, true, false, "Extra"); // inside, extra, ref

                if (!has_tod)
                {
                    tod_min = timestamp;
                    tod_max = timestamp;
                    has_tod = true;
                }
                else
                {
                    tod_min = min (timestamp, tod_min);
                    tod_max = max (timestamp, tod_max);
                }
            }
            else if (!skip_no_data_details)
            {
                addDetail(timestamp, tst_pos, false, false, false, "Tst outside"); // inside, extra, ref
            }
        }
    }

    bool ignore = false;
    string ignore_comment;

    if (num_extra && num_extra < min_num_updates_)
    {
        ignore = true;
        ignore_comment = "Minimum number of updates: "+to_string(num_extra)+" < "+to_string(min_num_updates_);
    }

    if (!ignore && has_tod && (tod_max-tod_min) < min_duration_)
    {
        ignore = true;
        ignore_comment = "Minimum duration: "+Time::toString(tod_max-tod_min,1)+" < "+Time::toString(min_duration_,1);
    }

    if (!ignore && ignore_primary_only_ && target_data.isPrimaryOnly())
    {
        ignore = true;
        ignore_comment = "Ignore primary-only";
    }

    bool has_extra_test_data = num_extra;

//    if (!ignore && test_data_only)
//        loginf << "'" << name_ << "': utn " << target_data.utn_
//               << " not ignored tdo, ref " << num_ref_inside << " num_ok " << num_ok << " num_extra " << num_extra;

    auto ret = make_shared<EvaluationRequirementResult::SingleExtraData>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_extra, num_ok, has_extra_test_data);
    
    if (ignore)                
        ret->setIgnoreResult(ignore_comment);

    return ret;
}

}
