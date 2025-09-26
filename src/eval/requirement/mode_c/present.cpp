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

#include "eval/requirement/mode_c/present.h"
#include "eval/standard/evaluationstandard.h"
#include "eval/results/mode_c/present.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "logger.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

ModeCPresent::ModeCPresent(const std::string& name, const std::string& short_name, const std::string& group_name,
                           double prob, COMPARISON_TYPE prob_check_type, EvaluationCalculator& calculator)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator)
{
}

std::shared_ptr<EvaluationRequirementResult::Single> ModeCPresent::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_;

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    ptime timestamp;

    ptime ref_lower, ref_upper;

    int num_updates {0};
    int num_no_ref_pos {0};
    //int num_no_ref_val {0};
    int num_pos_outside {0};
    int num_pos_inside {0};

    int num_no_ref_id {0};
    int num_present_id {0};
    int num_missing_id {0};

    typedef EvaluationRequirementResult::SingleModeCPresent Result;
    typedef EvaluationDetail                                Detail;
    typedef Result::EvaluationDetails                       Details;
    Details details;

    dbContent::TargetPosition pos_current;
    //unsigned int code;
    //bool code_ok;

    bool code_present_ref;
    bool code_present_tst;
    bool code_missing;

    //bool ref_exists;
    bool is_inside;
    //pair<dbContent::TargetPosition, bool> ret_pos;
    boost::optional<dbContent::TargetPosition> ref_pos;
    //bool ok;

    string comment;
    //bool lower_nok, upper_nok;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;
    bool skip_detail;

    auto addDetail = [ & ] (const ptime& ts,
            const dbContent::TargetPosition& tst_pos,
            const boost::optional<dbContent::TargetPosition>& ref_pos,
            const QVariant& ref_exists,
            const QVariant& pos_inside,
            const QVariant& is_not_ok,
            const QVariant& num_updates,
            const QVariant& num_no_ref,
            const QVariant& num_pos_inside,
            const QVariant& num_pos_outside,
            const QVariant& num_no_ref_id,
            const QVariant& num_present_id,
            const QVariant& num_missing_id,
            const std::string& comment)
    {
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::RefExists, ref_exists)
                          .setValue(Result::DetailKey::PosInside, pos_inside.isValid() ? pos_inside : "false")
                          .setValue(Result::DetailKey::IsNotOk, is_not_ok)
                          .setValue(Result::DetailKey::NumUpdates, num_updates)
                          .setValue(Result::DetailKey::NumNoRef, num_no_ref)
                          .setValue(Result::DetailKey::NumInside, num_pos_inside)
                          .setValue(Result::DetailKey::NumOutside, num_pos_outside)
                          .setValue(Result::DetailKey::NumNoRefVal, num_no_ref_id)
                          .setValue(Result::DetailKey::NumPresent, num_present_id)
                          .setValue(Result::DetailKey::NumMissing, num_missing_id)
                          .addPosition(ref_pos)
                          .generalComment(comment));
    };

    for (const auto& tst_id : tst_data)
    {
        //ref_exists = false;
        is_inside = false;
        comment = "";

        skip_detail = false;

        ++num_updates;

        timestamp = tst_id.first;
        pos_current = target_data.tstChain().pos(tst_id);

        if (!target_data.hasMappedRefData(tst_id, max_ref_time_diff))
        {
            if (!skip_no_data_details)
                addDetail(timestamp, pos_current, {},
                          false, {}, false, // ref_exists, pos_inside, is_not_ok
                          num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                          num_no_ref_id, num_present_id, num_missing_id, "No reference data");

            ++num_no_ref_pos;
            continue;
        }

        ref_pos = target_data.mappedRefPos(tst_id, max_ref_time_diff);

//        ref_pos = ret_pos.first;
//        ok = ret_pos.second;

        if (!ref_pos.has_value())
        {
            if (!skip_no_data_details)
                addDetail(timestamp, pos_current, {},
                          false, {}, false, // ref_exists, pos_inside, is_not_ok
                          num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                          num_no_ref_id, num_present_id, num_missing_id, "No reference position");

            ++num_no_ref_pos;
            continue;
        }

        is_inside = target_data.isTimeStampNotExcluded(timestamp)
                    && target_data.mappedRefPosInside(sector_layer, tst_id);

        if (!is_inside)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, pos_current, ref_pos,
                          true, is_inside, false, // ref_exists, pos_inside, is_not_ok
                          num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                          num_no_ref_id, num_present_id, num_missing_id, "Outside sector");

            ++num_pos_outside;
            continue;
        }

        ++num_pos_inside;

        // check if ref code exists
        code_present_ref = false;

        tie(ref_lower, ref_upper) = target_data.mappedRefTimes(tst_id, max_ref_time_diff);

        if ((!ref_lower.is_not_a_date_time() || !ref_upper.is_not_a_date_time())) // ref times possible
        {
            if ((!ref_lower.is_not_a_date_time()
                 && target_data.refChain().modeC(ref_lower, false, false).has_value())
                    || (!ref_upper.is_not_a_date_time()
                        && target_data.refChain().modeC(ref_upper, false, false).has_value()))
                // ref value(s) exist
            {
                code_present_ref = true;
            }
        }

        code_present_tst = target_data.tstChain().modeC(tst_id, false, false).has_value();

        code_missing = false;

        if (code_present_ref)
        {
            if (code_present_tst)
            {
                comment = "OK";
                ++num_present_id;
            }
            else
            {
                comment = "Id missing";
                ++num_missing_id;

                code_missing = true;
            }
        }
        else
        {
            comment = "No reference id";
            ++num_no_ref_id;

            // skip_detail?
        }

        if (!(skip_no_data_details && skip_detail))
            addDetail(timestamp, pos_current, ref_pos,
                      true, is_inside, code_missing, // ref_exists, pos_inside, is_not_ok
                      num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                      num_no_ref_id, num_present_id, num_missing_id, comment);
    }

    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " num_updates " << num_updates << " num_no_ref_pos " << num_no_ref_pos
              //<< " num_no_ref_val " << num_no_ref_val
           << " num_pos_outside " << num_pos_outside << " num_pos_inside " << num_pos_inside
           << " num_no_ref_id " << num_no_ref_id << " num_present_id " << num_present_id
           << " num_missing_id " << num_missing_id;

    traced_assert(num_updates - num_no_ref_pos == num_pos_inside + num_pos_outside);
    traced_assert(num_pos_inside == num_no_ref_id+num_present_id+num_missing_id);

    //assert (details.size() == tst_data.size());

    return make_shared<EvaluationRequirementResult::SingleModeCPresent>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_updates, num_no_ref_pos, num_pos_outside, num_pos_inside,
                num_no_ref_id, num_present_id, num_missing_id);
}
}
