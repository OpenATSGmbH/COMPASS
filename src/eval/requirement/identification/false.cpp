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

#include "eval/requirement/identification/false.h"
#include "eval/standard/evaluationstandard.h"
#include "eval/results/identification/false.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "logger.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

IdentificationFalse::IdentificationFalse(
        const std::string& name, const std::string& short_name, const std::string& group_name,
        double prob, COMPARISON_TYPE prob_check_type, EvaluationCalculator& calculator,
        bool require_all_false, bool use_mode_a, bool use_ms_ta, bool use_ms_ti)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator),
      require_all_false_(require_all_false),
      use_mode_a_(use_mode_a), use_ms_ta_(use_ms_ta), use_ms_ti_(use_ms_ti)
{
}

std::shared_ptr<EvaluationRequirementResult::Single> IdentificationFalse::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_;

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    ptime timestamp;

    int num_updates {0};
    int num_no_ref_pos {0};
    int num_no_ref_val {0};
    int num_pos_outside {0};
    int num_pos_inside {0};
    int num_unknown {0};
    int num_correct {0};
    int num_false {0};

    typedef EvaluationRequirementResult::SingleIdentificationFalse Result;
    typedef EvaluationDetail                                       Detail;
    typedef Result::EvaluationDetails                              Details;
    Details details;

    dbContent::TargetPosition pos_current;

    bool ref_exists;
    bool is_inside;
    //pair<dbContent::TargetPosition, bool> ret_pos;
    boost::optional<dbContent::TargetPosition> ref_pos;
    //bool ok;

    ValueComparisonResult cmp_res_ti;
    string cmp_res_ti_comment;
    bool ti_false;

    ValueComparisonResult cmp_res_ta;
    string cmp_res_ta_comment;
    bool ta_false;

    ValueComparisonResult cmp_res_ma;
    string cmp_res_ma_comment;
    bool ma_false;

    bool any_false;
    bool all_false;
    bool result_false;

    string comment;

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
                            const QVariant& num_unknown_id,
                            const QVariant& num_correct_id,
                            const QVariant& num_false_id,
                            const std::string& comment)
    {
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::RefExists, ref_exists)
                                             .setValue(Result::DetailKey::PosInside, pos_inside.isValid() ? pos_inside : "false")
                                             .setValue(Result::DetailKey::IsNotOk, is_not_ok)
                                             .setValue(Result::DetailKey::NumUpdates, num_updates)
                                             .setValue(Result::DetailKey::NumNoRef, num_no_ref)
                                             .setValue(Result::DetailKey::NumInside, num_pos_inside)
                                             .setValue(Result::DetailKey::NumOutside, num_pos_outside)
                                             .setValue(Result::DetailKey::NumUnknownID, num_unknown_id)
                                             .setValue(Result::DetailKey::NumCorrectID, num_correct_id)
                                             .setValue(Result::DetailKey::NumFalseID, num_false_id)
                                             .addPosition(ref_pos)
                                             .generalComment(comment));
    };

    for (const auto& tst_id : tst_data)
    {
        ref_exists = false;
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
                            false, {}, false, // ref_exists, pos_inside,
                            num_updates, num_no_ref_pos+num_no_ref_val, num_pos_inside, num_pos_outside,
                            num_unknown, num_correct, num_false, "No reference data");

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
                            false, {}, false, // ref_exists, pos_inside,
                            num_updates, num_no_ref_pos+num_no_ref_val, num_pos_inside, num_pos_outside,
                            num_unknown, num_correct, num_false, "No reference position");

            ++num_no_ref_pos;
            continue;
        }
        ref_exists = true;

        is_inside = target_data.isTimeStampNotExcluded(timestamp)
                    && target_data.mappedRefPosInside(sector_layer, tst_id);

        if (!is_inside)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, pos_current, ref_pos,
                            ref_exists, is_inside, false, // ref_exists, pos_inside,
                            num_updates, num_no_ref_pos+num_no_ref_val, num_pos_inside, num_pos_outside,
                            num_unknown, num_correct, num_false, "Outside sector");

            ++num_pos_outside;
            continue;
        }
        ++num_pos_inside;

        tie(cmp_res_ti, cmp_res_ti_comment) = compareTi(tst_id, target_data, max_ref_time_diff);
        tie(cmp_res_ta, cmp_res_ta_comment) = compareTa(tst_id, target_data, max_ref_time_diff);
        tie(cmp_res_ma, cmp_res_ma_comment) = compareModeA(tst_id, target_data, max_ref_time_diff);

        any_false = false;
        all_false = true;

        if (use_ms_ti_)
        {
            ti_false = cmp_res_ti == ValueComparisonResult::Different;

            if (ti_false)
                comment += "ACID false (" + cmp_res_ti_comment + ")";

            any_false |= ti_false;
            all_false &= ti_false;
        }

        if (use_ms_ta_)
        {
            ta_false = cmp_res_ta == ValueComparisonResult::Different;

            if (ta_false)
            {
                if (comment.size())
                    comment += ", ";

                comment += "ACAD false (" + cmp_res_ta_comment + ")";
            }

            any_false |= ta_false;
            all_false &= ta_false;
        }

        if (use_mode_a_)
        {
            ma_false = cmp_res_ma == ValueComparisonResult::Different;

            if (ma_false)
            {
                if (comment.size())
                    comment += ", ";
                comment += "M3A false (" + cmp_res_ma_comment + ")";
            }

            any_false |= ma_false;
            all_false &= ma_false;
        }

        if (!use_ms_ti_ && !use_ms_ta_ && !use_mode_a_)
            all_false = false; // if none is used, set to false

        if (require_all_false_)
        {
            result_false = all_false;
        }
        else // one correct ok
        {
            result_false = any_false;
        }

        if (result_false)
            ++num_false;
        else
            ++num_correct;

        if (!skip_detail)
            addDetail(timestamp, pos_current, ref_pos,
                        ref_exists, is_inside, result_false,
                        num_updates, num_no_ref_pos+num_no_ref_val, num_pos_inside, num_pos_outside,
                        num_unknown, num_correct, num_false, comment);
    }

    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " num_updates " << num_updates << " num_no_ref_pos " << num_no_ref_pos
           << " num_no_ref_val " << num_no_ref_val
           << " num_pos_outside " << num_pos_outside << " num_pos_inside " << num_pos_inside
           << " num_unknown " << num_unknown << " num_correct " << num_correct
           << " num_false " << num_false;

    traced_assert(num_updates - num_no_ref_pos == num_pos_inside + num_pos_outside);
    traced_assert(num_pos_inside == num_no_ref_val+num_unknown+num_correct+num_false);

    //assert (details.size() == tst_data.size());

    return make_shared<EvaluationRequirementResult::SingleIdentificationFalse>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_updates, num_no_ref_pos, num_no_ref_val, num_pos_outside, num_pos_inside,
                num_unknown, num_correct, num_false);
}

bool IdentificationFalse::requireAllFalse() const
{
    return require_all_false_;
}

bool IdentificationFalse::useModeA() const
{
    return use_mode_a_;
}

bool IdentificationFalse::useMsTa() const
{
    return use_ms_ta_;
}

bool IdentificationFalse::useMsTi() const
{
    return use_ms_ti_;
}

}
