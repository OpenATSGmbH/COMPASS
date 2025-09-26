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

#include "eval/requirement/identification/correct.h"
#include "eval/results/identification/correct.h"
#include "eval/standard/evaluationstandard.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "logger.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

IdentificationCorrect::IdentificationCorrect(
        const std::string& name, const std::string& short_name, const std::string& group_name,
        double prob, COMPARISON_TYPE prob_check_type, EvaluationCalculator& calculator,
        bool require_correctness_of_all, bool use_mode_a, bool use_ms_ta, bool use_ms_ti)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator),
      require_correctness_of_all_(require_correctness_of_all),
      use_mode_a_(use_mode_a), use_ms_ta_(use_ms_ta), use_ms_ti_(use_ms_ti)
{

}

std::shared_ptr<EvaluationRequirementResult::Single> IdentificationCorrect::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_;

    typedef EvaluationRequirementResult::SingleIdentificationCorrect Result;
    typedef EvaluationDetail                                         Detail;
    typedef Result::EvaluationDetails                                Details;

    if (target_data.isPrimaryOnly())
    {
        logdbg << "'" << name_ << "': utn " << target_data.utn_
               << " ignored since primary only";

        return make_shared<EvaluationRequirementResult::SingleIdentificationCorrect>(
                    "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                    calculator_, Details(), 0, 0, 0, 0, 0, 0, 0);
    }

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    ptime timestamp;

    unsigned int num_updates {0};
    unsigned int num_no_ref_pos {0};
    unsigned int num_no_ref_id {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside {0};
    unsigned int num_correct {0};
    unsigned int num_not_correct {0};

    Details details;

    dbContent::TargetPosition pos_current;

    bool ref_exists;
    bool is_inside;
    //pair<dbContent::TargetPosition, bool> ret_pos;
    boost::optional<dbContent::TargetPosition> ref_pos;
    //bool ok;

    string comment;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;
    bool skip_detail;

    ValueComparisonResult cmp_res_ti;
    string cmp_res_ti_comment;
    bool ti_no_ref;
    bool ti_correct_failed;

    ValueComparisonResult cmp_res_ta;
    string cmp_res_ta_comment;
    bool ta_no_ref;
    bool ta_correct_failed;

    ValueComparisonResult cmp_res_ma;
    string cmp_res_ma_comment;
    bool ma_no_ref;
    bool ma_correct_failed;

    bool all_no_ref;
    bool any_correct;
    bool all_correct;
    bool result_ok;

    auto addDetail = [ & ] (const ptime& ts,
                            const dbContent::TargetPosition& tst_pos,
                            const boost::optional<dbContent::TargetPosition>& ref_pos,
                            const QVariant& ref_exists,
                            const QVariant& pos_inside,
                            const QVariant& is_not_correct,
                            const QVariant& num_updates,
                            const QVariant& num_no_ref,
                            const QVariant& num_pos_inside,
                            const QVariant& num_pos_outside,
                            const QVariant& num_correct,
                            const QVariant& num_not_correct,
                            const std::string& comment)
    {
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::RefExists, ref_exists)
                                             .setValue(Result::DetailKey::PosInside, pos_inside.isValid() ? pos_inside : "false")
                                             .setValue(Result::DetailKey::IsNotCorrect, is_not_correct)
                                             .setValue(Result::DetailKey::NumUpdates, num_updates)
                                             .setValue(Result::DetailKey::NumNoRef, num_no_ref)
                                             .setValue(Result::DetailKey::NumInside, num_pos_inside)
                                             .setValue(Result::DetailKey::NumOutside, num_pos_outside)
                                             .setValue(Result::DetailKey::NumCorrect, num_correct)
                                             .setValue(Result::DetailKey::NumNotCorrect, num_not_correct)
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
                            false, {}, false, // ref_exists, pos_inside, is_not_correct
                            num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                            num_correct, num_not_correct, "No reference data");

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
                            false, {}, false, // ref_exists, pos_inside, is_not_correct
                            num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                            num_correct, num_not_correct, "No reference position");

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
                            ref_exists, is_inside, false, // ref_exists, pos_inside, is_not_correct
                            num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                            num_correct, num_not_correct, "Outside sector");

            ++num_pos_outside;
            continue;
        }
        ++num_pos_inside;

        any_correct = false;
        all_correct = true;
        all_no_ref = true;

        if (use_ms_ti_)
        {
            tie(cmp_res_ti, cmp_res_ti_comment) = compareTi(tst_id, target_data, max_ref_time_diff); //aircraft id

            ti_no_ref = cmp_res_ti == ValueComparisonResult::Unknown_NoRefData;
            all_no_ref &= ti_no_ref;

            ti_correct_failed = cmp_res_ti == ValueComparisonResult::Unknown_NoTstData
                    || cmp_res_ti == ValueComparisonResult::Different;

            if (ti_correct_failed)
                comment += "ACID failed (" + cmp_res_ti_comment + ")";

            any_correct |= !ti_correct_failed;
            all_correct &= !ti_correct_failed;
        }

        if (use_ms_ta_)
        {
            tie(cmp_res_ta, cmp_res_ta_comment) = compareTa(tst_id, target_data, max_ref_time_diff); //mode s

            ta_no_ref = cmp_res_ta == ValueComparisonResult::Unknown_NoRefData;
            all_no_ref &= ta_no_ref;

            ta_correct_failed = cmp_res_ta == ValueComparisonResult::Unknown_NoTstData
                    || cmp_res_ta == ValueComparisonResult::Different;

            if (ta_correct_failed)
            {
                if (comment.size())
                    comment += ", ";
                comment += "ACAD failed (" + cmp_res_ta_comment + ")";
            }

            any_correct |= !ta_correct_failed;
            all_correct &= !ta_correct_failed;
        }

        if (use_mode_a_)
        {
            tie(cmp_res_ma, cmp_res_ma_comment) = compareModeA(tst_id, target_data, max_ref_time_diff);

            ma_no_ref = cmp_res_ma == ValueComparisonResult::Unknown_NoRefData;
            all_no_ref &= ma_no_ref;

            ma_correct_failed = cmp_res_ma == ValueComparisonResult::Unknown_NoTstData
                    || cmp_res_ma == ValueComparisonResult::Different;

            if (ma_correct_failed)
            {
                if (comment.size())
                    comment += ", ";
                comment += "M3A failed (" + cmp_res_ma_comment + ")";
            }

            any_correct |= !ma_correct_failed;
            all_correct &= !ma_correct_failed;
        }

        if (all_no_ref) // none has a reference
        {
            if (!skip_no_data_details)
                addDetail(timestamp, pos_current, ref_pos,
                            ref_exists, is_inside, false, // ref_exists, pos_inside, is_not_correct
                            num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                            num_correct, num_not_correct, "No reference id");

            ++num_no_ref_id;

            continue;
        }

        if (!use_ms_ti_ && !use_ms_ta_ && !use_mode_a_)
            all_correct = false; // if none is used, set to false

        if (require_correctness_of_all_)
        {
            result_ok = all_correct;
        }
        else // one correct ok
        {
            result_ok = any_correct;
        }

        if (result_ok)
            ++num_correct;
        else
            ++num_not_correct;

        if (!skip_detail)
            addDetail(timestamp, pos_current, ref_pos,
                        ref_exists, is_inside, !result_ok,
                        num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                        num_correct, num_not_correct, comment);
    }

    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " num_updates " << num_updates << " num_no_ref_pos " << num_no_ref_pos
           << " num_no_ref_id " << num_no_ref_id
           << " num_pos_outside " << num_pos_outside << " num_pos_inside " << num_pos_inside
           << " num_correct " << num_correct << " num_not_correct " << num_not_correct;

    traced_assert(num_updates - num_no_ref_pos == num_pos_inside + num_pos_outside);
    traced_assert(num_pos_inside == num_no_ref_id+num_correct+num_not_correct);

    return make_shared<EvaluationRequirementResult::SingleIdentificationCorrect>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_updates, num_no_ref_pos, num_no_ref_id, num_pos_outside, num_pos_inside,
                num_correct, num_not_correct);
}

bool IdentificationCorrect::requireCorrectnessOfAll() const
{
    return require_correctness_of_all_;
}

bool IdentificationCorrect::useModeA() const
{
    return use_mode_a_;
}

bool IdentificationCorrect::useMsTa() const
{
    return use_ms_ta_;
}

bool IdentificationCorrect::useMsTi() const
{
    return use_ms_ti_;
}

}
