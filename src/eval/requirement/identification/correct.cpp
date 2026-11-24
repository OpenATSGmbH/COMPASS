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

#include "requirement/identification/correct.h"
#include "requirement/base/correcthelpers.h"
#include "results/identification/correct.h"
#include "standard/evaluationstandard.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "logger.h"
#include "timeconv.h"

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

    std::vector<ValueComparisonResult> cmp_results;
    bool result_ok;

    auto addCompare = [ & ] (const std::pair<ValueComparisonResult, std::string>& cmp, const std::string& value_name)
    {
        cmp_results.push_back(cmp.first);

        if (cmp.first == ValueComparisonResult::Unknown_NoTstData || cmp.first == ValueComparisonResult::Different)
        {
            if (comment.size())
                comment += ", ";
            comment += value_name + " failed (" + cmp.second + ")";
        }
    };

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
                                             .addPosition(ref_pos)
                                             .generalComment(comment));
    };

    for (const auto& tst_id : tst_data)
    {
        ref_exists = false;
        is_inside = false;
        comment = "";

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

        cmp_results.clear();

        if (use_ms_ti_)
            addCompare(compareTi(tst_id, target_data, max_ref_time_diff), "ACID"); // aircraft id

        if (use_ms_ta_)
            addCompare(compareTa(tst_id, target_data, max_ref_time_diff), "ACAD"); // mode s

        if (use_mode_a_)
            addCompare(compareModeA(tst_id, target_data, max_ref_time_diff), "M3A");

        auto report_class = CorrectHelpers::classify(cmp_results, require_correctness_of_all_);

        if (report_class == CorrectHelpers::ReportClass::NoRefValue)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, pos_current, ref_pos,
                            ref_exists, is_inside, false, // ref_exists, pos_inside, is_not_correct
                            num_updates, num_no_ref_pos, num_pos_inside, num_pos_outside,
                            num_correct, num_not_correct, "No reference id");

            ++num_no_ref_id;

            continue;
        }

        result_ok = report_class == CorrectHelpers::ReportClass::Correct;

        if (result_ok)
            ++num_correct;
        else
            ++num_not_correct;

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
