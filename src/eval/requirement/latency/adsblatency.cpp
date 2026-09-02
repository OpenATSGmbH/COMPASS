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

#include "eval/requirement/latency/adsblatency.h"
#include "eval/results/latency/adsblatency.h"

#include "evaluationmanager.h"
#include "logger.h"
#include "sectorlayer.h"

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

/**
*/
ADSBLatency::ADSBLatency(const std::string& name,
                         const std::string& short_name,
                         const std::string& group_name,
                         double prob,
                         COMPARISON_TYPE prob_check_type,
                         EvaluationCalculator& calculator,
                         float max_value_s)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator, false)
    , max_value_s_   (max_value_s)
{
}

/**
*/
float ADSBLatency::maxValue() const
{
    return max_value_s_;
}

/**
*/
std::shared_ptr<EvaluationRequirementResult::Single> ADSBLatency::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " max_value_s " << max_value_s_;

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    unsigned int num_pos         {0};
    unsigned int num_no_data     {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside  {0};
    unsigned int num_value_ok    {0};
    unsigned int num_value_nok   {0};

    typedef EvaluationRequirementResult::SingleADSBLatency Result;
    typedef EvaluationDetail                               Detail;
    typedef Result::EvaluationDetails                      Details;
    Details details;

    ptime timestamp;

    dbContent::TargetPosition tst_pos;

    std::vector<double> latencies;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    auto addDetail = [ & ] (const ptime& ts,
                            const dbContent::TargetPosition& tst_pos,
                            const QVariant& pos_inside,
                            const QVariant& value,
                            const QVariant& check_passed,
                            const std::string& comment)
    {
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::PosInside, pos_inside.isValid() ? pos_inside : "false")
                                             .setValue(Result::DetailKey::Value, value)
                                             .setValue(Result::DetailKey::CheckPassed, check_passed)
                                             .setValue(Result::DetailKey::NumPos, num_pos)
                                             .setValue(Result::DetailKey::NumNoData, num_no_data)
                                             .setValue(Result::DetailKey::NumInside, num_pos_inside)
                                             .setValue(Result::DetailKey::NumOutside, num_pos_outside)
                                             .setValue(Result::DetailKey::NumCheckPassed, num_value_ok)
                                             .setValue(Result::DetailKey::NumCheckFailed, num_value_nok)
                                             .generalComment(comment));
    };

    for (const auto& tst_id : tst_data)
    {
        ++num_pos;

        timestamp = tst_id.first;
        tst_pos = target_data.tstChain().pos(tst_id);

        bool is_inside = target_data.isTimeStampNotExcluded(timestamp)
                         && target_data.tstPosInside(sector_layer, tst_id);

        if (!is_inside)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos, is_inside, {}, {}, "Outside sector");

            ++num_pos_outside;
            continue;
        }
        ++num_pos_inside;

        auto tomr = target_data.tstChain().tomrPosition(tst_id);
        auto tort = target_data.tstChain().tort(tst_id);

        if (!tomr.has_value() || !tort.has_value())
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos, is_inside, {}, {}, "No time stamp data");

            ++num_no_data;
            continue;
        }

        // latency per ED-129C Section 3.3.4: report output time minus
        // message reception time, both in seconds of day
        double latency = (double)tort.value() - (double)tomr.value();

        // day wrap around midnight
        if (latency < -43200.0)
            latency += 86400.0;
        else if (latency > 43200.0)
            latency -= 86400.0;

        latencies.push_back(latency);

        bool value_ok = latency >= 0.0 && latency <= max_value_s_;

        string comment;

        if (value_ok)
            ++num_value_ok;
        else
        {
            ++num_value_nok;
            comment = latency < 0.0 ? "Negative latency" : "Latency not OK";
        }

        addDetail(timestamp, tst_pos, is_inside, latency, value_ok, comment);
    }

    traced_assert(num_pos == num_pos_inside + num_pos_outside);
    traced_assert(num_pos_inside == num_no_data + num_value_ok + num_value_nok);
    traced_assert(latencies.size() == num_value_ok + num_value_nok);

    return make_shared<EvaluationRequirementResult::SingleADSBLatency>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_pos, num_no_data, num_pos_outside, num_pos_inside,
                num_value_ok, num_value_nok, latencies);
}

}
