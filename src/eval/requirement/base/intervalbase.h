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

#include "probabilitybase.h"
#include "dbcontent/target/targetreportdefs.h"
#include "targetposition.h"

#include <string>

#include <boost/date_time/posix_time/ptime.hpp>

class TimePeriod;
class EvaluationDetail;
class TimePeriodCollection;

namespace dbContent
{
    class TargetPosition;
}

namespace EvaluationRequirement
{

/**
 * Describes certain events happening when traversing the reference data time periods.
 */
struct Event
{
    enum Type
    {
        TypeNoReference = 0,          // update is not part of an inside sector reference time period
        TypeEnterPeriod,              // a new in-sector reference period is entered
        TypeFirstValidUpdateInPeriod, // the first valid update of an in-sector reference period is encountered
        TypeValidFirst,               // a valid update is checked at the start of an in-sector reference period
        TypeValid,                    // a valid update is checked inside an in-sector reference period
        TypeValidLast,                // a valid update is checked at the end of an in-sector reference period
        TypeInvalid,                  // an invalid update was encountered inside an in-sector reference period
        TypeDataMissing,              // update misses (e.g. reference) data to perform validity check
        TypeLastValidUpdateInPeriod,  // the last valid update of an in-sector reference period is encountered
        TypeLeavePeriod,              // an in-sector reference period is being left
        TypeEmptyPeriod,              // an in-sector reference period without updates is encountered
    };

    boost::posix_time::time_duration dt() const;
    double dtSeconds() const;

    Type                            type;           // event type
    dbContent::TargetReport::DataID data_id;        // data id of the update causing the event (if an update is involved)
    boost::optional<uint32_t>       period;         // index to the period the event happened in
    std::string                     error;          // error description in case of an invalid update
    boost::posix_time::ptime        interval_time0; // start time of the event interval (note: =/= period start!)
    boost::posix_time::ptime        interval_time1; // end time of the event interval (note: =/= period end!)
    bool                            had_ref_data;   // reference data was available

    uint32_t                        misses = 0;     // misses caused by the event
};

/**
 * Base class for interval-based evaluation requirements.
 */
class IntervalBase : public ProbabilityBase
{
public:
    struct DetailInfo
    {
        boost::posix_time::ptime                                evt_time;             // time the event occurs at
        double                                                  evt_dt;               // event duration
        std::string                                             evt_comment;          // event comment
        dbContent::TargetPosition                               evt_position;         // event location
        boost::optional<dbContent::TargetPosition>              evt_position_ref;     // event reference position (e.g. start of interval)
        boost::optional<unsigned int>                           evt_ref_updates_idx0; // optional event reference updates start index
        boost::optional<unsigned int>                           evt_ref_updates_idx1; // optional event reference updates end index

        bool evt_has_misses;    // event has misses
        bool evt_has_ref;       // event has reference
        bool evt_has_dt;        // event has a valid tod diff

        bool generate_detail = false; // generate a detail for this event
    };

    struct Validity
    {
        enum class Value
        {
            Valid = 0,     // update is valid
            Invalid,       // update is invalid
            RefDataMissing // reference data is missing => could not check validity
        };

        Value        value = Value::Invalid; // validity value
        std::string  comment;                // additional textual info
    };

    IntervalBase(const std::string& name, 
                 const std::string& short_name, 
                 const std::string& group_name,
                 double prob, 
                 COMPARISON_TYPE prob_check_type, 
                 EvaluationCalculator& calculator,
                 float update_interval_s, 
                 const boost::optional<float>& min_gap_length_s = boost::optional<float>(),
                 const boost::optional<float>& max_gap_length_s = boost::optional<float>(),
                 const boost::optional<float>& miss_tolerance_s = boost::optional<float>(),
                 const boost::optional<float>& min_ref_period_s = boost::optional<float>(),
                 const boost::optional<bool>& must_hold_for_any_target = boost::optional<bool>());
    virtual ~IntervalBase() = default;

    virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate (
            const EvaluationTargetData& target_data, 
            std::shared_ptr<Base> instance,
            const SectorLayer& sector_layer) override;

    float updateInterval() const { return update_interval_s_; }

    const boost::optional<float>& minGapLength() const { return min_gap_length_s_; }
    const boost::optional<float>& maxGapLength() const { return max_gap_length_s_; }
    const boost::optional<float>& missTolerance() const { return miss_tolerance_s_; }

    float missThreshold() const;

protected:
    virtual uint32_t numMisses(double dt) const;
    virtual DetailInfo eventDetailInfo(const EvaluationTargetData& target_data,
                                       const Event& event,
                                       std::vector<dbContent::TargetPosition>& ref_updates) const;

    virtual Validity isValid(const dbContent::TargetReport::DataID& data_id,
                         const EvaluationTargetData& target_data,
                         const SectorLayer& sector_layer,
                         const boost::posix_time::time_duration& max_ref_time_diff) const = 0;
    virtual std::shared_ptr<EvaluationRequirementResult::Single> createResult(const std::string& result_id,
                                                                              std::shared_ptr<Base> instance, 
                                                                              const EvaluationTargetData& target_data,
                                                                              const SectorLayer& sector_layer, 
                                                                              const std::vector<EvaluationDetail>& details,
                                                                              const TimePeriodCollection& periods,
                                                                              unsigned int sum_uis,
                                                                              unsigned int misses_total) = 0;
private:
    std::vector<Event> periodEvents(const TimePeriod& period,
                                    const EvaluationTargetData& target_data,
                                    const SectorLayer& sector_layer,
                                    bool skip_no_data_details) const;
    std::vector<Event> periodEvents(const TimePeriodCollection& periods,
                                    const EvaluationTargetData& target_data,
                                    const SectorLayer& sector_layer,
                                    bool skip_no_data_details) const;

    float                  update_interval_s_;
    boost::optional<float> min_gap_length_s_;
    boost::optional<float> max_gap_length_s_;
    boost::optional<float> miss_tolerance_s_;
    boost::optional<float> min_ref_period_s_;
};

} // namespace EvaluationRequirement
