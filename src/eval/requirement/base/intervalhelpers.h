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

#include "dbcontent/target/targetreportchain.h"

#include <functional>
#include <string>
#include <vector>

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>

class TimePeriod;
class TimePeriodCollection;

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
        TypeEmptyPeriod,              // an in-sector reference period without (valid) updates is encountered
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
 * Validity of a test update in an interval-based requirement.
 */
struct IntervalValidity
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

/**
 * Algorithms of the interval-based requirements (e.g. Correct (Periods)), free of the
 * evaluation data access, so they can be unit tested on plain inputs.
 */
namespace IntervalHelpers
{

/// miss test parameters
struct MissParams
{
    float                  update_interval_s = 1.0f;
    boost::optional<float> min_gap_length_s;
    boost::optional<float> max_gap_length_s;
    boost::optional<float> miss_tolerance_s;
};

/// result counts of one target
struct Counts
{
    uint32_t sum_uis = 0; // expected update intervals
    uint32_t misses  = 0; // missed update intervals
};

typedef std::function<IntervalValidity(const dbContent::TargetReport::DataID&)> ValidityFunc;

/// number of missed update intervals in a gap of dt seconds
uint32_t numMisses(double dt_s, const MissParams& p);

/// adds outside periods up to the test data times and sorts the test updates (sorted by time) into the periods
void addTestUpdates(TimePeriodCollection& periods,
                    const std::vector<dbContent::TargetReport::DataID>& tst_updates);

/// events of one period
std::vector<Event> periodEvents(const TimePeriod& period,
                                const ValidityFunc& validity,
                                const MissParams& p,
                                bool skip_no_data_details);

/// events of all periods, inside periods get their inside period index
std::vector<Event> periodEvents(const TimePeriodCollection& periods,
                                const ValidityFunc& validity,
                                const MissParams& p,
                                bool skip_no_data_details);

/// expected and missed update intervals from the periods and their events
Counts counts(const TimePeriodCollection& periods,
              const std::vector<Event>& events,
              const MissParams& p,
              bool exclude_detection_misses);

/// removes the update intervals without any test update from the counts (e.g. EUROCAE ED-142 6.4.3)
void excludeDetectionMisses(const TimePeriodCollection& periods,
                            const MissParams& p,
                            Counts& counts);

} // namespace IntervalHelpers

} // namespace EvaluationRequirement
