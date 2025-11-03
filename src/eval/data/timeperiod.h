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

#include "boost/date_time/posix_time/ptime.hpp"
#include <boost/optional.hpp>

#include <vector>
#include "traced_assert.h"

#include <math.h>

class SectorLayer;
class EvaluationTargetData;

/**
*/
struct TimePeriodUpdate
{
    dbContent::TargetReport::DataID data_id;
};

/**
*/
class TimePeriod
{
public:
    typedef std::vector<TimePeriodUpdate> Updates;

    enum class Type 
    {
        InsideSector,
        OutsideSector
    };

    TimePeriod(boost::posix_time::ptime begin, boost::posix_time::ptime end, Type type = Type::InsideSector);
    virtual ~TimePeriod() = default;

    bool isInside (boost::posix_time::ptime time) const;
    bool isCloseToEnd (boost::posix_time::ptime time, boost::posix_time::time_duration max_time_d) const;

    void extend (boost::posix_time::ptime time);

    boost::posix_time::ptime begin() const;
    boost::posix_time::ptime end() const;
    boost::posix_time::time_duration duration() const;

    Type type() const { return type_; }

    void addUpdate(const TimePeriodUpdate& update);
    const Updates& getUpdates() const;

    std::string str() const;

protected:
    boost::posix_time::ptime begin_;
    boost::posix_time::ptime end_;
    Type                     type_;
    Updates                  updates_;
};

/**
*/
class TimePeriodCollection
{
public:
    TimePeriodCollection();
    virtual ~TimePeriodCollection() = default;

    void clear();

    void add (TimePeriod&& period);
    void createFromReference(const EvaluationTargetData& target_data, 
                             const SectorLayer& sector_layer,
                             const boost::posix_time::time_duration& max_ref_time_diff);

    void addUpdate(const TimePeriodUpdate& update);

    bool isInside (boost::posix_time::ptime time) const;

    unsigned int getPeriodIndex (boost::posix_time::ptime time) const;
    int getPeriodMaxIndexBefore (boost::posix_time::ptime time) const; // -1 if none

    unsigned int size() const;

    using TimePeriodIterator =
    typename std::vector<TimePeriod>::iterator;

    TimePeriodIterator begin();
    TimePeriodIterator end();

    boost::posix_time::ptime totalBegin() const;
    boost::posix_time::ptime totalEnd() const;

    TimePeriod& period (unsigned int index);
    const TimePeriod& period (unsigned int index) const;
    TimePeriod& lastPeriod();

    std::string print(bool line_breaks=false) const;

    void removeSmallPeriods (boost::posix_time::time_duration min_duration);

    unsigned int getUIs (float update_interval, 
                         bool inside_sector_only = true) const;

    void fillInOutsidePeriods(const boost::optional<boost::posix_time::ptime>& data_tmin = boost::optional<boost::posix_time::ptime>(),
                              const boost::optional<boost::posix_time::ptime>& data_tmax = boost::optional<boost::posix_time::ptime>());

protected:
    std::vector<TimePeriod> periods_;
};
