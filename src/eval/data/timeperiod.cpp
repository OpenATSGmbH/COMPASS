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

#include "timeperiod.h"
//#include "stringconv.h"
#include "util/timeconv.h"

#include "evaluationtargetdata.h"
#include "dbcontent/target/targetreportchain.h"

/*****************************************************************************************
 * TimePeriod
 *****************************************************************************************/

/**
*/
TimePeriod::TimePeriod (boost::posix_time::ptime begin, 
                        boost::posix_time::ptime end,
                        Type type)
    : begin_(begin), end_(end), type_(type)
{
    traced_assert(end_ >= begin_);
}

/**
*/
bool TimePeriod::isInside (boost::posix_time::ptime time) const
{
    return (begin_ <= time) && (time <= end_);
}

/**
*/
bool TimePeriod::isCloseToEnd (boost::posix_time::ptime time, boost::posix_time::time_duration max_time_d) const
{
    traced_assert(time >= end_);
    return (time-end_).abs() <= max_time_d;
}

/**
*/
void TimePeriod::extend (boost::posix_time::ptime time)
{
    traced_assert(time >= end_);
    end_ = time;
}

/**
*/
boost::posix_time::ptime TimePeriod::begin() const
{
    return begin_;
}

/**
*/
boost::posix_time::ptime TimePeriod::end() const
{
    return end_;
}

/**
*/
boost::posix_time::time_duration TimePeriod::duration() const
{
    return end_ - begin_;
}

/**
*/
std::string TimePeriod::str() const
{
    std::string s = "[" + Utils::Time::toString(begin_) + "," + Utils::Time::toString(end_) + "]";
    if (type_ == Type::OutsideSector)
        s += " Outside Sector";

    return s;
}

/**
*/
void TimePeriod::addUpdate(const TimePeriodUpdate& update)
{
    //check time range and order
    traced_assert(update.data_id.timestamp() >= begin_ && update.data_id.timestamp() <= end_);
    traced_assert(updates_.empty() || update.data_id.timestamp() >= updates_.back().data_id.timestamp());

    updates_.push_back(update);
}

/**
*/
const TimePeriod::Updates& TimePeriod::getUpdates() const
{
    return updates_;
}

/*****************************************************************************************
 * TimePeriodCollection
 *****************************************************************************************/

/**
*/
TimePeriodCollection::TimePeriodCollection() = default;

/**
*/
void TimePeriodCollection::clear()
{
    periods_.clear();
}

/**
*/
void TimePeriodCollection::add (TimePeriod&& period)
{
    if (periods_.size())
        traced_assert(periods_.rbegin()->end() <= period.begin());

    periods_.push_back(period);
}

/**
*/
void TimePeriodCollection::createFromReference(const EvaluationTargetData& target_data, 
                                               const SectorLayer& sector_layer,
                                               const boost::posix_time::time_duration& max_ref_time_diff)
{
    clear();

    const auto& ref_data = target_data.refChain().timestampIndexes();

    boost::posix_time::ptime timestamp;
    boost::optional<bool>    was_inside;

    for (auto& ref_it : ref_data)
    {
        timestamp = ref_it.first;

        bool is_inside = target_data.isTimeStampNotExcluded(timestamp)
                      && target_data.refPosInside(sector_layer, ref_it);

        if (is_inside)
        {
            if (!was_inside.has_value())
            {
                // first value => create first period
                add({timestamp, timestamp});
            }
            else if(was_inside.value())
            {
                //was inside and is now inside => check distance to last update
                if (lastPeriod().isCloseToEnd(timestamp, max_ref_time_diff)) // 4.9
                    lastPeriod().extend(timestamp); //near enough  => extend
                else
                    add({timestamp, timestamp});    //too far away => create new period
            }
            else //was not inside
            {
                // was not inside and is now inside => create new period
                add({timestamp, timestamp});
            }
        }

        was_inside = is_inside;
    }
}

/**
*/
void TimePeriodCollection::addUpdate(const TimePeriodUpdate& update)
{
    auto idx = getPeriodIndex(update.data_id.timestamp());
    periods_[ idx ].addUpdate(update);
}

/**
*/
bool TimePeriodCollection::isInside (boost::posix_time::ptime time) const
{
    for (const auto& period_it : periods_)
        if (period_it.isInside(time))
            return true;

    return false;
}

/**
*/
unsigned int TimePeriodCollection::getPeriodIndex (boost::posix_time::ptime time) const
{
    size_t n = periods_.size();

    for (size_t i = 0; i < n; ++i)
    {
        const auto& p = periods_.at(i);
        if (p.isInside(time))
        {
            //since our search is greedy, we check the begin of the next interval too in case we are labeled OutsideSector
            if (p.type() == TimePeriod::Type::OutsideSector && 
                i < n - 1 &&
                periods_[ i + 1 ].type() == TimePeriod::Type::InsideSector &&
                periods_[ i + 1 ].begin() == time)
            {
                return i + 1;
            }

            return i;
        }
    }

    throw std::runtime_error("TimePeriodCollection: time "+Utils::Time::toString(time)+" not inside");
}

/**
*/
int TimePeriodCollection::getPeriodMaxIndexBefore (boost::posix_time::ptime time) const
{
    int index {-1};

    for (unsigned int cnt=0; cnt < periods_.size(); ++cnt)
        if (periods_.at(cnt).end() < time)
            index = cnt;

    return index;
}

/**
*/
unsigned int TimePeriodCollection::size() const
{ 
    return periods_.size();
}

/**
*/
TimePeriodCollection::TimePeriodIterator TimePeriodCollection::begin() 
{ 
    return periods_.begin(); 
}

/**
*/
TimePeriodCollection::TimePeriodIterator TimePeriodCollection::end() 
{ 
    return periods_.end(); 
}

/**
*/
boost::posix_time::ptime TimePeriodCollection::totalBegin() const
{
    traced_assert(periods_.size());
    return periods_.at(0).begin();
}

/**
*/
boost::posix_time::ptime TimePeriodCollection::totalEnd() const
{
    traced_assert(periods_.size());
    return periods_.rbegin()->end();
}

/**
*/
TimePeriod& TimePeriodCollection::period (unsigned int index)
{
    traced_assert(index < periods_.size());
    return periods_.at(index);
}

/**
*/
const TimePeriod& TimePeriodCollection::period (unsigned int index) const
{
    traced_assert(index < periods_.size());
    return periods_.at(index);
}

/**
*/
TimePeriod& TimePeriodCollection::lastPeriod()
{
    traced_assert(periods_.size());
    return *periods_.rbegin();
}

/**
*/
std::string TimePeriodCollection::print(bool line_breaks) const
{
    std::string tmp;

    for (auto& period_it : periods_)
    {
        if (tmp.size())
        {
            if (line_breaks)
                tmp += ",\n";
            else
                tmp += ", ";
        }

        tmp += period_it.str();
    }

    return tmp;
}

/**
*/
void TimePeriodCollection::removeSmallPeriods(boost::posix_time::time_duration min_duration)
{
    std::vector<TimePeriod> tmp_periods;

    for (auto& period_it : periods_)
    {
        if (period_it.duration() >= min_duration)
            tmp_periods.push_back(period_it);
    }

    periods_ = tmp_periods;
}

/**
*/
unsigned int TimePeriodCollection::getUIs (float update_interval, 
                                           bool inside_sector_only) const
{
    unsigned int sum_uis = 0;

    for (auto& period_it : periods_)
    {
        //skip outside sectors?
        if (inside_sector_only && period_it.type() == TimePeriod::Type::OutsideSector)
            continue;

        sum_uis += floor(Utils::Time::partialSeconds(period_it.duration())/update_interval);
    }

    return sum_uis;
}

/**
*/
void TimePeriodCollection::fillInOutsidePeriods(const boost::optional<boost::posix_time::ptime>& data_tmin,
                                                const boost::optional<boost::posix_time::ptime>& data_tmax)
{
    //no periods?
    std::vector<TimePeriod> periods;

    //collect inside periods
    if (!periods_.empty())
    {
        periods.reserve(periods_.size());

        for (const auto& p : periods_)
            if (p.type() == TimePeriod::Type::InsideSector)
                periods.push_back(TimePeriod(p.begin(), p.end(), TimePeriod::Type::InsideSector));
    }

    //no inside periods?
    if (periods.empty())
    {
        //add a single outside sector if data bounds are provided
        if (data_tmin.has_value() && data_tmax.has_value())
            add({data_tmin.value(), data_tmax.value(), TimePeriod::Type::OutsideSector});

        return;
    }

    //inside periods start and end
    auto t0 = periods.front().begin();
    auto t1 = periods.back().end();

    clear();

    bool create_head = data_tmin.has_value() && data_tmin.value() < t0;
    bool create_tail = data_tmax.has_value() && data_tmax.value() > t1;

    if (create_head)
        add({data_tmin.value(), t0, TimePeriod::Type::OutsideSector});

    add(TimePeriod(periods[ 0 ].begin(), periods[ 0 ].end(), TimePeriod::Type::InsideSector));

    size_t n = periods.size();

    for (size_t i = 1; i < n; ++i)
    {
        const auto& p0 = periods[ i - 1 ];
        const auto& p1 = periods[ i     ];

        if (p0.end() < p1.begin())
            add(TimePeriod(p0.end(), p1.begin(), TimePeriod::Type::OutsideSector));

        add(TimePeriod(p1.begin(), p1.end(), TimePeriod::Type::InsideSector));
    }

    if (create_tail)
        add({t1, data_tmax.value(), TimePeriod::Type::OutsideSector});
}
