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

#include "datasourcesstatus.h"

#include "compass.h"
#include "datasourcemanager.h"

#include "timeconv.h"
#include "logger.h"


namespace sensor_status
{

/*************************************************************************************
 * Event
 *************************************************************************************/

/**
 */
std::string Event::typeToString(Type type)
{
    switch (type)
    {
        case Event::Type::MissingInformation:
            return "Information missing from CAT063 data";
        case Event::Type::NoCAT063Data:
            return "No CAT063 data found";
        case Event::Type::StatusChange:
            return "Status changed";
    }
    return "";
}

/**
 */
std::string Event::toString(bool add_tracker_info,
                            bool add_sensor_info) const
{
    std::string txt;

    if (!ts.is_not_a_date_time())
    {
        txt += Utils::Time::toTimeString(ts, false) + " ";
    }

    auto& ds_man = COMPASS::instance().dataSourceManager();

    if (add_tracker_info && tracker_key.has_value())
    {
        traced_assert(ds_man.hasDBDataSource(tracker_key.value().first));
        txt += ds_man.dbDataSource(tracker_key.value().first).name() + " ";
        txt += "L" + std::to_string(tracker_key.value().second + 1) + " ";
    }

    if (add_sensor_info && sensor_ds_id.has_value())
    {
        if (ds_man.hasDBDataSource(sensor_ds_id.value()))
            txt += ds_man.dbDataSource(sensor_ds_id.value()).name() + " ";
        else if (ds_man.hasConfigDataSource(sensor_ds_id.value()))
            txt += ds_man.configDataSource(sensor_ds_id.value()).name() + " ";
        else
            txt += std::to_string(sensor_ds_id.value()) + " ";
    }

    if (type == sensor_status::Event::Type::StatusChange)
    {
        traced_assert(status_change.has_value());

        auto state0_str = sensor_status::displayStringFromSensorStatus(status_change->first);
        auto state1_str = sensor_status::displayStringFromSensorStatus(status_change->second);

        txt += state0_str + " \u2192 " + state1_str + " ";
    }
    else if (type == sensor_status::Event::Type::MissingInformation)
    {
        txt  += Event::typeToString(type) + " ";
    }
    else if (type == sensor_status::Event::Type::NoCAT063Data)
    {
        txt  += Event::typeToString(type) + " ";
    }
    
    return txt;
}

/*************************************************************************************
 * EventQueue
 *************************************************************************************/

/**
 */
void EventQueue::addEvent(const Event& evt)
{
    events_.push_back(evt);

    if (max_queue_length_.has_value() && events_.size() > max_queue_length_.value())
        events_.pop_front();
}

/**
 */
std::vector<const Event*> EventQueue::consumeNewEvents()
{
    size_t idx1 = events_.size();
    size_t idx0 = events_.size();

    for (std::size_t i = idx1; i-- > 0; )
    {
        if (events_[ i ].consumed)
            break;
        idx0 = i;
    }

    traced_assert(idx0 <= idx1);

    size_t n = idx1 - idx0;
    if (n == 0)
        return {};

    //collect consumed events
    std::vector<const Event*> new_events;
    new_events.reserve(n);
    for (size_t i = idx0; i < idx1; ++i)
    {
        //consume event
        auto& evt = events_[ i ];
        evt.consumed = true;

        new_events.push_back(&evt);
    };

    return new_events;
}

/**
 */
void EventQueue::clear()
{
    events_.clear();
}

/*************************************************************************************
 * General
 *************************************************************************************/

/**
 */
SensorStatus sensorStatusFromCon(unsigned char con)
{
    if (con == 0)
        return SensorStatus::ConOperational;
    else if (con == 1)
        return SensorStatus::ConDegraded;
    else if (con == 2)
        return SensorStatus::ConInitialization;
    else if (con == 3)
        return SensorStatus::ConDisconnected;

    bool unknown_con_state = true;
    traced_assert(!unknown_con_state);

    return SensorStatus::Unknown;
}

/**
 */
std::string stringFromSensorStatus(SensorStatus status)
{
    switch (status)
    {
        case SensorStatus::ConOperational:
            return "Operational";
        case SensorStatus::ConDegraded:
            return "Degraded";
        case SensorStatus::ConInitialization:
            return "Initializing";
        case SensorStatus::ConDisconnected:
            return "Not Connected";
        case SensorStatus::Fresh:
            return "Fresh";
        case SensorStatus::Coasting:
            return "Coasting";
        case SensorStatus::Unknown:
            return "Unknown";
    }
    return "Unknown";
}

/**
 */
std::string displayStringFromSensorStatus(sensor_status::SensorStatus status)
{
    if (status == sensor_status::SensorStatus::Fresh)
        return stringFromSensorStatus(sensor_status::SensorStatus::Unknown);

    return stringFromSensorStatus(status);
}

} // namespace sensor_status
