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

#include "event_log.h"
#include "util/timeconv.h"
#include "json.hpp"

namespace logger
{

/***********************************************************************************
 * EventLog
 ***********************************************************************************/

uint32_t EventLog::event_ids_ = 0;

/**
*/
EventLog::EventLog(size_t max_events_per_category)
:   max_events_(max_events_per_category)
{
}

/**
 * Adds a new event to the event log under the given log priority.
 */
void EventLog::addEvent(const Event& evt, int priority)
{
    //thread safety first
    boost::lock_guard<boost::mutex> lock{event_mutex_};

    auto& buffer = events_[ priority ];

    //init buffer if needed
    if (buffer.capacity() == 0)
        buffer.set_capacity(max_events_);

    //assign event id
    Event evt_with_id = evt;
    evt_with_id.id = event_ids_++;

    //add event to buffer
    buffer.push_back(evt_with_id);
}

/**
 * Retrieves certain events using the given query.
 */
EventLog::EventMap EventLog::getEvents(const EventQuery& query) const
{
    EventMap events;

    //thread safety first
    boost::lock_guard<boost::mutex> lock{event_mutex_};

    for (auto& elem : events_)
    {
        //correct event type?
        if (query.event_type == EventQuery::EventType::Warnings && elem.first != log4cpp::Priority::WARN)
            continue;
        if (query.event_type == EventQuery::EventType::Errors && elem.first != log4cpp::Priority::ERROR)
            continue;

        auto& evt_buffer = elem.second;
        auto& evts_out   = events[ elem.first ];

        //prereserve predicted size
        size_t n_buffer = evt_buffer.size();
        size_t n_target = query.query_type == EventQuery::QueryType::All ? n_buffer : query.num_items;
        size_t n_res    = std::min(n_buffer, n_target);

        evts_out.reserve(n_res);

        //iterate over buffer items
        auto itend = evt_buffer.rend();
        for (auto it = evt_buffer.rbegin(); it != itend; ++it)
        {
            //query limit hit?
            if (query.query_type != EventQuery::QueryType::All && evts_out.size() >= query.num_items)
                break;

            //only add fresh events?
            if (query.fresh_only && !it->consume())
                continue;

            evts_out.push_back(*it);
        }

        evts_out.shrink_to_fit();
    }

    return events;
}

/**
 * Retrieves certain events as json using the given query.
 */
nlohmann::json EventLog::getEventsAsJSON(const EventQuery& query) const
{
    auto createJSON = [ & ] (const logger::EventLog::EventMap& events)
    {
        nlohmann::json archive_json;

        auto warnings = nlohmann::json::array();
        auto errors   = nlohmann::json::array();
        
        auto it_wrn = events.find(log4cpp::Priority::WARN);
        auto it_err = events.find(log4cpp::Priority::ERROR);

        auto addEntry = [ & ] (nlohmann::json& arr, const logger::Event& event)
        {
            nlohmann::json e;
            e["id"       ] = event.id;
            e["timestamp"] = Utils::Time::toString(boost::posix_time::from_time_t((time_t)event.timestamp));
            e["message"  ] = event.message;
            
            arr.push_back(e);
        };

        //add warnings
        if (it_wrn != events.end())
        {
            for (const auto& entry : it_wrn->second)
                addEntry(warnings, entry);
        }

        //add errors
        if (it_err != events.end())
        {
            for (const auto& entry : it_err->second)
                addEntry(errors, entry);
        }

        archive_json["warnings"] = warnings;
        archive_json["errors"]   = errors;

        return archive_json;
    };

    auto events = getEvents(query);

    nlohmann::json evts_json = createJSON(events);

    return evts_json;
}

/***********************************************************************************
 * EventAppender
 ***********************************************************************************/

/**
*/
void EventAppender::_append(const log4cpp::LoggingEvent& event)
{
    //only collect warnings and errors
    if (event_log_ && (event.priority == log4cpp::Priority::WARN ||
                       event.priority == log4cpp::Priority::ERROR))
    {
        //std::cout << "adding type " << event.priority << " log message '" << event.message << "' to archive" << std::endl;

        boost::mutex m;

        Event evt;
        evt.timestamp = event.timeStamp.getSeconds();
        evt.message   = event.message;

        event_log_->addEvent(evt, event.priority);
    }
}

} // namespace logger
