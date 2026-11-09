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

#include "datasourcesstatus.h"

#include <QObject>

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class DBContentManager;

namespace context
{
    class DBContextManager;
}

/**
 * Tracks the sensor status reported by the trackers (CAT063) in live mode.
 * - parses the distributed view data on every data distribution, independent of any widget
 * - keeps the sensor states and status events per tracker line, and the active tracker line
 * - shown and modified by the Sensor Status tool (DataSourcesStatusToolWidget), read by get_sensor_status
 */
class SensorStatusTracker : public QObject
{
    Q_OBJECT

signals:
    void activeTrackerChanged();
    void eventsAdded();
    void refreshed();

public slots:
    void dataDistributedSlot();

public:
    /**
     * Sensor status related information for a certain tracker line and sensor.
     */
    struct SensorStatusCache
    {
        void initForParse();

        sensor_status::SensorState state_current;
        sensor_status::SensorState state_last_update;
        sensor_status::SensorState state_last_item;

        bool last_update_found_in_scan = false;
    };

    typedef std::map<unsigned int, SensorStatusCache> SensorStateMap; // sensor id => state

    /**
     * Holds all important state information related to a certain tracker (+line).
     */
    struct TrackerStates
    {
        void initForParse();
        void clearOutdatedEvents(double max_event_age);
        bool queueCandidateEvents();

        SensorStateMap                 states;           // sensor states
        boost::posix_time::ptime       last_update_ts;   // latest update received for a sensor from this tracker
        sensor_status::EventQueue      events;           // tracker status events
        std::set<sensor_status::Event> added_events;     // set protecting from duplicate event logging
        std::set<sensor_status::Event> candidates;
    };

    SensorStatusTracker(context::DBContextManager& ctx_man,
                        DBContentManager& dbcontent_man);
    virtual ~SensorStatusTracker();

    void reset();

    void setActiveTracker(unsigned int ds_id, unsigned int line_id);
    void unsetActiveTracker();
    bool hasActiveTracker() const { return active_tracker_.has_value(); }

    const boost::optional<sensor_status::TrackerKey>& activeTracker() const { return active_tracker_; }
    const boost::posix_time::ptime& lastRefresh() const { return last_refresh_ts_; }
    const sensor_status::EventQueue& currentEventQueue() const;
    sensor_status::EventQueue& currentEventQueue();

    bool hasActiveTrackerStates() const;
    const TrackerStates& activeTrackerStates() const;
    bool hasActiveTrackerSensorState(unsigned int ds_id) const;
    const sensor_status::SensorState& activeTrackerSensorState(unsigned int ds_id) const;

private:
    void logEvent(TrackerStates* tracker_states,
                  sensor_status::Event::Type type,
                  const boost::posix_time::ptime& ts,
                  const boost::optional<sensor_status::TrackerKey>& tracker_key = boost::optional<sensor_status::TrackerKey>(),
                  const boost::optional<unsigned int>& sensor_ds_id = boost::optional<unsigned int>(),
                  const boost::optional<sensor_status::StatusChange>& status_change = boost::optional<sensor_status::StatusChange>());
    void addEvent(TrackerStates& tracker_states,
                  const sensor_status::Event& evt,
                  bool check_duplicates);
    void addEvent(sensor_status::EventQueue& event_queue,
                  const sensor_status::Event& evt);
    void initTrackerStates(const sensor_status::TrackerKey& key,
                           TrackerStates& tracker_states) const;

    void initSensorStatesForParse();
    void updateSensorStatus();

    context::DBContextManager& ctx_man_;
    DBContentManager&          dbcontent_man_;

    boost::optional<sensor_status::TrackerKey> active_tracker_;

    std::map<sensor_status::TrackerKey, TrackerStates> tracker_states_;

    boost::posix_time::ptime  last_refresh_ts_;
    sensor_status::EventQueue general_events_;
};
