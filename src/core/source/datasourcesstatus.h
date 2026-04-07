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

#include "traced_assert.h"

namespace context { class DBContextManager; }

#include <map>
#include <deque>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <QColor>

namespace sensor_status
{

/**
 * Sensor status enum.
 * - The first items are directly mapped from CON states
 * - The other items are internally determined states
 */
enum class SensorStatus
{
    //con states
    ConOperational    = 0, // sensor operational  (from CON)
    ConDegraded       = 1, // sensor degraded     (from CON)
    ConInitialization = 2, // sensor initializing (from CON)
    ConDisconnected   = 3, // sensor disconnected (from CON)

    //internal states
    Fresh             = 4, // sensor status uninit
    Unknown           = 5  // sensor status unknown
};

/**
 * Current sensor state.
 */
struct SensorState
{
    bool isFresh() const { return status == SensorStatus::Fresh; }
    bool isUnknown() const { return status == SensorStatus::Unknown; }
    bool isCON() const { return status == SensorStatus::ConOperational    || 
                                status == SensorStatus::ConDegraded       ||
                                status == SensorStatus::ConInitialization ||
                                status == SensorStatus::ConDisconnected; }

    SensorStatus             status = SensorStatus::Fresh;      // current sensor status
    boost::posix_time::ptime ts_con;                            // timestamp the last CON status was obtained
    unsigned long            rec_num_con;                       // cat063 rec num of con status
};

typedef std::pair<unsigned int, unsigned int> TrackerKey;     // sensor id, line id
typedef std::pair<SensorStatus, SensorStatus> StatusChange;   // old and new sensor status

/**
 * A sensor status event (e.g. a status change for a certain sensor at a certain time)
 */
struct Event
{
    enum class Type
    {
        StatusChangeIntermediate = 0, // intermediate sensor status change
        StatusChangeFinal,            // final sensor status change
        MissingInformation,           // important sensor status related information was missing from CAT063 buffer (timestamp, CON, sensor id, etc.)
        NoCAT063Data                  // no CAT063 data received from buffers
    };

    bool isGeneral() const { return !tracker_key.has_value(); }
    bool isTrackerSpecific() const { return tracker_key.has_value(); }
    bool isTrackerSpecific(const TrackerKey& key) const { return isTrackerSpecific() && tracker_key.value() == key; }

    std::string toString(context::DBContextManager& ctx_man,
                         bool add_tracker_info,
                         bool add_sensor_info) const;

    bool operator<(const Event& other) const;

    static std::string typeToString(Type type);

    Type                                  type;           // event type
    boost::posix_time::ptime              ts;             // event timestamp
    boost::optional<TrackerKey>           tracker_key;    // tracker the event is assigned to (might be empty in general events)
    boost::optional<unsigned int>         sensor_ds_id;   // id of the involved sensor (might be empty in general events)
    boost::optional<StatusChange>         status_change;  // old and new status (used for StatusChange type events)

    bool consumed = false;
};

/**
 * Event queue.
 */
class EventQueue
{
public:
    EventQueue() = default;
    EventQueue(size_t max_queue_length) : max_queue_length_(max_queue_length) {}
    virtual ~EventQueue() = default;

    void addEvent(const Event& evt);
    void clear();
    std::vector<const Event*> consumeNewEvents();

    const std::deque<Event>& queue() const { return events_; }

private:
    std::deque<Event>       events_;
    boost::optional<size_t> max_queue_length_;
};

extern SensorStatus sensorStatusFromCon(unsigned char con);
extern std::string stringFromSensorStatus(SensorStatus status);
extern std::string displayStringFromSensorStatus(sensor_status::SensorStatus status);

} // namespace sensor_status
