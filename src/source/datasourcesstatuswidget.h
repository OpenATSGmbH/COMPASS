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

#include "datasourceswidgetbase.h"

#include <map>
#include <deque>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class DBContentManager;
class DataSourcesStatusWidget;
class DataSourcesWidgetItemBase;

class QLabel;

/**
 */
class DataSourceStatusItem : public DataSourceItemBase
{
public:
    DataSourceStatusItem(DataSourcesStatusWidget* widget,
                         DataSourcesWidgetItemBase* parent);

    virtual ~DataSourceStatusItem() = default;

protected:
    virtual void init_impl() override;
    virtual void updateContentChanges_impl() override;
    virtual void updateContent_impl() override;

private:
    void showStatus(const std::string& msg, const QColor& color);
    void showStatus(const std::pair<std::string, QColor>& info);

    DataSourcesStatusWidget* status_widget_ = nullptr;
    QLabel*                  status_label_  = nullptr;
};



/**
 */
class DataSourcesStatusWidget : public DataSourcesWidgetBase
{
    Q_OBJECT

signals:
    void eventAdded();
    void refreshed();

public:
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
        Coasting          = 5  // sensor coasting (last received CON status too old)
    };

    /**
     * Current sensor state.
     */
    struct SensorState
    {
        bool isFresh() const { return status == SensorStatus::Fresh; }
        bool isCoasting() const { return status == SensorStatus::Coasting; }
        bool isCON() const { return status == SensorStatus::ConOperational    || 
                                    status == SensorStatus::ConDegraded       ||
                                    status == SensorStatus::ConInitialization ||
                                    status == SensorStatus::ConDisconnected; }

        SensorStatus             status = SensorStatus::Fresh; // current sensor status
        boost::posix_time::ptime ts_con;                       // timestamp the last CON status was obtained
    };

    typedef std::map<unsigned int, SensorState>   SensorStateMap;     // sensor id => state
    typedef std::pair<unsigned int, unsigned int> TrackerKey;         // sensor id, line id
    typedef std::pair<SensorStatus, SensorStatus> StatusChange;       // old and new sensor status

    /**
     * A sensor status event (e.g. a status change for a certain sensor at a certain time)
     */
    struct Event
    {
        enum class Type
        {
            StatusChange = 0,   // sensor status changed
            MissingInformation, // important sensor status related information was missing from CAT063 buffer (timestamp, CON, sensor id, etc.)
            NoCAT063Data        // no CAT063 data received from buffers
        };

        bool isGeneral() const { return !tracker_key.has_value(); }
        bool isTrackerSpecific() const { return tracker_key.has_value(); }
        bool isTrackerSpecific(const TrackerKey& key) const { return isTrackerSpecific() && tracker_key.value() == key; }

        std::pair<std::string, QColor> displayInfo(bool tracker_info,
                                                   bool sensor_info) const;

        Type                                  type;           // event type
        boost::posix_time::ptime              ts;             // event timestamp
        boost::optional<TrackerKey>           tracker_key;    // tracker the event is assigned to (might be empty in general events)
        boost::optional<unsigned int>         sensor_ds_id;   // id of the involved sensor (might be empty in general events)
        boost::optional<StatusChange>         status_change;  // old and new status (used for StatusChange type events)
        std::string                           info;           // additional information
    };

    /**
     * Holds all important state information related to a certain tracker (+line).
     */
    struct TrackerStates
    {
        SensorStateMap           states;         // sensor states
        boost::posix_time::ptime last_update_ts; // latest update received for a sensor from this tracker
        std::deque<Event>        events;         // tracker statuts events
    };

    DataSourcesStatusWidget(DataSourceManager& ds_man, 
                            DBContentManager& dbcontent_man,
                            bool init_ui = true);
    virtual ~DataSourcesStatusWidget();

    virtual void addActionsToConfigMenu(QMenu* menu) override;

    void reset();

    void setActiveTracker(unsigned int ds_id, unsigned int line_id);
    void unsetActiveTracker();
    bool hasActiveTracker() const { return active_tracker_.has_value(); }

    const boost::optional<TrackerKey>& activeTracker() const { return active_tracker_; }
    const boost::posix_time::ptime& lastRefresh() const { return last_refresh_ts_; }
    const Event* lastTrackerEvent() const;

    bool hasActiveTrackerStates() const;
    const TrackerStates& activeTrackerStates() const;
    bool hasActiveTrackerSensorState(unsigned int ds_id) const;
    const SensorState& activeTrackerSensorState(unsigned int ds_id) const;

    void showLastUpdates(bool show);

    static SensorStatus sensorStatusFromCon(unsigned char con);

    static std::string stringFromSensorStatus(SensorStatus status);
    static QColor colorFromSensorStatus(SensorStatus status);
    static std::pair<std::string, QColor> displayInfoFromSensorStatus(SensorStatus status);

    static const std::pair<std::string, QColor> InfoConOperational;
    static const std::pair<std::string, QColor> InfoConDegraded;
    static const std::pair<std::string, QColor> InfoConInitialization;
    static const std::pair<std::string, QColor> InfoConDisconnected;
    static const std::pair<std::string, QColor> InfoCoasting;
    static const std::pair<std::string, QColor> InfoStatusUnknown;
    static const std::pair<std::string, QColor> InfoStatusInfoMissing;
    static const std::pair<std::string, QColor> InfoNoData;

    static const int StatusColumn;
    static const int LastUpdateColumn;

    static const int MaximumEventCount;

protected:
    virtual QStringList getCustomColumnHeaders() const override;

    virtual bool showDSType(const std::string& ds_type_name) const override;
    virtual bool showDS(unsigned int ds_id) const override;

    virtual DataSourceItemBase* createDSItem(DataSourcesWidgetItemBase* parent = nullptr) override;

private:
    void logEvent(TrackerStates* tracker_states,
                  Event::Type type,
                  const boost::posix_time::ptime& ts,
                  const boost::optional<TrackerKey>& tracker_key = boost::optional<TrackerKey>(),
                  const boost::optional<unsigned int>& sensor_ds_id = boost::optional<unsigned int>(),
                  const boost::optional<StatusChange>& status_change = boost::optional<StatusChange>(),
                  const std::string& info = "");
    void addEvent(TrackerStates& tracker_states,
                  const Event& evt);

    void dataLoaded();
    void updateSensorStatus();

    DBContentManager& dbcontent_man_;

    boost::optional<TrackerKey> active_tracker_;

    std::map<TrackerKey, TrackerStates> tracker_states_;
    boost::posix_time::ptime            last_refresh_ts_;
};
