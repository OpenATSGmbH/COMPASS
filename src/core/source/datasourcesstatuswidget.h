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
#include "datasourcesstatus.h"

#include <set>
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

    DataSourcesStatusWidget* status_widget_ = nullptr;
    QLabel*                  status_label_  = nullptr;
};

/**
 */
class DataSourcesStatusWidget : public DataSourcesWidgetBase
{
    Q_OBJECT

signals:
    void activeTrackerChanged();
    void eventsAdded();
    void refreshed();

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

    DataSourcesStatusWidget(context::DBContextManager& ctx_man,
                            DBContentManager& dbcontent_man,
                            bool init_ui = true);
    virtual ~DataSourcesStatusWidget();

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

    void showLastUpdates(bool show);

    static QColor colorFromSensorStatus(sensor_status::SensorStatus status);
    static QColor colorFromEvent(const sensor_status::Event& evt);
    static QFont  fontFromEvent(const sensor_status::Event& evt);

    static const QColor ColorConOperational;
    static const QColor ColorConDegraded;
    static const QColor ColorConInitialization;
    static const QColor ColorConDisconnected;
    static const QColor ColorCoasting;
    static const QColor ColorStatusUnknown;
    static const QColor ColorStatusInfoMissing;
    static const QColor ColorNoData;

    static const int StatusColumn;
    static const int LastUpdateColumn;

protected:
    virtual QStringList getCustomColumnHeaders() const override;

    virtual bool showDSType(const std::string& ds_type_name) const override;
    virtual bool showDS(unsigned int ds_id) const override;

    virtual DataSourceItemBase* createDSItem(DataSourcesWidgetItemBase* parent = nullptr) override;

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

    void dataLoaded();
    void initSensorStatesForParse();
    void updateSensorStatus();

    DBContentManager& dbcontent_man_;

    boost::optional<sensor_status::TrackerKey> active_tracker_;

    std::map<sensor_status::TrackerKey, TrackerStates> tracker_states_;
    std::map<unsigned long, boost::posix_time::ptime>  processed_rec_nums_;

    boost::posix_time::ptime  last_refresh_ts_;
    sensor_status::EventQueue general_events_;
};
