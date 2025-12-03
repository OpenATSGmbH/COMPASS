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

    static const std::pair<std::string, QColor> InfoNoData;
    static const std::pair<std::string, QColor> InfoCoasting;
    static const std::pair<std::string, QColor> InfoNoStatus;

    static const std::pair<std::string, QColor> InfoConOperational;
    static const std::pair<std::string, QColor> InfoConDegraded;
    static const std::pair<std::string, QColor> InfoConInitialization;
    static const std::pair<std::string, QColor> InfoConDisconnected;
    static const std::pair<std::string, QColor> InfoConUnknown;

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

public:
    struct SensorStatus
    {
        bool                     con_valid = false;
        bool                     coasting = false;
        unsigned char            con;
        boost::posix_time::ptime ts;
    };

    struct Event
    {
        enum class Type
        {
            FirstStatus = 0,
            GoingIntoCoasting,
            RegainedStatus,
            MissingInformation,
            NoData
        };

        enum class Severity
        {
            Info = 0,
            Warning,
            Error
        };

        std::string displayInfo() const;

        Type                          type;
        Severity                      severity;
        boost::posix_time::ptime      ts;
        boost::optional<unsigned int> tracker_ds_id;
        boost::optional<unsigned int> tracker_line_id;
        boost::optional<unsigned int> sensor_ds_id;
        std::string                   info;
    };

    typedef std::map<unsigned int, SensorStatus>  SensorStatusMap; // sensor id => status info
    typedef std::pair<unsigned int, unsigned int> TrackerKey;      // sensor id, line id

    DataSourcesStatusWidget(DataSourceManager& ds_man, 
                            DBContentManager& dbcontent_man,
                            bool init_ui = true);
    virtual ~DataSourcesStatusWidget();

    void setActiveTracker(unsigned int ds_id, unsigned int line_id);
    void unsetActiveTracker();
    bool hasActiveTracker() const { return active_tracker_.has_value(); }
    const boost::optional<TrackerKey>& activeTracker() const { return active_tracker_; }

    void reset();

    const std::map<TrackerKey, SensorStatusMap>& sensorStatus() const { return sensor_status_; }
    const std::vector<Event>& events() const { return events_; }

    bool hasCurrentTrackerStatus() const;
    bool hasCurrentTrackerStatus(unsigned int ds_id) const;
    const SensorStatusMap& currentTrackerStatus() const;
    const SensorStatus& currentTrackerStatus(unsigned int ds_id) const;

    static const int StatusColumn;
    static const int InfoColumn;

protected:
    virtual QStringList getCustomColumnHeaders() const override;

    virtual bool showDSType(const std::string& ds_type_name) const override;
    virtual bool showDS(unsigned int ds_id) const override;
    virtual bool showDSLine(unsigned int ds_id, unsigned int ds_line) const override;

    virtual DataSourceItemBase* createDSItem(DataSourcesWidgetItemBase* parent = nullptr) override;

private:
    void logEvent(Event::Type type,
                  const boost::posix_time::ptime& ts,
                  const boost::optional<unsigned int>& tracker_ds_id = boost::optional<unsigned int>(),
                  const boost::optional<unsigned int>& tracker_line_id = boost::optional<unsigned int>(),
                  const boost::optional<unsigned int>& sensor_ds_id = boost::optional<unsigned int>(),
                  const std::string& info = "");
    void dataLoaded();
    void updateSensorStatus();

    DBContentManager& dbcontent_man_;

    boost::optional<TrackerKey> active_tracker_;

    std::map<TrackerKey, SensorStatusMap> sensor_status_;
    std::vector<Event>                    events_;
};
