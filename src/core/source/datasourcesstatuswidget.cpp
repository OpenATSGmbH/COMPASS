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

#include "datasourcesstatuswidget.h"
#include "sensorstatustracker.h"
#include "datasourcebase.h"
#include "db_context_manager.h"
#include "data_source.h"

#include "appstate.h"

#include "timeconv.h"

#include <QLabel>

/***********************************************************************************************************
 * DataSourceStatusItem
 ***********************************************************************************************************/

/**
 */
DataSourceStatusItem::DataSourceStatusItem(DataSourcesStatusWidget* widget,
                                           DataSourcesWidgetItemBase* parent)
:   DataSourceItemBase(widget, parent)
,   status_widget_    (widget)
{
}

/**
 */
void DataSourceStatusItem::init_impl()
{
    DataSourceItemBase::init_impl();

    status_label_ = new QLabel;

    auto f = status_label_->font();
    f.setBold(true);

    status_label_->setFont(f);

    setItemWidget(DataSourcesStatusWidget::StatusColumn, status_label_);
    //setTextAlignment(DataSourcesStatusWidget::LastUpdateColumn, Qt::AlignHCenter);
}

/**
 */
void DataSourceStatusItem::updateContentChanges_impl()
{
    DataSourceItemBase::updateContentChanges_impl();

    //nothing to do yet
}

/**
 */
void DataSourceStatusItem::updateContent_impl()
{
    DataSourceItemBase::updateContent_impl();

    const auto& tracker = status_widget_->tracker();

    //no sensor status available?
    if (!tracker.hasActiveTrackerSensorState(dsID()))
    {
        auto txt = sensor_status::displayStringFromSensorStatus(sensor_status::SensorStatus::Unknown);
        auto col = DataSourcesStatusWidget::colorFromSensorStatus(sensor_status::SensorStatus::Unknown);

        showStatus(txt, col);

        return;
    }

    const auto& sensor_state = tracker.activeTrackerSensorState(dsID());

    //should be init at this point
    traced_assert(!sensor_state.isFresh());

    auto txt = sensor_status::displayStringFromSensorStatus(sensor_state.status);
    auto col = DataSourcesStatusWidget::colorFromSensorStatus(sensor_state.status);

    showStatus(txt, col);

    std::string last_update_str;
    if (!sensor_state.ts_con.is_not_a_date_time())
        last_update_str = Utils::Time::toTimeString(sensor_state.ts_con, false);

    setText(DataSourcesStatusWidget::LastUpdateColumn, QString::fromStdString(last_update_str));
}

/**
 */
void DataSourceStatusItem::showStatus(const std::string& msg, const QColor& color)
{
    status_label_->setText(QString::fromStdString(msg));
    status_label_->setStyleSheet("color: " + color.name() + ";");   // hex color
}

/***********************************************************************************************************
 * DataSourcesStatusWidget
 ***********************************************************************************************************/

// display colors for various states and events
const QColor DataSourcesStatusWidget::ColorConOperational    = Qt::darkGreen;
const QColor DataSourcesStatusWidget::ColorConDegraded       = QColor(255, 165, 0);
const QColor DataSourcesStatusWidget::ColorConInitialization = Qt::blue;
const QColor DataSourcesStatusWidget::ColorConDisconnected   = Qt::red;
const QColor DataSourcesStatusWidget::ColorCoasting          = QColor(255, 165, 0);
const QColor DataSourcesStatusWidget::ColorStatusUnknown     = Qt::darkGray;
const QColor DataSourcesStatusWidget::ColorStatusInfoMissing = Qt::red;
const QColor DataSourcesStatusWidget::ColorNoData            = Qt::darkGray;

const int DataSourcesStatusWidget::StatusColumn     = 1;
const int DataSourcesStatusWidget::LastUpdateColumn = 2;

/**
 */
DataSourcesStatusWidget::DataSourcesStatusWidget(context::DBContextManager& ctx_man,
                                                 SensorStatusTracker& tracker,
                                                 bool init_ui)
:   DataSourcesWidgetBase(ctx_man, Source::All, false, false)
,   tracker_(tracker)
{
    if (init_ui)
        init();

    showLastUpdates(ctx_man.sensorConfig().sensor_status_show_last_updates);

    // the tracker state changed => show it
    connect(&tracker_, &SensorStatusTracker::activeTrackerChanged, this, [ this ] () { updateContent(); });
    connect(&tracker_, &SensorStatusTracker::refreshed, this, [ this ] () { updateContent(); });
}

/**
 */
DataSourcesStatusWidget::~DataSourcesStatusWidget() = default;

/**
 */
QColor DataSourcesStatusWidget::colorFromSensorStatus(sensor_status::SensorStatus status)
{
    switch (status)
    {
        case sensor_status::SensorStatus::ConOperational:
            return ColorConOperational;
        case sensor_status::SensorStatus::ConDegraded:
            return ColorConDegraded;
        case sensor_status::SensorStatus::ConInitialization:
            return ColorConInitialization;
        case sensor_status::SensorStatus::ConDisconnected:
            return ColorConDisconnected;
        case sensor_status::SensorStatus::Fresh:
            return ColorStatusUnknown;
        case sensor_status::SensorStatus::Unknown:
            return ColorStatusUnknown;
    }
    return ColorStatusUnknown;
}

/**
 */
QColor DataSourcesStatusWidget::colorFromEvent(const sensor_status::Event& evt)
{
    QColor color;

    if (evt.type == sensor_status::Event::Type::StatusChangeIntermediate ||
        evt.type == sensor_status::Event::Type::StatusChangeFinal)
        color = DataSourcesStatusWidget::colorFromSensorStatus(evt.status_change->second);
    else if (evt.type == sensor_status::Event::Type::MissingInformation)
        color = DataSourcesStatusWidget::ColorStatusInfoMissing;
    else if (evt.type == sensor_status::Event::Type::NoCAT063Data)
        color = DataSourcesStatusWidget::ColorNoData;
    
    return color;
}

/**
 */
QFont DataSourcesStatusWidget::fontFromEvent(const sensor_status::Event& evt)
{
    QFont f;
    f.setBold(evt.type == sensor_status::Event::Type::StatusChangeFinal);

    return f;
}

/**
 */
void DataSourcesStatusWidget::reset()
{
    tracker_.reset();

    updateContent();
}

/**
 */
QStringList DataSourcesStatusWidget::getCustomColumnHeaders() const 
{ 
    QStringList headers;
    headers << "Tracker Status";
    headers << "Last Update";

    return headers;
}

/**
 */
bool DataSourcesStatusWidget::showDSType(const std::string& ds_type_name) const
{
    //filter some ds types
    if (dbContent::DataSourceBase::dsTypeFromString(ds_type_name) == dbContent::DataSourceType::Tracker ||
        dbContent::DataSourceBase::dsTypeFromString(ds_type_name) == dbContent::DataSourceType::RefTraj ||
        dbContent::DataSourceBase::dsTypeFromString(ds_type_name) == dbContent::DataSourceType::Other)
        return false;

    return true;
}

/**
 */
bool DataSourcesStatusWidget::showDS(unsigned int ds_id) const
{
    //no live mode => show nothing
    if (ctx_man_.appState().appMode() != AppMode::LiveRunning)
        return false;

    //no active tracker or no sensor status for active tracker yet => show nothing
    if (!tracker_.hasActiveTrackerStates())
        return false;

    //is db data source? => always show
    if (ctx_man_.hasDataSource(ds_id))
        return true;

    //config data source => has sensor status for given ds id?
    return tracker_.activeTrackerStates().states.count(ds_id) > 0;
}

/**
 */
DataSourceItemBase* DataSourcesStatusWidget::createDSItem(DataSourcesWidgetItemBase* parent)
{
    return new DataSourceStatusItem(this, parent);
}

/**
 */
void DataSourcesStatusWidget::showLastUpdates(bool show)
{
    ctx_man_.sensorConfig().sensor_status_show_last_updates = show;
    showColumn(LastUpdateColumn, show);
}

