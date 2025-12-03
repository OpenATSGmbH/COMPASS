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
#include "datasourcebase.h"
#include "datasourcemanager.h"

#include "dbcontent.h"
#include "dbcontentmanager.h"

#include "compass.h"

#include "timeconv.h"

#include <QLabel>

/***********************************************************************************************************
 * DataSourceStatusItem
 ***********************************************************************************************************/

const std::pair<std::string, QColor> DataSourceStatusItem::InfoNoData            = { "No Data", Qt::gray };
const std::pair<std::string, QColor> DataSourceStatusItem::InfoCoasting          = { "Coasting", Qt::gray };
const std::pair<std::string, QColor> DataSourceStatusItem::InfoNoStatus          = { "No Status", Qt::gray };

const std::pair<std::string, QColor> DataSourceStatusItem::InfoConOperational    = { "Operational", Qt::green };
const std::pair<std::string, QColor> DataSourceStatusItem::InfoConDegraded       = { "Degraded", QColor(255, 165, 0) };
const std::pair<std::string, QColor> DataSourceStatusItem::InfoConInitialization = { "Initializing", Qt::blue };
const std::pair<std::string, QColor> DataSourceStatusItem::InfoConDisconnected   = { "Not Connected", Qt::red };
const std::pair<std::string, QColor> DataSourceStatusItem::InfoConUnknown        = { "Unknown CON", Qt::red };

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
    status_label_ = new QLabel;

    auto f = status_label_->font();
    f.setBold(true);

    status_label_->setFont(f);

    setItemWidget(DataSourcesStatusWidget::StatusColumn, status_label_);
}

/**
 */
void DataSourceStatusItem::updateContentChanges_impl()
{
    //nothing to do yet
}

/**
 */
void DataSourceStatusItem::updateContent_impl()
{
    if (!status_widget_->hasCurrentTrackerStatus(dsID()))
    {
        //set to no data (should never be displayed actually)
        showStatus(InfoNoData);
        return;
    }

    const auto& status = status_widget_->currentTrackerStatus(dsID());

    if (status.con_valid)
    {
        //set to con state
        if (status.con == 0)
            showStatus(InfoConOperational);
        else if (status.con == 1)
            showStatus(InfoConDegraded);
        else if (status.con == 2)
            showStatus(InfoConInitialization);
        else if (status.con == 3)
            showStatus(InfoConDisconnected);
        else
            showStatus(InfoConUnknown);
    }
    else if (status.coasting)
    {
        //set to coasting
        showStatus(InfoCoasting);
    }
    else
    {
        //set to no status (little bit weird)
        showStatus(InfoNoStatus);
    }

    std::string info;
    if (!status.ts.is_not_a_date_time())
        info = "Last update: " + Utils::Time::toString(status.ts);

    setText(DataSourcesStatusWidget::InfoColumn, QString::fromStdString(info));
}

/**
 */
void DataSourceStatusItem::showStatus(const std::string& msg, const QColor& color)
{
    status_label_->setText(QString::fromStdString(msg));
    status_label_->setStyleSheet("color: " + color.name() + ";");   // hex color
}

/**
 */
void DataSourceStatusItem::showStatus(const std::pair<std::string, QColor>& info)
{
    showStatus(info.first, info.second);
}

/***********************************************************************************************************
 * DataSourcesStatusWidget::Event
 ***********************************************************************************************************/

/**
 */
std::string DataSourcesStatusWidget::Event::displayInfo() const
{
    std::string txt;

    if (!ts.is_not_a_date_time())
    {
        txt += Utils::Time::toString(ts) + " ";
    }

    auto& ds_man = COMPASS::instance().dataSourceManager();

    if (tracker_ds_id.has_value())
    {
        txt += "Tracker ";

        traced_assert(ds_man.hasDBDataSource(tracker_ds_id.value()));
        txt += ds_man.dbDataSource(tracker_ds_id.value()).name() + " ";

        if (tracker_line_id.has_value())
        {
            txt += "L" + std::to_string(tracker_line_id.value() + 1) + " ";
        }
    }

    if (sensor_ds_id.has_value())
    {
        txt += "Sensor ";

        traced_assert(ds_man.hasDBDataSource(sensor_ds_id.value()));
        txt += ds_man.dbDataSource(sensor_ds_id.value()).name() + " ";
    }

    switch (type)
    {
        case Type::FirstStatus:
            txt += "First status obtained ";
        case Type::GoingIntoCoasting:
            txt += "Going into coasting ";
        case Type::RegainedStatus:
            txt += "Regained status ";
        case Type::MissingInformation:
            txt += "Status information missing ";
        case Type::NoData:
            txt += "No data available ";
    }
    
    return txt;
}

/***********************************************************************************************************
 * DataSourcesStatusWidget
 ***********************************************************************************************************/

const int DataSourcesStatusWidget::StatusColumn = 1;
const int DataSourcesStatusWidget::InfoColumn   = 2;

/**
 */
DataSourcesStatusWidget::DataSourcesStatusWidget(DataSourceManager& ds_man, 
                                                 DBContentManager& dbcontent_man,
                                                 bool init_ui)
:   DataSourcesWidgetBase(ds_man, false, false)
,   dbcontent_man_(dbcontent_man)
{
    if (init_ui)
        init();

    connect(&dbcontent_man_, &DBContentManager::loadedDataSignal, this, &DataSourcesStatusWidget::dataLoaded);
}

/**
 */
DataSourcesStatusWidget::~DataSourcesStatusWidget() = default;

/**
 */
void DataSourcesStatusWidget::setActiveTracker(unsigned int ds_id, unsigned int line_id)
{
    if (active_tracker_.has_value() && 
        active_tracker_->first == ds_id && 
        active_tracker_->second == line_id)
        return;

    traced_assert(ds_man_.hasDBDataSource(ds_id));
    const auto& ds = ds_man_.dbDataSource(ds_id);

    loginf << "setting active tracker to " << ds.name() << " L" << line_id + 1;

    //change active tracker
    active_tracker_ = TrackerKey(ds_id, line_id);

    updateContent();
}

/**
 */
void DataSourcesStatusWidget::unsetActiveTracker()
{
    if (!active_tracker_.has_value())
        return;

    loginf << "unsetting active tracker";

    active_tracker_.reset();

    updateContent();
}

/**
 */
bool DataSourcesStatusWidget::hasCurrentTrackerStatus() const
{
    return (active_tracker_.has_value() && sensor_status_.count(active_tracker_.value()) > 0);
}

/**
 */
bool DataSourcesStatusWidget::hasCurrentTrackerStatus(unsigned int ds_id) const
{
    return (active_tracker_.has_value() && 
            sensor_status_.count(active_tracker_.value()) > 0 && 
            sensor_status_.at(active_tracker_.value()).count(ds_id) > 0);
}

/**
 */
const DataSourcesStatusWidget::SensorStatusMap& DataSourcesStatusWidget::currentTrackerStatus() const
{
    traced_assert(hasCurrentTrackerStatus());
    return sensor_status_.at(active_tracker_.value());
}

/**
 */
const DataSourcesStatusWidget::SensorStatus& DataSourcesStatusWidget::currentTrackerStatus(unsigned int ds_id) const
{
    traced_assert(hasCurrentTrackerStatus(ds_id));
    return sensor_status_.at(active_tracker_.value()).at(ds_id);
}

/**
 */
void DataSourcesStatusWidget::reset()
{
    sensor_status_.clear();
    events_.clear();

    updateContent();
}

/**
 */
QStringList DataSourcesStatusWidget::getCustomColumnHeaders() const 
{ 
    QStringList headers;
    headers << "Status";
    headers << "Info";

    return headers;
}

namespace 
{
    DataSourcesStatusWidget::Event::Severity severityFromType(DataSourcesStatusWidget::Event::Type type)
    {
        switch(type) 
        {
            case DataSourcesStatusWidget::Event::Type::FirstStatus:
                return DataSourcesStatusWidget::Event::Severity::Info;
            case DataSourcesStatusWidget::Event::Type::GoingIntoCoasting:
                return DataSourcesStatusWidget::Event::Severity::Warning;
            case DataSourcesStatusWidget::Event::Type::RegainedStatus:
                return DataSourcesStatusWidget::Event::Severity::Info;
            case DataSourcesStatusWidget::Event::Type::MissingInformation:
                return DataSourcesStatusWidget::Event::Severity::Error;
            case DataSourcesStatusWidget::Event::Type::NoData:
                return DataSourcesStatusWidget::Event::Severity::Error;
        }
        return DataSourcesStatusWidget::Event::Severity::Info;
    }
}

/**
 */
void DataSourcesStatusWidget::logEvent(Event::Type type,
                                       const boost::posix_time::ptime& ts,
                                       const boost::optional<unsigned int>& tracker_ds_id,
                                       const boost::optional<unsigned int>& tracker_line_id,
                                       const boost::optional<unsigned int>& sensor_ds_id,
                                       const std::string& info)
{


    Event evt;
    evt.type            = type;
    evt.severity        = severityFromType(type);
    evt.ts              = ts.is_not_a_date_time() ? Utils::Time::currentUTCTime() : ts;
    evt.tracker_ds_id   = tracker_ds_id;
    evt.tracker_line_id = tracker_line_id;
    evt.sensor_ds_id    = sensor_ds_id;
    evt.info            = info;

    events_.push_back(evt);

    emit eventAdded();
}

/**
 */
void DataSourcesStatusWidget::dataLoaded()
{
    if (COMPASS::instance().appMode() != AppMode::LiveRunning)
        return;

    const std::string DBCType = "CAT063";

    //update sensor status from data
    auto data = dbcontent_man_.data();
    auto it = data.find(DBCType);

    if (it != data.end() && it->second->size() > 0)
    {
        auto buffer = it->second;

        bool has_ts_var    = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_timestamp_);
        bool has_ds_id_var = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_ds_id_);
        bool has_line_var  = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_line_id_);
        auto var_ts        = has_ts_var    ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_timestamp_) : nullptr;
        auto var_ds        = has_ds_id_var ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_ds_id_    ) : nullptr;
        auto var_line      = has_line_var  ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_line_id_  ) : nullptr;

        bool has_ts_id     = var_ts   && buffer->hasAnyPropertyNamed(var_ts->name());
        bool has_ds_id     = var_ds   && buffer->hasAnyPropertyNamed(var_ds->name());
        bool has_line      = var_line && buffer->hasAnyPropertyNamed(var_line->name());
        bool has_con       = buffer->hasAnyPropertyNamed(DBContent::var_cat063_con_.name());
        bool has_sen_sac   = buffer->hasAnyPropertyNamed(DBContent::var_cat063_sensor_sac_.name());
        bool has_sen_sic   = buffer->hasAnyPropertyNamed(DBContent::var_cat063_sensor_sic_.name());

        traced_assert(has_ts_id && has_ds_id && has_line && has_con && has_sen_sac && has_sen_sic);

        auto& ts_vec      = buffer->get<boost::posix_time::ptime>(var_ts->name());
        auto& ds_id_vec   = buffer->get<unsigned int>(var_ds->name());
        auto& line_vec    = buffer->get<unsigned int>(var_line->name());
        auto& con_vec     = buffer->get<unsigned char>(DBContent::var_cat063_con_.name());
        auto& sen_sac_vec = buffer->get<unsigned char>(DBContent::var_cat063_sensor_sac_.name());
        auto& sen_sic_vec = buffer->get<unsigned char>(DBContent::var_cat063_sensor_sic_.name());

        auto ts_cur = Utils::Time::currentUTCTime();

        bool missing_info = false;

        size_t n = buffer->size() - 1;
        for (size_t i = n; i-- > 0; )
        {
            if (ts_vec.isNull(i)      ||
                con_vec.isNull(i)     ||
                sen_sac_vec.isNull(i) ||
                sen_sic_vec.isNull(i))
            {
                //information missing
                missing_info = true;
                continue;
            }

            auto& ts = ts_vec.getRef(i);
            
            //break if scan period has been elapsed
            if (ts <= ts_cur && Utils::Time::partialSeconds(ts_cur - ts) > ds_man_.config().sensor_status_max_secs_scan_)
                break;

            auto ds_id   = ds_id_vec.get(i);
            auto line_id = line_vec.get(i);

            TrackerKey key(ds_id, line_id);

            unsigned int sensor_id = (unsigned int)sen_sac_vec.get(i) * 255 + sen_sic_vec.get(i);

            auto& sen_stat_map = sensor_status_[ key ];
            bool init = sen_stat_map.count(sensor_id) > 0;

            auto& sen_stat = sen_stat_map[ sensor_id ];
            if (!sen_stat.con_valid || ts >= sen_stat.ts)
            {
                //log no status -> status
                if (!sen_stat.con_valid)
                    logEvent(init ? Event::Type::FirstStatus : Event::Type::RegainedStatus, ts, ds_id, line_id, sensor_id);

                sen_stat.ts        = ts;
                sen_stat.con       = con_vec.get(i);
                sen_stat.con_valid = true;
                sen_stat.coasting  = false;
            }
        }

        //log missing information
        if (missing_info)
            logEvent(Event::Type::MissingInformation, {});
    }
    else
    {
        //no data in buffer
        logEvent(Event::Type::NoData, {});
    }

    //update status from new information
    updateSensorStatus();

    //update content to new status 
    updateContent();
}

/**
 */
void DataSourcesStatusWidget::updateSensorStatus()
{
    auto ts_cur = Utils::Time::currentUTCTime();

    for (auto& sen_stat_map : sensor_status_)
    {
        for (auto& sen_stat : sen_stat_map.second)
        {
            if (sen_stat.second.con_valid &&
                sen_stat.second.ts <= ts_cur && 
                Utils::Time::partialSeconds(ts_cur - sen_stat.second.ts) > ds_man_.config().sensor_status_max_secs_coast_)
            {
                //set sensor to coasting
                sen_stat.second.con_valid = false;
                sen_stat.second.coasting  = true;

                logEvent(Event::Type::GoingIntoCoasting, 
                         ts_cur, 
                         sen_stat_map.first.first,
                         sen_stat_map.first.second,
                         sen_stat.first);
            }
        }
    }
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
    if (COMPASS::instance().appMode() != AppMode::LiveRunning)
        return false;

    //no active tracker => show nothing
    if (!active_tracker_.has_value())
        return false;

    //no sensor status for active tracker yet => show nothing
    auto it = sensor_status_.find(active_tracker_.value());
    if (it == sensor_status_.end())
        return false;

    //has sensor status for given ds id?
    return it->second.count(ds_id) > 0;
}

/**
 */
bool DataSourcesStatusWidget::showDSLine(unsigned int ds_id, unsigned int ds_line) const
{
    return true;
}

/**
 */
DataSourceItemBase* DataSourcesStatusWidget::createDSItem(DataSourcesWidgetItemBase* parent)
{
    return new DataSourceStatusItem(this, parent);
}
