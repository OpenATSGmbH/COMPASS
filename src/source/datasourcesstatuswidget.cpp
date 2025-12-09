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
    setTextAlignment(DataSourcesStatusWidget::LastUpdateColumn, Qt::AlignHCenter);
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

    //no sensor status available?
    if (!status_widget_->hasActiveTrackerSensorState(dsID()))
    {
        showStatus(DataSourcesStatusWidget::InfoStatusUnknown);
        return;
    }

    const auto& sensor_state = status_widget_->activeTrackerSensorState(dsID());

    //should be init at this point
    traced_assert(!sensor_state.isFresh());

    auto display_info = DataSourcesStatusWidget::displayInfoFromSensorStatus(sensor_state.status);
    showStatus(display_info);

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
std::pair<std::string, QColor> DataSourcesStatusWidget::Event::displayInfo(bool tracker_info,
                                                                           bool sensor_info) const
{
    std::string txt;

    if (!ts.is_not_a_date_time())
    {
        txt += Utils::Time::toTimeString(ts, false) + " ";
    }

    auto& ds_man = COMPASS::instance().dataSourceManager();

    if (tracker_info && tracker_key.has_value())
    {
        traced_assert(ds_man.hasDBDataSource(tracker_key.value().first));
        txt += ds_man.dbDataSource(tracker_key.value().first).name() + " ";
        txt += "L" + std::to_string(tracker_key.value().second + 1) + " ";
    }

    if (sensor_info && sensor_ds_id.has_value())
    {
        if (ds_man.hasDBDataSource(sensor_ds_id.value()))
            txt += ds_man.dbDataSource(sensor_ds_id.value()).name() + " ";
        else if (ds_man.hasConfigDataSource(sensor_ds_id.value()))
            txt += ds_man.configDataSource(sensor_ds_id.value()).name() + " ";
        else
            txt += std::to_string(sensor_ds_id.value()) + " ";
    }

    QColor color;

    if (type == Type::StatusChange)
    {
        traced_assert(status_change.has_value());

        auto state0_str = DataSourcesStatusWidget::stringFromSensorStatus(status_change->first);
        auto state1_di  = DataSourcesStatusWidget::displayInfoFromSensorStatus(status_change->second);

        txt += state0_str + " \u2192 " + state1_di.first + " ";

        color = state1_di.second;
    }
    else if (type == Type::MissingInformation)
    {
        txt += DataSourcesStatusWidget::InfoStatusInfoMissing.first + " ";
        color = DataSourcesStatusWidget::InfoStatusInfoMissing.second;
    }
    else if (type == Type::NoCAT063Data)
    {
        txt += DataSourcesStatusWidget::InfoNoData.first + " ";
        color = DataSourcesStatusWidget::InfoNoData.second;
    }
    
    return std::make_pair(txt, color);
}

/***********************************************************************************************************
 * DataSourcesStatusWidget
 ***********************************************************************************************************/

const std::pair<std::string, QColor> DataSourcesStatusWidget::InfoConOperational    = { "Operational"               , Qt::darkGreen       };
const std::pair<std::string, QColor> DataSourcesStatusWidget::InfoConDegraded       = { "Degraded"                  , QColor(255, 165, 0) };
const std::pair<std::string, QColor> DataSourcesStatusWidget::InfoConInitialization = { "Initializing"              , Qt::blue            };
const std::pair<std::string, QColor> DataSourcesStatusWidget::InfoConDisconnected   = { "Not Connected"             , Qt::red             };
const std::pair<std::string, QColor> DataSourcesStatusWidget::InfoCoasting          = { "Coasting"                  , QColor(255, 165, 0) };
const std::pair<std::string, QColor> DataSourcesStatusWidget::InfoStatusUnknown     = { "Status Unknown"            , Qt::darkGray        };
const std::pair<std::string, QColor> DataSourcesStatusWidget::InfoStatusInfoMissing = { "Status information missing", Qt::red             };
const std::pair<std::string, QColor> DataSourcesStatusWidget::InfoNoData            = { "No data available"         , Qt::darkGray        };

const int DataSourcesStatusWidget::StatusColumn     = 1;
const int DataSourcesStatusWidget::LastUpdateColumn = 2;

const int DataSourcesStatusWidget::MaximumEventCount = 1000;

/**
 */
DataSourcesStatusWidget::DataSourcesStatusWidget(DataSourceManager& ds_man, 
                                                 DBContentManager& dbcontent_man,
                                                 bool init_ui)
:   DataSourcesWidgetBase(ds_man, Source::All, false, false)
,   dbcontent_man_(dbcontent_man)
{
    if (init_ui)
        init();

    showLastUpdates(ds_man.config().sensor_status_show_last_updates_);

    connect(&dbcontent_man_, &DBContentManager::loadedDataSignal, this, &DataSourcesStatusWidget::dataLoaded, Qt::QueuedConnection);
}

/**
 */
DataSourcesStatusWidget::~DataSourcesStatusWidget() = default;

/**
 */
DataSourcesStatusWidget::SensorStatus DataSourcesStatusWidget::sensorStatusFromCon(unsigned char con)
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
}

/**
 */
std::pair<std::string, QColor> DataSourcesStatusWidget::displayInfoFromSensorStatus(SensorStatus status)
{
    switch (status)
    {
        case SensorStatus::ConOperational:
            return DataSourcesStatusWidget::InfoConOperational;
        case SensorStatus::ConDegraded:
            return DataSourcesStatusWidget::InfoConDegraded;
        case SensorStatus::ConInitialization:
            return DataSourcesStatusWidget::InfoConInitialization;
        case SensorStatus::ConDisconnected:
            return DataSourcesStatusWidget::InfoConDisconnected;
        case SensorStatus::Fresh:
            return DataSourcesStatusWidget::InfoStatusUnknown;
        case SensorStatus::Coasting:
            return DataSourcesStatusWidget::InfoCoasting;
    }
    return DataSourcesStatusWidget::InfoStatusUnknown;
}

/**
 */
std::string DataSourcesStatusWidget::stringFromSensorStatus(SensorStatus status)
{
    return displayInfoFromSensorStatus(status).first;
}

/**
 */
QColor DataSourcesStatusWidget::colorFromSensorStatus(SensorStatus status)
{
    return displayInfoFromSensorStatus(status).second;
}

/**
 */
void DataSourcesStatusWidget::addActionsToConfigMenu(QMenu* menu)
{
    auto last_update_action = menu->addAction("Show Last Sensor Updates");
    last_update_action->setCheckable(true);
    last_update_action->setChecked(ds_man_.config().sensor_status_show_last_updates_);

    connect(last_update_action, &QAction::toggled, [ this ] (bool ok) { this->showLastUpdates(ok); });
}

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
const DataSourcesStatusWidget::Event* DataSourcesStatusWidget::lastTrackerEvent() const
{
    if  (!hasActiveTrackerStates())
        return nullptr;

    const auto& ats = activeTrackerStates();
    if (ats.events.empty())
        return nullptr;

    const auto& evt = ats.events.back();

    return &evt;
}

/**
 */
bool DataSourcesStatusWidget::hasActiveTrackerStates() const
{
    return (active_tracker_.has_value() && tracker_states_.count(active_tracker_.value()) > 0);
}

/**
 */
bool DataSourcesStatusWidget::hasActiveTrackerSensorState(unsigned int ds_id) const
{
    return (active_tracker_.has_value() && 
            tracker_states_.count(active_tracker_.value()) > 0 && 
            tracker_states_.at(active_tracker_.value()).states.count(ds_id) > 0);
}

/**
 */
const DataSourcesStatusWidget::TrackerStates& DataSourcesStatusWidget::activeTrackerStates() const
{
    traced_assert(hasActiveTrackerStates());
    return tracker_states_.at(active_tracker_.value());
}

/**
 */
const DataSourcesStatusWidget::SensorState& DataSourcesStatusWidget::activeTrackerSensorState(unsigned int ds_id) const
{
    traced_assert(hasActiveTrackerSensorState(ds_id));
    return tracker_states_.at(active_tracker_.value()).states.at(ds_id);
}

/**
 */
void DataSourcesStatusWidget::reset()
{
    tracker_states_.clear();

    updateContent();
}

/**
 */
QStringList DataSourcesStatusWidget::getCustomColumnHeaders() const 
{ 
    QStringList headers;
    headers << "Status";
    headers << "Last Update";

    return headers;
}

/**
 */
void DataSourcesStatusWidget::logEvent(TrackerStates* tracker_states,
                                       Event::Type type,
                                       const boost::posix_time::ptime& ts,
                                       const boost::optional<TrackerKey>& tracker_key,
                                       const boost::optional<unsigned int>& sensor_ds_id,
                                       const boost::optional<StatusChange>& status_change,
                                       const std::string& info )
{
    traced_assert(type != Event::Type::StatusChange || status_change.has_value());

    Event evt;
    evt.type            = type;
    evt.ts              = ts.is_not_a_date_time() ? Utils::Time::currentUTCTime() : ts;
    evt.tracker_key     = tracker_key;
    evt.sensor_ds_id    = sensor_ds_id;
    evt.status_change   = status_change;
    evt.info            = info;

    bool signal_changes = false;

    if (tracker_states)
    {
        //tracker specific event => add to tracker states
        traced_assert(tracker_key.has_value());

        addEvent(*tracker_states, evt);

        signal_changes = active_tracker_.has_value() && (active_tracker_.value() == tracker_key.value());
    }
    else
    {
        //general event => distribute to all tracker states
        traced_assert(type != Event::Type::StatusChange);
        traced_assert(!tracker_key.has_value());
        traced_assert(!sensor_ds_id.has_value());
        traced_assert(!status_change.has_value());

        for (auto& ts : tracker_states_)
            addEvent(ts.second, evt);

        signal_changes = true;
    }
    
    if (signal_changes)
        emit eventAdded();
}

/**
 */
void DataSourcesStatusWidget::addEvent(TrackerStates& tracker_states,
                                       const Event& evt)
{
    tracker_states.events.push_back(evt);

    //remove old events if limit is reached
    if (tracker_states.events.size() > MaximumEventCount)
        tracker_states.events.pop_front();
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
        bool has_ds_id_var = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_ds_id_    );
        bool has_line_var  = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_line_id_  );

        auto var_ts        = has_ts_var    ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_timestamp_) : nullptr;
        auto var_ds        = has_ds_id_var ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_ds_id_    ) : nullptr;
        auto var_line      = has_line_var  ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_line_id_  ) : nullptr;

        bool has_ts        = var_ts   && buffer->hasAnyPropertyNamed(var_ts->name());
        bool has_ds_id     = var_ds   && buffer->hasAnyPropertyNamed(var_ds->name());
        bool has_line      = var_line && buffer->hasAnyPropertyNamed(var_line->name());
        bool has_con       = buffer->hasAnyPropertyNamed(DBContent::var_cat063_con_.name());
        bool has_sen_sac   = buffer->hasAnyPropertyNamed(DBContent::var_cat063_sensor_sac_.name());
        bool has_sen_sic   = buffer->hasAnyPropertyNamed(DBContent::var_cat063_sensor_sic_.name());

        traced_assert(has_ts && has_ds_id && has_line && has_con && has_sen_sac && has_sen_sic);

        auto& ts_vec      = buffer->get<boost::posix_time::ptime>(var_ts->name());
        auto& ds_id_vec   = buffer->get<unsigned int>(var_ds->name());
        auto& line_vec    = buffer->get<unsigned int>(var_line->name());
        auto& con_vec     = buffer->get<unsigned char>(DBContent::var_cat063_con_.name());
        auto& sen_sac_vec = buffer->get<unsigned char>(DBContent::var_cat063_sensor_sac_.name());
        auto& sen_sic_vec = buffer->get<unsigned char>(DBContent::var_cat063_sensor_sic_.name());

        // auto ts0 = ts_vec.get(0);
        // auto ts1 = ts_vec.get(buffer->size() - 1);
        // loginf << "TS NUM RANGE " << Utils::Time::toString(ts0) << " - " << Utils::Time::toString(ts1);

        auto ts_cur = Utils::Time::currentUTCTime();

        bool missing_info = false;

        boost::posix_time::ptime last_ts;

        size_t n = buffer->size() - 1;
        for (size_t i = n; i-- > 0; )
        {
            //ds + line never expected to be null
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

            //!timestamps are expected to be ordered!
            traced_assert(last_ts.is_not_a_date_time() || ts <= last_ts);
            
            //break if scan period has been parsed
            if (ts <= ts_cur && Utils::Time::partialSeconds(ts_cur - ts) > ds_man_.config().sensor_status_max_secs_scan_)
                break;

            auto ds_id   = ds_id_vec.get(i);
            auto line_id = line_vec.get(i);

            TrackerKey key(ds_id, line_id);

            unsigned int sensor_id = (unsigned int)sen_sac_vec.get(i) * 255 + sen_sic_vec.get(i);

            auto& tracker_states = tracker_states_[ key ];
            auto& sen_state = tracker_states.states[ sensor_id ];

            if (sen_state.isFresh() ||
                ts >= sen_state.ts_con)
            {
                auto old_status        = sen_state.status;
                bool old_status_is_con = sen_state.isCON();

                sen_state.ts_con = ts;
                sen_state.status = DataSourcesStatusWidget::sensorStatusFromCon(con_vec.get(i));

                //store newest status update
                if (tracker_states.last_update_ts.is_not_a_date_time() || ts > tracker_states.last_update_ts)
                    tracker_states.last_update_ts = ts;

                //log regain of con status
                if (!old_status_is_con)
                {
                    logEvent(&tracker_states,
                             Event::Type::StatusChange,
                             ts, 
                             key, 
                             sensor_id,
                             StatusChange(old_status, sen_state.status));
                }
            }

            last_ts = ts;
        }

        //log missing information
        if (missing_info)
            logEvent(nullptr, Event::Type::MissingInformation, {});
    }
    else
    {
        //no data in buffer
        logEvent(nullptr, Event::Type::NoCAT063Data, {});
    }

    //update status from new information
    updateSensorStatus();

    //update content to new status 
    updateContent();

    last_refresh_ts_ = Utils::Time::currentUTCTime();

    emit refreshed();
}

/**
 */
void DataSourcesStatusWidget::updateSensorStatus()
{
    auto ts_cur = Utils::Time::currentUTCTime();

    for (auto& tracker_status : tracker_states_)
    {
        for (auto& sen_stat : tracker_status.second.states)
        {
            //state should be init at this point
            traced_assert(!sen_stat.second.isFresh());

            //if not yet coasting check if coasting now
            if (!sen_stat.second.isCoasting() &&
                sen_stat.second.ts_con <= ts_cur && 
                Utils::Time::partialSeconds(ts_cur - sen_stat.second.ts_con) > ds_man_.config().sensor_status_max_secs_valid_)
            {
                auto old_status = sen_stat.second.status;

                //set sensor status to coasting
                sen_stat.second.status = SensorStatus::Coasting;

                logEvent(&tracker_status.second,
                         Event::Type::StatusChange, 
                         ts_cur, 
                         tracker_status.first,
                         sen_stat.first,
                         StatusChange(old_status, sen_stat.second.status));
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
    auto it = tracker_states_.find(active_tracker_.value());
    if (it == tracker_states_.end())
        return false;

    //is db data source? => always show
    if (ds_man_.hasDBDataSource(ds_id))
        return true;

    //config data source => has sensor status for given ds id?
    return it->second.states.count(ds_id) > 0;
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
    ds_man_.config().sensor_status_show_last_updates_ = show;
    showColumn(LastUpdateColumn, show);
}
