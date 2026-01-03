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

#define DEBUG_SENSOR_STATUS_TIMING

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

    //no sensor status available?
    if (!status_widget_->hasActiveTrackerSensorState(dsID()))
    {
        auto txt = sensor_status::displayStringFromSensorStatus(sensor_status::SensorStatus::Unknown);
        auto col = DataSourcesStatusWidget::colorFromSensorStatus(sensor_status::SensorStatus::Unknown);

        showStatus(txt, col);

        return;
    }

    const auto& sensor_state = status_widget_->activeTrackerSensorState(dsID());

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
 * DataSourcesStatusWidget::SensorStatusCache
 ***********************************************************************************************************/

/**
 */
void DataSourcesStatusWidget::SensorStatusCache::initForParse()
{
    last_update_found_in_scan = false;

    //backup last update state
    state_last_update = state_current;

    //reset last item state
    state_last_item = {};
}

/***********************************************************************************************************
 * DataSourcesStatusWidget::TrackerStates
 ***********************************************************************************************************/

/**
 */
void DataSourcesStatusWidget::TrackerStates::initForParse()
{
    //clear event candidates
    candidates.clear();

    for (auto& sen_stat : states)
        sen_stat.second.initForParse();
}

/**
 */
void DataSourcesStatusWidget::TrackerStates::clearOutdatedEvents(double max_event_age)
{
    if (added_events.empty())
        return;

    auto ts_cur = Utils::Time::currentUTCTime();

    //events are sorted time first => find range older than scan period
    auto it0 = added_events.begin();
    auto it1 = added_events.begin();
    size_t n = 0;
    for (; it1 != added_events.end(); ++it1, ++n)
        if (it1->ts > ts_cur || Utils::Time::partialSeconds(ts_cur - it1->ts) <= max_event_age)
            break;

    //remove too old range
    if (it0 != it1)
    {
        loginf << "removing " << n << " old events from duplicate check, " << added_events.size() - n << " remaining";
        added_events.erase(it0, it1);
    }
}

/**
 */
bool DataSourcesStatusWidget::TrackerStates::queueCandidateEvents()
{
    if (candidates.empty())
        return false;

    for (const auto& evt : candidates)
    {
        events.addEvent(evt);
    }

    candidates.clear();

    return true;
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
DataSourcesStatusWidget::DataSourcesStatusWidget(DataSourceManager& ds_man, 
                                                 DBContentManager& dbcontent_man,
                                                 bool init_ui)
:   DataSourcesWidgetBase(ds_man, Source::All, false, false)
,   dbcontent_man_(dbcontent_man)
,   general_events_(ds_man.config().sensor_status_max_event_buf_size_)
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
    active_tracker_ = sensor_status::TrackerKey(ds_id, line_id);

    updateContent();

    emit activeTrackerChanged();
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

    emit activeTrackerChanged();
}

/**
 */
const sensor_status::EventQueue& DataSourcesStatusWidget::currentEventQueue() const
{
    if (!hasActiveTrackerStates())
        return general_events_;

    return activeTrackerStates().events;
}

/**
 */
sensor_status::EventQueue& DataSourcesStatusWidget::currentEventQueue()
{
    if (!hasActiveTrackerStates())
        return general_events_;

    return tracker_states_.at(active_tracker_.value()).events;
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
const sensor_status::SensorState& DataSourcesStatusWidget::activeTrackerSensorState(unsigned int ds_id) const
{
    traced_assert(hasActiveTrackerSensorState(ds_id));
    return tracker_states_.at(active_tracker_.value()).states.at(ds_id).state_current;
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
    headers << "Tracker Status";
    headers << "Last Update";

    return headers;
}

/**
 */
void DataSourcesStatusWidget::logEvent(TrackerStates* tracker_states,
                                       sensor_status::Event::Type type,
                                       const boost::posix_time::ptime& ts,
                                       const boost::optional<sensor_status::TrackerKey>& tracker_key,
                                       const boost::optional<unsigned int>& sensor_ds_id,
                                       const boost::optional<sensor_status::StatusChange>& status_change)
{
    traced_assert((type != sensor_status::Event::Type::StatusChangeIntermediate &&
                   type != sensor_status::Event::Type::StatusChangeFinal) || status_change.has_value());

    sensor_status::Event evt;
    evt.type            = type;
    evt.ts              = ts.is_not_a_date_time() ? Utils::Time::currentUTCTime() : ts;
    evt.tracker_key     = tracker_key;
    evt.sensor_ds_id    = sensor_ds_id;
    evt.status_change   = status_change;

    if (tracker_states)
    {
        //tracker specific event => add to tracker states
        traced_assert(tracker_key.has_value());

        addEvent(*tracker_states, evt, true);

        //signal is sent later on
    }
    else
    {
        //general event => distribute to all existing tracker event queues
        traced_assert(type != sensor_status::Event::Type::StatusChangeIntermediate &&
                      type != sensor_status::Event::Type::StatusChangeFinal);

        traced_assert(!tracker_key.has_value());
        traced_assert(!sensor_ds_id.has_value());
        traced_assert(!status_change.has_value());

        for (auto& ts : tracker_states_)
            addEvent(ts.second, evt, false);

        //also add to general event queue
        addEvent(general_events_, evt);

        //immediately send signal for general events
        emit eventsAdded();
    }  
}

/**
 */
void DataSourcesStatusWidget::addEvent(TrackerStates& tracker_states,
                                       const sensor_status::Event& evt,
                                       bool check_duplicates)
{
    //check duplicates?
    if (check_duplicates)
    {
        //event already added
        if (tracker_states.added_events.count(evt) > 0)
            return;

        //remember logged event
        tracker_states.added_events.insert(evt);
        tracker_states.candidates.insert(evt);

        return;
    } 

    //add to queue
    addEvent(tracker_states.events, evt);
}

/**
 */
void DataSourcesStatusWidget::addEvent(sensor_status::EventQueue& event_queue,
                                       const sensor_status::Event& evt)
{
    event_queue.addEvent(evt);
}

/**
 */
void DataSourcesStatusWidget::initTrackerStates(const sensor_status::TrackerKey& key,
                                                TrackerStates& tracker_states) const
{
    //duplicate general event queue
    tracker_states.events = general_events_;
}

/**
 */
void DataSourcesStatusWidget::dataLoaded()
{
    if (COMPASS::instance().appMode() != AppMode::LiveRunning)
        return;

#ifdef DEBUG_SENSOR_STATUS_TIMING
    auto time0 = boost::posix_time::microsec_clock::local_time();
#endif

    const std::string DBCType = "CAT063";

    //init sensor states for parsing new data
    initSensorStatesForParse();

    //determine new sensor status from data
    auto data = dbcontent_man_.data();
    auto it = data.find(DBCType);

    if (it != data.end() && it->second->size() > 0)
    {
        auto buffer = it->second;

        bool has_rn_var    = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_rec_num_  );
        bool has_ts_var    = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_timestamp_);
        bool has_ds_id_var = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_ds_id_    );
        bool has_line_var  = dbcontent_man_.metaCanGetVariable(DBCType, DBContent::meta_var_line_id_  );

        auto var_rn        = has_rn_var    ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_rec_num_  ) : nullptr;
        auto var_ts        = has_ts_var    ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_timestamp_) : nullptr;
        auto var_ds        = has_ds_id_var ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_ds_id_    ) : nullptr;
        auto var_line      = has_line_var  ? &dbcontent_man_.metaGetVariable(DBCType, DBContent::meta_var_line_id_  ) : nullptr;

        bool has_rn        = var_rn   && buffer->hasAnyPropertyNamed(var_rn->name());
        bool has_ts        = var_ts   && buffer->hasAnyPropertyNamed(var_ts->name());
        bool has_ds_id     = var_ds   && buffer->hasAnyPropertyNamed(var_ds->name());
        bool has_line      = var_line && buffer->hasAnyPropertyNamed(var_line->name());
        bool has_con       = buffer->hasAnyPropertyNamed(DBContent::var_cat063_con_.name());
        bool has_sen_sac   = buffer->hasAnyPropertyNamed(DBContent::var_cat063_sensor_sac_.name());
        bool has_sen_sic   = buffer->hasAnyPropertyNamed(DBContent::var_cat063_sensor_sic_.name());

        bool all_vars_present = has_rn && has_ts && has_ds_id && has_line && has_con && has_sen_sac && has_sen_sic;

        if (!all_vars_present)
        {
            loginf << "missing required variables for sensor status update:";
            loginf << "  rec num: " << (has_rn ? "ok" : "missing");
            loginf << "  timestamp: " << (has_ts ? "ok" : "missing");
            loginf << "  ds id: " << (has_ds_id ? "ok" : "missing");
            loginf << "  line id: " << (has_line ? "ok" : "missing");
            loginf << "  con: " << (has_con ? "ok" : "missing");
            loginf << "  sensor sac: " << (has_sen_sac ? "ok" : "missing");
            loginf << "  sensor sic: " << (has_sen_sic ? "ok" : "missing");
        }

        if(!all_vars_present)
        {
            logwrn << "required variables missing, skipping update";
            return;
        }

        auto& rec_num_vec = buffer->get<unsigned long>(var_rn->name());
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

        const double max_status_age = ds_man_.config().sensorStatusMaxStatusAgeValue();

        //parse in reverse
        for (size_t i = n; i-- > 0; )
        {
            //rec num + ds + line never expected to be null
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
            if (ts <= ts_cur && Utils::Time::partialSeconds(ts_cur - ts) > max_status_age)
                break;

            auto rec_num = rec_num_vec.get(i);
            auto ds_id   = ds_id_vec.get(i);
            auto line_id = line_vec.get(i);

            sensor_status::TrackerKey key(ds_id, line_id);

            unsigned int sensor_id = (unsigned int)sen_sac_vec.get(i) * 255 + sen_sic_vec.get(i);

            bool first_tracker_state = tracker_states_.count(key) == 0;

            auto& tracker_states  = tracker_states_[ key ];
            auto& sen_state_cache = tracker_states.states[ sensor_id ];

            auto& state_cur       = sen_state_cache.state_current;
            auto& state_last_item = sen_state_cache.state_last_item;
            auto& state_last_upd  = sen_state_cache.state_last_update;

            if (first_tracker_state)
                initTrackerStates(key, tracker_states);

            if (state_last_upd.isCON() && rec_num == state_last_upd.rec_num_con)
                sen_state_cache.last_update_found_in_scan = true;

            auto status = sensor_status::sensorStatusFromCon(con_vec.get(i));

            if (state_cur.isFresh() ||
                ts >= state_cur.ts_con)
            {
                //set new CON status
                state_cur.status      = status;
                state_cur.ts_con      = ts;
                state_cur.rec_num_con = rec_num;
                
                //store newest status update
                if (tracker_states.last_update_ts.is_not_a_date_time() || ts > tracker_states.last_update_ts)
                    tracker_states.last_update_ts = ts;
            }

            //detailed scan: check against sensor status of last buffer row
            if (state_last_item.status != sensor_status::SensorStatus::Fresh && // must have last item in batch
                status != state_last_item.status)                               // only log if status has changed
            {
                auto status0 = status;
                auto status1 = state_last_item.status;
                auto ts1     = state_last_item.ts_con;

                //log status change
                logEvent(&tracker_states,
                         sensor_status::Event::Type::StatusChangeIntermediate,
                         ts1, 
                         key,
                         sensor_id,
                         sensor_status::StatusChange(status0, status1));
            }
            
            state_last_item.status      = status;
            state_last_item.ts_con      = ts;
            state_last_item.rec_num_con = rec_num;

            last_ts = ts;
        }

        //after scan: log state change from last scan's final state to new scan's first state
        for (auto& tracker_status : tracker_states_)
        {
            for (auto& sen_stat : tracker_status.second.states)
            {
                const auto& state_last_upd  = sen_stat.second.state_last_update;
                const auto& state_last_item = sen_stat.second.state_last_item;

                bool has_last_item_state = !state_last_item.isFresh();
                bool status_changed      = state_last_item.status != state_last_upd.status;
                bool already_scanned     = sen_stat.second.last_update_found_in_scan;

                if (status_changed)
                    loginf << tracker_status.first.first << ":" << sen_stat.first 
                        << " has last item " << has_last_item_state
                        << " already scanned " << already_scanned
                        << " status changed " << status_changed;

                //log if:
                // - found a state during scan
                // - the status has changed compared to last scan's final state
                // - the last scan final state's item has not been scanned yet in this run
                if (has_last_item_state && status_changed && !already_scanned)
                {
                    logEvent(&tracker_status.second,
                             sensor_status::Event::Type::StatusChangeIntermediate,
                             state_last_item.ts_con, 
                             tracker_status.first,
                             sen_stat.first,
                             sensor_status::StatusChange(state_last_upd.status, state_last_item.status));
                }
            }
        }

        //log missing information
        if (missing_info)
            logEvent(nullptr, sensor_status::Event::Type::MissingInformation, {});
    }
    else
    {
        //no data in buffer
        logEvent(nullptr, sensor_status::Event::Type::NoCAT063Data, {});
    }

    //update status from new information
    updateSensorStatus();

    //update content to new status 
    updateContent();

    last_refresh_ts_ = Utils::Time::currentUTCTime();

#ifdef DEBUG_SENSOR_STATUS_TIMING
    auto time1 = boost::posix_time::microsec_clock::local_time();
    loginf << "took " << (time1 - time0).total_milliseconds() << "ms";
#endif

    emit refreshed();
}

/**
 */
void DataSourcesStatusWidget::initSensorStatesForParse()
{
    for (auto& tracker_status : tracker_states_)
        tracker_status.second.initForParse();
}

/**
 */
void DataSourcesStatusWidget::updateSensorStatus()
{
    auto ts_cur = Utils::Time::currentUTCTime();

    const double max_status_age           = ds_man_.config().sensorStatusMaxStatusAgeValue();
    const double max_status_age_max_value = ds_man_.config().sensorStatusMaxStatusAgeMaxValue();

    //determine final sensor status and finalize scan
    for (auto& tracker_status : tracker_states_)
    {
        for (auto& sen_stat : tracker_status.second.states)
        {
            auto&       state_cur       = sen_stat.second.state_current;
            const auto& state_last_upd  = sen_stat.second.state_last_update;

            //state should be init at this point
            traced_assert(!state_cur.isFresh());

            //check for going into unknown status due to maximum age
            if (!state_cur.isUnknown() &&
                state_cur.ts_con <= ts_cur && 
                Utils::Time::partialSeconds(ts_cur - state_cur.ts_con) > max_status_age)
            {
                //set sensor status to unknown
                state_cur.status = sensor_status::SensorStatus::Unknown;

                //log state change
                logEvent(&tracker_status.second,
                          sensor_status::Event::Type::StatusChangeIntermediate,
                          ts_cur, 
                          tracker_status.first,
                          sen_stat.first,
                          sensor_status::StatusChange(state_last_upd.status, state_cur.status));
            }

            //log final update status change
            // if (state_last_upd.status != state_cur.status)
            // {
            //     logEvent(&tracker_status.second,
            //              sensor_status::Event::Type::StatusChangeFinal,
            //              state_cur.status == sensor_status::SensorStatus::Unknown ? ts_cur : state_cur.ts_con, 
            //              tracker_status.first,
            //              sen_stat.first,
            //              sensor_status::StatusChange(state_last_upd.status, state_cur.status));
            // }
        }
 
        //add existing event candidates to queue
        bool events_added = tracker_status.second.queueCandidateEvents();
            
        //cleanup outdated events (use maximum possible max status age value)
        tracker_status.second.clearOutdatedEvents(max_status_age_max_value);

        //send signal if events have been added that are related to the currently selected tracker line
        if (events_added && active_tracker_.has_value() && active_tracker_.value() == tracker_status.first)
            emit eventsAdded();
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
