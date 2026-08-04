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

#include "dbcontentdataengine.h"
#include "dbcontentmanager.h"
#include "dbcontent.h"
#include "loadoperation.h"
#include "dbcontentdeletedbjob.h"
#include "dbcontentinsertdbjob.h"
#include "livedatafeed.h"
#include "db_context_manager.h"
#include "filtermanager.h"
#include "buffer.h"
#include "util/number.h"
#include "util/timeconv.h"
#include "buffer_utils.h"
#include "compass.h"
#include "jobmanager.h"
#include "dbinterface.h"
#include "db_context_manager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "base/idbvariableresolver.h"
#include "logger.h"

#include <QCoreApplication>
#include <QThread>
#include <QEventLoop>

using namespace std;
using namespace dbContent;

/**
 */
DBContentDataEngine::DBContentDataEngine(DBContentManager& dbcont_man)
:   dbcont_man_(dbcont_man)
{
}

/**
 */
DBContentDataEngine::~DBContentDataEngine() = default;

/**
 * Executes one load: resolves the target set, composes each content's WHERE and
 * read set, augments with the core meta-vars, and fans out one read job per
 * content into the operation. Assumes no other operation is running.
 */
void DBContentDataEngine::load(std::shared_ptr<LoadOperation> op)
{
    traced_assert(op);

    // single-operation safety net (all load paths funnel through here): only one load
    // runs at a time. If one is still running, wait for it (pumping events) rather than
    // asserting or quitting it - a consumer may be awaiting its result.
    if (isLoading())
    {
        logwrn << "waiting for a still-running load to finish";
        while (isLoading())
        {
            QCoreApplication::processEvents();
            QThread::msleep(1);
        }
    }

    // each content of the previous load delivered exactly one terminal event before it
    // finished; contentReadDoneSlot's name-only staleness check relies on that
    traced_assert(pending_contents_.empty());

    current_op_ = op;
    op->clearBuffers();

    load_failed_ = false;
    load_error_.clear();

    connect(op.get(), &LoadOperation::cancelRequestedSignal,
            this, &DBContentDataEngine::cancelCurrentSlot, Qt::QueuedConnection);

    op->setState(LoadOperation::State::Running);

    const LoadSpec& spec = op->spec();

    if (spec.measure_db_performance_)
        dbcont_man_.compass().dbInterface().startPerformanceMetrics();

    set<string> targets = resolveTargetSet(spec);

    for (const auto& name : targets)
    {
        DBContent& obj = dbcont_man_.dbContent(name);

        VariableSet read_set = spec.read_set_ ? spec.read_set_(name) : dbcont_man_.getReadSet(name);

        if (read_set.getSize() == 0)
        {
            logwrn << "skipping loading of " << name << ": empty read set";
            continue;
        }

        std::string where = composeWhereClause(name, spec, read_set);

        // always-present core meta-vars (timestamp promoted into the core: the read
        // job always orders by it); utn stays optional, not forced onto custom loads
        dbcont_man_.addStandardVariables(name, read_set, /*add_utn_if_available=*/false);

        // the read job is created + owned by the DBContent (like its update/delete jobs);
        // it massages + announces the finished buffer via readDoneSignal, collected below
        connect(&obj, &DBContent::readDoneSignal,
                this, &DBContentDataEngine::contentReadDoneSlot, Qt::UniqueConnection);
        connect(&obj, &DBContent::readFailedSignal,
                this, &DBContentDataEngine::contentReadFailedSlot, Qt::UniqueConnection);

        pending_contents_.insert(name);

        obj.loadInternal(read_set, where);
    }

    if (pending_contents_.empty())
        finish();
}

/**
 * One content finished reading (announced by its DBContent, which owns the job and already
 * massaged the buffer): store it into the operation and announce it; a null buffer (empty
 * result or obsoleted on cancel) is just dropped. Finish once all target contents are in.
 */
void DBContentDataEngine::contentReadDoneSlot(const string& name, shared_ptr<Buffer> buffer)
{
    if (!current_op_ || !pending_contents_.count(name)) // stale / not part of this load
        return;

    if (buffer && buffer->size())
    {
        current_op_->setBuffer(name, buffer);
        current_op_->emitChanged({name}, /*reset=*/false, /*last=*/false);
    }

    pending_contents_.erase(name);

    if (pending_contents_.empty())
        finish();
}

/**
 * A content's read errored (DB error). A load is atomic: fail the whole operation, obsolete
 * the remaining pending reads (they drain via readDoneSignal(null)), and finish as Failed.
 */
void DBContentDataEngine::contentReadFailedSlot(const string& name, const string& error)
{
    if (!current_op_ || !pending_contents_.count(name)) // stale / not part of this load
        return;

    logerr << "read failed for " << name << ": " << error;

    if (!load_failed_) // first error wins
    {
        load_failed_ = true;
        load_error_  = error;
    }

    pending_contents_.erase(name);

    for (const auto& other : pending_contents_)
        dbcont_man_.dbContent(other).quitLoading();

    if (pending_contents_.empty())
        finish();
}

/**
 * Cancel the running load (if any) via the operation's own cooperative cancel - which
 * emits cancelRequestedSignal, handled by cancelCurrentSlot below. The single cancel path.
 */
void DBContentDataEngine::cancelLoad()
{
    if (current_op_)
        current_op_->cancel();
}

/**
 * Cancel request from the operation: obsolete the in-flight read jobs; they
 * drain through the done/obsolete slots and the op finishes as Cancelled.
 */
void DBContentDataEngine::cancelCurrentSlot()
{
    // the read jobs are owned by the DBContents; flag each pending one obsolete. The job
    // sees the flag between chunks, drops its buffer and completes normally, so it drains
    // through readJobDoneSlot -> readDoneSignal(null) -> contentReadDoneSlot
    for (const auto& name : pending_contents_)
        dbcont_man_.dbContent(name).quitLoading();
}

/**
 * Terminal transition for the current operation: emits the final (last) change
 * event and the operation's finished state (Cancelled if a cancel was requested).
 */
void DBContentDataEngine::finish()
{
    auto op = current_op_;
    current_op_ = nullptr;
    pending_contents_.clear();

    if (op->spec().measure_db_performance_)
    {
        DBInterface& db_interface = dbcont_man_.compass().dbInterface();
        if (db_interface.hasActivePerformanceMetrics())
            loginf << db_interface.stopPerformanceMetrics().asString();
    }

    // terminal state (Failed takes precedence over a concurrent cancel); on failure the
    // partial buffers are discarded. The Result carries the message for consumers.
    LoadOperation::State state;
    if (load_failed_)
    {
        state = LoadOperation::State::Failed;
        op->clearBuffers();
        op->result_ = Result::failed(load_error_);
    }
    else
    {
        state = op->cancel_requested_ ? LoadOperation::State::Cancelled
                                      : LoadOperation::State::Done;
        op->result_ = Result::succeeded();
    }

    op->emitChanged({}, /*reset=*/false, /*last=*/true);

    // result set before the state transition so a finishedSignal consumer sees it
    op->setState(state);
}

/**
 * Delete by criteria: wipe the in-memory dataset, then run a cleanup delete job;
 * on completion the deleted info is applied to the context and dataDeletedSignal fires.
 */
void DBContentDataEngine::deleteData(const nlohmann::json& delete_info)
{
    traced_assert(!delete_job_); // caller checks hasActiveDeleteJob()

    delete_info_ = delete_info;

    delete_job_ = make_shared<DBContentDeleteDBJob>(dbcont_man_.compass().dbInterface());
    delete_job_->setDeleteInfo(delete_info_);
    delete_job_->cleanupDB(true);

    connect(delete_job_.get(), &DBContentDeleteDBJob::doneSignal,
            this, &DBContentDataEngine::finishDeletingSlot, Qt::QueuedConnection);

    dbcont_man_.compass().jobManager().addDBJob(delete_job_);
}

/**
 * Deletes everything older than the given timestamp (bounds the live DB cache).
 */
void DBContentDataEngine::deleteOlderThan(boost::posix_time::ptime before_timestamp)
{
    traced_assert(!delete_job_); // caller checks hasActiveDeleteJob()

    delete_job_ = make_shared<DBContentDeleteDBJob>(dbcont_man_.compass().dbInterface());
    delete_job_->setBeforeTimestamp(before_timestamp);

    connect(delete_job_.get(), &DBContentDeleteDBJob::doneSignal,
            this, &DBContentDataEngine::deleteJobDoneSlot, Qt::QueuedConnection);

    dbcont_man_.compass().jobManager().addDBJob(delete_job_);
}

/**
 */
void DBContentDataEngine::finishDeletingSlot()
{
    traced_assert(delete_job_);

    dbcont_man_.compass().dbContextManager().applyDeleteInfo(delete_info_);

    delete_job_  = nullptr;
    delete_info_ = nlohmann::json{};

    emit dataDeletedSignal();
}

/**
 */
void DBContentDataEngine::deleteJobDoneSlot()
{
    traced_assert(delete_job_);
    delete_job_ = nullptr;
}

/**
 * Highest record number (without DBContent id) across contents present in the DB.
 */
unsigned long DBContentDataEngine::queryMaxRecordNumberWODBContentID() const
{
    traced_assert(dbcont_man_.compass().dbInterface().ready());

    unsigned long result = 0;

    for (auto& obj_it : dbcont_man_)
    {
        if (obj_it.second->existsInDB())
        {
            unsigned long with_id = dbcont_man_.compass().dbInterface().getMaxRecordNumber(*obj_it.second);
            result = std::max(Utils::Number::recNumGetWithoutDBContId(with_id), result);
        }
    }

    return result;
}

/**
 */
unsigned int DBContentDataEngine::queryMaxRefTrajTrackNum() const
{
    traced_assert(dbcont_man_.compass().dbInterface().ready());
    return dbcont_man_.compass().dbInterface().getMaxRefTrackTrackNum();
}

/**
 */
void DBContentDataEngine::loadMaxRecordNumberWODBContentID()
{
    max_rec_num_wo_dbcontid_ = queryMaxRecordNumberWODBContentID();
    loginf << "start" << *max_rec_num_wo_dbcontid_;
}

/**
 */
unsigned long DBContentDataEngine::maxRecordNumberWODBContentID() const
{
    traced_assert(max_rec_num_wo_dbcontid_.has_value());
    return *max_rec_num_wo_dbcontid_;
}

/**
 */
void DBContentDataEngine::maxRecordNumberWODBContentID(unsigned long value)
{
    logdbg << "start" << value;
    max_rec_num_wo_dbcontid_ = value;
}

/**
 */
void DBContentDataEngine::loadMaxRefTrajTrackNum()
{
    max_reftraj_track_num_ = queryMaxRefTrajTrackNum();
    loginf << "start" << *max_reftraj_track_num_;
}

/**
 */
unsigned int DBContentDataEngine::maxRefTrajTrackNum() const
{
    traced_assert(max_reftraj_track_num_.has_value());
    return *max_reftraj_track_num_;
}

/**
 */
void DBContentDataEngine::maxRefTrajTrackNum(unsigned int value)
{
    logdbg << "start" << value;
    max_reftraj_track_num_ = value;
}

/**
 */
void DBContentDataEngine::clearMaxNumbers()
{
    max_rec_num_wo_dbcontid_.reset();
    max_reftraj_track_num_.reset();
}

/**
 * Seed the dataset-extent metadata from the DB properties (on DB open).
 */
void DBContentDataEngine::loadMinMaxInfo()
{
    auto& db_interface = dbcont_man_.compass().dbInterface();

    if (db_interface.hasProperty(PROP_TIMESTAMP_MIN_NAME))
    {
        timestamp_min_ = Utils::Time::fromLong(stol(db_interface.getProperty(PROP_TIMESTAMP_MIN_NAME)));
        traced_assert(!timestamp_min_->is_not_a_date_time());
    }
    if (db_interface.hasProperty(PROP_TIMESTAMP_MAX_NAME))
    {
        timestamp_max_ = Utils::Time::fromLong(stol(db_interface.getProperty(PROP_TIMESTAMP_MAX_NAME)));
        traced_assert(!timestamp_max_->is_not_a_date_time());
    }

    if (hasMinMaxTimestamp())
        loginf << "timestamp_min_ " << Utils::Time::toString(*timestamp_min_)
               << " timestamp_max_ " << Utils::Time::toString(*timestamp_max_);
    else
        loginf << "no min/max timestamp";

    if (db_interface.hasProperty(PROP_LATITUDE_MIN_NAME))
        latitude_min_ = stod(db_interface.getProperty(PROP_LATITUDE_MIN_NAME));
    if (db_interface.hasProperty(PROP_LATITUDE_MAX_NAME))
        latitude_max_ = stod(db_interface.getProperty(PROP_LATITUDE_MAX_NAME));

    if (db_interface.hasProperty(PROP_LONGITUDE_MIN_NAME))
        longitude_min_ = stod(db_interface.getProperty(PROP_LONGITUDE_MIN_NAME));
    if (db_interface.hasProperty(PROP_LONGITUDE_MAX_NAME))
        longitude_max_ = stod(db_interface.getProperty(PROP_LONGITUDE_MAX_NAME));

    if (hasMinMaxPosition())
        loginf << "latitude_min_ " << *latitude_min_
               << " latitude_max_ " << *latitude_max_ << " longitude_min_ " << *longitude_min_
               << " longitude_max_ " << *longitude_max_;
    else
        loginf << "no min/max position";
}

/**
 */
void DBContentDataEngine::clearMinMaxInfo()
{
    timestamp_min_.reset();
    timestamp_max_.reset();
    latitude_min_.reset();
    latitude_max_.reset();
    longitude_min_.reset();
    longitude_max_.reset();
}

/**
 * Expand + persist the dataset-extent metadata from a freshly inserted chunk,
 * then announce the change (e.g. updates the timestamps in the data sources tool).
 */
void DBContentDataEngine::updateInsertMinMax(const std::map<std::string, std::shared_ptr<Buffer>>& insert_data)
{
    using namespace boost::posix_time;

    // use db column names since the buffers were transformed during insert
    for (auto& buf_it : insert_data)
    {
        string dbcont_name = buf_it.first;

        traced_assert(dbcont_man_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_timestamp_));

        unsigned int buffer_size = buf_it.second->size();

        // timestamp
        {
            Variable& var = dbcont_man_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_timestamp_);
            if (buf_it.second->has<boost::posix_time::ptime>(var.dbColumnName()))
            {
                NullableVector<boost::posix_time::ptime>& data_vec = buf_it.second->get<boost::posix_time::ptime>(
                    var.dbColumnName());

                bool has_vec_min_max;
                ptime ts_vec_min, ts_vec_max;

                tie(has_vec_min_max, ts_vec_min, ts_vec_max) = data_vec.minMaxValues();

                if (has_vec_min_max)
                {
                    if (hasMinMaxTimestamp())
                    {
                        timestamp_min_ = std::min(*timestamp_min_, ts_vec_min);
                        timestamp_max_ = std::max(*timestamp_max_, ts_vec_max);
                    }
                    else
                    {
                        timestamp_min_ = ts_vec_min;
                        timestamp_max_ = ts_vec_max;
                    }
                }
            }
        }

        if (hasMinMaxTimestamp())
        {
            dbcont_man_.compass().dbInterface().setProperty(PROP_TIMESTAMP_MIN_NAME,
                                                        to_string(Utils::Time::toLong(*timestamp_min_)));
            dbcont_man_.compass().dbInterface().setProperty(PROP_TIMESTAMP_MAX_NAME,
                                                        to_string(Utils::Time::toLong(*timestamp_max_)));
        }

        // lat & long
        if (dbcont_man_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_longitude_)
            && dbcont_man_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_latitude_))
        {
            Variable& lat_var = dbcont_man_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_latitude_);
            Variable& lon_var = dbcont_man_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_longitude_);

            if (buf_it.second->has<double>(lat_var.dbColumnName())
                && buf_it.second->has<double>(lon_var.dbColumnName()))
            {
                NullableVector<double>& lat_vec = buf_it.second->get<double>(lat_var.dbColumnName());
                NullableVector<double>& lon_vec = buf_it.second->get<double>(lon_var.dbColumnName());

                bool has_min_max = hasMinMaxPosition();

                for (unsigned int cnt=0; cnt < buffer_size; cnt++)
                {
                    if (!lat_vec.isNull(cnt) && !lon_vec.isNull(cnt))
                    {
                        if (has_min_max)
                        {
                            latitude_min_ = std::min(*latitude_min_, lat_vec.get(cnt));
                            latitude_max_ = std::max(*latitude_max_, lat_vec.get(cnt));

                            longitude_min_ = std::min(*longitude_min_, lon_vec.get(cnt));
                            longitude_max_ = std::max(*longitude_max_, lon_vec.get(cnt));
                        }
                        else
                        {
                            latitude_min_ = lat_vec.get(cnt);
                            latitude_max_ = lat_vec.get(cnt);

                            longitude_min_ = lon_vec.get(cnt);
                            longitude_max_ = lon_vec.get(cnt);

                            has_min_max = true;
                        }
                    }
                }

                if (has_min_max)
                {
                    dbcont_man_.compass().dbInterface().setProperty(PROP_LATITUDE_MIN_NAME, to_string(*latitude_min_));
                    dbcont_man_.compass().dbInterface().setProperty(PROP_LATITUDE_MAX_NAME, to_string(*latitude_max_));

                    dbcont_man_.compass().dbInterface().setProperty(PROP_LONGITUDE_MIN_NAME, to_string(*longitude_min_));
                    dbcont_man_.compass().dbInterface().setProperty(PROP_LONGITUDE_MAX_NAME, to_string(*longitude_max_));
                }
            }
        }
    }

    // min/max are up to date now - announce per inserted chunk
    emit dbcont_man_.dbContentStatusChanged();
}

/**
 */
bool DBContentDataEngine::hasMinMaxInfo() const
{
    return timestamp_min_.has_value() || timestamp_max_.has_value()
           || latitude_min_.has_value() || latitude_max_.has_value()
           || longitude_min_.has_value() || longitude_max_.has_value();
}

/**
 */
bool DBContentDataEngine::hasMinMaxTimestamp() const
{
    return timestamp_min_.has_value() && timestamp_max_.has_value();
}

/**
 */
void DBContentDataEngine::setMinMaxTimestamp(boost::posix_time::ptime min, boost::posix_time::ptime max)
{
    traced_assert(!min.is_not_a_date_time());
    traced_assert(!max.is_not_a_date_time());

    timestamp_min_ = min;
    timestamp_max_ = max;

    dbcont_man_.compass().dbInterface().setProperty(PROP_TIMESTAMP_MIN_NAME, to_string(Utils::Time::toLong(*timestamp_min_)));
    dbcont_man_.compass().dbInterface().setProperty(PROP_TIMESTAMP_MAX_NAME, to_string(Utils::Time::toLong(*timestamp_max_)));
}

/**
 */
std::pair<boost::posix_time::ptime, boost::posix_time::ptime> DBContentDataEngine::minMaxTimestamp() const
{
    traced_assert(hasMinMaxTimestamp());
    return {*timestamp_min_, *timestamp_max_};
}

/**
 */
bool DBContentDataEngine::hasMinMaxPosition() const
{
    return latitude_min_.has_value() || latitude_max_.has_value()
           || longitude_min_.has_value() || longitude_max_.has_value();
}

/**
 */
void DBContentDataEngine::setMinMaxLatitude(double min, double max)
{
    latitude_min_ = min;
    latitude_max_ = max;

    dbcont_man_.compass().dbInterface().setProperty(PROP_LATITUDE_MIN_NAME, to_string(*latitude_min_));
    dbcont_man_.compass().dbInterface().setProperty(PROP_LATITUDE_MAX_NAME, to_string(*latitude_max_));
}

/**
 */
std::pair<double, double> DBContentDataEngine::minMaxLatitude() const
{
    traced_assert(hasMinMaxPosition());
    return {*latitude_min_, *latitude_max_};
}

/**
 */
void DBContentDataEngine::setMinMaxLongitude(double min, double max)
{
    longitude_min_ = min;
    longitude_max_ = max;

    dbcont_man_.compass().dbInterface().setProperty(PROP_LONGITUDE_MIN_NAME, to_string(*longitude_min_));
    dbcont_man_.compass().dbInterface().setProperty(PROP_LONGITUDE_MAX_NAME, to_string(*longitude_max_));
}

/**
 */
std::pair<double, double> DBContentDataEngine::minMaxLongitude() const
{
    traced_assert(hasMinMaxPosition());
    return {*longitude_min_, *longitude_max_};
}

/**
 * Names to load: wildcard "*" = all loadable + loading-wanted contents; else the
 * explicitly named loadable ones.
 */
std::set<std::string> DBContentDataEngine::resolveTargetSet(const LoadSpec& spec) const
{
    auto& ctx_man = dbcont_man_.compass().dbContextManager();

    std::set<std::string> targets;

    const bool wildcard = spec.dbcontents_.count("*") > 0;

    for (auto& object : dbcont_man_)
    {
        if (!object.second->loadable())
            continue;

        if (wildcard)
        {
            if (!ctx_man.loadingWanted(object.first))
                continue;
            targets.insert(object.first);
        }
        else if (spec.dbcontents_.count(object.first))
        {
            targets.insert(object.first);
        }
    }

    return targets;
}

/**
 * WHERE for a content: data-source/line filter (if applied), then FilterManager
 * conditions (which may add required vars to read_set), then the custom clause -
 * AND-ed. Empty wanted sets emit the "1=0" no-row sentinel.
 */
std::string DBContentDataEngine::composeWhereClause(const std::string& name, const LoadSpec& spec,
                                                    dbContent::VariableSet& read_set)
{
    std::vector<FilterClause> parts;

    // datasource/line constraints
    parts.push_back(dataSourceClause(name, spec));

    // view filters
    if (spec.apply_view_filters_ && dbcont_man_.compass().filterManager().useFilters())
        parts.push_back(dbcont_man_.compass().filterManager().viewClause(name));

    // issuer-supplied ad-hoc clause
    if (spec.custom_filter_clause_)
    {
        FilterClause custom;
        custom.sql = spec.custom_filter_clause_(name);
        parts.push_back(custom);
    }

    FilterClause where = combineAnd(parts);

    // augment the read set with the referenced vars (explicit output, replacing the read-set
    // side effect the old getSQLCondition had)
    read_set.add(where.required_vars);

    return where.sql;
}

/**
 * Builds the datasource/line WHERE fragment for a content from the context manager's
 * loading selection (ds_id IN (...) or per-DS line-scoped OR groups; "1=0" empty sentinel).
 */
FilterClause DBContentDataEngine::dataSourceClause(const std::string& name, const LoadSpec& spec)
{
    FilterClause clause;

    if (!spec.apply_datasrc_filters_)
        return clause;

    auto selection = dbcont_man_.compass().dbContextManager().loadingSelection(name);

    if (!selection)          // unconstrained -> no clause
        return clause;

    if (selection->empty())  // nothing wanted
    {
        clause.sql = "1=0";
        return clause;
    }

    DBContent& dbc = dbcont_man_.dbContent(name);

    traced_assert(dbc.hasVariable(dbcontent_vars::meta_var_ds_id_.name()));
    Variable& ds_var = dbc.variable(dbcontent_vars::meta_var_ds_id_.name());
    traced_assert(ds_var.dataType() == PropertyDataType::UINT);

    traced_assert(dbc.hasVariable(dbcontent_vars::meta_var_line_id_.name()));
    Variable& line_var = dbc.variable(dbcontent_vars::meta_var_line_id_.name());
    traced_assert(line_var.dataType() == PropertyDataType::UINT);

    // one OR group per wanted ds: (ds = X AND line IN (its wanted lines))
    clause.sql = "(";
    bool first_group = true;
    for (const auto& [ds_id, lines] : *selection)
    {
        if (!first_group) clause.sql += " OR ";
        first_group = false;

        clause.sql += "(" + ds_var.dbColumnName() + " = " + std::to_string(ds_id)
                          + " AND " + line_var.dbColumnName() + " IN (";
        bool first_line = true;
        for (unsigned int line : lines)
        {
            if (!first_line) clause.sql += ",";
            clause.sql += std::to_string(line);
            first_line = false;
        }
        clause.sql += "))";
    }
    clause.sql += ")";

    return clause;
}

/**
 * Inserts decoded buffers into the DB: prepares the DBContents, updates data
 * sources, and submits the insert job. Waits out any running load first.
 */
void DBContentDataEngine::insert(std::map<std::string, std::shared_ptr<Buffer>> data)
{
    while (isLoading()) // pending insert during a load
    {
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        QThread::msleep(1);
    }

    traced_assert(!insert_in_progress_);
    traced_assert(!insert_data_.size());
    traced_assert(!insert_job_);

    insert_in_progress_ = true;
    insert_data_        = data;

    for (auto& buf_it : insert_data_)
    {
        traced_assert(dbcont_man_.existsDBContent(buf_it.first));
        dbcont_man_.dbContent(buf_it.first).prepareInsert(buf_it.second);
    }

    // update data sources from dbcontents (single-threaded)
    bool ds_added_any = false;
    for (auto& buf_it : insert_data_)
        ds_added_any |= dbcont_man_.dbContent(buf_it.first).updateDataSourcesBeforeInsert(buf_it.second);

    auto& ctx_man = dbcont_man_.compass().dbContextManager();
    if (ds_added_any)
        emit ctx_man.dataSourcesChangedSignal();
    emit ctx_man.countsChangedSignal();

    insert_job_ = make_shared<DBContentInsertDBJob>(dbcont_man_.compass().dbInterface(), dbcont_man_, insert_data_, false);

    connect(insert_job_.get(), &DBContentInsertDBJob::doneSignal,
            this, &DBContentDataEngine::finishInsertingSlot, Qt::QueuedConnection);

    dbcont_man_.compass().jobManager().addDBJob(insert_job_);
}

/**
 * Insert job finished: finalize the DBContents, announce completion, update the
 * DB min/max metadata, then either drop the staged buffers (offline/paused) or
 * hand them to the live feed and run a live tick (live).
 */
void DBContentDataEngine::finishInsertingSlot()
{
    insert_job_ = nullptr;

    for (auto& buf_it : insert_data_)
        dbcont_man_.dbContent(buf_it.first).finalizeInsert(buf_it.second);

    insert_in_progress_ = false;
    emit insertDoneSignal();

    // update DB metadata (timestamp/position min-max) from the inserted buffers
    updateInsertMinMax(insert_data_);

    // announce the fresh buffers unconditionally - the engine is pure DB I/O and does not
    // know about app mode. The live/offline gating lives with the consumer: LiveController
    // stages + ticks only while its session is running, and drops the announcement otherwise.
    emit insertedDataSignal(insert_data_);
    insert_data_.clear();
}
