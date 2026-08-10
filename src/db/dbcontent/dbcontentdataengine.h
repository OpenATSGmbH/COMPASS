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

#include "boost/date_time/posix_time/posix_time.hpp"

#include "json.hpp"
#include "loadspec.h"
#include "filterclause.h"

#include <QObject>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>

class DBContentManager;
class DBContentDeleteDBJob;
class DBContentInsertDBJob;
class LoadOperation;
class Buffer;

// Single door to the DB connection for DBContent data: executes one
// LoadOperation at a time (fills its buffers, drives its state) and owns the
// insert/delete DB writes. Serialized behind the one DB connection via JobManager.
class DBContentDataEngine : public QObject
{
    Q_OBJECT

signals:
    void dataDeletedSignal(); // after a delete finished and context was updated
    void insertDoneSignal();  // after an insert job finished (forwarded by the manager)
    // freshly inserted buffers, taken over by the receiver: it may prune/transform/empty them
    // (the LiveController-owned feed does), so single consumer by design
    void insertedDataSignal(std::map<std::string, std::shared_ptr<Buffer>> buffers);

public slots:
    // one content finished reading (announced by the DBContent that owns the job)
    void contentReadDoneSlot(const std::string& dbcontent_name, std::shared_ptr<Buffer> buffer);
    // one content's read errored (DB error) -> fail the whole operation
    void contentReadFailedSlot(const std::string& dbcontent_name, const std::string& error);
    void cancelCurrentSlot();

    void finishDeletingSlot();
    void deleteJobDoneSlot();

    void finishInsertingSlot();

public:
    DBContentDataEngine(DBContentManager& dbcont_man);
    virtual ~DBContentDataEngine();

    // any DB work in flight (load, insert or delete)
    bool isBusy() const { return isLoading() || insertInProgress() || hasActiveDeleteJob(); }
    void waitUntilIdle(); // pumps events until isBusy() clears - before the DB is closed

    void load(std::shared_ptr<LoadOperation> op);
    bool isLoading() const { return (bool) current_op_; }
    std::shared_ptr<LoadOperation> currentOp() const { return current_op_; } // running load (null if none)
    void cancelLoad(); // cooperatively cancel the running load (no-op if none)

    // load-query construction (also used by the manager for the progress count)
    std::set<std::string> resolveTargetSet(const LoadSpec& spec) const;

    // write-path allocation counters (seeded from the DB on open, then bumped by
    // the insert paths as fresh rec_nums / track_nums are handed out)
    void loadMaxRecordNumberWODBContentID();   // query DB into the cache
    bool          hasMaxRecordNumberWODBContentID() const { return max_rec_num_wo_dbcontid_.has_value(); }
    unsigned long maxRecordNumberWODBContentID() const;    // cached
    void          maxRecordNumberWODBContentID(unsigned long value);

    void loadMaxRefTrajTrackNum();             // query DB into the cache
    bool         hasMaxRefTrajTrackNum() const { return max_reftraj_track_num_.has_value(); }
    unsigned int maxRefTrajTrackNum() const;   // cached
    void         maxRefTrajTrackNum(unsigned int value);

    // DB closed: drop the transient load/insert/delete state + the cached DB metadata
    void onDatabaseClose();

    // DB-persisted dataset-extent metadata (loaded from properties on open,
    // maintained + persisted from the inserted buffers, cleared on close)
    void loadMinMaxInfo();                     // seed from DB properties on open
    void updateInsertMinMax(const std::map<std::string, std::shared_ptr<Buffer>>& insert_data);

    bool hasMinMaxInfo() const;
    bool hasMinMaxTimestamp() const;
    void setMinMaxTimestamp(boost::posix_time::ptime min, boost::posix_time::ptime max);
    std::pair<boost::posix_time::ptime, boost::posix_time::ptime> minMaxTimestamp() const;
    bool hasMinMaxPosition() const;
    void setMinMaxLatitude(double min, double max);
    std::pair<double, double> minMaxLatitude() const;
    void setMinMaxLongitude(double min, double max);
    std::pair<double, double> minMaxLongitude() const;

    bool hasActiveDeleteJob() const { return (bool) delete_job_; }

    void deleteData(const nlohmann::json& delete_info);       // delete by criteria
    void deleteOlderThan(boost::posix_time::ptime before_timestamp); // bound the live cache

    void insert(std::map<std::string, std::shared_ptr<Buffer>> data);
    bool insertInProgress() const { return insert_in_progress_; }

private:
    void finish();

    void clearMaxNumbers();
    void clearMinMaxInfo();

    std::string composeWhereClause(const std::string& dbcontent_name, const LoadSpec& spec,
                                   dbContent::VariableSet& read_set);

    // datasource/line WHERE fragment for a content (empty when no ds filter applies)
    FilterClause dataSourceClause(const std::string& dbcontent_name, const LoadSpec& spec);

    unsigned long queryMaxRecordNumberWODBContentID() const; // DB scan
    unsigned int  queryMaxRefTrajTrackNum() const;           // DB scan

    DBContentManager& dbcont_man_;

    std::optional<unsigned long> max_rec_num_wo_dbcontid_;
    std::optional<unsigned int>  max_reftraj_track_num_;

    std::optional<boost::posix_time::ptime> timestamp_min_;
    std::optional<boost::posix_time::ptime> timestamp_max_;
    std::optional<double>                   latitude_min_;
    std::optional<double>                   latitude_max_;
    std::optional<double>                   longitude_min_;
    std::optional<double>                   longitude_max_;

    std::shared_ptr<LoadOperation> current_op_;
    std::set<std::string>          pending_contents_; // contents still reading for current_op_
    bool                           load_failed_ {false}; // a content's read errored
    std::string                    load_error_;          // first read error message

    std::shared_ptr<DBContentDeleteDBJob> delete_job_;
    nlohmann::json                        delete_info_;

    std::map<std::string, std::shared_ptr<Buffer>> insert_data_;
    bool                                           insert_in_progress_ {false};
    std::shared_ptr<DBContentInsertDBJob>          insert_job_;
};
