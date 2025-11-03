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

#include "dbcontentreaddbjob.h"
#include "buffer.h"
#include "dbinterface.h"
#include "dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "logger.h"
#include "compass.h"
#include "viewmanager.h"

using namespace dbContent;

DBContentReadDBJob::DBContentReadDBJob(DBInterface& db_interface, DBContent& dbcontent, VariableSet read_list,
                           std::string custom_filter_clause)
    : Job("DBContentReadDBJob"),
      db_interface_(db_interface),
      dbcontent_(dbcontent),
      read_list_(read_list),
      custom_filter_clause_(custom_filter_clause)
{
    traced_assert(dbcontent_.existsInDB());

    use_order_ = true; // always order
    traced_assert(COMPASS::instance().dbContentManager().metaCanGetVariable(
                dbcontent_.name(), DBContent::meta_var_timestamp_));

    // always order by timestamp
    order_variable_ = &COMPASS::instance().dbContentManager().metaGetVariable(
                dbcontent_.name(), DBContent::meta_var_timestamp_);
}

DBContentReadDBJob::~DBContentReadDBJob() {}

void DBContentReadDBJob::run_impl()
{
    logdbg << "start" << dbcontent_.name() << ": start";
    started_ = true;

    if (obsolete_)
    {
        logdbg << "start" << dbcontent_.name() << ": obsolete before prepared";
        done_ = true;
        return;
    }

    start_time_ = boost::posix_time::microsec_clock::local_time();

    db_interface_.prepareRead(dbcontent_, read_list_, custom_filter_clause_,
                              use_order_, order_variable_);

    unsigned int cnt = 0;

    //ViewManager &view_manager = COMPASS::instance().viewManager();

    bool last_buffer;

    while (!obsolete_)
    {
        std::shared_ptr<Buffer> buffer;
        std::tie (buffer, last_buffer) = db_interface_.readDataChunk(dbcontent_);
        traced_assert(buffer);

        cnt++;

        if (obsolete_)
            break;

        traced_assert(buffer->dbContentName() == dbcontent_.name());

        logdbg << "start" << dbcontent_.name() << ": intermediate signal, #buffers "
               << cnt << " last one " << last_buffer;
        row_count_ += buffer->size();

        // add data to cache
        if (!cached_buffer_) // no cache
            cached_buffer_ = buffer;
        else
        {
            cached_buffer_->seizeBuffer(*buffer);
            buffer = nullptr;
        }

        if (obsolete_)
            break;

        if (last_buffer) // distribute data, !view_manager.isProcessingData() || 
        {
            logdbg << dbcontent_.name()
                   << ": emitting intermediate read, size " << cached_buffer_->size();
            //cached_buffer_->printProperties();

            emit intermediateSignal(cached_buffer_);

            cached_buffer_ = nullptr;
        }

        if (last_buffer)
        {
            logdbg << "start" << dbcontent_.name() << ": last buffer";
            break;
        }
    }

    if (obsolete_)
        cached_buffer_ = nullptr;

    traced_assert(!cached_buffer_);

    logdbg << "start" << dbcontent_.name() << ": finalizing statement";
    db_interface_.finalizeReadStatement(dbcontent_);

    stop_time_ = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = stop_time_ - start_time_;

    if (diff.total_seconds() > 0)
    {
        logdbg << "start" << dbcontent_.name() << ": done after " << diff << ", "
               << 1000.0 * row_count_ / diff.total_milliseconds() << " el/s";
    }
    else
    {
        logdbg << "start" << dbcontent_.name() << ": done";
    }

    done_ = true;

    return;
}

unsigned int DBContentReadDBJob::rowCount() const { return row_count_; }
