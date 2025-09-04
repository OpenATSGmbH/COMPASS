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

#include "updatebufferdbjob.h"

#include "buffer.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "stringconv.h"

#include "boost/date_time/posix_time/posix_time.hpp"

using namespace Utils::String;

UpdateBufferDBJob::UpdateBufferDBJob(DBInterface& db_interface, DBContent& dbcontbject,
                                     dbContent::Variable& key_var, std::shared_ptr<Buffer> buffer)
    : Job("UpdateBufferDBJob"),
      db_interface_(db_interface),
      dbcontbject_(dbcontbject),
      key_var_(key_var),
      buffer_(buffer)
{
    traced_assert(buffer_);
    traced_assert(dbcontbject_.existsInDB());
}

UpdateBufferDBJob::~UpdateBufferDBJob() {}

void UpdateBufferDBJob::run_impl()
{
    logdbg;

    started_ = true;

    boost::posix_time::ptime loading_start_time_;
    boost::posix_time::ptime loading_stop_time_;

    loading_start_time_ = boost::posix_time::microsec_clock::local_time();

    unsigned int steps = buffer_->size() / 10000;

    logdbg << "writing object " << dbcontbject_.name() << " key "
           << key_var_.name() << " size " << buffer_->size() << " steps " << steps;

    unsigned int index_from = 0;
    unsigned int index_to = 0;

    for (unsigned int cnt = 0; cnt <= steps; cnt++)
    {
        index_from = cnt * 10000;
        index_to = index_from + 10000;

        if (index_to > buffer_->size() - 1)
            index_to = buffer_->size() - 1;

        logdbg << "step " << cnt << " steps " << steps << " from "
               << index_from << " to " << index_to;

        db_interface_.updateBuffer(dbcontbject_.dbTableName(), key_var_.dbColumnName(),
                                   buffer_, index_from, index_to);

        emit updateProgressSignal(100.0 * index_to / buffer_->size());
    }

    loading_stop_time_ = boost::posix_time::microsec_clock::local_time();

    double load_time;
    boost::posix_time::time_duration diff = loading_stop_time_ - loading_start_time_;
    load_time = diff.total_milliseconds() / 1000.0;

    logdbg << "start" << dbcontbject_.name()
           << " write done (" << doubleToStringPrecision(load_time, 2) << " s).";

    done_ = true;
}
