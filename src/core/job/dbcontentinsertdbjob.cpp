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

#include "dbcontentinsertdbjob.h"

#include "buffer.h"
#include "dbinterface.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"

#include "stringconv.h"
#include "files.h"
#include "timeconv.h"

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/filesystem.hpp"

using namespace Utils::String;

/**
 */
DBContentInsertDBJob::DBContentInsertDBJob(DBInterface& db_interface, 
                                           DBContentManager& db_content_man,
                                           std::map<std::string, std::shared_ptr<Buffer>>& buffers,
                                           bool emit_change)
:   Job            ("DBContentInsertDBJob")
,   db_interface_  (db_interface)
,   db_content_man_(db_content_man)
,   buffers_       (buffers)
,   emit_change_   (emit_change)
{
    for (const auto& b : buffers)
        traced_assert(b.second && b.second->size() > 0);
}

/**
 */
DBContentInsertDBJob::~DBContentInsertDBJob() = default;

/**
 */
void DBContentInsertDBJob::run_impl()
{
    logdbg;

    started_ = true;

    boost::posix_time::ptime loading_start_time;
    boost::posix_time::ptime loading_stop_time;

    loading_start_time = boost::posix_time::microsec_clock::local_time();

    logdbg << "writing " << buffers_.size() << " object(s)";

    unsigned int buffer_cnt {0};

    for (auto& buf_it : buffers_)
        buffer_cnt += buf_it.second->size();

    db_interface_.insertDBContent(buffers_);

    loading_stop_time = boost::posix_time::microsec_clock::local_time();

    boost::posix_time::time_duration diff = loading_stop_time - loading_start_time;

    logdbg << "writing buffers done, size " << buffer_cnt
           << " (" << doubleToStringPrecision(diff.total_milliseconds(), 2) << " ms).";

    done_ = true;
}

/**
 */
bool DBContentInsertDBJob::emitChange() const
{ 
    return emit_change_; 
}
