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

#include "WriteBufferDBJob.h"

#include "Buffer.h"
#include "DBInterface.h"
#include "String.h"

using namespace Utils::String;

WriteBufferDBJob::WriteBufferDBJob(JobOrderer* orderer, boost::function<void(Job*)> done_function,
                                   boost::function<void(Job*)> obsolete_function,
                                   DBInterface* db_interface, Buffer* buffer)
    : DBJob(orderer, done_function, obsolete_function, db_interface), buffer_(buffer)
{
    traced_assert(buffer_);
}

WriteBufferDBJob::~WriteBufferDBJob() {}

void WriteBufferDBJob::execute()
{
    logdbg;

    boost::posix_time::ptime loading_start_time_;
    boost::posix_time::ptime loading_stop_time_;

    loading_start_time_ = boost::posix_time::microsec_clock::local_time();

    logdbg << "writing type " << buffer_->getDBContType() << " size "
           << buffer_->getSize();
    db_interface_->writeBuffer(buffer_);

    loading_stop_time_ = boost::posix_time::microsec_clock::local_time();

    double load_time;
    boost::posix_time::time_duration diff = loading_stop_time_ - loading_start_time_;
    load_time = diff.total_milliseconds() / 1000.0;

    logdbg << "buffer write done (" << doubleToString(load_time)
           << " s).";
    done_ = true;
}
