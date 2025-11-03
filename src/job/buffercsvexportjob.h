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

#include <memory>

//#include "boost/date_time/posix_time/posix_time.hpp"
#include "buffer.h"
#include "dbcontent/variable/variableset.h"
#include "job.h"

class BufferCSVExportJob : public Job
{
  public:
    BufferCSVExportJob(std::shared_ptr<Buffer> buffer, const dbContent::VariableSet& read_set,
                       const std::string& file_name, bool overwrite, bool only_selected,
                       bool use_presentation);
    virtual ~BufferCSVExportJob();

  protected:
    virtual void run_impl();

    std::shared_ptr<Buffer> buffer_;
    dbContent::VariableSet read_set_;

    std::string file_name_;
    bool overwrite_;
    bool only_selected_;
    bool use_presentation_;

    boost::posix_time::ptime start_time_;
    boost::posix_time::ptime stop_time_;
};
