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

#include "buffercsvexportjob.h"
#include "buffer_value_string.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"

#include <fstream>
#include <sstream>

BufferCSVExportJob::BufferCSVExportJob(std::shared_ptr<Buffer> buffer,
                                       const dbContent::VariableSet& read_set, const std::string& file_name,
                                       bool overwrite, bool only_selected, bool use_presentation)
    : Job("BufferCSVExportJob"),
      buffer_(buffer),
      read_set_(read_set),
      file_name_(file_name),
      overwrite_(overwrite),
      only_selected_(only_selected),
      use_presentation_(use_presentation)
{
    traced_assert(file_name_.size());
}

BufferCSVExportJob::~BufferCSVExportJob() {}

void BufferCSVExportJob::run_impl()
{
    logdbg;
    started_ = true;

    start_time_ = boost::posix_time::microsec_clock::local_time();

    std::ofstream output_file;

    if (overwrite_)
        output_file.open(file_name_, std::ios_base::out);
    else
        output_file.open(file_name_, std::ios_base::app);

    if (output_file)
    {
        size_t read_set_size = read_set_.getSize();
        size_t buffer_size = buffer_->size();
        std::stringstream ss;
        std::string value_str;
        size_t row = 0;

        ss << "Selected";

        for (size_t col = 0; col < read_set_size; col++)
        {
            ss << ";";
            ss << read_set_.getVariable(col).name();
        }
        output_file << ss.str() << "\n";

        traced_assert(buffer_->has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buffer_->get<bool>(DBContent::selected_var.name());

        for (; row < buffer_size; ++row)
        {
            if (only_selected_ && (selected_vec.isNull(row) || !selected_vec.get(row)))
                continue;

            ss.str("");

            if (selected_vec.isNull(row))
                ss << "0";
            else
                ss << selected_vec.get(row);

            for (size_t col = 0; col < read_set_size; col++)
            {
                dbContent::Variable& variable = read_set_.getVariable(col);

                bool is_null = false;
                value_str = buffer_utils::getValueString(
                    variable, *buffer_, row, use_presentation_, is_null);

                ss << ";";
                ss << value_str;
            }

            output_file << ss.str() << "\n";
        }

        stop_time_ = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration diff = stop_time_ - start_time_;

        if (diff.total_seconds() > 0)
            loginf << "done after " << diff << ", "
                   << 1000.0 * row / diff.total_milliseconds() << " el/s";
    }
    else
    {
        logerr << "failure opening " << file_name_;
    }

    done_ = true;

    logdbg << "done";
    return;
}
