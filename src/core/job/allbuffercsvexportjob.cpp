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

#include "allbuffercsvexportjob.h"
#include "buffer_utils.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableorderedset.h"
#include "dbcontent/variable/metavariable.h"
#include "global.h"

#include <fstream>
#include <sstream>

using namespace dbContent;

AllBufferCSVExportJob::AllBufferCSVExportJob(
    std::map<std::string, std::shared_ptr<Buffer>> buffers, VariableOrderedSet* read_set,
    std::map<unsigned int, std::string> number_to_dbcont,
    const std::vector<std::pair<unsigned int, unsigned int>>& row_indexes,
    const std::string& file_name, bool overwrite, bool only_selected, bool use_presentation)
    : Job("AllBufferCSVExportJob"),
      buffers_(buffers),
      read_set_(read_set),
      number_to_dbcont_(number_to_dbcont),
      row_indexes_(row_indexes),
      file_name_(file_name),
      overwrite_(overwrite),
      only_selected_(only_selected),
      use_presentation_(use_presentation)
{
    traced_assert(read_set_);
    traced_assert(file_name_.size());
}

AllBufferCSVExportJob::~AllBufferCSVExportJob() {}

void AllBufferCSVExportJob::run_impl()
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
        unsigned int dbcont_num;
        unsigned int buffer_index;

        unsigned int read_set_size = read_set_->getSize();
        std::shared_ptr<Buffer> buffer;

        std::string dbcontent_name;
        std::string variable_dbcontent_name;
        std::string variable_name;

        std::stringstream ss;
        std::string value_str;

        // write the columns
        ss << "Selected;DBContent";

        for (size_t col = 0; col < read_set_size; col++)
        {
            ss << ";" << read_set_->variableDefinition(col).second;
        }
        output_file << ss.str() << "\n";

        // write the data
        DBContentManager& manager = read_set_->dbContentManager();

        for (auto& row_index_it : row_indexes_)
        {
            // set up everything to access the data
            dbcont_num = row_index_it.first;
            buffer_index = row_index_it.second;

            traced_assert(number_to_dbcont_.count(dbcont_num) == 1);
            dbcontent_name = number_to_dbcont_.at(dbcont_num);

            traced_assert(buffers_.count(dbcontent_name) == 1);
            buffer = buffers_.at(dbcontent_name);

            traced_assert(buffer_index < buffer->size());

            traced_assert(buffer->has<bool>(DBContent::selected_var.name()));
            NullableVector<bool>& selected_vec = buffer->get<bool>(DBContent::selected_var.name());

            traced_assert(buffer->has<unsigned long>(DBContent::meta_var_rec_num_.name()));

            // check if skipped because not selected
            if (only_selected_ &&
                (selected_vec.isNull(buffer_index) || !selected_vec.get(buffer_index)))
                continue;

            ss.str("");

            // set selected flag
            if (selected_vec.isNull(buffer_index))
                ss << "0;";
            else
                ss << selected_vec.get(buffer_index) << ";";

            ss << dbcontent_name;  // set dbcontname

            for (unsigned int col = 0; col < read_set_size; ++col)
            {
                std::tie(variable_dbcontent_name, variable_name) = read_set_->variableDefinition(col);

                // check if data & variables exist
                if (variable_dbcontent_name == META_OBJECT_NAME)
                {
                    traced_assert(manager.existsMetaVariable(variable_name));
                    if (!manager.metaVariable(variable_name)
                             .existsIn(dbcontent_name))
                    {
                        ss << ";";
                        continue;
                    }
                }
                else
                {
                    if (dbcontent_name != variable_dbcontent_name)
                    {
                        ss << ";";
                        continue;
                    }

                    traced_assert(manager.existsDBContent(dbcontent_name));
                    traced_assert(manager.dbContent(dbcontent_name).hasVariable(variable_name));
                }

                Variable& variable = (variable_dbcontent_name == META_OBJECT_NAME)
                                            ? manager.metaVariable(variable_name).getFor(dbcontent_name)
                                            : manager.dbContent(dbcontent_name).variable(variable_name);

                bool is_null = false;
                value_str = buffer_utils::getValueString(
                    variable, *buffer, buffer_index, use_presentation_, is_null);

                ss << ";";
                ss << value_str;
            }
            output_file << ss.str() << "\n";
        }
    }
    else
    {
        logerr << "failure opening " << file_name_;
    }

    done_ = true;

    logdbg << "done";
    return;
}
