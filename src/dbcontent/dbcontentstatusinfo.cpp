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

#include "dbcontentstatusinfo.h"

#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/target/targetreportaccessor.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"

namespace dbContent
{
DBContentStatusInfo::DBContentStatusInfo() {}

dbContent::VariableSet DBContentStatusInfo::getReadSetFor(const std::string& dbcontent_name) const
{
    dbContent::VariableSet read_set;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    auto& dbcont = dbcont_man.dbContent(dbcontent_name);
    traced_assert(dbcont.containsStatusContent());

    // ds id
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));

    // line id
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_line_id_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_line_id_));

    // timestamp
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));

    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_message_type_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_message_type_));

    return read_set;
}
void DBContentStatusInfo::process(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    scan_info_.clear();

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    for (auto& buf_it : buffers)
    {
        const std::string dbcontent_name = buf_it.first;

        if (!dbcont_man.dbContent(buf_it.first).containsStatusContent())
            continue;

        logdbg << "dbcontent " << dbcontent_name << " size " << buf_it.second->size()
        << " properties " << buf_it.second->properties().size();
        //buf_it.second->printProperties();

        auto& dbcontent = dbcont_man.dbContent(dbcontent_name);

        if (!dbcontent.containsStatusContent())
            continue;

        // cat002: 001 North marker message;002 Sector crossing message;
        // cat010: 002 Start of Update Cycle
        // cat019: 001 Start of Update Cycle
        // cat023: ?
        // cat034: 001 North marker message;002 Sector crossing message;
        // cat065: 002 End of Batch, I020 Batch Number 

        unsigned char message_type_cycle{255};

        if (dbcontent_name == "CAT002")
            message_type_cycle = 1;
        else if (dbcontent_name == "CAT010")
            message_type_cycle = 2;
        else if (dbcontent_name == "CAT019")
            message_type_cycle = 1;
        else if (dbcontent_name == "CAT023")
            continue;
        else if (dbcontent_name == "CAT034")
            message_type_cycle = 1;
        else if (dbcontent_name == "CAT065")
            message_type_cycle = 2;
        else
        {
            logwrn << dbcontent_name << " not yet implemented";
            continue;
        }

        traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));
        traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_line_id_));
        traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));
        traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_message_type_));

        Variable& ds_id_var = dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_);
        Variable& line_id_var = dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_line_id_);
        Variable& timestamp_var = dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_);
        Variable& message_type_var = dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_message_type_);

        traced_assert(buf_it.second->has<unsigned int>(ds_id_var.name()));
        NullableVector<unsigned int>& ds_id_vec = buf_it.second->get<unsigned int>(ds_id_var.name());
        traced_assert(buf_it.second->has<unsigned int>(line_id_var.name()));
        NullableVector<unsigned int>& line_id_vec = buf_it.second->get<unsigned int>(line_id_var.name());
        traced_assert(buf_it.second->has<boost::posix_time::ptime>(timestamp_var.name()));
        NullableVector<boost::posix_time::ptime>& timestamp_vec = buf_it.second->get<boost::posix_time::ptime>(timestamp_var.name());
        traced_assert(buf_it.second->has<unsigned char>(message_type_var.name()));
        NullableVector<unsigned char>& message_type_vec = buf_it.second->get<unsigned char>(message_type_var.name());

        NullableVector<unsigned char>* batch_number_vec {nullptr};

        if (dbcontent_name == "CAT065")
        {
            traced_assert(dbcont_man.canGetVariable(dbcontent_name, DBContent::var_cat065_batch_number_));
            Variable& batch_number_var = dbcont_man.getVariable(dbcontent_name, DBContent::var_cat065_batch_number_);
            traced_assert(buf_it.second->has<unsigned char>(batch_number_var.name()));
            batch_number_vec = &buf_it.second->get<unsigned char>(batch_number_var.name());
        }

        traced_assert(ds_id_vec.isNeverNull());
        traced_assert(line_id_vec.isNeverNull());

        unsigned int buffer_size = buf_it.second->size();

        for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
        {
            if (!message_type_vec.isNull(cnt) 
                && message_type_vec.get(cnt) == message_type_cycle 
                && !timestamp_vec.isNull(cnt))
            {
                // skip if batch number does not mark beginning of batch
                if (batch_number_vec && !batch_number_vec->isNull(cnt) && batch_number_vec->get(cnt) != 0)
                    continue;

                scan_info_[ds_id_vec.get(cnt)][line_id_vec.get(cnt)].push_back(
                    timestamp_vec.get(cnt));
            }
        }
    }
}

bool DBContentStatusInfo::hasInfo(unsigned int ds_id, unsigned int line_id) const
{
    if (!scan_info_.count(ds_id))
        return false;

    if (!scan_info_.at(ds_id).count(line_id))
        return false;

    return true;
}

std::vector<boost::posix_time::ptime> DBContentStatusInfo::getInfo(
    unsigned int ds_id, unsigned int line_id)
{
    traced_assert(hasInfo(ds_id, line_id));

    auto outer_it = scan_info_.find(ds_id);
    auto inner_it = outer_it->second.find(line_id);

    std::vector<boost::posix_time::ptime> result = std::move(inner_it->second);
    outer_it->second.erase(inner_it);
    if (outer_it->second.empty())
        scan_info_.erase(outer_it);

    return result;
}

}  // namespace dbContent
