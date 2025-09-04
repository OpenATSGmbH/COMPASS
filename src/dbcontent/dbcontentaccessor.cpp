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

#include "dbcontentaccessor.h"
#include "compass.h"
#include "dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "bufferaccessor.h"
#include "targetreportaccessor.h"
#include "timeconv.h"
#include "reconstructorbase.h"

using namespace std;
using namespace Utils;

namespace dbContent 
{

/**
*/
DBContentAccessor::DBContentAccessor()
{
}

/**
*/
bool DBContentAccessor::add(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    loginf;

    bool something_changed = false;

    for (auto& buf_it : buffers)
    {
        traced_assert(buf_it.second);

        if (!buf_it.second->size()) // empty buffer
            continue;

        if (buffers_.count(buf_it.first))
        {
            logdbg << "adding buffer dbcont " << buf_it.first
                   << " adding size " << buf_it.second->size() << " current size " << buffers_.at(buf_it.first)->size();

            buffers_.at(buf_it.first)->seizeBuffer(*buf_it.second.get());

            logdbg << "new buffer dbcont " << buf_it.first
                   << " size " << buffers_.at(buf_it.first)->size();
        }
        else
        {
            buffers_[buf_it.first] = std::move(buf_it.second);

            logdbg << "created buffer dbcont " << buf_it.first
                   << " size " << buffers_.at(buf_it.first)->size();
        }
        something_changed = true;
    }

    if (something_changed)
        updateDBContentLookup();

    loginf << "done";

    return something_changed;
}

void DBContentAccessor::removeContentBeforeTimestamp(boost::posix_time::ptime remove_before_time)
{
    loginf;

    unsigned int buffer_size;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
    loginf << "remove_before_time "
           << Time::toString(remove_before_time);
#endif

    for (auto buf_it = buffers_.begin(); buf_it != buffers_.end();) // iterate below
    {
        buffer_size = buf_it->second->size();

        logdbg << "dbcont " << buf_it->first
               << " size " << buf_it->second->size();

        if (buffer_size == 0)
        {
            ++buf_it;
            continue;
        }

        traced_assert(dbcont_man.metaVariable(DBContent::meta_var_timestamp_.name()).existsIn(buf_it->first));

        dbContent::Variable& ts_var = dbcont_man.metaVariable(
                                                    DBContent::meta_var_timestamp_.name()).getFor(buf_it->first);

        Property ts_prop {ts_var.name(), ts_var.dataType()};
        traced_assert(buf_it->second->hasProperty(ts_prop));

        NullableVector<boost::posix_time::ptime>& ts_vec = buf_it->second->get<boost::posix_time::ptime>(ts_var.name());

#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
        traced_assert(ts_vec.isNeverNull());
#endif

        unsigned int index=0;
        bool cutoff_found = false;

        logdbg << "looking for cutoff";

        for (; index < buffer_size; ++index)
        {
            if (!ts_vec.isNull(index) && ts_vec.get(index) > remove_before_time)
            {
#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
                loginf << "found " << buf_it->first
                       << " cutoff tod index " << index
                       << " ts " << Time::toString(ts_vec.get(index));
#endif

                cutoff_found = true;
                break; // index is on first index where ts > remove_before_time
            }
        }

        logdbg << "cutoff_found " << cutoff_found;

        if (!cutoff_found) // no ts bigger than remove:ts found, remove all data
        {
#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
            loginf << "removing full buffer " << buf_it->first
                   << " last index time "
                   << (ts_vec.isNull(buffer_size-1) ? "null" : Time::toString(ts_vec.get(buffer_size-1)));
#endif

            buf_it = buffers_.erase(buf_it);

            logdbg << "removing full buffer done";
        }
        else if (cutoff_found && index != 0) // if index == 0, all ok, otherwise remove
        {
            logdbg << "found cutoff at index " << index;

            traced_assert(index >= 1);
            traced_assert(index < buffer_size);

            index--; // cut at previous

#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
            loginf << "cutting " << buf_it->first
                   << " up to index " << index
                   << " total size " << buffer_size
                   << " index time " << (ts_vec.isNull(index) ? "null" : Time::toString(ts_vec.get(index)));
#endif

            buf_it->second->cutUpToIndex(index); // delete

#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING

            buffer_size = buf_it->second->size();
            index=0;

            for (; index < buffer_size; ++index)
            {
                if (!ts_vec.isNull(index))
                {
                    if (ts_vec.get(index) <= remove_before_time)
                        logerr << "index " << index
                               << " " << Time::toString(ts_vec.get(index));

                    traced_assert(ts_vec.get(index) > remove_before_time);
                }
            }
#endif
            ++buf_it;

        }
        else
        {
#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
            loginf << "keeping full buffer " << buf_it->first
                   << (ts_vec.isNull(buffer_size-1) ? "null" : Time::toString(ts_vec.get(buffer_size-1)));

            buffer_size = buf_it->second->size();
            index=0;

            for (; index < buffer_size; ++index)
            {
                if (ts_vec.get(index) <= remove_before_time)
                    logerr << "index " << index
                           << " " << Time::toString(ts_vec.get(index));

                traced_assert(ts_vec.get(index) > remove_before_time);
            }
#endif
            ++buf_it;
        }

        logdbg << "processing buffer done";
    }

    logdbg << "removing empty buffers";

    removeEmptyBuffers();

    logdbg << "done";
}

void DBContentAccessor::removeEmptyBuffers()
{
    logdbg << "start";

    bool something_changed = false;

    // remove empty buffers
    std::map<std::string, std::shared_ptr<Buffer>> tmp_data = buffers_;

    for (auto& buf_it : tmp_data)
        if (!buf_it.second->size())
        {
            buffers_.erase(buf_it.first);
            something_changed = true;
        }

    if (something_changed)
        updateDBContentLookup();

    logdbg << "done";
}

/**
*/
void DBContentAccessor::clear()
{
    buffers_.clear();
    dbcontent_lookup_.clear();
}

/**
*/
bool DBContentAccessor::has(const std::string& dbcontent_name) const
{
    return buffers_.count(dbcontent_name);
}

/**
*/
std::shared_ptr<Buffer> DBContentAccessor::get(const std::string& dbcontent_name)
{
    traced_assert(has(dbcontent_name));
    return buffers_.at(dbcontent_name);
}

/**
*/
void DBContentAccessor::updateDBContentLookup()
{
    loginf;

    dbcontent_lookup_.clear();

    for (auto& buf_it : buffers_)
    {
        //generate lookup for buffer's dbcontent
        auto& lookup = dbcontent_lookup_[buf_it.first];
        lookup = std::shared_ptr<DBContentVariableLookup>(new DBContentVariableLookup(buf_it.first, buf_it.second));
        lookup->update(COMPASS::instance().dbContentManager());
    }
}

/**
*/
BufferAccessor DBContentAccessor::bufferAccessor(const std::string& dbcontent_name) const
{
    traced_assert(has(dbcontent_name));
    return BufferAccessor(dbcontent_lookup_.at(dbcontent_name));
}

/**
*/
TargetReportAccessor DBContentAccessor::targetReportAccessor(const std::string& dbcontent_name) const
{
    traced_assert(has(dbcontent_name));
    return TargetReportAccessor(dbcontent_lookup_.at(dbcontent_name));
}

/**
*/
void DBContentAccessor::print() const 
{
    for (const auto& elem : dbcontent_lookup_)
    {
        elem.second->print();
        std::cout << std::endl;
    }
}

} // namespace dbContent
