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

#include "buffer.h"
#include "dbcontentvariablelookup.h"

class DBContentManager;

namespace dbContent 
{

class BufferAccessor;
class TargetReportAccessor;

/**
*/
class DBContentAccessor
{
public:
    DBContentAccessor();
    virtual ~DBContentAccessor() = default;

    bool add(std::map<std::string, std::shared_ptr<Buffer>> buffers); // something changed flag
    void removeContentBeforeTimestamp(boost::posix_time::ptime remove_before_time);
    void removeEmptyBuffers();
    void clear();

    bool has(const std::string& dbcontent_name) const;
    std::shared_ptr<Buffer> get(const std::string& dbcontent_name);
    std::map<std::string, std::shared_ptr<Buffer>> buffers() { return buffers_; }

    template <typename T>
    bool hasVar(const std::string& dbcontent_name, const Property& var_property) const;
    template <typename T>
    NullableVector<T>& getVar(const std::string& dbcontent_name, const Property& var_property);

    template <typename T>
    bool hasMetaVar(const std::string& dbcontent_name, const Property& metavar_property) const;
    template <typename T>
    NullableVector<T>& getMetaVar(const std::string& dbcontent_name, const Property& metavar_property);

    using BufferIterator = typename std::map<std::string, std::shared_ptr<Buffer>>::iterator;
    BufferIterator begin() { return buffers_.begin(); }
    BufferIterator end() { return buffers_.end(); }

    BufferAccessor bufferAccessor(const std::string& dbcontent_name) const;
    TargetReportAccessor targetReportAccessor(const std::string& dbcontent_name) const;

    void print() const;

protected:
    void updateDBContentLookup();

    std::map<std::string, std::shared_ptr<Buffer>>                  buffers_;          // dbcont name -> buffer
    std::map<std::string, std::shared_ptr<DBContentVariableLookup>> dbcontent_lookup_; // dbcont name -> var lookup
};

/**
*/
template <typename T>
inline bool DBContentAccessor::hasVar(const std::string& dbcontent_name, const Property& var_property) const
{
    return dbcontent_lookup_.count(dbcontent_name) && dbcontent_lookup_.at(dbcontent_name)->hasVar<T>(var_property);
}

/**
*/
template <typename T>
inline NullableVector<T>& DBContentAccessor::getVar(const std::string& dbcontent_name, const Property& var_property)
{
    if (!has(dbcontent_name))
    {
        logerr << "dbcontent " << dbcontent_name << " not present";
        traced_assert(has(dbcontent_name));
    }
    return dbcontent_lookup_.at(dbcontent_name)->getVar<T>(var_property);
}

/**
*/
template <typename T>
inline bool DBContentAccessor::hasMetaVar(const std::string& dbcontent_name, const Property& metavar_property) const
{
    return dbcontent_lookup_.count(dbcontent_name) && dbcontent_lookup_.at(dbcontent_name)->hasMetaVar<T>(metavar_property);
}

/**
*/
template <typename T>
inline NullableVector<T>& DBContentAccessor::getMetaVar(const std::string& dbcontent_name, const Property& metavar_property)
{
    if (!has(dbcontent_name))
    {
        logerr << "dbcontent " << dbcontent_name << " not present";
        traced_assert(has(dbcontent_name));
    }
    return dbcontent_lookup_.at(dbcontent_name)->getMetaVar<T>(metavar_property);
}

} // namespace dbContent
