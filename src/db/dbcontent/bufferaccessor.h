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

/**
*/
class BufferAccessor
{
public:
    BufferAccessor(const std::string& dbcontent_name, 
                   const std::shared_ptr<Buffer>& buffer,
                   const DBContentManager& dbcont_man);
    BufferAccessor(const std::shared_ptr<DBContentVariableLookup>& lookup);
    virtual ~BufferAccessor();

    template <typename T>
    bool hasVar(const Property& var_property) const;
    template <typename T>
    NullableVector<T>& getVar(const Property& var_property);

    template <typename T>
    bool hasMetaVar(const Property& metavar_property) const;
    template <typename T>
    NullableVector<T>& getMetaVar(const Property& metavar_property);

    unsigned int size() const;

protected:
    std::shared_ptr<DBContentVariableLookup> lookup_;

private:
    void updateLookup(const DBContentManager& dbcontent_manager);
};

/**
*/
template <typename T>
bool BufferAccessor::hasVar(const Property& var_property) const
{
    return lookup_->hasVar<T>(var_property);
}

/**
*/
template <typename T>
NullableVector<T>& BufferAccessor::getVar(const Property& var_property)
{
    return lookup_->getVar<T>(var_property);
}

/**
*/
template <typename T>
bool BufferAccessor::hasMetaVar(const Property& metavar_property) const
{
    return lookup_->hasMetaVar<T>(metavar_property);
}

/**
*/
template <typename T>
NullableVector<T>& BufferAccessor::getMetaVar(const Property& metavar_property)
{
    return lookup_->getMetaVar<T>(metavar_property);
}


} // namespace dbContent
