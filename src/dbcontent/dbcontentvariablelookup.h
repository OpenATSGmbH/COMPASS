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

class DBContentManager;

namespace dbContent 
{

/**
 * DBContent-specific data held in a DBContentAccessor.
 * Reuasable interface.
 */
class DBContentVariableLookup 
{
public:
    DBContentVariableLookup(const std::string& dbcontent_name,
                            const std::shared_ptr<Buffer>& buffer);
    virtual ~DBContentVariableLookup();

    const std::string dbContentName() const { return dbcontent_name_; }

    template <typename T>
    bool hasVar(const Property& var_property) const;
    template <typename T>
    NullableVector<T>& getVar(const Property& var_property);

    template <typename T>
    bool hasMetaVar(const Property& metavar_property) const;
    template <typename T>
    NullableVector<T>& getMetaVar(const Property& metavar_property);

    void update(const DBContentManager& dbcontent_manager);

    void print() const;

    unsigned int size() const;


private:
    std::string                        dbcontent_name_;
    std::shared_ptr<Buffer>            buffer_;
    std::map<std::string, std::string> meta_var_lookup_; // metavarname -> varname
};

/**
*/
template <typename T>
inline bool DBContentVariableLookup::hasVar(const Property& var_property) const
{
    return buffer_->has<T>(var_property.name());
}

/**
*/
template <typename T>
inline NullableVector<T>& DBContentVariableLookup::getVar(const Property& var_property)
{
    if (!hasVar<T>(var_property))
    {
        logerr << "property " << var_property.name() << " not present";
        traced_assert(hasVar<T>(var_property));
    }
    return buffer_->get<T>(var_property.name());
}

/**
*/
template <typename T>
inline bool DBContentVariableLookup::hasMetaVar(const Property& metavar_property) const
{
    return meta_var_lookup_.count(metavar_property.name()) && 
           buffer_->has<T>(meta_var_lookup_.at(metavar_property.name()));
}

/**
*/
template <typename T>
inline NullableVector<T>& DBContentVariableLookup::getMetaVar(const Property& metavar_property)
{
    if (!hasMetaVar<T>(metavar_property))
    {
        logerr << "property " << metavar_property.name() << " not present";
        traced_assert(hasMetaVar<T>(metavar_property));
    }
    return buffer_->get<T>(meta_var_lookup_.at(metavar_property.name()));
}

} // namespace dbContent
