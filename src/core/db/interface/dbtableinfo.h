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

#include "property.h"
#include "propertylist.h"
#include "traced_assert.h"

#include <map>
#include <string>

#include <boost/optional.hpp>

/**
 */
class DBTableColumnInfo
{
public:
    /**
     */
    DBTableColumnInfo(const std::string& name, 
                      PropertyDataType type_prop,
                      bool key,
                      bool null_allowed = true,
                      const std::string& comment = "",
                      bool precise_db_types = true)
    :   name_        (name)
    ,   type_prop_   (type_prop)
    ,   null_allowed_(null_allowed)
    ,   key_         (key)
    ,   comment_     (comment)
    {
        const auto& types2str_db = Property::dbDataTypes2Strings(precise_db_types);
        if (types2str_db.count(type_prop))
            type_db_ = types2str_db.at(type_prop);
    }

    /**
     */
    DBTableColumnInfo(const std::string& name, 
                      const std::string& type_db,
                      bool key,
                      bool null_allowed = true,
                      const std::string& comment = "")
    :   name_        (name)
    ,   type_db_     (type_db)
    ,   null_allowed_(null_allowed)
    ,   key_         (key)
    ,   comment_     (comment)
    {
        const auto& str2types_db = Property::strings2DBDataTypes();
        if (str2types_db.count(type_db))
            type_prop_ = str2types_db.at(type_db);
    }

    virtual ~DBTableColumnInfo() {}

    std::string name() const { return name_; }

    /**
     */
    bool hasPropertyType() const
    {
        return type_prop_.has_value();
    }

    /**
     */
    PropertyDataType propertyType() const
    { 
        traced_assert(type_prop_.has_value());
        return type_prop_.value();
    }

    /**
     */
    bool hasDBType() const
    {
        return type_db_.has_value();
    }

    /**
     */
    const std::string& dbType() const 
    {
        traced_assert(type_db_.has_value());
        return type_db_.value();
    }

    bool nullAllowed() const { return null_allowed_; }
    bool key() const { return key_; }
    std::string comment() const { return comment_; }

protected:
    std::string                       name_;
    boost::optional<PropertyDataType> type_prop_;
    boost::optional<std::string>      type_db_;
    bool                              null_allowed_;
    bool                              key_;
    std::string                       comment_;
};

/**
 */
class DBTableInfo
{
public:
    DBTableInfo() {}
    DBTableInfo(const std::string name) : name_(name) {}
    virtual ~DBTableInfo() {}

    void name(const std::string& name) { name_ = name; }
    const std::string& name() const { return name_; }

    /**
     */
    bool hasColumn(const std::string& name) const 
    { 
        return column_map_.count(name) > 0; 
    }

    /**
     */
    const DBTableColumnInfo& column(const std::string& name) const 
    { 
        auto it = column_map_.find(name);
        traced_assert(it != column_map_.end());

        return columns_.at(it->second);
    }

    /**
     */
    void addColumn(const std::string& name, 
                   const std::string& type, 
                   bool key, 
                   bool null_allowed,
                   const std::string& comment)
    {
        traced_assert(column_map_.count(name) == 0);
        size_t idx = columns_.size();
        columns_.push_back(DBTableColumnInfo(name, type, key, null_allowed, comment));
        column_map_[ name ] = idx;
    }

    /**
     */
    void addColumn(const std::string& name, 
                   PropertyDataType type, 
                   bool key, 
                   bool null_allowed,
                   const std::string& comment,
                   bool precise_db_types = true)
    {
        traced_assert(column_map_.count(name) == 0);
        size_t idx = columns_.size();
        columns_.push_back(DBTableColumnInfo(name, type, key, null_allowed, comment, precise_db_types));
        column_map_[ name ] = idx;
    }

    unsigned int size() { return columns_.size(); }

    const std::vector<DBTableColumnInfo>& columns() const { return columns_; }

    /**
     */
    boost::optional<PropertyList> tableProperties() const
    {
        PropertyList p;

        for (const auto& c : columns_)
        {
            if (!c.hasPropertyType())
                return boost::optional<PropertyList>();
            
            p.addProperty(c.name(), c.propertyType());
        }
        
        return p;
    }

protected:
    std::string                    name_;
    std::vector<DBTableColumnInfo> columns_;
    std::map<std::string, size_t>  column_map_;
};
