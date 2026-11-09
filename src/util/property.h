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

#include <map>
#include <vector>
#include <string>

enum class PropertyDataType
{
    BOOL,
    CHAR,
    UCHAR,
    INT,
    UINT,
    LONGINT,
    ULONGINT,
    FLOAT,
    DOUBLE,
    STRING,
    JSON,
    TIMESTAMP
};

/**
 * @brief Base class for a data item identifier
 *
 * Contains a data type (based on PROPERTY_DATA_TYPE) and a string identifier. Also a size in bytes
 * is stored, which is based on the data type.
 *
 */
class Property
{
protected:
    /// @brief Default constructor. Use only if members are overwritten.
    Property() {}

public:
    /// @brief Constructor
    Property(std::string id, PropertyDataType type);
    /// @brief Destructor
    virtual ~Property() {}

    PropertyDataType dataType() const { return data_type_; }
    PropertyDataType& dataTypeRef() { return data_type_; }

    const std::string& dataTypeString() const { return data_type_str_; }
    const std::string& dbDataTypeString(bool precise_type) const;
    std::string& dataTypeStringRef() { return data_type_str_; }
    void dataType(PropertyDataType type)
    {
        data_type_ = type;
        data_type_str_ = asString(data_type_);
    }

    const std::string& name() const { return name_; }
    void rename(const std::string& name) { name_ = name; }; // for buffer nullablevector renaming

    static const std::string& asString(PropertyDataType type);
    static PropertyDataType asDataType(const std::string& type);
    static const std::string& asDBString(PropertyDataType type, bool precise_type);
    static PropertyDataType asDBDataType(const std::string& db_type);

    static const std::map<PropertyDataType, std::string>& dataTypes2Strings();
    static const std::map<PropertyDataType, std::string>& dbDataTypes2Strings(bool precise_types);
    static const std::map<std::string, PropertyDataType>& strings2DataTypes();
    static const std::map<std::string, PropertyDataType>& strings2DBDataTypes();

protected:
    /// Data type
    PropertyDataType data_type_;
    std::string data_type_str_;
    /// String identifier
    std::string name_;

    /// Mappings from PropertyDataType to strings, and back.
    //    static const std::map<PropertyDataType, std::string> data_types_2_strings_;
    //    static const std::map<std::string, PropertyDataType> strings_2_data_types_;
};
