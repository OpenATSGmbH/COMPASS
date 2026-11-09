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

#include "property.h"
#include "logger.h"
#include "traced_assert.h"

#include <boost/assign/list_of.hpp>

#include <limits>

/**
 * PropertyDataType => string repr
 */
const std::map<PropertyDataType, std::string>& Property::dataTypes2Strings()
{
    static const auto* map = new std::map<PropertyDataType, std::string>
    {{PropertyDataType::BOOL, "BOOL"},
        {PropertyDataType::CHAR, "CHAR"},
        {PropertyDataType::UCHAR, "UCHAR"},
        {PropertyDataType::INT, "INT"},
        {PropertyDataType::UINT, "UINT"},
        {PropertyDataType::LONGINT, "LONGINT"},
        {PropertyDataType::ULONGINT, "ULONGINT"},
        {PropertyDataType::FLOAT, "FLOAT"},
        {PropertyDataType::DOUBLE, "DOUBLE"},
        {PropertyDataType::STRING, "STRING"},
        {PropertyDataType::JSON, "JSON"},
        {PropertyDataType::TIMESTAMP, "TIMESTAMP"}};
    return *map;
}

/**
 * PropertyDataType => sql data type string repr
 */
const std::map<PropertyDataType, std::string>& Property::dbDataTypes2Strings(bool precise_types)
{
    static const auto* map = new std::map<PropertyDataType, std::string>
       {{PropertyDataType::BOOL, "TINYINT"},
        {PropertyDataType::CHAR, "TINYINT"},
        {PropertyDataType::UCHAR, "TINYINT"},
        {PropertyDataType::INT, "INT"},
        {PropertyDataType::UINT, "INT"},
        {PropertyDataType::LONGINT, "BIGINT"},
        {PropertyDataType::ULONGINT, "BIGINT"},
        {PropertyDataType::FLOAT, "FLOAT"},
        {PropertyDataType::DOUBLE, "DOUBLE"},
        {PropertyDataType::STRING, "TEXT"},
        {PropertyDataType::JSON, "TEXT"},
        {PropertyDataType::TIMESTAMP, "BIGINT"}};

    static const auto* map_precise = new std::map<PropertyDataType, std::string>
        {{PropertyDataType::BOOL, "BOOLEAN"},
         {PropertyDataType::CHAR, "TINYINT"},
         {PropertyDataType::UCHAR, "UTINYINT"},
         {PropertyDataType::INT, "INTEGER"},
         {PropertyDataType::UINT, "UINTEGER"},
         {PropertyDataType::LONGINT, "BIGINT"},
         {PropertyDataType::ULONGINT, "UBIGINT"},
         {PropertyDataType::FLOAT, "FLOAT"},
         {PropertyDataType::DOUBLE, "DOUBLE"},
         {PropertyDataType::STRING, "VARCHAR"},
         {PropertyDataType::JSON, "VARCHAR"},
         {PropertyDataType::TIMESTAMP, "BIGINT"}};

    return precise_types ? *map_precise : *map;
}

/**
 * string repr => PropertyDataType
 */
const std::map<std::string, PropertyDataType>& Property::strings2DataTypes()
{
    static const auto* map = new std::map<std::string, PropertyDataType>
    {{"BOOL", PropertyDataType::BOOL},
        {"CHAR", PropertyDataType::CHAR},
        {"UCHAR", PropertyDataType::UCHAR},
        {"INT", PropertyDataType::INT},
        {"UINT", PropertyDataType::UINT},
        {"LONGINT", PropertyDataType::LONGINT},
        {"ULONGINT", PropertyDataType::ULONGINT},
        {"FLOAT", PropertyDataType::FLOAT},
        {"DOUBLE", PropertyDataType::DOUBLE},
        {"STRING", PropertyDataType::STRING},
        {"JSON", PropertyDataType::JSON},
        {"TIMESTAMP", PropertyDataType::TIMESTAMP}};
    return *map;
}

/**
 * sql data type string repr => PropertyDataType
 */
const std::map<std::string, PropertyDataType>& Property::strings2DBDataTypes()
{
    static const auto* map = new std::map<std::string, PropertyDataType>
        {{"BOOLEAN" , PropertyDataType::BOOL},
         {"LOGICAL" , PropertyDataType::BOOL},
         {"TINYINT" , PropertyDataType::CHAR},
         {"INT1"    , PropertyDataType::CHAR},
         {"UTINYINT", PropertyDataType::UCHAR},
         {"INTEGER" , PropertyDataType::INT},
         {"INT4"    , PropertyDataType::INT},
         {"INT"     , PropertyDataType::INT},
         {"SIGNED"  , PropertyDataType::INT},
         {"UINTEGER", PropertyDataType::UINT},
         {"BIGINT"  , PropertyDataType::LONGINT},
         {"INT8"    , PropertyDataType::LONGINT},
         {"LONG"    , PropertyDataType::LONGINT},
         {"UBIGINT" , PropertyDataType::ULONGINT},
         {"FLOAT"   , PropertyDataType::FLOAT},
         {"FLOAT4"  , PropertyDataType::FLOAT},
         {"REAL"    , PropertyDataType::FLOAT},
         {"DOUBLE"  , PropertyDataType::DOUBLE},
         {"FLOAT8"  , PropertyDataType::DOUBLE},
         {"VARCHAR" , PropertyDataType::STRING},
         {"CHAR"    , PropertyDataType::STRING},
         {"BPCHAR"  , PropertyDataType::STRING},
         {"TEXT"    , PropertyDataType::STRING},
         {"STRING"  , PropertyDataType::STRING}};

    return *map;
}

Property::Property(std::string id, PropertyDataType type) : data_type_(type), name_(id)
{
    data_type_str_ = asString(data_type_);
}

const std::string& Property::dbDataTypeString(bool precise_type) const
{
    return Property::asDBString(data_type_, precise_type);
}

const std::string& Property::asString(PropertyDataType type)
{
    if (!dataTypes2Strings().count(type))
    {
        //std::cout << "Property: asString: unkown type " << (unsigned int) type << std::endl;
        logerr << "unkown type " << (unsigned int) type;
    }

    traced_assert(dataTypes2Strings().count(type) > 0);
    return dataTypes2Strings().at(type);
}

PropertyDataType Property::asDataType(const std::string& type)
{
    logdbg << "start" << type;

    if (!strings2DataTypes().count(type))
    {
        //std::cout << "Property: asDataType: unkown type " << type << std::endl;
        logerr << "unkown type " << type;
    }

    traced_assert(strings2DataTypes().count(type) > 0);
    return strings2DataTypes().at(type);
}

const std::string& Property::asDBString(PropertyDataType type, bool precise_type)
{
    if (!dbDataTypes2Strings(precise_type).count(type))
    {
        //std::cout << "Property: asDBString: unkown type " << (unsigned int)type << std::endl;
        logerr << "unkown type " << (unsigned int)type;
    }

    traced_assert(dbDataTypes2Strings(precise_type).count(type) > 0);
    return dbDataTypes2Strings(precise_type).at(type);
}

PropertyDataType Property::asDBDataType(const std::string& db_type)
{
    logdbg << "start" << db_type;

    if (!strings2DBDataTypes().count(db_type))
    {
        //std::cout << "Property: asDBDataType: unkown type " << db_type << std::endl;
        logerr << "unkown type " << db_type;
    }

    traced_assert(strings2DBDataTypes().count(db_type) > 0);
    return strings2DBDataTypes().at(db_type);
}
