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
#include "nullablevector.h"

#include "timeconv.h"

#include <type_traits>

#include <QString>

#include <boost/optional.hpp>

/*****************************************************************************************
 * Helpers for switching property data types.
 *****************************************************************************************/

#define SwitchPropertyDataTypeEntry(Name, Type, Suffix, Func) \
    case Name:                                                \
    {                                                         \
        Func(Name, Type, Suffix)                              \
        break;                                                \
    }

#define SwitchPropertyDataType(Var, PTypeFunc, DefaultFunc)                                                          \
        switch(Var)                                                                                                  \
        {                                                                                                            \
            SwitchPropertyDataTypeEntry(PropertyDataType::BOOL     , bool                    , bool     , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::CHAR     , char                    , char     , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::UCHAR    , unsigned char           , uchar    , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::INT      , int                     , int      , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::UINT     , unsigned int            , uint     , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::LONGINT  , long int                , long     , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::ULONGINT , unsigned long int       , ulong    , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::FLOAT    , float                   , float    , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::DOUBLE   , double                  , double   , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::STRING   , std::string             , string   , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::JSON     , nlohmann::json          , json     , PTypeFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::TIMESTAMP, boost::posix_time::ptime, timestamp, PTypeFunc) \
            default:                                                                                                 \
            {                                                                                                        \
                DefaultFunc                                                                                          \
                break;                                                                                               \
            }                                                                                                        \
        }
        
#define SwitchPropertyDataTypeNumeric(Var, PTypeFunc, StringFunc, JSONFunc, DefaultFunc)                              \
        switch(Var)                                                                                                   \
        {                                                                                                             \
            SwitchPropertyDataTypeEntry(PropertyDataType::BOOL     , bool                    , bool     , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::CHAR     , char                    , char     , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::UCHAR    , unsigned char           , uchar    , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::INT      , int                     , int      , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::UINT     , unsigned int            , uint     , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::LONGINT  , long int                , long     , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::ULONGINT , unsigned long int       , ulong    , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::FLOAT    , float                   , float    , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::DOUBLE   , double                  , double   , PTypeFunc ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::STRING   , std::string             , string   , StringFunc) \
            SwitchPropertyDataTypeEntry(PropertyDataType::JSON     , nlohmann::json          , json     , JSONFunc  ) \
            SwitchPropertyDataTypeEntry(PropertyDataType::TIMESTAMP, boost::posix_time::ptime, timestamp, PTypeFunc ) \
            default:                                                                                                  \
            {                                                                                                         \
                DefaultFunc                                                                                           \
                break;                                                                                                \
            }                                                                                                         \
        }

namespace property_templates
{

/**
*/
template <typename TFunc>
bool invokeFunctor(PropertyDataType dtype, TFunc& func)
{
    switch (dtype)
    {
    case PropertyDataType::BOOL:
        return func.template operator()<bool, PropertyDataType::BOOL>();
    case PropertyDataType::CHAR:
        return func.template operator()<char, PropertyDataType::CHAR>();
    case PropertyDataType::UCHAR:
        return func.template operator()<unsigned char, PropertyDataType::UCHAR>();
    case PropertyDataType::INT:
        return func.template operator()<int, PropertyDataType::INT>();
    case PropertyDataType::UINT:
        return func.template operator()<unsigned int, PropertyDataType::UINT>();
    case PropertyDataType::LONGINT:
        return func.template operator()<long int, PropertyDataType::LONGINT>();
    case PropertyDataType::ULONGINT:
        return func.template operator()<unsigned long int, PropertyDataType::ULONGINT>();
    case PropertyDataType::FLOAT:
        return func.template operator()<float, PropertyDataType::FLOAT>();
    case PropertyDataType::DOUBLE:
        return func.template operator()<double, PropertyDataType::DOUBLE>();
    case PropertyDataType::TIMESTAMP:
        return func.template operator()<boost::posix_time::ptime, PropertyDataType::TIMESTAMP>();
    case PropertyDataType::STRING:
        return func.template operator()<std::string, PropertyDataType::STRING>();
    case PropertyDataType::JSON:
        return func.template operator()<nlohmann::json, PropertyDataType::JSON>();
    default:
        func.error(dtype);
    }
    return false;
}

/*****************************************************************************************
 * Template value to double.
 *****************************************************************************************/

template <typename T>
inline double toDouble(const T& value, bool* ok = nullptr)
{
    if (ok)
        *ok = true;

    return static_cast<double>(value);
}
template <>
inline double toDouble(const boost::posix_time::ptime& value, bool* ok)
{
    if (ok)
        *ok = true;

    return static_cast<double>(Utils::Time::toLong(value));
}
template <>
inline double toDouble(const std::string& value, bool* ok)
{
    //@TODO: maybe just throw?
    if (ok)
        *ok = false;
    
    return 0.0;
}
template <>
inline double toDouble(const nlohmann::json& value, bool* ok)
{
    //@TODO: maybe just throw?
    if (ok)
        *ok = false;
    
    return 0.0;
}

/*****************************************************************************************
 * Template value from double.
 *****************************************************************************************/

template <typename T>
inline T fromDouble(double value)
{
    return static_cast<T>(value);
}
template <>
inline boost::posix_time::ptime fromDouble(double value)
{
    return Utils::Time::fromLong(static_cast<unsigned long>(value));
}
template <>
inline std::string fromDouble(double value)
{
    return std::to_string(value);
}
template <>
inline nlohmann::json fromDouble(double value)
{
    return nlohmann::json();
}

/*****************************************************************************************
 * Template value to string.
 *****************************************************************************************/

template <typename T>
inline std::string toString(const T& value, int decimals)
{
    return std::to_string(value);
}
template <>
inline std::string toString(const std::string& value, int decimals)
{
    return value;
}
template <>
inline std::string toString(const double& value, int decimals)
{
    return QString::number(value, 'f', decimals).toStdString();
}
template <>
inline std::string toString(const float& value, int decimals)
{
    return QString::number(value, 'f', decimals).toStdString();
}
template <>
inline std::string toString(const boost::posix_time::ptime& value, int decimals)
{
    if (value.is_not_a_date_time())
        return "";
    
    return Utils::Time::toString(value);
}
template <>
inline std::string toString(const nlohmann::json& value, int decimals)
{
    return value.dump();
}

/*****************************************************************************************
 * Template value from string.
 *****************************************************************************************/

template <typename T>
inline boost::optional<T> fromString(const std::string& value)
{
    double v;
    try
    {
        size_t n;
        v = std::stod(value, &n);

        if (n != value.size())
            return {};
    }
    catch(...)
    {
        return {};
    }
    return static_cast<T>(v);
}
template <>
inline boost::optional<std::string> fromString(const std::string& value)
{
    return value;
}
template <>
inline boost::optional<boost::posix_time::ptime> fromString(const std::string& value)
{
    if (value.empty())
        return {};

    bool ok;
    boost::posix_time::ptime t = Utils::Time::fromString(value, &ok);
    if (!ok || t.is_not_a_date_time())
        return {};

    return t;
}
template <>
inline boost::optional<nlohmann::json> fromString(const std::string& value)
{
    nlohmann::json j = nlohmann::json::parse(value);
    if (j.is_null())
        return {};

    return j;
}

/*****************************************************************************************
 * Double to string.
 *****************************************************************************************/

struct Double2StringFunctor
{
    template <typename T, PropertyDataType DType>
    bool operator()()
    {
        T value = fromDouble<T>(v);
        v_str = toString<T>(value, dec);
        return true;
    }

    void error(PropertyDataType dtype) {}

    double      v;
    std::string v_str;
    int         dec = 6;
};

inline std::string double2String(PropertyDataType dtype, double v, int decimals)
{
    Double2StringFunctor func;
    func.v   = v;
    func.dec = decimals;
    invokeFunctor(dtype, func);

    return func.v_str;
}

/*****************************************************************************************
 * String to double.
 *****************************************************************************************/

struct String2DoubleFunctor
{
    template <typename T, PropertyDataType DType>
    bool operator()()
    {
        auto value = fromString<T>(v_str);
        if (!value.has_value())
            return false;

        v = toDouble<T>(value.value());
        return true;
    }

    void error(PropertyDataType dtype) {}
    
    std::string v_str;
    double      v;
};

inline boost::optional<double> string2Double(PropertyDataType dtype, const std::string& v_str)
{
    String2DoubleFunctor func;
    func.v_str = v_str;
    if (!invokeFunctor(dtype, func))
        return {};

    return func.v;
}

/*****************************************************************************************
 * Num color steps.
 *****************************************************************************************/

template <typename T>
inline size_t suggestedNumColorSteps(const T& vmin, const T& vmax, size_t steps_default)
{
    traced_assert(vmax >= vmin);

    if (vmin == vmax)
        return 1;

    if (std::is_floating_point<T>::value)
    {
        return steps_default;
    }
    else if (std::is_integral<T>::value)
    {
        return std::min(steps_default, (size_t)(vmax - vmin + 1));
    }

    return steps_default;
}
template <>
inline size_t suggestedNumColorSteps(const std::string& vmin, const std::string& vmax, size_t steps_default)
{
    return 0;
}
template <>
inline size_t suggestedNumColorSteps(const boost::posix_time::ptime& vmin, const boost::posix_time::ptime& vmax, size_t steps_default)
{
    return 0;
}
template <>
inline size_t suggestedNumColorSteps(const nlohmann::json& vmin, const nlohmann::json& vmax, size_t steps_default)
{
    return 0;
}
template <>
inline size_t suggestedNumColorSteps(const bool& vmin, const bool& vmax, size_t steps_default)
{
    return (vmin == vmax ? 1 : 2);
}

struct SuggestedNumColorStepsFunctor
{
    template <typename T, PropertyDataType DType>
    bool operator()()
    {
        T v0 = fromDouble<T>(vmin);
        T v1 = fromDouble<T>(vmax);

        steps = suggestedNumColorSteps<T>(v0, v1, steps_default);

        return true;
    }

    void error(PropertyDataType dtype) {}
    
    size_t steps_default;
    double vmin;
    double vmax;
    size_t steps;
};

inline size_t suggestedNumColorSteps(PropertyDataType dtype, 
                                     double vmin, 
                                     double vmax, 
                                     size_t steps_default)
{
    SuggestedNumColorStepsFunctor func;
    func.steps_default = steps_default;
    func.vmin          = vmin;
    func.vmax          = vmax;

    if (!invokeFunctor(dtype, func))
        return 0;

    return func.steps;
}

} // property_templates
