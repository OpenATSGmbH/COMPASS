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

#include "configurableparameter.h"
#include "traced_assert.h"

#include "logger.h"
#include "stringconv.h"

using namespace Utils;

/************************************************************************************
 * ConfigurableParameter
 ************************************************************************************/

/**
*/
ConfigurableParameter::ConfigurableParameter(const std::string& parameter_id)
:   parameter_id_(parameter_id)
{
}

// /**
// */
// ConfigurableParameter::ConfigurableParameter(const ConfigurableParameter& source)
// {
//     operator=(source);
// }

// /**
// */
// ConfigurableParameter& ConfigurableParameter::operator=(const ConfigurableParameter& source)
// {
//     parameter_id_ = source.parameter_id_;

//     return *this;
// }

/**
*/
const std::string& ConfigurableParameter::getParameterId() const
{
    return parameter_id_;
}

/************************************************************************************
 * ConfigurableParameterT
 ************************************************************************************/

/**
*/
template <typename T>
ConfigurableParameterT<T>::ConfigurableParameterT(const std::string& parameter_id)
:   ConfigurableParameter(parameter_id)
{
}

/**
*/
template <typename T>
ConfigurableParameterT<T>::ConfigurableParameterT(const std::string& parameter_id,
                                                  T* pointer,
                                                  const T& config_value,
                                                  const T& default_value)
:   ConfigurableParameter(parameter_id)
,   pointer_             (pointer)
,   config_value_        (config_value)
,   default_value_       (default_value)
{
}

/**
 * Returns the stored value of the parameter, either the value of the stored pointer or the stored config value.
*/
template <typename T>
const T* ConfigurableParameterT<T>::getValuePointer() const
{
    //return pointer if stored
    if (hasStoredPointer())
        return pointer_;

    //otherwise return pointer to config value
    return &config_value_;
}

/**
 * Returns the stored value of the parameter, either the value of the stored pointer or the stored config value.
*/
template <typename T>
T* ConfigurableParameterT<T>::getValuePointer()
{
    //return pointer if stored
    if (hasStoredPointer())
        return pointer_;

    //otherwise return pointer to config value
    return &config_value_;
}

/**
*/
template <typename T>
T ConfigurableParameterT<T>::getParameterValue() const
{
    auto ptr = getValuePointer();
    return *ptr;
}

/**
*/
template <typename T>
const T& ConfigurableParameterT<T>::getDefaultValue() const
{
    return default_value_;
}

/**
*/
template <typename T>
const T& ConfigurableParameterT<T>::getConfigValue() const
{
    return config_value_;
}

/**
*/
template <typename T>
const T* ConfigurableParameterT<T>::getStoredPointer() const
{
    return pointer_;
}

/**
*/
template <typename T>
bool ConfigurableParameterT<T>::hasStoredPointer() const
{
    return (pointer_ != nullptr);
}

/**
*/
template <typename T>
std::string ConfigurableParameterT<T>::getParameterType() const
{
    //not implemented for this type, throw
    throw std::runtime_error("ConfigurableParameter: getParameterType: unknown class type");
}

/**
*/
template <typename T>
std::string ConfigurableParameterT<T>::getParameterValueString() const
{
    //basic version just uses to string (might result in compile time error if to_string is not available for type)
    return std::to_string(getParameterValue());
}

/**
*/
template <typename T>
void ConfigurableParameterT<T>::resetToDefault()
{
    std::stringstream ss;
    ss << "ConfigurableParameter: resetToDefault: parameter '" << parameter_id_ << "' default value '" << default_value_ << "'";

    if (hasStoredPointer())
        ss << " ptr not null value '" << *pointer_ << "'";
    else
        ss << " ptr null";

    auto ptr = getValuePointer();
    *ptr = default_value_;

    loginf << ss.str();
}

/**
*/
template <typename T>
void ConfigurableParameterT<T>::toJSON(nlohmann::json& json_obj) const
{
    json_obj[getParameterId()] = getParameterValue();
}

/**
*/
template <typename T>
void ConfigurableParameterT<T>::update(T* pointer, 
                                       const T& config_value, 
                                       const T& default_value, 
                                       bool update_pointer)
{
    traced_assert(pointer);

    pointer_       = pointer;
    config_value_  = config_value;
    default_value_ = default_value;

    if (update_pointer)
        *pointer = config_value_;
}

/**
*/
template <typename T>
void ConfigurableParameterT<T>::update(T* pointer, 
                                       const T& default_value, 
                                       bool update_pointer)
{
    traced_assert(pointer);

    pointer_       = pointer;
    default_value_ = default_value;

    if (update_pointer)
        *pointer = config_value_;
}

/**
*/
template <typename T>
void ConfigurableParameterT<T>::update(T* pointer)
{
    traced_assert(pointer);

    pointer_ = pointer;
}

/**
 * Sets the pointer value to the value contained in the json object.
 */
template <typename T>
void ConfigurableParameterT<T>::setValue(const nlohmann::json& json_value)
{
    traced_assert(hasStoredPointer());

    *pointer_ = valueFromJSON(json_value);
}

/**
*/
template <>
std::string ConfigurableParameterT<float>::getParameterValueString() const
{
    return Utils::String::doubleToStringPrecision(getParameterValue(), 8);
}

/**
*/
template <>
std::string ConfigurableParameterT<double>::getParameterValueString() const
{
    return Utils::String::doubleToStringPrecision(getParameterValue(), 12);
}

/**
*/
template <>
std::string ConfigurableParameterT<std::string>::getParameterValueString() const
{
    return getParameterValue();
}

/**
*/
template <>
std::string ConfigurableParameterT<nlohmann::json>::getParameterValueString() const
{
    return getValuePointer()->dump(4);
}

/**
*/
template <typename T>
T ConfigurableParameterT<T>::valueFromJSON(const nlohmann::json& json_value)
{
    if (std::is_same<T,nlohmann::json>::value == true)
        return json_value;

    T value;
    try
    {
        //try to explicitely convert to the template type => might throw
        value = json_value.get<T>();
    }
    catch(...)
    {
        //bad conversion
        traced_assert(false);
    }
    
    return value;
}

template class ConfigurableParameterT<bool>;
template class ConfigurableParameterT<int>;
template class ConfigurableParameterT<unsigned int>;
template class ConfigurableParameterT<float>;
template class ConfigurableParameterT<double>;
template class ConfigurableParameterT<std::string>;
template class ConfigurableParameterT<nlohmann::json>;
