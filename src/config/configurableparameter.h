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

#include "traced_assert.h"

#include <string>

#include "json.hpp"

class Configurable;

template<typename T>
class ConfigurableParameterT;

/**
 * Base class for configurable parameters, used to hide the concrete value type.
 */
class ConfigurableParameter
{
public:
    explicit ConfigurableParameter(const std::string& parameter_id);
    virtual ~ConfigurableParameter() = default;

    //ConfigurableParameter(const ConfigurableParameter& source);
    //virtual ConfigurableParameter& operator=(const ConfigurableParameter& source);

    template<typename T>
    ConfigurableParameterT<T>* as()
    {
        ConfigurableParameterT<T>* param = dynamic_cast<ConfigurableParameterT<T>*>(this);
        traced_assert(param);
        return param;
    }

    template<typename T>
    const ConfigurableParameterT<T>* as() const
    {
        const ConfigurableParameterT<T>* param = dynamic_cast<const ConfigurableParameterT<T>*>(this);
        traced_assert(param);
        return param;
    }

    template<typename T>
    bool isType() const
    {
        const ConfigurableParameterT<T>* param = dynamic_cast<const ConfigurableParameterT<T>*>(this);
        return (param != nullptr);
    }

    /// Returns parameter identifier
    const std::string& getParameterId() const;

    virtual void resetToDefault() = 0;
    virtual std::string getParameterType() const = 0;
    virtual std::string getParameterValueString() const = 0;
    virtual bool hasStoredPointer() const = 0;
    virtual void toJSON(nlohmann::json& json_obj) const = 0;

protected:
    friend class Configuration;

    virtual void setValue(const nlohmann::json& json_value) = 0;

    /// Parameter identifier
    std::string parameter_id_;
};

/**
 * @brief Configuration parameter template class
 *
 * @details Implements a configurable parameter of a concrete type using a templated class.
 */
template <class T>
class ConfigurableParameterT : public ConfigurableParameter
{
  public:
    /// Constructor
    explicit ConfigurableParameterT(const std::string& parameter_id);
    explicit ConfigurableParameterT(const std::string& parameter_id,
                                    T* pointer,
                                    const T& config_value,
                                    const T& default_value);
    /// Destructor
    virtual ~ConfigurableParameterT() = default;

    /// Returns the parameter value
    T getParameterValue() const;

    /// Returns the default value
    const T& getDefaultValue() const;

    /// Returns the config value
    const T& getConfigValue() const;

    /// Returns the stored value pointer
    const T* getStoredPointer() const;

    /**
     * Returns the parameter type as string
     *
     * \exception std::runtime_error if unknown class is used
     */
    std::string getParameterType() const override final;

    /// Returns parameter value as string (using a stringstream)
    std::string getParameterValueString() const override final;

    /// Sets pointer_ to default value if valid, otherwise sets config_value_ to default_value_
    void resetToDefault() override final;

    /// Checks if the parameter obtains a stored external pointer
    bool hasStoredPointer() const override final;

    /// Writes the parameter's value to the json object
    void toJSON(nlohmann::json& json_obj) const override final;

    /// Updates the parameter members
    void update(T* pointer, const T& config_value, const T& default_value, bool update_pointer);
    void update(T* pointer, const T& default_value, bool update_pointer);
    void update(T* pointer);

    static T valueFromJSON(const nlohmann::json& json_value);

protected:
    const T* getValuePointer() const;
    T* getValuePointer();

    void setValue(const nlohmann::json& json_value) override final;

    /// Template pointer to real value
    T* pointer_ = nullptr;
    /// Template configuration value
    T config_value_;
    /// Template default value as given by registerParameter
    T default_value_;
};

template <>
std::string ConfigurableParameterT<float>::getParameterValueString() const;
template <>
std::string ConfigurableParameterT<double>::getParameterValueString() const;
template <>
std::string ConfigurableParameterT<std::string>::getParameterValueString() const;
template <>
std::string ConfigurableParameterT<nlohmann::json>::getParameterValueString() const;
