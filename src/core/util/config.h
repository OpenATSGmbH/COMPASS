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

#include <string>

#include "json.hpp"

/**
 * @brief Provides access to a simple configuration file
 *
 * Simple configuration file for reading a file containing string identifiers and values
 */

class SimpleConfig
{
  protected:
    bool opened_{false};
    std::string config_filename_;

    /// Container with all defined identifier -> value pairings
    // std::map<std::string,std::string> config_;
    nlohmann::json config_;

    /// @brief Initialises the config class and reads file given by filename
    void loadFile();

  public:
    /// @brief Constructor
    SimpleConfig(const std::string& config_filename);
    /// @brief Destructor
    virtual ~SimpleConfig();

    /// @brief Returns if a given identifier exists
    bool existsId(const std::string& id);

    /// @brief  Gets value from map by id string and converts it to bool
    bool getBool(const std::string& id);
    /// @brief  Gets value from map by id string and converts it to int
    int getInt(const std::string& id);
    /// @brief  Gets value from map by id string and converts it to unsigned int
    unsigned int getUnsignedInt(const std::string& id);
    /// @brief  Gets value from map by id string and converts it to double
    double getDouble(const std::string& id);
    /// @brief  Gets value from map by id string and doesn't convert it
    std::string getString(const std::string& id);
};
