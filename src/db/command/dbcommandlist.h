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

#include "propertylist.h"

/**
 * @brief Encapsulation of several database SQL commands
 *
 * @details Contains the command strings and the PropertyList of the expected data structure
 * returned by the command. The PropertyList is copied when set and deleted in the destructor.
 */
class DBCommandList
{
  public:
    /// @brief Constructor
    DBCommandList() : expect_data_result_(false) {}
    /// @brief  Desctructor
    virtual ~DBCommandList() { commands_.clear(); }

    /// @brief Adds a command string
    void addCommandString(std::string command) { commands_.push_back(command); }
    /// @brief Adds a vector of command strings
    void addCommandStrings(std::vector<std::string> commands)
    {
        commands_.insert(commands_.end(), commands.begin(), commands.end());
    }
    /// @brief Sets the PropertyList of the expected data
    void setPropertyList(PropertyList list)
    {
        result_list_ = list;
        expect_data_result_ = true;
    }

    /// @brief Returns command string at index i
    const std::string& getCommandString(unsigned int i) const
    {
        traced_assert(i < commands_.size());
        return commands_.at(i);
    }
    /// @brief Returns number of commands
    unsigned int getNumCommands() const { return commands_.size(); }
    /// @brief Returns command container
    const std::vector<std::string>& getCommands() const { return commands_; }
    /// @brief Returns flag indicating if returned data is expected
    bool getExpectDataResult() const { return expect_data_result_; }
    /// @brief Returns PropertyList of the expected returned data
    const PropertyList& getResultList() const { return result_list_; }

  protected:
    /// Command list container
    std::vector<std::string> commands_;
    /// Flag indicating if returned data is expected
    bool expect_data_result_;
    /// PropertyList of the expected returned data
    PropertyList result_list_;
};
