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
 * @brief Encapsulation of a database SQL command
 *
 * @details Contains the command string and the PropertyList of the expected data structure returned
 * by the command. The PropertyList is copied when set and deleted in the destructor.
 */
class DBCommand
{
public:
    /// @brief Constructor
    DBCommand() : expect_data_result_(false) {}
    DBCommand(const PropertyList& list) : expect_data_result_(true), result_list_(list) {}
    /// @brief Destructor
    virtual ~DBCommand() {}

    /// @brief Sets command string
    void set(const std::string& command) { command_ = command; }
    /// @brief Sets PropertyList of exptected data.
    void list(const PropertyList& list)
    {
        result_list_ = list;
        expect_data_result_ = true;
    }

    /// @brief Returns command string
    const std::string& get() const { return command_; }
    /// @brief Returns flag indicating if returned data is expected.
    bool expectsResult() const { return expect_data_result_; }
    /// @brief Returns PropertyList of expected data.
    const PropertyList& resultList() const { return result_list_; }

protected:
    /// SQL Command
    std::string command_;
    /// Flag if return of data is expected
    bool expect_data_result_;
    /// PropertyList of expected data
    PropertyList result_list_;
};

using DBCommandVector = std::vector<DBCommand>;
