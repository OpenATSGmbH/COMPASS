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

#include <memory>
#include <string>

class Buffer;

/**
 * @brief Generalized result of database query
 *
 * Simple encapsulation of a buffer (result data from query) and a flag indicating if a buffer was
 * set.
 */
class DBResult
{
  public:
    /// @brief Constructor with parameters
    DBResult(std::shared_ptr<Buffer> buffer) : has_more_(false), contains_data_(true), buffer_(buffer) {}
    /// @brief Constructor with parameters
    DBResult(std::shared_ptr<Buffer> buffer, bool has_more) : has_more_(has_more), contains_data_(true), buffer_(buffer) {}
    /// @brief Default constructor
    DBResult() : has_more_(false), contains_data_(false) {}
    /// @brief Destructor
    virtual ~DBResult() {}

    /// @brief Sets the result buffer
    void buffer(std::shared_ptr<Buffer> buffer)
    {
        buffer_ = buffer;
        contains_data_ = true;
    }

    /// @brief Returns the result buffer
    std::shared_ptr<Buffer> buffer() const { return buffer_; }

    /// @brief Sets the has more flag
    void hasMore(bool has_more)
    {
        has_more_ = has_more;
    }

    /// @brief Returns the has more flag
    bool hasMore() const { return has_more_; }

    /// @brief Returns if contains data flag was set
    bool containsData() const { return contains_data_; }

    /// @brief Sets error state
    void setError(const std::string& err)
    {
        has_error_ = true;
        error_msg_ = err;
    }

    /// @brief Checks error state
    bool hasError() const
    {
        return has_error_;
    }

    /// @brief Returns the current error message
    const std::string& error() const
    {
        return error_msg_;
    }

    std::string printResult() const;

  private:
    /// @brief Flag signalling if more data is to be expected
    bool has_more_ = false;
    /// @brief Contains result data flag
    bool contains_data_;
    /// @brief Result data buffer
    std::shared_ptr<Buffer> buffer_;
    /// @brief Errors
    bool has_error_ = false;
    std::string error_msg_;
};
