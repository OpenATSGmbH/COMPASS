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
#include <sstream>
#include <functional>

/**
 */
class LogStream
{
public:
    using CommitFunc = std::function<void(const std::string&)>;

    LogStream(CommitFunc commit)
        : commit_func_(std::move(commit)) {}

    ~LogStream() 
    {
        if (commit_func_)
            commit_func_(buffer_.str());
    }

    LogStream(const LogStream&) = delete;
    LogStream& operator=(const LogStream&) = delete;

    LogStream(LogStream&& other) noexcept
    :   buffer_     (std::move(other.buffer_)     ) // Move the stream buffer
    ,   commit_func_(std::move(other.commit_func_)) // Move the commit function
    {
        // After moving, 'other' is left in a valid but unspecified state.
    }

    // Move assignment operator
    LogStream& operator=(LogStream&& other) noexcept 
    {
        if (this != &other) 
        {
            // Move assign the stream buffer.
            buffer_ = std::move(other.buffer_);
            // Move assign the commit function.
            commit_func_ = std::move(other.commit_func_);
            // 'other' is now in a moved-from state, and its destructor
            // will not perform the commit action if commit_func_ is empty.
        }
        return *this;
    }

    template <typename T>
    LogStream& operator<<(const T& val) 
    {
        buffer_ << val;
        return *this;
    }

private:
    std::ostringstream buffer_;
    CommitFunc         commit_func_;
};
