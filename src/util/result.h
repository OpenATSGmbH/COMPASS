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

#include <boost/optional.hpp>

/**
 */
class Result : public std::pair<bool, std::string>
{
public:
    Result() : std::pair<bool, std::string>(false, "") {}
    Result(bool ok, const std::string& err = "") : std::pair<bool, std::string>(ok, err) {}

    virtual ~Result() = default;

    bool ok() const { return this->first; }
    const std::string& error() const { return this->second; }

    static Result failed(const std::string& err = "") { return Result(false, err); }
    static Result succeeded() { return Result(true, ""); }
};

/**
 */
template<typename T>
class ResultT : public Result
{
public:
    ResultT() = default;
    ResultT(bool ok, const std::string& err = "") : Result(ok, err) {}
    ResultT(bool ok, const std::string& err, const boost::optional<T>& result) : Result(ok, err), result_(result) {}
    ResultT(const Result& result) : Result(result) {}

    virtual ~ResultT() = default;

    void setResult(const T& result) { result_ = result; }
    void resetResult() { result_.reset(); }
    bool hasResult() const { return result_.has_value(); }
    const T& result() const { traced_assert(hasResult()); return result_.value(); }

    static ResultT<T> succeeded(const boost::optional<T>& result) { return ResultT<T>(true, "", result); }

private:
    boost::optional<T> result_;
};
