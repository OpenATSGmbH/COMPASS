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

#include "timewindow.h"
#include "timeconv.h"
#include "traced_assert.h"
#include "json.hpp"

using namespace std;

namespace Utils
{

TimeWindow::TimeWindow() {}

TimeWindow::TimeWindow(const boost::posix_time::ptime& begin, const boost::posix_time::ptime& end)
{
    std::get<0>(time_window_) = begin;
    std::get<1>(time_window_) = end;

    traced_assert(valid());
}

bool TimeWindow::valid() const
{
    if (std::get<0>(time_window_).is_not_a_date_time())
        return false;

    if (std::get<1>(time_window_).is_not_a_date_time())
        return false;

    return std::get<0>(time_window_) <= std::get<1>(time_window_);
}

void TimeWindow::setFrom(const nlohmann::json& json)
{
    traced_assert(json.is_array());
    traced_assert(json.size() == 2);

    traced_assert(json.at(0).is_string());
    traced_assert(json.at(1).is_string());

    time_window_ = {{},{}};

    std::get<0>(time_window_) = Time::fromString(json.at(0));
    std::get<1>(time_window_) = Time::fromString(json.at(1));

    traced_assert(valid());
}

nlohmann::json TimeWindow::getAsJson() const
{
    traced_assert(valid());

    nlohmann::json json_result = nlohmann::json::array();

    json_result.push_back(Time::toString(std::get<0>(time_window_)));
    json_result.push_back(Time::toString(std::get<1>(time_window_)));

    return json_result;
}

std::string TimeWindow::asStr() const
{
    return Time::toString(std::get<0>(time_window_)) + " - " + Time::toString(std::get<1>(time_window_));
}

bool TimeWindow::contains(const boost::posix_time::ptime& ts) const
{
    return (std::get<0>(time_window_) <= ts && std::get<1>(time_window_) >= ts);
}

bool TimeWindow::contains(const TimeWindow& tw) const
{
    return (std::get<0>(time_window_) <= tw.begin() && std::get<1>(time_window_) >= tw.end());
}

const boost::posix_time::ptime& TimeWindow::begin() const
{
    return std::get<0>(time_window_);
}
const boost::posix_time::ptime& TimeWindow::end() const
{
    return std::get<1>(time_window_);
}


// --------------------

TimeWindowCollection::TimeWindowCollection()
{}

bool TimeWindowCollection::valid() const
{
    for (const auto& time_window : time_windows_)
        if (!time_window.valid())
            return false;

    return true;
}

bool TimeWindowCollection::contains(const boost::posix_time::ptime& ts) const
{
    for (const auto& time_window : time_windows_)
        if (time_window.contains(ts))
            return true;

    return false;
}

void TimeWindowCollection::setFrom(nlohmann::json& json)
{
    time_windows_.clear();
    traced_assert(json.is_array());

    for (const auto& time_window_array : json)
    {
        traced_assert(time_window_array.is_array());

        TimeWindow time_window;

        time_window.setFrom(time_window_array);
        traced_assert(time_window.valid());


        time_windows_.push_back(std::move(time_window));
    }

    traced_assert(valid());
}

nlohmann::json TimeWindowCollection::asJSON() const
{
    nlohmann::json json_result = nlohmann::json::array();

    for (const auto& time_window : time_windows_)
        json_result.push_back(time_window.getAsJson());

    return json_result;
}

std::string TimeWindowCollection::asString() const
{
    ostringstream ss;

    for (const auto& tw_it : time_windows_)
    {
        if (ss.str().size())
            ss << endl;

        ss << tw_it.asStr();
    }

    return ss.str();
}

const Utils::TimeWindow& TimeWindowCollection::get(unsigned int index)
{
    traced_assert(index < time_windows_.size());
    return time_windows_.at(index);
}


void TimeWindowCollection::add(const TimeWindow& time_window)
{
    traced_assert(time_window.valid());
    time_windows_.push_back(time_window);
}

bool TimeWindowCollection::contains(const TimeWindow& time_window)
{
    for (const auto& tw_it : time_windows_)
    {
        if (tw_it.contains(time_window))
            return true;
    }

    return false;
}

void TimeWindowCollection::erase(unsigned int index)
{
    traced_assert(index < time_windows_.size());
    time_windows_.erase(time_windows_.begin() + index);
}

void TimeWindowCollection::clear()
{
    time_windows_.clear();
}

}


