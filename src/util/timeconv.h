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

#include "boost/date_time/posix_time/posix_time.hpp"

#include <QDateTime>

#include <string>

namespace Utils
{
namespace Time
{
extern const std::string QT_DATETIME_FORMAT;
extern const std::string QT_DATETIME_FORMAT_SHORT;

extern boost::posix_time::ptime fromString(const std::string& value, bool* ok = nullptr);
extern boost::posix_time::ptime fromString(const std::string& value, const std::string& facet, bool* ok = nullptr);
extern boost::posix_time::ptime fromLong(unsigned long value);
extern long toLong(const boost::posix_time::ptime& value);
extern long toLongQtUTC(const boost::posix_time::ptime& value);
extern long correctLongQtUTC(long t);
extern long decorrectLongQtUTC(long t);
extern std::string toString(const boost::posix_time::ptime& value, unsigned int partial_digits=3, bool* ok = nullptr);
extern std::string toString(const boost::posix_time::time_duration& duration, unsigned int partial_digits=3);
extern std::string toStringLong(unsigned long value);
extern std::string toTimeString(const boost::posix_time::ptime& value);
extern std::string toDateString(const boost::posix_time::ptime& value);
extern boost::posix_time::ptime fromDateString(const std::string& value);
extern boost::posix_time::ptime currentUTCTime();
// can be negative but not exceed maxint seconds
extern boost::posix_time::time_duration partialSeconds(double seconds, bool ignore_full_seconds=false);
extern double partialSeconds(const boost::posix_time::time_duration& seconds);

extern QDateTime qtFrom (const boost::posix_time::ptime& value, bool include_ms=true);
extern boost::posix_time::ptime truncateToFullSeconds(const boost::posix_time::ptime& timestamp);

}  // namespace Time

}  // namespace Utils


