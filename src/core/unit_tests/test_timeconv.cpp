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
 *
 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#include "catch.hpp"
#include "timeconv.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

using namespace Utils;
using namespace boost::posix_time;
using namespace boost::gregorian;

TEST_CASE("toTimeString show_digits", "[time][timeconv]")
{
    // 12:34:56.789
    const ptime t(date(2024, 1, 1), time_duration(12, 34, 56) + millisec(789));

    SECTION("with digits shows milliseconds")
    {
        REQUIRE(Time::toTimeString(t, true) == "12:34:56.789");
    }

    SECTION("without digits omits milliseconds")
    {
        REQUIRE(Time::toTimeString(t, false) == "12:34:56");
    }

    SECTION("show_digits is honored per call, not frozen by the first invocation")
    {
        // Regression guard: toTimeString used a single function-local static
        // std::locale whose facet was chosen from show_digits on the *first*
        // call, so a first call with show_digits=false permanently suppressed
        // milliseconds for every later show_digits=true call. Both formats must
        // work regardless of call order.
        const std::string with    = Time::toTimeString(t, true);
        const std::string without = Time::toTimeString(t, false);

        REQUIRE(with    == "12:34:56.789");
        REQUIRE(without == "12:34:56");
        REQUIRE(with != without);

        // reverse order too - result must not depend on which was called first
        REQUIRE(Time::toTimeString(t, false) == "12:34:56");
        REQUIRE(Time::toTimeString(t, true)  == "12:34:56.789");
    }
}

TEST_CASE("toTimeString zero milliseconds still shows .000 with digits", "[time][timeconv]")
{
    // The value is fine (fractional part exactly zero); with digits the
    // fractional field must still be printed rather than dropped.
    const ptime t(date(2024, 1, 1), time_duration(0, 0, 0)); // 00:00:00.000

    REQUIRE(Time::toTimeString(t, true)  == "00:00:00.000");
    REQUIRE(Time::toTimeString(t, false) == "00:00:00");
}

TEST_CASE("toTimeString truncates sub-millisecond to 3 digits", "[time][timeconv]")
{
    // 01:02:03.789123 -> milliseconds shown, microseconds dropped
    const ptime t(date(2024, 1, 1), time_duration(1, 2, 3) + microsec(789123));

    REQUIRE(Time::toTimeString(t, true) == "01:02:03.789");
}
