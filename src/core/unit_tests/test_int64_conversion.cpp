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

// 64-bit integer conversions. long is 32 bit on Windows (LLP64), so COMPASS uses
// std::int64_t and std::uint64_t. These tests check that no value range or precision
// is lost for timestamps, buffers, JSON and string parsing.
// Expected epoch milliseconds were computed independently (Python datetime, UTC).

#include "catch.hpp"
#include "timeconv.h"
#include "buffer.h"

#include <json.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <type_traits>

using namespace Utils;
using namespace boost::posix_time;
using namespace boost::gregorian;

namespace
{
const std::int64_t Int32Max = std::numeric_limits<std::int32_t>::max();

struct TimestampCase
{
    ptime        time;
    std::int64_t epoch_ms;
};

std::vector<TimestampCase> timestampCases()
{
    return {
        { ptime(date(1970, 1, 1)), 0 },
        { ptime(date(2026, 11, 3), hours(12) + minutes(34) + seconds(56) + milliseconds(789)), 1793709296789LL },
        // first millisecond after 2^31 seconds (the 32-bit time_t limit)
        { ptime(date(2038, 1, 19), hours(3) + minutes(14) + seconds(8) + milliseconds(1)), 2147483648001LL },
        { ptime(date(2100, 12, 31), hours(23) + minutes(59) + seconds(59) + milliseconds(999)), 4133980799999LL }
    };
}
}

TEST_CASE("int64 types are 64 bit and equal to long on 64-bit Linux", "[int64]")
{
    static_assert(sizeof(std::int64_t) == 8, "std::int64_t must have 8 bytes");
    static_assert(sizeof(std::uint64_t) == 8, "std::uint64_t must have 8 bytes");

#if defined(__linux__) && defined(__x86_64__)
    // the replacement of long by std::int64_t does not change the Linux types
    static_assert(std::is_same<std::int64_t, long>::value, "std::int64_t is long on 64-bit Linux");
    static_assert(std::is_same<std::uint64_t, unsigned long>::value, "std::uint64_t is unsigned long on 64-bit Linux");
#endif

    SUCCEED();
}

TEST_CASE("Time toLong and fromLong keep milliseconds beyond 32 bit", "[int64][timeconv]")
{
    for (const auto& c : timestampCases())
    {
        INFO("timestamp " << to_simple_string(c.time));

        std::int64_t ms = Time::toLong(c.time);
        CHECK(ms == c.epoch_ms);

        // round trip is exact to the millisecond
        CHECK(Time::fromLong(static_cast<std::uint64_t>(ms)) == c.time);
    }

    // current data is far above the 32-bit range
    CHECK(Time::toLong(timestampCases().at(1).time) > Int32Max);
}

TEST_CASE("Time toLong truncates sub-millisecond parts only", "[int64][timeconv]")
{
    ptime t = ptime(date(2026, 11, 3), hours(12) + minutes(34) + seconds(56) + microseconds(789999));

    CHECK(Time::toLong(t) == 1793709296789LL);
}

TEST_CASE("Time Qt UTC correction is reversible with 64-bit values", "[int64][timeconv]")
{
    for (const auto& c : timestampCases())
    {
        INFO("timestamp " << to_simple_string(c.time));

        std::int64_t corrected = Time::correctLongQtUTC(c.epoch_ms);

        // the correction is a time zone offset: at most 14 hours
        CHECK(std::abs(corrected - c.epoch_ms) <= 14LL * 3600 * 1000);
        CHECK(Time::decorrectLongQtUTC(corrected) == c.epoch_ms);
        CHECK(Time::toLongQtUTC(c.time) == corrected);
    }
}

TEST_CASE("Buffer LONGINT and ULONGINT keep the full 64-bit range", "[int64][buffer]")
{
    PropertyList pl;
    pl.addProperty("l", PropertyDataType::LONGINT);
    pl.addProperty("ul", PropertyDataType::ULONGINT);

    Buffer buf(pl);

    const std::vector<std::int64_t> long_values = {
        std::numeric_limits<std::int64_t>::min(), -1, 0, Int32Max + 1, 1793709296789LL,
        std::numeric_limits<std::int64_t>::max() };

    const std::vector<std::uint64_t> ulong_values = {
        0, 4294967296ULL, 9007199254740993ULL, 1793709296789ULL, 12345678901234567890ULL,
        std::numeric_limits<std::uint64_t>::max() };

    REQUIRE(buf.has<std::int64_t>("l"));
    REQUIRE(buf.has<std::uint64_t>("ul"));

    auto& l_vec  = buf.get<std::int64_t>("l");
    auto& ul_vec = buf.get<std::uint64_t>("ul");

    for (size_t i = 0; i < long_values.size(); ++i)
    {
        l_vec.set(i, long_values[ i ]);
        ul_vec.set(i, ulong_values[ i ]);
    }

    for (size_t i = 0; i < long_values.size(); ++i)
    {
        INFO("index " << i);
        REQUIRE_FALSE(l_vec.isNull(i));
        CHECK(l_vec.get(i) == long_values[ i ]);
        CHECK(ul_vec.get(i) == ulong_values[ i ]);
    }
}

TEST_CASE("JSON keeps 64-bit integers", "[int64][json]")
{
    const std::int64_t  big_signed   = 1793709296789LL;
    const std::int64_t  min_signed   = std::numeric_limits<std::int64_t>::min();
    const std::uint64_t max_unsigned = std::numeric_limits<std::uint64_t>::max();

    nlohmann::json j;
    j["ts"]  = big_signed;
    j["min"] = min_signed;
    j["max"] = max_unsigned;

    // round trip through text, as for config files and view points
    nlohmann::json parsed = nlohmann::json::parse(j.dump());

    CHECK(parsed["ts"].get<std::int64_t>() == big_signed);
    CHECK(parsed["min"].get<std::int64_t>() == min_signed);
    CHECK(parsed["max"].get<std::uint64_t>() == max_unsigned);
}

TEST_CASE("String parsing keeps 64-bit integers", "[int64][string]")
{
    CHECK(std::stoll("1793709296789") == 1793709296789LL);
    CHECK(std::stoll("-9223372036854775808") == std::numeric_limits<std::int64_t>::min());
    CHECK(std::stoull("18446744073709551615") == std::numeric_limits<std::uint64_t>::max());
    CHECK(std::stoull("4294967296") == 4294967296ULL);
}
