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

// DuckDB round trip of 64-bit columns through the COMPASS write path (DBPrepare::executeBuffer,
// bind_long / bind_ulong / bind_timestamp) and the read path (DuckDBExecResult, bulk column
// copy into the Buffer). long is 32 bit on Windows, these columns need std::int64_t /
// std::uint64_t buffers.
// Column types as in Property (property.cpp): LONGINT, ULONGINT and TIMESTAMP are BIGINT,
// a TIMESTAMP is stored as epoch milliseconds. ULONGINT in a signed BIGINT column holds
// values up to 2^63-1 only.

#include "catch.hpp"
#include "duckdbprepare.h"
#include "duckdbexecresult.h"
#include "dbresult.h"
#include "buffer.h"

#include <duckdb.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

using namespace boost::posix_time;
using namespace boost::gregorian;

namespace
{
const std::vector<std::int64_t> LongValues = {
    std::numeric_limits<std::int64_t>::min(), -1, 0, 2147483648LL, 1793709296789LL,
    std::numeric_limits<std::int64_t>::max() };

// up to 2^63-1, the limit of the signed BIGINT column
const std::vector<std::uint64_t> ULongValues = {
    0, 4294967296ULL, 9007199254740993ULL, 1793709296789ULL, 4611686018427387904ULL,
    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) };

std::vector<ptime> timestampValues()
{
    return {
        ptime(date(1970, 1, 1)),
        ptime(date(2026, 11, 3), hours(12) + minutes(34) + seconds(56) + milliseconds(789)),
        ptime(date(2038, 1, 19), hours(3) + minutes(14) + seconds(8) + milliseconds(1)),
        ptime(date(2100, 12, 31), hours(23) + minutes(59) + seconds(59) + milliseconds(999)),
        ptime(date(2026, 11, 3), hours(0)),
        ptime(date(2026, 11, 3), hours(23) + minutes(59) + seconds(59) + milliseconds(999)) };
}

PropertyList testProperties()
{
    PropertyList pl;
    pl.addProperty("l", PropertyDataType::LONGINT);
    pl.addProperty("ul", PropertyDataType::ULONGINT);
    pl.addProperty("ts", PropertyDataType::TIMESTAMP);
    return pl;
}
}

TEST_CASE("DuckDB round trip of 64-bit columns", "[int64][duckdb]")
{
    duckdb_database db;
    duckdb_connection con;

    REQUIRE(duckdb_open(nullptr, &db) == DuckDBSuccess);
    REQUIRE(duckdb_connect(db, &con) == DuckDBSuccess);

    duckdb_result create_result;
    REQUIRE(duckdb_query(con, "CREATE TABLE t (id INTEGER, l BIGINT, ul BIGINT, ts BIGINT)",
                         &create_result) == DuckDBSuccess);
    duckdb_destroy_result(&create_result);

    const auto timestamps = timestampValues();
    const size_t n = LongValues.size();

    // write path: a Buffer with id and the 64-bit columns, one NULL row at the end
    {
        PropertyList pl;
        pl.addProperty("id", PropertyDataType::INT);

        const PropertyList value_properties = testProperties();
        for (const auto& p : value_properties.properties())
            pl.addProperty(p);

        auto buffer = std::make_shared<Buffer>(pl);

        for (size_t i = 0; i < n; ++i)
        {
            buffer->get<int>("id").set(i, static_cast<int>(i));
            buffer->get<std::int64_t>("l").set(i, LongValues[ i ]);
            buffer->get<std::uint64_t>("ul").set(i, ULongValues[ i ]);
            buffer->get<ptime>("ts").set(i, timestamps[ i ]);
        }

        buffer->get<int>("id").set(n, static_cast<int>(n));
        buffer->get<std::int64_t>("l").setNull(n);
        buffer->get<std::uint64_t>("ul").setNull(n);
        buffer->get<ptime>("ts").setNull(n);

        DuckDBPrepare insert(con);
        REQUIRE(insert.init("INSERT INTO t VALUES (?, ?, ?, ?)"));
        REQUIRE(insert.executeBuffer(buffer).ok());
        insert.cleanup();
    }

    // check the stored values directly with the DuckDB C API (independent of the read path)
    {
        duckdb_result r;
        REQUIRE(duckdb_query(con, "SELECT l, ul FROM t ORDER BY id", &r) == DuckDBSuccess);
        REQUIRE(duckdb_row_count(&r) == n + 1);

        for (size_t i = 0; i < n; ++i)
        {
            INFO("row " << i);
            CHECK(duckdb_value_int64(&r, 0, i) == LongValues[ i ]);
            CHECK(duckdb_value_uint64(&r, 1, i) == ULongValues[ i ]);
        }

        CHECK(duckdb_value_is_null(&r, 0, n));
        CHECK(duckdb_value_is_null(&r, 1, n));
        duckdb_destroy_result(&r);

        // timestamps are stored as epoch milliseconds, expected values computed independently
        const std::vector<std::int64_t> expected_ms = {
            0, 1793709296789LL, 2147483648001LL, 4133980799999LL, 1793664000000LL, 1793750399999LL };

        REQUIRE(duckdb_query(con, "SELECT ts FROM t ORDER BY id", &r) == DuckDBSuccess);
        for (size_t i = 0; i < n; ++i)
        {
            INFO("row " << i);
            CHECK(duckdb_value_int64(&r, 0, i) == expected_ms[ i ]);
        }
        CHECK(duckdb_value_is_null(&r, 0, n));
        duckdb_destroy_result(&r);
    }

    // read path: select into a Buffer through DuckDBExecResult
    // (with the non-NULL id column, as real loads have: setNull() does not increase the
    // Buffer size, so a last row with only NULL values would not be counted)
    {
        PropertyList pl;
        pl.addProperty("id", PropertyDataType::INT);

        const PropertyList value_properties = testProperties();
        for (const auto& p : value_properties.properties())
            pl.addProperty(p);

        DuckDBPrepare select(con);
        REQUIRE(select.init("SELECT id, l, ul, ts FROM t ORDER BY id"));

        auto result = select.execute(DBPrepare::ExecOptions().bufferProperties(pl));
        select.cleanup();
        REQUIRE(result);
        REQUIRE(result->buffer());

        auto& buffer = *result->buffer();
        REQUIRE(buffer.size() == n + 1);

        for (size_t i = 0; i < n; ++i)
        {
            INFO("row " << i);
            REQUIRE_FALSE(buffer.get<std::int64_t>("l").isNull(i));
            CHECK(buffer.get<std::int64_t>("l").get(i) == LongValues[ i ]);
            CHECK(buffer.get<std::uint64_t>("ul").get(i) == ULongValues[ i ]);
            CHECK(buffer.get<ptime>("ts").get(i) == timestamps[ i ]);
        }

        CHECK(buffer.get<std::int64_t>("l").isNull(n));
        CHECK(buffer.get<std::uint64_t>("ul").isNull(n));
        CHECK(buffer.get<ptime>("ts").isNull(n));
    }

    duckdb_disconnect(&con);
    duckdb_close(&db);
}
