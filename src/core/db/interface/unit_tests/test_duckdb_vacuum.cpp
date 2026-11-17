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

// Reuse of deleted rows in a DuckDB file of the storage version of COMPASS
// (DuckDBSettings::StorageCompatibilityVersion). A table with a primary key, as the data tables
// from SQLGenerator, is filled and then changed in cycles as in live mode: insert new rows, delete
// the oldest rows, checkpoint. The number of blocks of the file must stay level.
// Storage version v1.5.0 does not reuse deleted rows in a table with a primary key, there the file
// grows in each cycle (hidden test case, run it with: compass_tests "[vacuum]").

#include "catch.hpp"
#include "sqlgenerator.h"
#include "dbtableinfo.h"
#include "dbdefs.h"
#include "duckdbsettings.h"

#include <duckdb.h>

#include <boost/filesystem.hpp>

#include <string>
#include <vector>

namespace
{
const idx_t BatchRows   = 122880; // one DuckDB row group
const idx_t InitBatches = 2;
const idx_t Cycles      = 8;

/**
 * Fills the test table and changes it in cycles, returns the number of blocks of the database file
 * after the initial fill (index 0) and after each cycle (index 1 to Cycles).
 */
std::vector<idx_t> blocksPerCycle(const std::string& storage_version)
{
    auto path = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("compass_vacuum_%%%%%%%%.db");
    std::string file = path.string();

    duckdb_config config;
    REQUIRE(duckdb_create_config(&config) == DuckDBSuccess);
    REQUIRE(duckdb_set_config(config, "storage_compatibility_version", storage_version.c_str()) == DuckDBSuccess);

    duckdb_database db;
    duckdb_connection con;
    char* error = nullptr;
    REQUIRE(duckdb_open_ext(file.c_str(), &db, config, &error) == DuckDBSuccess);
    duckdb_destroy_config(&config);
    REQUIRE(duckdb_connect(db, &con) == DuckDBSuccess);

    auto query = [ & ] (const std::string& sql)
    {
        duckdb_result res;
        bool ok = duckdb_query(con, sql.c_str(), &res) == DuckDBSuccess;
        if (!ok)
            FAIL(std::string(duckdb_result_error(&res)) + " in '" + sql + "'");
        duckdb_destroy_result(&res);
    };

    auto blocks = [ & ] ()
    {
        query("CHECKPOINT");

        duckdb_result res;
        REQUIRE(duckdb_query(con, "SELECT total_blocks FROM pragma_database_size() "
                                  "WHERE database_name = current_database()", &res) == DuckDBSuccess);
        REQUIRE(duckdb_row_count(&res) == 1);
        idx_t n = duckdb_value_uint64(&res, 0, 0);
        duckdb_destroy_result(&res);
        return n;
    };

    idx_t next_rec_num = 0;

    auto insertBatch = [ & ] ()
    {
        //timestamps are stored as milliseconds since epoch
        query("INSERT INTO data_test SELECT i, 1767225600000 + i, "
              "(hash(i) % 1000000) / 1000000.0, 'CS' || (i % 5000)::VARCHAR "
              "FROM range(" + std::to_string(next_rec_num) + ", " + std::to_string(next_rec_num + BatchRows) + ") r(i)");
        next_rec_num += BatchRows;
    };

    //table as created by DBConnection for DuckDB: precise types, primary key, no other indexes
    db::SQLConfig sql_config;
    sql_config.precise_types = true;
    sql_config.indexing      = false;
    sql_config.verbose       = false;

    std::vector<DBTableColumnInfo> columns = {
        DBTableColumnInfo("rec_num", PropertyDataType::ULONGINT, true, false),
        DBTableColumnInfo("timestamp", PropertyDataType::TIMESTAMP, false),
        DBTableColumnInfo("latitude", PropertyDataType::DOUBLE, false),
        DBTableColumnInfo("callsign", PropertyDataType::STRING, false)
    };
    query(SQLGenerator(sql_config).getCreateTableStatement("data_test", columns, {}));

    std::vector<idx_t> result;

    for (idx_t b = 0; b < InitBatches; ++b)
        insertBatch();
    result.push_back(blocks());

    idx_t oldest_rec_num = 0;
    for (idx_t c = 0; c < Cycles; ++c)
    {
        insertBatch();
        oldest_rec_num += BatchRows;
        query("DELETE FROM data_test WHERE rec_num < " + std::to_string(oldest_rec_num));
        result.push_back(blocks());
    }

    duckdb_disconnect(&con);
    duckdb_close(&db);

    boost::filesystem::remove(path);
    boost::filesystem::remove(file + ".wal");

    return result;
}

/**
 * Upper limit of the number of blocks: the first cycle allocates the blocks for one more batch,
 * later cycles reuse the blocks of the deleted rows.
 */
idx_t blockLimit(const std::vector<idx_t>& blocks)
{
    return blocks.at(1) + blocks.at(1) / 2;
}
}

TEST_CASE("DuckDB reuse of deleted rows in a table with primary key", "[duckdb][vacuum]")
{
    auto blocks = blocksPerCycle(DuckDBSettings::StorageCompatibilityVersion);
    REQUIRE(blocks.size() == Cycles + 1);

    idx_t limit = blockLimit(blocks);
    for (idx_t c = 2; c <= Cycles; ++c)
    {
        INFO("cycle " << c << " blocks " << blocks[c] << " limit " << limit);
        REQUIRE(blocks[c] <= limit);
    }
}

TEST_CASE("DuckDB storage v1.5.0 does not reuse deleted rows in a table with primary key", "[.][duckdb][vacuum]")
{
    auto blocks = blocksPerCycle("v1.5.0");
    REQUIRE(blocks.size() == Cycles + 1);

    INFO("last cycle blocks " << blocks.back() << " limit " << blockLimit(blocks));
    REQUIRE(blocks.back() > blockLimit(blocks));
}
