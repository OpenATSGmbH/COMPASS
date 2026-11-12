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

// zstd column compression of large json texts (task result tables, see TaskResultDB):
// the CREATE TABLE statement of SQLGenerator, and the storage in a DuckDB file. DuckDB uses the
// column compression only in files of a newer storage version (DuckDBSettings::StorageCompatibilityVersion),
// a file of the default storage version stores the column uncompressed and stays usable.

#include "catch.hpp"
#include "sqlgenerator.h"
#include "duckdbsettings.h"
#include "propertylist.h"

#include <duckdb.h>

#include <boost/filesystem.hpp>

#include <string>

namespace
{
PropertyList testProperties()
{
    PropertyList pl;
    pl.addProperty("content_id", PropertyDataType::UINT);
    pl.addProperty("json_content", PropertyDataType::STRING);
    return pl;
}

db::SQLConfig duckdbSQLConfig(bool zstd)
{
    db::SQLConfig config;
    config.precise_types        = true;
    config.supports_column_zstd = zstd;
    config.verbose              = false;
    return config;
}

/**
 * Creates the test table in a new database file, fills it with repetitive json texts and returns
 * the compression method of the json column, the storage version is set if not empty.
 */
std::string jsonColumnCompression(const std::string& storage_version)
{
    auto path = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("compass_zstd_%%%%%%%%.db");
    std::string file = path.string();

    duckdb_config config;
    REQUIRE(duckdb_create_config(&config) == DuckDBSuccess);
    if (!storage_version.empty())
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

    SQLGenerator gen(duckdbSQLConfig(true));
    query(gen.getCreateTableStatement("contents", testProperties(), 0, { "json_content" }));
    query("INSERT INTO contents SELECT i, '{\"caption\":\"Target Overview\",\"rows\":[[\"UTN\",' || i::VARCHAR || "
          "'],[\"Use\",false]],\"annotations\":' || repeat('{\"on_demand\":false,\"style\":0},', 20) || '[]}' "
          "FROM range(200000) r(i)");
    query("CHECKPOINT");

    duckdb_result res;
    REQUIRE(duckdb_query(con, "SELECT DISTINCT compression FROM pragma_storage_info('contents') "
                              "WHERE column_name = 'json_content' AND segment_type = 'VARCHAR'", &res) == DuckDBSuccess);
    std::string compression;
    for (idx_t r = 0; r < duckdb_row_count(&res); ++r)
    {
        char* v = duckdb_value_varchar(&res, 0, r);
        compression += (compression.empty() ? "" : ",") + std::string(v);
        duckdb_free(v);
    }
    duckdb_destroy_result(&res);

    //the texts read back unchanged
    REQUIRE(duckdb_query(con, "SELECT count(*) FROM contents WHERE json_content LIKE '%\"UTN\",199999]%'", &res) == DuckDBSuccess);
    REQUIRE(duckdb_value_int64(&res, 0, 0) == 1);
    duckdb_destroy_result(&res);

    duckdb_disconnect(&con);
    duckdb_close(&db);

    boost::filesystem::remove(path);
    boost::filesystem::remove(file + ".wal");

    return compression;
}
}

TEST_CASE("SQLGenerator zstd columns in CREATE TABLE", "[duckdb][zstd]")
{
    SECTION("backend supports zstd: only the named column is compressed")
    {
        SQLGenerator gen(duckdbSQLConfig(true));
        auto sql = gen.getCreateTableStatement("contents", testProperties(), 0, { "json_content" });

        REQUIRE(sql.find("json_content VARCHAR USING COMPRESSION zstd") != std::string::npos);
        REQUIRE(sql.find("content_id UINTEGER USING") == std::string::npos);
        REQUIRE(sql.find("PRIMARY KEY (content_id)") != std::string::npos);
    }

    SECTION("backend without zstd support or no named column: no compression clause")
    {
        SQLGenerator gen_no(duckdbSQLConfig(false));
        REQUIRE(gen_no.getCreateTableStatement("contents", testProperties(), 0, { "json_content" }).find("USING") == std::string::npos);

        SQLGenerator gen(duckdbSQLConfig(true));
        REQUIRE(gen.getCreateTableStatement("contents", testProperties(), 0).find("USING") == std::string::npos);
    }
}

TEST_CASE("DuckDB zstd json column per storage version", "[duckdb][zstd]")
{
    SECTION("storage version of COMPASS: the column is stored with zstd")
    {
        REQUIRE(jsonColumnCompression(DuckDBSettings::StorageCompatibilityVersion) == "ZSTD");
    }

    SECTION("default storage version: the clause is accepted, the column is not stored with zstd")
    {
        auto compression = jsonColumnCompression("");
        REQUIRE(!compression.empty());
        REQUIRE(compression.find("ZSTD") == std::string::npos);
    }
}
