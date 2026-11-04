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

/**
 * Sector filtering in the WHERE clause through a DuckDB scalar function, measured on a real
 * database. Hidden test, it needs the database file and runs only when selected:
 *
 *   ./build/bin/compass_tests "[sector_db]"
 *
 * The database path can be overridden with the environment variable COMPASS_SECTOR_TEST_DB.
 * The stored sectors of the database's Data Context are used. Compared per data table:
 *
 *   bbox          latitude/longitude between the sector bounds widened by delta
 *   bbox+in(+d)   bbox and inside the sectors inflated by delta
 *   in(+d)        inside the inflated sectors, no bbox
 *   in(0)         inside the plain sectors
 *   in(-d)        inside the sectors deflated by delta
 *   out(-d)       not inside the deflated sectors (the outside test)
 *   band          inside inflated and not inside deflated (combination of both)
 *   bbox+band     the same with the bbox in front
 *
 * Each variant is run as a count and as a load-like select (record number, timestamp, position,
 * ordered by timestamp), best of several repetitions. The function results are cross-checked
 * row by row against direct Sector::isInside() calls.
 */

#include "catch.hpp"
#include "duckdbsectorfunctions.h"
#include "sector.h"
#include "sectorinsidecache.h"
#include "json.hpp"

#include <duckdb.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    const char*  DefaultDBPath   = "/home/sk/data/test/at_20230422/temp/at_20230422_2h.db";
    const double DeltaDeg        = 0.1;    //about 11 km in latitude
    const int    NumReps         = 3;      //best of
    const idx_t  CrossCheckRows  = 200000; //rows compared against direct Sector::isInside() calls

    const std::vector<std::string> Tables = { "data_cat062", "data_cat048", "data_cat021",
                                              "data_cat020", "data_cat001" };

    std::string dbPath()
    {
        const char* env = std::getenv("COMPASS_SECTOR_TEST_DB");
        return (env && *env) ? std::string(env) : std::string(DefaultDBPath);
    }

    bool fileExists(const std::string& path)
    {
        std::ifstream f(path);
        return f.good();
    }

    /**
     * Read-only DuckDB handle.
     */
    struct Database
    {
        duckdb_database   db  = nullptr;
        duckdb_connection con = nullptr;

        ~Database()
        {
            if (con)
                duckdb_disconnect(&con);
            if (db)
                duckdb_close(&db);
        }

        bool open(const std::string& path, std::string& error)
        {
            duckdb_config config;
            if (duckdb_create_config(&config) == DuckDBError)
            {
                error = "could not create config";
                return false;
            }

            duckdb_set_config(config, "access_mode", "READ_ONLY");

            char* err = nullptr;
            auto state = duckdb_open_ext(path.c_str(), &db, config, &err);
            duckdb_destroy_config(&config);

            if (state == DuckDBError)
            {
                error = err ? std::string(err) : std::string("unknown error");
                if (err)
                    duckdb_free(err);
                return false;
            }

            if (duckdb_connect(db, &con) == DuckDBError)
            {
                error = "could not connect";
                return false;
            }

            return true;
        }
    };

    /**
     * Result handle with automatic destruction.
     */
    struct Result
    {
        duckdb_result result;
        bool          valid = false;

        ~Result()
        {
            if (valid)
                duckdb_destroy_result(&result);
        }

        bool run(duckdb_connection con, const std::string& sql)
        {
            if (duckdb_query(con, sql.c_str(), &result) == DuckDBError)
            {
                valid = true;
                FAIL("query failed: " << sql << "\n" << duckdb_result_error(&result));
                return false;
            }

            valid = true;
            return true;
        }
    };

    int64_t scalarInt(duckdb_connection con, const std::string& sql)
    {
        Result r;
        r.run(con, sql);
        return duckdb_value_int64(&r.result, 0, 0);
    }

    std::string scalarString(duckdb_connection con, const std::string& sql)
    {
        Result r;
        r.run(con, sql);

        char* value = duckdb_value_varchar(&r.result, 0, 0);
        std::string str = value ? value : "";
        if (value)
            duckdb_free(value);

        return str;
    }

    /**
     * Runs the query reps times and returns the best wall time in seconds. The result row
     * count of the last run is stored in rows.
     */
    double timeQuery(duckdb_connection con, const std::string& sql, int reps, int64_t& rows)
    {
        double best = -1.0;
        rows = 0;

        for (int rep = 0; rep < reps; ++rep)
        {
            auto t0 = std::chrono::steady_clock::now();

            Result r;
            r.run(con, sql);

            auto t1 = std::chrono::steady_clock::now();

            double secs = std::chrono::duration<double>(t1 - t0).count();
            if (best < 0.0 || secs < best)
                best = secs;

            rows = (int64_t) duckdb_row_count(&r.result);
        }

        return best;
    }

    bool tableHasPosition(duckdb_connection con, const std::string& table)
    {
        int64_t num_tables = scalarInt(con,
            "SELECT count(*) FROM information_schema.tables WHERE table_name = '" + table + "'");
        if (num_tables != 1)
            return false;

        int64_t num_cols = scalarInt(con,
            "SELECT count(*) FROM information_schema.columns WHERE table_name = '" + table +
            "' AND column_name IN ('latitude', 'longitude', 'record_number', 'timestamp')");

        return num_cols == 4;
    }

    /**
     * Loads the sectors stored in the Data Context of the database. Every call creates new
     * Sector objects, since one Sector holds the prepared inside test for one delta only.
     */
    std::vector<std::shared_ptr<Sector>> loadSectors(duckdb_connection con)
    {
        std::vector<std::shared_ptr<Sector>> sectors;

        std::string json_str = scalarString(con, "SELECT json FROM db_context WHERE section = 'sectors'");
        REQUIRE(!json_str.empty());

        nlohmann::json j = nlohmann::json::parse(json_str);
        REQUIRE(j.contains("data"));

        for (const auto& s : j.at("data"))
        {
            auto sector = std::make_shared<Sector>(s.at("id").get<unsigned int>(),
                                                   s.at("name").get<std::string>(),
                                                   s.at("layer_name").get<std::string>(),
                                                   false);
            REQUIRE(sector->readJSON(s));

            sectors.push_back(sector);
        }

        return sectors;
    }

    bool insideAny(const std::vector<std::shared_ptr<Sector>>& sectors, double lat, double lon, double delta)
    {
        for (const auto& s : sectors)
        {
            if (s->isExclusionSector())
                continue;
            if (s->isInside(lat, lon, delta))
                return true;
        }
        return false;
    }

    std::string fmtSecs(double secs)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3) << secs;
        return ss.str();
    }

    struct Variant
    {
        std::string name;
        std::string where;
        int64_t     count      = 0;
        double      count_secs = 0.0;
        int64_t     load_rows  = 0;
        double      load_secs  = 0.0;
    };
}

TEST_CASE("DuckDB sector inside function filters a database load", "[.][sector_db]")
{
    const std::string path = dbPath();

    if (!fileExists(path))
    {
        WARN("test database not found, skipping: " << path);
        return;
    }

    Database database;
    std::string error;
    REQUIRE(database.open(path, error));

    duckdb_connection con = database.con;

    //three independent sector sets, one per delta
    auto sectors_zero  = loadSectors(con);
    auto sectors_plus  = loadSectors(con);
    auto sectors_minus = loadSectors(con);

    REQUIRE(!sectors_zero.empty());

    WARN("database " << path << ", sectors " << sectors_zero.size() << ", delta_deg " << DeltaDeg);

    //bounding box of the non-exclusion sectors, widened by delta
    double lat_min = 0, lat_max = 0, lon_min = 0, lon_max = 0;
    bool first = true;

    for (const auto& s : sectors_zero)
    {
        if (s->isExclusionSector())
            continue;

        auto lat = s->getMinMaxLatitude();
        auto lon = s->getMinMaxLongitude();

        if (first)
        {
            lat_min = lat.first; lat_max = lat.second;
            lon_min = lon.first; lon_max = lon.second;
            first = false;
        }
        else
        {
            lat_min = std::min(lat_min, lat.first); lat_max = std::max(lat_max, lat.second);
            lon_min = std::min(lon_min, lon.first); lon_max = std::max(lon_max, lon.second);
        }
    }

    REQUIRE(!first);

    lat_min -= DeltaDeg; lat_max += DeltaDeg;
    lon_min -= DeltaDeg; lon_max += DeltaDeg;

    //register the three functions
    REQUIRE(DuckDBSectorFunctions::registerInsideFunction(con, "sector_inside",       sectors_zero,  0.0,       &error));
    REQUIRE(DuckDBSectorFunctions::registerInsideFunction(con, "sector_inside_plus",  sectors_plus,  DeltaDeg,  &error));
    REQUIRE(DuckDBSectorFunctions::registerInsideFunction(con, "sector_inside_minus", sectors_minus, -DeltaDeg, &error));

    //the C API documentation says a second registration under the same name fails, DuckDB 1.5.5
    //accepts it. Recorded only, the queries below show whether the function still works after it.
    bool duplicate_ok = DuckDBSectorFunctions::registerInsideFunction(con, "sector_inside_dup", sectors_zero, 0.0, &error);
    duplicate_ok = duplicate_ok && DuckDBSectorFunctions::registerInsideFunction(con, "sector_inside_dup", sectors_zero, 0.0, &error);
    WARN("duplicate registration of 'sector_inside_dup' accepted " << (duplicate_ok ? "yes" : "no"));

    //the cache function of the Sectors filter, fed with the stored layers
    auto cache = std::make_shared<SectorInsideCache>();
    std::vector<std::string> layer_names;
    {
        auto sectors_cache = loadSectors(con);

        std::map<std::string, std::vector<std::shared_ptr<Sector>>> by_layer;
        for (const auto& s : sectors_cache)
            by_layer[ s->layerName() ].push_back(s);

        for (const auto& it : by_layer)
        {
            cache->setLayer(it.first, it.second);
            layer_names.push_back(it.first);
        }
    }

    REQUIRE(DuckDBSectorFunctions::registerCacheFunction(con, cache, &error));

    //OR over all stored layers, as the Sectors filter renders it
    auto cacheExpr = [&](double delta)
    {
        std::ostringstream ss;
        ss << std::setprecision(10);

        bool first = true;
        for (const auto& name : layer_names)
        {
            if (!first)
                ss << " OR ";
            ss << SectorInsideCache::DBFunctionName << "('" << name << "', latitude, longitude, NULL, " << delta << ")";
            first = false;
        }

        return layer_names.size() > 1 ? "(" + ss.str() + ")" : ss.str();
    };

    //an unknown layer fails the query
    {
        duckdb_result r;
        auto state = duckdb_query(con, ("SELECT count(*) FROM data_cat062 WHERE " +
                                        std::string(SectorInsideCache::DBFunctionName) +
                                        "('no_such_layer', latitude, longitude, NULL, 0.0)").c_str(), &r);
        CHECK(state == DuckDBError);
        if (state == DuckDBError)
            WARN("unknown layer error: " << duckdb_result_error(&r));
        duckdb_destroy_result(&r);
    }

    std::ostringstream bbox_ss;
    bbox_ss << std::setprecision(12)
            << "latitude BETWEEN " << lat_min << " AND " << lat_max
            << " AND longitude BETWEEN " << lon_min << " AND " << lon_max;
    const std::string bbox = bbox_ss.str();

    const std::string in_plus  = "sector_inside_plus(latitude, longitude)";
    const std::string in_zero  = "sector_inside(latitude, longitude)";
    const std::string in_minus = "sector_inside_minus(latitude, longitude)";

    for (const auto& table : Tables)
    {
        if (!tableHasPosition(con, table))
        {
            WARN(table << ": not in database or without position, skipped");
            continue;
        }

        const int64_t total      = scalarInt(con, "SELECT count(*) FROM " + table);
        const int64_t null_coord = scalarInt(con, "SELECT count(*) FROM " + table +
                                                  " WHERE latitude IS NULL OR longitude IS NULL");

        std::vector<Variant> variants = {
            { "bbox",        bbox },
            { "bbox+in(+d)", bbox + " AND " + in_plus },
            { "in(+d)",      in_plus },
            { "in(0)",       in_zero },
            { "in(-d)",      in_minus },
            { "out(-d)",     "NOT " + in_minus },
            { "band",        in_plus + " AND NOT " + in_minus },
            { "bbox+band",   bbox + " AND " + in_plus + " AND NOT " + in_minus },
            { "cache(+d)",   cacheExpr(DeltaDeg) },
            { "cache(0)",    cacheExpr(0.0) },
            { "bbox+cache",  bbox + " AND " + cacheExpr(DeltaDeg) }
        };

        int64_t rows = 0;

        //full scan time without any condition, as the reference
        double total_count_secs = timeQuery(con, "SELECT count(*) FROM " + table, NumReps, rows);
        double total_load_secs  = timeQuery(con, "SELECT record_number, timestamp, latitude, longitude FROM " +
                                                 table + " ORDER BY timestamp", NumReps, rows);

        for (auto& v : variants)
        {
            v.count_secs = timeQuery(con, "SELECT count(*) FROM " + table + " WHERE " + v.where, NumReps, rows);
            v.count      = scalarInt(con, "SELECT count(*) FROM " + table + " WHERE " + v.where);
            v.load_secs  = timeQuery(con, "SELECT record_number, timestamp, latitude, longitude FROM " + table +
                                          " WHERE " + v.where + " ORDER BY timestamp", NumReps, v.load_rows);
        }

        //report
        {
            std::ostringstream ss;
            ss << "\n" << table << ": rows " << total << ", null positions " << null_coord
               << ", full scan count " << fmtSecs(total_count_secs) << " s, full load "
               << fmtSecs(total_load_secs) << " s\n"
               << std::left << std::setw(14) << "variant" << std::right
               << std::setw(10) << "rows" << std::setw(10) << "count_s" << std::setw(10) << "load_s" << "\n";

            for (const auto& v : variants)
                ss << std::left << std::setw(14) << v.name << std::right
                   << std::setw(10) << v.count << std::setw(10) << fmtSecs(v.count_secs)
                   << std::setw(10) << fmtSecs(v.load_secs) << "\n";

            WARN(ss.str());
        }

        auto count = [&](const std::string& name) -> int64_t
        {
            for (const auto& v : variants)
                if (v.name == name)
                    return v.count;
            FAIL("unknown variant " << name);
            return 0;
        };

        //consistency of the counts
        INFO(table);
        for (const auto& v : variants)
            CHECK(v.load_rows == v.count);

        CHECK(count("in(-d)") <= count("in(0)"));
        CHECK(count("in(0)")  <= count("in(+d)"));
        CHECK(count("in(+d)") <= count("bbox"));
        CHECK(count("bbox+in(+d)") == count("in(+d)"));       //the inflated sector lies in the widened box
        CHECK(count("band") == count("in(+d)") - count("in(-d)"));
        CHECK(count("bbox+band") == count("band"));
        CHECK(count("out(-d)") + count("in(-d)") == total - null_coord);

        //the cache function gives the same rows as the direct functions
        CHECK(count("cache(+d)") == count("in(+d)"));
        CHECK(count("cache(0)") == count("in(0)"));
        CHECK(count("bbox+cache") == count("in(+d)"));

        //null coordinates give null
        CHECK(scalarInt(con, "SELECT count(*) FROM " + table + " WHERE " + in_zero + " IS NULL") == null_coord);

        //row by row against direct calls, once on a plain scan and once behind a filter, since a
        //filter changes the vector layout the function receives
        for (const std::string& pre : { std::string("latitude IS NOT NULL AND longitude IS NOT NULL"), bbox })
        {
            Result r;
            r.run(con, "SELECT latitude, longitude, " + in_zero + ", " + in_plus + ", " + in_minus +
                       " FROM " + table + " WHERE " + pre + " LIMIT " + std::to_string(CrossCheckRows));

            const idx_t n = duckdb_row_count(&r.result);
            int64_t mismatches = 0;

            for (idx_t row = 0; row < n; ++row)
            {
                double lat = duckdb_value_double(&r.result, 0, row);
                double lon = duckdb_value_double(&r.result, 1, row);

                if (duckdb_value_boolean(&r.result, 2, row) != insideAny(sectors_zero,  lat, lon, 0.0))       ++mismatches;
                if (duckdb_value_boolean(&r.result, 3, row) != insideAny(sectors_plus,  lat, lon, DeltaDeg))  ++mismatches;
                if (duckdb_value_boolean(&r.result, 4, row) != insideAny(sectors_minus, lat, lon, -DeltaDeg)) ++mismatches;
            }

            INFO("cross check '" << pre << "' rows " << n);
            CHECK(mismatches == 0);
        }
    }
}

TEST_CASE("DuckDB sector inside function with altitude and exclusion", "[sectors][sector_function]")
{
    duckdb_database db;
    duckdb_connection con;

    REQUIRE(duckdb_open(nullptr, &db) == DuckDBSuccess);
    REQUIRE(duckdb_connect(db, &con) == DuckDBSuccess);

    //normal sector 47-48 / 15-16 up to 5000 ft, exclusion sector 47.4-47.6 / 15.4-15.6
    SectorInsideCache::SectorGeometry normal;
    normal.id           = 1;
    normal.name         = "S1";
    normal.points       = { {47.0, 15.0}, {47.0, 16.0}, {48.0, 16.0}, {48.0, 15.0} };
    normal.max_altitude = 5000.0;

    SectorInsideCache::SectorGeometry exclusion;
    exclusion.id        = 2;
    exclusion.name      = "X1";
    exclusion.exclusion = true;
    exclusion.points    = { {47.4, 15.4}, {47.4, 15.6}, {47.6, 15.6}, {47.6, 15.4} };

    auto cache = std::make_shared<SectorInsideCache>();
    cache->setLayer("L1", { normal, exclusion });

    std::string error;
    REQUIRE(DuckDBSectorFunctions::registerCacheFunction(con, cache, &error));

    auto query = [&](const std::string& sql)
    {
        duckdb_result r;
        REQUIRE(duckdb_query(con, sql.c_str(), &r) == DuckDBSuccess);
        int64_t value = duckdb_value_int64(&r, 0, 0);
        duckdb_destroy_result(&r);
        return value;
    };

    query("CREATE TABLE t (id INTEGER, latitude DOUBLE, longitude DOUBLE, alt FLOAT)");
    query("INSERT INTO t VALUES "
          "(1, 47.2, 15.2, 3000),"   //inside
          "(2, 47.2, 15.2, NULL),"   //inside, no altitude
          "(3, 47.2, 15.2, 6000),"   //above the normal sector
          "(4, 47.5, 15.5, 3000),"   //in the exclusion sector
          "(5, 47.5, 15.5, NULL),"   //in the exclusion sector, no altitude
          "(6, 49.0, 15.5, NULL),"   //outside
          "(7, NULL, 15.5, 3000)");  //no position

    CHECK(query("SELECT count(*) FROM t WHERE sector_inside('L1', latitude, longitude, alt, 0.0)") == 2);
    CHECK(query("SELECT sum(id) FROM t WHERE sector_inside('L1', latitude, longitude, alt, 0.0)") == 3);

    //a NULL altitude literal for a content without altitude
    CHECK(query("SELECT count(*) FROM t WHERE sector_inside('L1', latitude, longitude, NULL, 0.0)") == 3);

    //a delta larger than the exclusion sector removes it
    CHECK(query("SELECT count(*) FROM t WHERE sector_inside('L1', latitude, longitude, alt, 0.2)") == 4);

    duckdb_disconnect(&con);
    duckdb_close(&db);
}
