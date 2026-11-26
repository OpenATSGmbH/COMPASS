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

#include <duckdb.h>

#include <memory>
#include <string>
#include <vector>

class Sector;
class SectorInsideCache;

/**
 * Sector inside checks as DuckDB scalar functions, for use in WHERE clauses.
 *
 * A registered function is evaluated per row inside the DuckDB scan, in parallel over the row
 * groups. Zone maps cannot prune on it, so a bounding box condition on latitude and longitude
 * should precede it in the WHERE clause.
 *
 * Functions registered through the C API cannot be removed again, and a second registration
 * under the same name has no defined effect. Register once per database.
 */
class DuckDBSectorFunctions
{
public:
    /**
     * Registers the boolean scalar function
     *   sector_inside(layer VARCHAR, latitude DOUBLE, longitude DOUBLE, delta_deg DOUBLE)
     * (SectorInsideCache::DBFunctionName) in the database the connection belongs to. The function returns true if the position is
     * inside any non-exclusion sector of the named layer of the cache, inflated (delta_deg > 0) or
     * deflated (delta_deg < 0) by delta_deg. A null argument gives null. An unknown layer fails
     * the query with an error.
     *
     * The layer geometry comes from the cache at query time, so layers can change without a new
     * registration. The cache is kept alive by the function for the lifetime of the database.
     */
    static bool registerCacheFunction(duckdb_connection connection,
                                      const std::shared_ptr<SectorInsideCache>& cache,
                                      std::string* error = nullptr);

    /**
     * Registers the boolean scalar function name(latitude DOUBLE, longitude DOUBLE) in the
     * database the connection belongs to. The function returns true if the position is inside
     * any non-exclusion sector of the list, inflated (delta_deg > 0) or deflated (delta_deg < 0)
     * by delta_deg. A null coordinate gives null.
     *
     * The fast inside test of every sector is prepared here for the given delta, since DuckDB
     * calls the function from several threads at once and the lazy build is not thread safe.
     * The sectors are kept alive by the function for the lifetime of the database.
     * A sector may hold the prepared test for one delta only, so sectors passed with one delta
     * must not be passed to a second registration with another delta.
     */
    static bool registerInsideFunction(duckdb_connection connection,
                                       const std::string& name,
                                       const std::vector<std::shared_ptr<Sector>>& sectors,
                                       double delta_deg,
                                       std::string* error = nullptr);
};
