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

#include "duckdbsettings.h"
#include "dbinterface.h"
#include "logger.h"

#include <duckdb.h>

using namespace std;

/**
 */
std::string DuckDBSettings::accessModeAsString(AccessMode mode)
{
    return mode == AccessMode::ReadOnly ? "READ_ONLY" : "READ_WRITE"; 
}

/**
 */
std::string DuckDBSettings::sortOrderAsString(SortOrder order)
{
    return order == SortOrder::Ascending ? "ASC" : "DESC"; 
}

/**
 */
void DuckDBSettings::configure(duckdb_config* config, const DBInterface& dbinterface, bool db_in_memory) const
{
    duckdb_set_config(*config, "access_mode", DuckDBSettings::accessModeAsString(access_mode).c_str()); // or READ_ONLY
    duckdb_set_config(*config, "threads", std::to_string(dbinterface.numThreads()).c_str());
    
    if (db_in_memory)
    {
        loginf << "duckDB in memory limit: " << dbinterface.maxRAMinMemGB() << "GB"; // % also possible
        duckdb_set_config(*config, "memory_limit", (std::to_string(dbinterface.maxRAMinMemGB()) + "GB").c_str());
    }
    else
    {
        loginf << "duckDB file ram limit: " << dbinterface.maxRAMFileGB() << "GB";
        duckdb_set_config(*config, "memory_limit", (std::to_string(dbinterface.maxRAMFileGB()) + "GB").c_str());
    }

    string preserve_order = dbinterface.preserveInsertOrder() ? "true" : "false";

    duckdb_set_config(*config, "preserve_insertion_order", preserve_order.c_str());
    
    duckdb_set_config(*config, "default_order", DuckDBSettings::sortOrderAsString(sort_order_default).c_str());
}
