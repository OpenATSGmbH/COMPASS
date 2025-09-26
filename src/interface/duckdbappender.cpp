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

#include "duckdbappender.h"
#include "traced_assert.h"

#include "logger.h"

/**
 */
DuckDBScopedAppender::DuckDBScopedAppender(duckdb_connection connection, 
                                           const std::string& table_name) 
{
    auto result = duckdb_appender_create(connection, NULL, table_name.c_str(), &appender_);
    ok_ = result == DuckDBSuccess;
    //loginf << "appender ok? " << ok_;

    if (!ok_)
        setError("could not create appender for table '" + table_name + "'");
}

/**
 */
DuckDBScopedAppender::~DuckDBScopedAppender()
{
    //will also flush appender
    //if (ok_)
    //    duckdb_appender_close(appender_);
    duckdb_appender_destroy(&appender_);
}

/**
 */
bool DuckDBScopedAppender::valid() const
{
    return ok_;
}

/**
 */
size_t DuckDBScopedAppender::endRow()
{
    traced_assert(ok_);

    auto state = duckdb_appender_end_row(appender_);

    if (state != DuckDBSuccess)
        logerr << "failed: " << duckdb_appender_error(appender_);
    traced_assert(state == DuckDBSuccess);

    size_t last_appended = appended_;
    appended_ = 0;

    return last_appended;
}

/**
 */
bool DuckDBScopedAppender::flush()
{
    traced_assert(ok_);
    return duckdb_appender_flush(appender_) == DuckDBSuccess;
}

/**
 */
size_t DuckDBScopedAppender::columnCount() const
{
    traced_assert(ok_);
    return duckdb_appender_column_count(appender_);
}

/**
 */
std::string DuckDBScopedAppender::lastAppenderError() const
{
    traced_assert(ok_);
    return std::string(duckdb_appender_error(appender_));
}
