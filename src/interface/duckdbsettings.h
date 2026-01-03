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

#include <string>
#include "dbinterface.h"


class DBInterface;
/**
 */
struct DuckDBSettings
{
    enum class AccessMode
    {
        ReadOnly = 0,
        ReadWrite
    };

    enum class SortOrder
    {
        Ascending = 0,
        Descending
    };

    void configure(duckdb_config* config, const DBInterface& dbinterface) const;

    static std::string accessModeAsString(AccessMode mode);
    static std::string sortOrderAsString(SortOrder order);

    AccessMode   access_mode        = AccessMode::ReadWrite;
    SortOrder    sort_order_default = SortOrder::Ascending;
};
