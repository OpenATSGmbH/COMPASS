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

#include "dbdefs.h"

#include <sstream>

namespace db
{

/**
 */
std::string PerformanceMetrics::asString(int flags) const
{
    std::stringstream ss;

    ss << "DB Performance metrics:\n";

    if (flags & PM_Read)
    {
        ss << " * Read " << read_num_rows << " DB rows(s) in " << read_num_chunks << " chunk(s) in " << read_time / 1000.0 << "s" << "\n";
    }
    if (flags & PM_Insert)
    {
        ss << " * Inserted " << insert_num_rows << " DB rows(s) in " << insert_num_chunks << " chunk(s) in " << insert_time / 1000.0 << "s" << "\n";
    }
    if (flags & PM_Update)
    {
        ss << " * Updated " << update_num_rows << " DB rows(s) in " << update_num_chunks << " chunk(s) in " << update_time / 1000.0 << "s" << "\n";
    }

    return ss.str();
}

}
