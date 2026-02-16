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

#include "sqlitereader.h"
#include "dbresult.h"
#include "buffer.h"
#include "property_templates.h"

#include "logger.h"

/**
 */
SQLiteReader::SQLiteReader(sqlite3* connection)
:   connection_(connection)
{
    traced_assert(connection_);
}

/**
 */
SQLiteReader::~SQLiteReader() = default;

/**
 */
bool SQLiteReader::init_impl()
{
    //@TODO
    return false;
}

/**
 */
void SQLiteReader::finish_impl()
{
    //@TODO
}

/**
 */
std::shared_ptr<DBResult> SQLiteReader::readChunk_impl()
{
    //@TODO
    return std::shared_ptr<DBResult>();
}

/**
 */
size_t SQLiteReader::numLeft() const
{
    //@TODO
    return 0;
}
