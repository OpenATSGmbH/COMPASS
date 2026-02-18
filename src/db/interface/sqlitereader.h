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

#include <sqlite3.h>

#include "dbreader.h"

class Buffer;
class DBResult;

/**
 */
class SQLiteReader : public DBReader
{
public:
    SQLiteReader(sqlite3* connection);
    virtual ~SQLiteReader();

    size_t numLeft() const override final;

protected:
    bool init_impl() override final;
    void finish_impl() override final;
    std::shared_ptr<DBResult> readChunk_impl() override final;

private:
    sqlite3* connection_ = nullptr;
};

/**
 * Handles db reader scope.
 */
class SQLiteScopedReader : public DBScopedReader
{
public:
    SQLiteScopedReader(sqlite3* connection,
                       const std::shared_ptr<DBCommand>& select_cmd,
                       size_t offset, 
                       size_t chunk_size)
    :   DBScopedReader(std::shared_ptr<DBReader>(new SQLiteReader(connection)), select_cmd, offset, chunk_size) {}

    virtual ~SQLiteScopedReader() = default;
};
