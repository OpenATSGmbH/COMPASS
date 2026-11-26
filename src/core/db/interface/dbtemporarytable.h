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

#include "result.h"

#include <string>
#include <vector>

#include <boost/optional.hpp>

class DBConnection;
class DBTableColumnInfo;

namespace db
{
    struct Index;
}

class Buffer;

/**
 */
class DBTemporaryTable
{
public:
    DBTemporaryTable(DBConnection* connection);
    virtual ~DBTemporaryTable();

    ResultT<std::string> create(const std::vector<DBTableColumnInfo>& column_infos,
                                const std::vector<db::Index>& indices = std::vector<db::Index>());
    void remove();

    bool valid() const;
    const std::string& name() const;

    static std::string createTempTableName();

private:
    DBConnection*                connection_ = nullptr;
    boost::optional<std::string> table_name_;
};

/**
 */
class DBScopedTemporaryTable
{
public:
    DBScopedTemporaryTable(DBConnection* connection, 
                           const std::vector<DBTableColumnInfo>& column_infos,
                           const std::vector<db::Index>& indices = std::vector<db::Index>());
    virtual ~DBScopedTemporaryTable();

    bool valid() const;
    const Result& result() const;
    const std::string& name() const;

protected:
    DBTemporaryTable temp_table_;
    Result           result_;
};
