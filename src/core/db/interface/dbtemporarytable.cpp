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

#include "dbtemporarytable.h"
#include "dbdefs.h"
#include "dbconnection.h"

#include "logger.h"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#include <QString>

/***************************************************************************************
 * DBTemporaryTable
 ***************************************************************************************/

/**
 */
DBTemporaryTable::DBTemporaryTable(DBConnection* connection)
:   connection_(connection)
{
    traced_assert(connection_);
}

/**
 */
DBTemporaryTable::~DBTemporaryTable()
{
    //@TODO: autoremove table if created?
}

/**
 */
ResultT<std::string> DBTemporaryTable::create(const std::vector<DBTableColumnInfo>& column_infos,
                                              const std::vector<db::Index>& indices)
{
    if (valid())
        remove();

    traced_assert(!valid());

    auto table_name = DBTemporaryTable::createTempTableName();

    auto res = connection_->createTableInternal(table_name, column_infos, indices, false);
    if (!res.ok())
        return res;

    table_name_ = table_name;

    return ResultT<std::string>::succeeded(table_name);
}

/**
 */                         
void DBTemporaryTable::remove()
{
    if (!valid())
        return;

    auto res = connection_->deleteTable(table_name_.value());
    if (!res.ok())
        logerr << "could not remove temporary table '" << table_name_.value() << "': " << res.error();
    traced_assert(res.ok());

    table_name_.reset();
}

/**
 */
bool DBTemporaryTable::valid() const
{
    return table_name_.has_value();
}

/**
 */
const std::string& DBTemporaryTable::name() const
{
    traced_assert(valid());
    return table_name_.value();
}

/**
 */
std::string DBTemporaryTable::createTempTableName()
{
    boost::uuids::uuid uuid       = boost::uuids::random_generator()();
    std::string        uuid_str   = boost::lexical_cast<std::string>(uuid);
    std::string        table_name = "table_temp_" + uuid_str;

    return QString::fromStdString(table_name).replace('-', '_').toStdString();
}

/***************************************************************************************
 * DBScopedTemporaryTable
 ***************************************************************************************/

/**
 */
DBScopedTemporaryTable::DBScopedTemporaryTable(DBConnection* connection, 
                                               const std::vector<DBTableColumnInfo>& column_infos,
                                               const std::vector<db::Index>& indices)
:   temp_table_(connection)
{
    result_ = temp_table_.create(column_infos, indices);
}

/**
 */
DBScopedTemporaryTable::~DBScopedTemporaryTable()
{
    temp_table_.remove();
    result_ = Result::succeeded();
}

/**
 */
bool DBScopedTemporaryTable::valid() const
{
    return temp_table_.valid();
}

/**
 */
const Result& DBScopedTemporaryTable::result() const
{
    return result_;
}

/**
 */
const std::string& DBScopedTemporaryTable::name() const
{
    return temp_table_.name();
}
