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

#include "dbinstance.h"
#include "dbdefs.h"

class DBInterface;
class DBConnection;

/**
 */
class SQLiteInstance : public DBInstance
{
public:
    SQLiteInstance(DBInterface* interface); 
    virtual ~SQLiteInstance();
    
protected:
    Result open_impl(const std::string& file_name) override final;
    void close_impl() override final;
    ResultT<DBConnection*> createConnection_impl(bool verbose) override final;
    Result exportToFile_impl(const std::string& file_name) override final;

    /**
     */
    db::SQLConfig sqlConfiguration_impl() const override final
    {
        db::SQLConfig config;

        //no precise types needed, e.g. suitable integer types will be chosen in the background
        config.precise_types = false;

        //needed for query speed
        config.indexing = true;

        //multithreaded access not supported
        config.supports_mt = false;

        //in-memory db not supported
        config.supports_in_mem = false;

        config.placeholder = db::SQLPlaceholder::AtVar;
        config.use_conflict_resolution = false;

        return config;
    }

private:
    /// Database handle to execute queries
    sqlite3* db_handle_{nullptr};
};
