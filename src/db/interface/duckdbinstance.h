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

#include "dbinstance.h"
#include "duckdbsettings.h"

class DBInterface;
class DBConnection;

/**
 * Returns the memory DuckDB's allocator holds back to the operating system.
 *
 * DuckDB statically links its own jemalloc under prefixed symbols (duckdb_je_*), which
 * neither malloc_trim nor a preloaded allocator reaches, so its freed pages stay in the
 * process without this. Meant to be called after work that read a lot of data, not per
 * query. Does nothing if the allocator control symbol is not present.
 */
void duckDBClean();

/**
 */
class DuckDBInstance : public DBInstance
{
public:
    DuckDBInstance(DBInterface* interface); 
    virtual ~DuckDBInstance();

    DuckDBSettings& settings() { return settings_; }
    
protected:
    Result open_impl(const std::string& file_name) override final;
    void close_impl() override final;
    ResultT<DBConnection*> createConnection_impl(bool verbose) override final;
    Result cleanupDB_impl(const std::string& db_fn) override final;
    Result cleanupDB_impl() override final;
    Result cleanupDBInMem_impl() override final;
    Result exportToFile_impl(const std::string& file_name) override final;

    std::vector<db::SQLPragma> sqlPragmas() const override final;

    /**
     */
    db::SQLConfig sqlConfiguration_impl() const override final
    {
        db::SQLConfig config;

        //duckdb needs precise data types in its tables
        config.precise_types = true;

        //no indexing, queries should be quite fast in duckdb 
        //and indexing blows up the db file consiferably
        config.indexing = false;

        //duckdb can only comprehend ? as a placeholder
        config.placeholder = db::SQLPlaceholder::QuestionMark;

        //duckdb does not allow 'replace into' directly, this is handled via 'insert ... on conflict'
        config.use_conflict_resolution = true;

        //duckdb supports multithreaded access
        config.supports_mt = true;

        //duckdb supports in-memory db
        config.supports_in_mem = true;

        return config;
    }

private:
    friend class DuckDBConnection;

    duckdb_database db_ = nullptr;

    DuckDBSettings settings_;
};
