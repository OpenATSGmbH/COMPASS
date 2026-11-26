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

#include "duckdbinstance.h"
#include "dbinterface.h"
#include "duckdbconnection.h"

#include "logger.h"

#ifndef _WIN32
#include <dlfcn.h>
#endif

namespace
{
    class DuckDBScopedConfig
    {
    public:
        DuckDBScopedConfig()
        {
            auto state = duckdb_create_config(&config_);
            ok_ = state == DuckDBSuccess;
        }

        virtual ~DuckDBScopedConfig()
        {
            if (ok_)
                duckdb_destroy_config(&config_);
        }

        void configure(const DuckDBSettings& settings, const DBInterface& dbinterface, bool db_in_memory)
        {
            settings.configure(&config_, dbinterface, db_in_memory);
        }

        bool valid() const { return ok_; }
        duckdb_config* configuration() { return &config_; }

    private:
        duckdb_config config_;
        bool ok_ = false;
    };
}

/**
 */
void duckDBClean()
{
#ifdef _WIN32
    //the Windows build of DuckDB does not use jemalloc
#else
    //jemalloc's mallctl, looked up at runtime: the symbol is prefixed and internal to
    //libduckdb, so it may vanish with a DuckDB update - then this is a no-op
    using MallctlFn = int (*)(const char*, void*, size_t*, void*, size_t);
    static MallctlFn mallctl = (MallctlFn) dlsym(RTLD_DEFAULT, "duckdb_je_mallctl");

    if (!mallctl)
        return;

    //arena 4096 is jemalloc's MALLCTL_ARENAS_ALL: purge the dirty pages of every arena
    int ret = mallctl("arena.4096.purge", nullptr, nullptr, nullptr, 0);

    if (ret != 0)
        logwrn << "purging duckdb allocator arenas failed with " << ret;
#endif
}

/**
 */
DuckDBInstance::DuckDBInstance(DBInterface* interface)
:   DBInstance(interface)
{
}

/**
 */
DuckDBInstance::~DuckDBInstance()
{
    close();
}

/**
 */
Result DuckDBInstance::open_impl(const std::string& file_name)
{
    // create the configuration object
    DuckDBScopedConfig config;
    if (!config.valid()) 
        return Result::failed("Could not create db configuration");

    bool db_in_memory = file_name.empty();

    //configure
    config.configure(settings_, interface(), db_in_memory);

    //open db
    char* error = nullptr;
    auto state = duckdb_open_ext(file_name.c_str(), &db_, *config.configuration(), &error);
    if (state != DuckDBSuccess)
    {
        std::string err_str = error ? std::string(error) : std::string("Unknown error");
        return Result::failed(err_str);
    }

    return Result::succeeded();
}

/**
 */
void DuckDBInstance::close_impl()
{
    duckdb_close(&db_);
    db_ = nullptr;
}

/**
 */
ResultT<DBConnection*> DuckDBInstance::createConnection_impl(bool verbose)
{
    auto c = new DuckDBConnection(this, verbose);
    auto res = c->connect();

    if (!res.ok())
    {
        delete c;
        return ResultT<DBConnection*>::failed(res.error());
    }

    return ResultT<DBConnection*>::succeeded(c);
}

/**
 */
Result DuckDBInstance::cleanupDB_impl()
{
    loginf << "executing CHECKPOINT";
    auto res = defaultConnection().execute("CHECKPOINT");
    if (!res.ok())
        return res;

    loginf << "executing VACUUM";
    res = defaultConnection().execute("VACUUM");
    if (!res.ok())
        return res;

    return Result::succeeded();
}

/**
 */
Result DuckDBInstance::exportToFile_impl(const std::string& file_name)
{
    //sync to db file (also merges wal-file)
    auto res = defaultConnection().execute("FORCE CHECKPOINT;");
    if (!res.ok())
        return res;

    std::string current_db = dbInMem() ? "" : dbFilename();

    std::string sql = dbInMem() ? "ATTACH '" + file_name + "' AS db2;" +
                                  "COPY FROM DATABASE memory TO db2;" + 
                                  "DETACH db2;" :
                                  "ATTACH '" + file_name   + "' AS db2;" +
                                  "COPY FROM DATABASE '" + current_db + "' TO db2;" +
                                  "DETACH db2;";

    res = defaultConnection().execute(sql);
    if (!res.ok())
        return res;

    return Result::succeeded();
}

/**
 */
std::vector<db::SQLPragma> DuckDBInstance::sqlPragmas() const
{
    std::vector<db::SQLPragma> pragmas;

    //add config pragmas
    //pragmas.emplace_back("force_compression", "'chimp'");

    return pragmas;
}
