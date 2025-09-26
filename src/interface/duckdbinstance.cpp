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
#include "duckdbconnection.h"

#include "logger.h"
#include "files.h"

#include <boost/filesystem.hpp>

#include <QString>
#include <QFile>

using namespace Utils;

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

        void configure(const DuckDBSettings& settings)
        {
            settings.configure(&config_);
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

    //configure
    config.configure(settings_);

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
Result DuckDBInstance::cleanupDB_impl(const std::string& db_fn)
{
    //this method should only be called if there is no active connection
    traced_assert(!dbOpen());

    // std::string dir      = boost::filesystem::path(db_fn).parent_path().string();
    // std::string basename = boost::filesystem::path(db_fn).stem().string();
    // std::string ext      = boost::filesystem::path(db_fn).extension().string();

    //std::string fn_temp  = dir + "/" + basename + ext + ".tmp";

    std::string fn_temp = db_fn + ".tmp";

    if (Files::fileExists(fn_temp))
    {
        logwrn << "temp file '" << fn_temp << "' already exists, deleting";
        Files::deleteFile(fn_temp);

        if (Files::fileExists(fn_temp))
        {
            logerr << "deleting temp file '" << fn_temp << "' failed, aborting";
            return Result::failed("Could not delete temorary database");
        }
    }

    loginf << "moving db '" << db_fn << "' exists " << Files::fileExists(db_fn) << " to '"
           << fn_temp << "' exists " << Files::fileExists(fn_temp);

    //try to prepare current database for compression

    if (!Files::moveFile(db_fn, fn_temp))
        return Result::failed("Could not create temorary database");

    // if (!QFile::rename(QString::fromStdString(db_fn), QString::fromStdString(fn_temp)))
    //     return Result::failed("Could not create temorary database");

    //connect to in-mem db
    duckdb_database   db;
    duckdb_connection con;

    if (duckdb_open(NULL, &db) == DuckDBError) 
    {
        //revert back to old file (if possible)
        //QFile::rename(QString::fromStdString(fn_temp), QString::fromStdString(db_fn));
        Files::moveFile(fn_temp, db_fn);

        return Result::failed("Could not open memory db");
    }
    if (duckdb_connect(db, &con) == DuckDBError) 
    {
        //revert back to old file (if possible)
        //QFile::rename(QString::fromStdString(fn_temp), QString::fromStdString(db_fn));

        Files::moveFile(fn_temp, db_fn);

        return Result::failed("Could not open connection to memory db");
    }

    //perform compression into new file
    std::string sql = "ATTACH '" + fn_temp + "' AS db1;" +
                      "ATTACH '" + db_fn   + "' AS db2;" +
                      "COPY FROM DATABASE db1 TO db2;";

    auto state = duckdb_query(con, sql.c_str(), nullptr);
    if (state != DuckDBSuccess)
    {
        //remove any failed result + revert back to old file (if possible)
        QFile::remove(QString::fromStdString(db_fn));
        //QFile::rename(QString::fromStdString(fn_temp), QString::fromStdString(db_fn));

        Files::moveFile(fn_temp, db_fn);
        return Result::failed("Compressing database failed");
    }

    //compression successful => try to remove old file
    if (!QFile::remove(QString::fromStdString(fn_temp)))
        logwrn << "Could not remove intermediate database file";
    
    duckdb_disconnect(&con);
    duckdb_close(&db);

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
