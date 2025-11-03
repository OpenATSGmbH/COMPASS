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

#include "sqliteinstance.h"
#include "sqliteconnection.h"

#include "logger.h"

/**
 */
SQLiteInstance::SQLiteInstance(DBInterface* interface)
:   DBInstance(interface)
{
    loginf << "SQLITE_VERSION " << SQLITE_VERSION;
}

/**
 */
SQLiteInstance::~SQLiteInstance()
{
    close();
}

/**
 */
Result SQLiteInstance::open_impl(const std::string& file_name)
{
    loginf << "'" << file_name << "'";

    int result = sqlite3_open_v2(file_name.c_str(), &db_handle_,
                                 SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);

//    int result = sqlite3_open(":memory:", &db_handle_);

    if (result != SQLITE_OK)
    {
        // Even in case of an error we get a valid db_handle (for the
        // purpose of calling sqlite3_errmsg on it ...)
        std::string err(sqlite3_errmsg(db_handle_));
        sqlite3_close(db_handle_);
        return Result::failed(err);
    }

    char* sErrMsg = 0;
    sqlite3_exec(db_handle_, "PRAGMA SYNCHRONOUS = OFF", NULL, NULL, &sErrMsg);
    sqlite3_exec(db_handle_, "PRAGMA TEMP_STORE = 2", NULL, NULL, &sErrMsg);
    sqlite3_exec(db_handle_, "PRAGMA JOURNAL_MODE = OFF", NULL, NULL, &sErrMsg);
    sqlite3_exec(db_handle_, "PRAGMA LOCKING_MODE = EXCLUSIVE", NULL, NULL, &sErrMsg);
    sqlite3_exec(db_handle_, "PRAGMA CACHE_SIZE = 500", NULL, NULL, &sErrMsg);

    //sqlite3_exec(db_handle_, "PRAGMA locking_mode = EXCLUSIVE", NULL, NULL, &sErrMsg);

    return Result::succeeded();
}

/**
 */
void SQLiteInstance::close_impl()
{
    if (db_handle_)
    {
        sqlite3_close(db_handle_);
        db_handle_ = nullptr;
    }
}

/**
 */
ResultT<DBConnection*> SQLiteInstance::createConnection_impl(bool verbose)
{
    auto c = new SQLiteConnection(this, db_handle_, verbose);
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
Result SQLiteInstance::exportToFile_impl(const std::string& file_name)
{
    std::string tmp_sql = "VACUUM INTO '" + file_name + "';";

    return defaultConnection().execute(tmp_sql);
}
