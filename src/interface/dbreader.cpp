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

#include "dbreader.h"
#include "dbresult.h"
#include "buffer.h"
#include "property_templates.h"
#include "dbcommand.h"

#include "logger.h"

#include <QString>

/***************************************************************************************
 * DBReader
 ***************************************************************************************/

/**
 */
DBReader::DBReader()
{
}

/**
 */
DBReader::~DBReader()
{
    if (ready_)
        logerr << "finish not called";
    traced_assert(!ready_);
}

/**
 */
bool DBReader::init(const std::shared_ptr<DBCommand>& select_cmd,
                    size_t offset, 
                    size_t chunk_size)
{
    traced_assert(!ready_);
    traced_assert(select_cmd);

    ready_      = false;
    select_cmd_ = select_cmd;
    offset_     = offset;
    chunk_size_ = chunk_size;

    if (!select_cmd_->expectsResult())
    {
        setError("select command does not provided expected properties");
        return false;
    }
    if (!QString::fromStdString(select_cmd_->get()).startsWith("SELECT "))
    {
        setError("not a valid select command");
        return false;
    }

    ready_ = init_impl();
    return ready_;
}

/**
 */
void DBReader::finish()
{
    if (ready_)
    {
        finish_impl();
        ready_ = false;
    }
}

/**
 */
std::shared_ptr<DBResult> DBReader::readChunk()
{
    traced_assert(isReady());
    return readChunk_impl();
}

/**
 */
bool DBReader::hasError()  const
{ 
    return error_.has_value(); 
}

/**
 */
std::string DBReader::lastError() const 
{ 
    return hasError() ? error_.value() : ""; 
}

/***************************************************************************************
 * DBScopedReader
 ***************************************************************************************/

/**
 */
DBScopedReader::DBScopedReader(const std::shared_ptr<DBReader>& db_reader, 
                               const std::shared_ptr<DBCommand>& select_cmd,
                               size_t offset, 
                               size_t chunk_size) 
:   db_reader_(db_reader) 
{
    traced_assert(db_reader_);
    db_reader_->init(select_cmd, offset, chunk_size);
};

/**
 */
DBScopedReader::~DBScopedReader()
{
    db_reader_->finish();
}

/**
 */
bool DBScopedReader::isReady() const 
{ 
    return db_reader_->isReady(); 
}

/**
 */
std::shared_ptr<DBResult> DBScopedReader::readChunk() 
{
    return db_reader_->readChunk(); 
}

/**
 */
bool DBScopedReader::hasError()  const
{ 
    return db_reader_->hasError(); 
}

/**
 */
std::string DBScopedReader::lastError() const 
{ 
    return db_reader_->lastError(); 
}

/**
 */
size_t DBScopedReader::numLeft() const
{
    return db_reader_->numLeft(); 
}
