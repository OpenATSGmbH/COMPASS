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

#include "propertylist.h"

#include <string>
#include <memory>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

#include <json.hpp>

class Buffer;
class DBResult;
class DBCommand;

/**
 * Handles reading from a db.
 */
class DBReader
{
public:
    DBReader();
    virtual ~DBReader();

    bool isReady() const { return ready_; }

    bool init(const std::shared_ptr<DBCommand>& select_cmd, size_t offset, size_t chunk_size);
    void finish();

    std::shared_ptr<DBResult> readChunk();

    bool hasError() const;
    std::string lastError() const;

    virtual size_t numLeft() const = 0;

protected:
    void setError(const std::string& error) { error_ = error; }

    DBCommand* selectCommand() const { return select_cmd_.get(); }
    size_t offset() const { return offset_; }
    size_t chunkSize() const { return chunk_size_; }

    virtual bool init_impl() = 0;
    virtual void finish_impl() = 0;
    virtual std::shared_ptr<DBResult> readChunk_impl() = 0;

private:
    bool                         ready_      = false;
    std::shared_ptr<DBCommand>   select_cmd_;
    size_t                       offset_     = 0;
    size_t                       chunk_size_ = 0;
    boost::optional<std::string> error_;
};

/**
 * Handles db reader scope.
 */
class DBScopedReader
{
public:
    DBScopedReader(const std::shared_ptr<DBReader>& db_reader, 
                   const std::shared_ptr<DBCommand>& select_cmd,
                   size_t offset, 
                   size_t chunk_size);
    virtual ~DBScopedReader();

    bool isReady() const;
    std::shared_ptr<DBResult> readChunk();

    bool hasError() const;
    std::string lastError() const;

    size_t numLeft() const;

private:
    std::shared_ptr<DBReader> db_reader_;
};
