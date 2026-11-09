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

#include "duckdbreader.h"
#include "duckdbprepare.h"
#include "dbresult.h"
#include "buffer.h"
#include "property_templates.h"
#include "dbcommand.h"
#include "duckdbexecresult.h"

#include "logger.h"

/**
 */
DuckDBReader::DuckDBReader(duckdb_connection connection)
:   connection_(connection)
{
    traced_assert(connection_);
}

/**
 */
DuckDBReader::~DuckDBReader() = default;

/**
 */
bool DuckDBReader::init_impl()
{
    auto select_cmd = selectCommand();

    cur_idx_     = offset();
    result_rows_ = 0;
    result_cols_ = 0;

    //prepare select statement
    DuckDBScopedPrepare prepare(connection_, select_cmd->get(), false);

    //succeeded?
    if (!prepare.valid())
    {
        setError("could not prepare select statement");
        return false;
    }

    //execute and obtain duckdb result
    result_ = prepare.executeDuckDB();
    if (!result_ || result_->hasError())
    {
        setError("could not execute select statement");
        result_.reset();
        return false;
    }

    auto nr = result_->numRows();
    auto nc = result_->numColumns();
    traced_assert(nr.has_value());
    traced_assert(nc.has_value());

    result_rows_ = nr.value();
    result_cols_ = nc.value();

    if (result_cols_ != select_cmd->resultList().size())
    {
        setError("unexpected result size");
        result_.reset();
        return false;
    }

    return true;
}

/**
 */
void DuckDBReader::finish_impl()
{
    //free result
    result_.reset();
}

/**
 */
std::shared_ptr<DBResult> DuckDBReader::readChunk_impl()
{
    auto select_cmd = selectCommand();

    std::shared_ptr<DBResult> result(new DBResult);
    std::shared_ptr<Buffer> b(new Buffer(select_cmd->resultList()));
    result->buffer(b);

    //no more results to fetch?
    if (cur_idx_ >= result_rows_)
    {
        result->hasMore(false);
        return result;
    }

    bool has_more = false;

#if 0
    //read lines from result (deprecated)
    if (!result_->toBuffer(*b, cur_idx_, chunkSize()))
    {
        result->setError("result buffer could not be fetched");
        return result;
    }

    cur_idx_ += chunkSize();

    has_more = cur_idx_ < result_rows_;
#else
    //fetch chunks from result
    auto res = result_->readNextChunk(*b, chunkSize());
    if (!res.ok())
    {
        result->setError("result buffer could not be fetched: " + res.error());
        return result;
    }

    cur_idx_ += b->size();

    has_more = res.result();
    traced_assert(has_more || cur_idx_ == result_rows_);
#endif

    result->hasMore(has_more);

    return result;
}

/**
 */
size_t DuckDBReader::numLeft() const
{
    return (cur_idx_ >= result_rows_ ? 0 : result_rows_ - cur_idx_);
}
