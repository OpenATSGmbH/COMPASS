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

#include "result.h"

class Buffer;
class DBResult;

/**
 * Handles preparation and execution of sql statements.
 */
class DBPrepare
{
public:
    struct ExecOptions
    {
        bool dataExpected() const { return buffer_properties.has_value(); }
        bool dataBatchExpected() const { return dataExpected() && result_offset.has_value() && result_max_entries.has_value(); }
    
        ExecOptions& offset(size_t o) { result_offset = o; return *this; }
        ExecOptions& maxEntries(size_t me) { result_max_entries = me; return *this; }
        ExecOptions& bufferProperties(const PropertyList& props) { buffer_properties = props; return *this; }

        boost::optional<size_t>       result_offset;
        boost::optional<size_t>       result_max_entries;
        boost::optional<PropertyList> buffer_properties;
    };

    DBPrepare();
    virtual ~DBPrepare();

    bool init(const std::string& sql_statement);
    void cleanup();

    bool valid() const { return prepared_stmnt_ok_; }

    bool beginTransaction();
    bool commitTransaction();

    bool hasActiveTransaction() const { return active_transaction_; }
    bool hasActiveBinds() const { return active_binds_; }

    bool bind_null(size_t idx);
    bool bind_bool(size_t idx, bool v);
    bool bind_char(size_t idx, char v);
    bool bind_uchar(size_t idx, unsigned char v);
    bool bind_int(size_t idx, int v);
    bool bind_uint(size_t idx, unsigned int v);
    bool bind_long(size_t idx, long v);
    bool bind_ulong(size_t idx, unsigned long v);
    bool bind_float(size_t idx, float v);
    bool bind_double(size_t idx, double v);
    bool bind_string(size_t idx, const std::string& v);
    bool bind_json(size_t idx, const nlohmann::json& v);
    bool bind_timestamp(size_t idx, const boost::posix_time::ptime& v);

    std::shared_ptr<DBResult> execute(const ExecOptions& options = ExecOptions());
    bool execute(const ExecOptions* options = nullptr, 
                 DBResult* result = nullptr);
    Result executeBuffer(const std::shared_ptr<Buffer>& buffer,
                         const boost::optional<size_t>& idx_from = boost::optional<size_t>(), 
                         const boost::optional<size_t>& idx_to = boost::optional<size_t>());

    bool hasError() const { return error_.has_value(); }
    std::string lastError() const { return hasError() ? error_.value() : ""; }

protected:
    void setError(const std::string& err) { error_ = err; }

    virtual bool init_impl(const std::string& sql_statement) = 0;
    virtual void cleanup_impl() = 0;
    virtual void cleanupBinds_impl() {}

    virtual bool beginTransaction_impl() = 0;
    virtual bool commitTransaction_impl() = 0;
    virtual bool rollbackTransaction_impl() = 0;

    virtual bool bind_null_impl(size_t idx) = 0;
    virtual bool bind_bool_impl(size_t idx, bool v) = 0;
    virtual bool bind_char_impl(size_t idx, char v) = 0;
    virtual bool bind_uchar_impl(size_t idx, unsigned char v) = 0;
    virtual bool bind_int_impl(size_t idx, int v) = 0;
    virtual bool bind_uint_impl(size_t idx, unsigned int v) = 0;
    virtual bool bind_long_impl(size_t idx, long v) = 0;
    virtual bool bind_ulong_impl(size_t idx, unsigned long v) = 0;
    virtual bool bind_float_impl(size_t idx, float v) = 0;
    virtual bool bind_double_impl(size_t idx, double v) = 0;
    virtual bool bind_string_impl(size_t idx, const std::string& v) = 0;
    virtual bool bind_json_impl(size_t idx, const nlohmann::json& v) = 0;
    virtual bool bind_timestamp_impl(size_t idx, const boost::posix_time::ptime& v) = 0;

    virtual bool executeBinds_impl() = 0;
    virtual bool execute_impl(const ExecOptions* options, DBResult* result) = 0;

private:
    bool prepared_stmnt_ok_  = false;
    bool active_transaction_ = false;
    bool active_binds_       = false;

    boost::optional<std::string> error_;
};

/**
 * Handles scoped preparation and execution of sql statements.
 */
class DBScopedPrepare
{
public:
    DBScopedPrepare(const std::shared_ptr<DBPrepare>& db_prepare, 
                    const std::string& sql_statement,
                    bool begin_transaction = false);
    virtual ~DBScopedPrepare();

    bool valid() const { return db_prepare_->valid(); }

    bool bind_null(size_t idx) { return db_prepare_->bind_null(idx); }
    bool bind_bool(size_t idx, bool v) { return db_prepare_->bind_bool(idx, v); }
    bool bind_char(size_t idx, char v) { return db_prepare_->bind_char(idx, v); }
    bool bind_uchar(size_t idx, unsigned char v) { return db_prepare_->bind_uchar(idx, v); }
    bool bind_int(size_t idx, int v) { return db_prepare_->bind_int(idx, v); }
    bool bind_uint(size_t idx, unsigned int v) { return db_prepare_->bind_uint(idx, v); }
    bool bind_long(size_t idx, long v) { return db_prepare_->bind_long(idx, v); }
    bool bind_ulong(size_t idx, unsigned long v) { return db_prepare_->bind_ulong(idx, v); }
    bool bind_float(size_t idx, float v) { return db_prepare_->bind_float(idx, v); }
    bool bind_double(size_t idx, double v) { return db_prepare_->bind_double(idx, v); }
    bool bind_string(size_t idx, const std::string& v) { return db_prepare_->bind_string(idx, v); }
    bool bind_json(size_t idx, const nlohmann::json& v) { return db_prepare_->bind_json(idx, v); }
    bool bind_timestamp(size_t idx, const boost::posix_time::ptime& v) { return db_prepare_->bind_timestamp(idx, v); }

    std::shared_ptr<DBResult> execute(const DBPrepare::ExecOptions& options = DBPrepare::ExecOptions());
    bool execute(const DBPrepare::ExecOptions* options = nullptr, 
                 DBResult* result = nullptr);
    Result executeBuffer(const std::shared_ptr<Buffer>& buffer,
                         const boost::optional<size_t>& idx_from = boost::optional<size_t>(), 
                         const boost::optional<size_t>& idx_to = boost::optional<size_t>());

    bool hasError() const;
    std::string lastError() const;

protected:
    std::shared_ptr<DBPrepare> db_prepare_;
};
