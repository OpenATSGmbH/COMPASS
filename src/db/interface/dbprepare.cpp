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

#include "dbprepare.h"
#include "dbresult.h"
#include "buffer.h"
#include "property_templates.h"

#include "logger.h"

//#define DEBUG_BINDS

/***************************************************************************************
 * DBPrepare
 ***************************************************************************************/

/**
 */
DBPrepare::DBPrepare()
{
}

/**
 */
DBPrepare::~DBPrepare()
{
    if (active_transaction_)
        logerr << "transaction still active";
    traced_assert(!active_transaction_);

    if (active_binds_)
        logerr << "binds still active";
    traced_assert(!active_binds_);

    if (prepared_stmnt_ok_)
        logerr << "cleanup not called";
    traced_assert(!prepared_stmnt_ok_);
}

/**
 */
bool DBPrepare::init(const std::string& sql_statement)
{
    traced_assert(!prepared_stmnt_ok_);
    prepared_stmnt_ok_ = init_impl(sql_statement);

    return prepared_stmnt_ok_;
}

/**
 */
void DBPrepare::cleanup()
{
    if (active_transaction_)
        logerr << "transaction still active";
    traced_assert(!active_transaction_);

    if (active_binds_)
        logerr << "binds still active";
    traced_assert(!active_binds_);

    if (prepared_stmnt_ok_)
    {
        cleanup_impl();
        prepared_stmnt_ok_ = false;
    }
}

/**
 */
std::shared_ptr<DBResult> DBPrepare::execute(const ExecOptions& options)
{
    //create result
    std::shared_ptr<DBResult> result(new DBResult);

    //execute
    execute(&options, result.get());

    return result;
}

/**
 */
bool DBPrepare::execute(const ExecOptions* options, 
                        DBResult* result)
{
    if (!prepared_stmnt_ok_)
        logerr << "prepared statement invalid";
    traced_assert(prepared_stmnt_ok_);

    bool ok = false;

    if (active_binds_)
    {
        #ifdef DEBUG_BINDS
            loginf << "   executing binds...";
        #endif

        //binds are active => execute bind statement as efficiently as possible
        ok = executeBinds_impl();
        cleanupBinds_impl();

        active_binds_ = false;
    }
    else
    {
        //any data expected?
        if (result && options && options->dataExpected())
        {
            //create needed buffer (@TODO: could be inferred from result in duckdb...)
            std::shared_ptr<Buffer> b;
            b.reset(new Buffer(options->buffer_properties.value()));

            result->buffer(b);
        }

        //execute statement and optionally fetch data
        ok = execute_impl(options, result);
    }

    return ok;
}

/**
 */
Result DBPrepare::executeBuffer(const std::shared_ptr<Buffer>& buffer,
                                const boost::optional<size_t>& idx_from, 
                                const boost::optional<size_t>& idx_to)
{
    if (!prepared_stmnt_ok_)
        logerr << "prepared statement invalid";
    traced_assert(prepared_stmnt_ok_);

    traced_assert(buffer);

    if (buffer->size() == 0)
        return false;

    std::shared_ptr<Buffer> b = buffer;

    size_t idx0 = idx_from.has_value() ? idx_from.value() : 0;
    size_t idx1 = idx_to.has_value()   ? idx_to.value()   : b->size() - 1;
    traced_assert(idx0 <= idx1);
    traced_assert(idx1 < b->size());

    const auto& properties = b->properties().properties();
    size_t np = properties.size();

    #define UpdateFunc(PDType, DType, Suffix)                                                           \
        bool is_null = b->isNull(p, r);                                                                 \
        bool ok = is_null ? bind_null(bind_idx) : bind_##Suffix(bind_idx, b->get<DType>(pname).get(r)); \
        if (!ok)                                                                                        \
            logerr << "updating '" << pname << "' failed";                    \
        traced_assert(ok);

    #define NotFoundFunc                                                                           \
        logerr << "unknown property type " << Property::asString(dtype); \
        traced_assert(false);

    for (size_t r = idx0; r <= idx1; ++r)
    {
        #ifdef DEBUG_BINDS
            loginf << "binding buffer row " << r;
        #endif

        for (size_t c = 0; c < np; ++c)
        {
            const auto& p     = properties[ c ];
            auto        dtype = p.dataType();
            const auto& pname = p.name();

            //bind index = 1-based
            size_t bind_idx = c + 1;

            SwitchPropertyDataType(dtype, UpdateFunc, NotFoundFunc)
        }

        //use minimal execution version
        bool ok = execute(nullptr, nullptr);

        #ifdef DEBUG_BINDS
            loginf << "   executed " << np << " bind(s): " << ok;
        #endif
        
        if (!ok)
            logerr << "updating buffer row " << r << " failed";
        traced_assert(ok);
    }

    return Result::succeeded();
}

/**
 */
bool DBPrepare::beginTransaction()
{
    if (!prepared_stmnt_ok_)
        logerr << "prepared statement invalid";
    traced_assert(prepared_stmnt_ok_);

    if (active_transaction_)
        logerr << "transaction already active";
    traced_assert(!active_transaction_);

    bool ok = beginTransaction_impl();
    if (!ok)
        return false;

    active_transaction_ = true;

    return true;
}

/**
 */
bool DBPrepare::commitTransaction()
{
    if (!prepared_stmnt_ok_)
        logerr << "prepared statement invalid";
    traced_assert(prepared_stmnt_ok_);

    if (!active_transaction_)
        logerr << "no transaction active";
    traced_assert(active_transaction_);

    bool ok = commitTransaction_impl();

    //try to rollback in case commit failed
    if (!ok)
        rollbackTransaction_impl();

    //@TODO: guess there is nothing else we can do in case a transaction could be commited?
    active_transaction_ = false;

    return ok;
}

/**
 */
bool DBPrepare::bind_null(size_t idx) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_null_impl(idx); 
#ifdef DEBUG_BINDS
    loginf << "   bind_null @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_bool(size_t idx, bool v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_bool_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_bool @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_char(size_t idx, char v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_char_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_char @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_uchar(size_t idx, unsigned char v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_uchar_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_uchar @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_int(size_t idx, int v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_int_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_int @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_uint(size_t idx, unsigned int v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_uint_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_uint @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_long(size_t idx, long v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_long_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_long @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_ulong(size_t idx, unsigned long v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_ulong_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_ulong @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_float(size_t idx, float v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_float_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_float @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_double(size_t idx, double v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_double_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_double @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_string(size_t idx, const std::string& v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_string_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_string @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_json(size_t idx, const nlohmann::json& v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_json_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_json @" << idx << ": " << ok;
#endif
    return ok;
}

/**
 */
bool DBPrepare::bind_timestamp(size_t idx, const boost::posix_time::ptime& v) 
{ 
    traced_assert(prepared_stmnt_ok_);
    active_binds_ = true; 
    bool ok = bind_timestamp_impl(idx, v); 
#ifdef DEBUG_BINDS
    loginf << "   bind_timestamp @" << idx << ": " << ok;
#endif
    return ok;
}

/***************************************************************************************
 * DBScopedPrepare
 ***************************************************************************************/

/**
 */
DBScopedPrepare::DBScopedPrepare(const std::shared_ptr<DBPrepare>& db_prepare, 
                                 const std::string& sql_statement,
                                 bool begin_transaction) 
:   db_prepare_(db_prepare) 
{
    traced_assert(db_prepare_);
    bool ok = db_prepare_->init(sql_statement);

    if (ok && begin_transaction)
        db_prepare_->beginTransaction();
};

/**
 */
DBScopedPrepare::~DBScopedPrepare()
{
    if (db_prepare_->hasActiveTransaction())
        db_prepare_->commitTransaction();

    db_prepare_->cleanup();
}

/**
 */
std::shared_ptr<DBResult> DBScopedPrepare::execute(const DBPrepare::ExecOptions& options) 
{ 
    return db_prepare_->execute(options); 
}

/**
 */
bool DBScopedPrepare::execute(const DBPrepare::ExecOptions* options, 
                              DBResult* result)
{
    return db_prepare_->execute(options, result);
}

/**
 */
Result DBScopedPrepare::executeBuffer(const std::shared_ptr<Buffer>& buffer,
                                      const boost::optional<size_t>& idx_from, 
                                      const boost::optional<size_t>& idx_to)
{
    return db_prepare_->executeBuffer(buffer, idx_from, idx_to);
}

/**
 */
bool DBScopedPrepare::hasError() const 
{ 
    return db_prepare_->hasError();
}

/**
 */
std::string DBScopedPrepare::lastError() const 
{ 
    return db_prepare_->lastError();
}
