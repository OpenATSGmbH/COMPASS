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

#include "dbcontentdb.h"
#include "dbcontent.h"
#include "dbcontentmanager.h"
#include "dbcontentdataengine.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/target/target.h"

#include "dbinterface.h"
#include "dbinstance.h"
#include "dbtableinfo.h"
#include "reporttable.h"
#include "dbcommand.h"

#include "base/idbvariableresolver.h"
#include "buffer.h"
#include "propertylist.h"
#include "number.h"
#include "timeconv.h"
#include "logger.h"

#include <set>
#include <sstream>

using namespace Utils;
using namespace dbContent;

/**
 */
DBContentDB::DBContentDB(DBInterface& db_interface, DBContentManager& dbcont_man)
:   db_interface_(db_interface)
,   dbcont_man_  (dbcont_man)
{
}

/**
 * Creates the table of the DB content, with indices on some meta variables if the database uses indexing.
 */
void DBContentDB::createTable(const DBContent& dbcontent)
{
    loginf << "obj " << dbcontent.name();

    traced_assert(db_interface_.ready());

    if (db_interface_.existsTable(dbcontent.dbTableName()))
    {
        logerr << "table " << dbcontent.dbTableName() << " already exists";
        return;
    }

    //collect needed columns
    std::vector<DBTableColumnInfo> column_infos;
    for (auto& var_it : dbcontent.variables())
    {
        const auto& v = var_it.second;

        if (!v->dbColumnName().size())
            continue;

        column_infos.push_back(DBTableColumnInfo(v->dbColumnName(), v->dataType(), v->isKey()));
    }

    //enable indexing on some metavars?
    std::vector<db::Index> indices;
    if (db_interface_.dbInstance().sqlConfiguration().indexing)
    {
        const std::string& name = dbcontent.name();

        indices.emplace_back("TIMESTAMP_INDEX_" + name,
                             dbcont_man_.metaGetVariable(name, dbcontent_vars::meta_var_timestamp_).dbColumnName());
        indices.emplace_back("DS_ID_INDEX_" + name,
                             dbcont_man_.metaGetVariable(name, dbcontent_vars::meta_var_ds_id_).dbColumnName());
        indices.emplace_back("LINE_ID_INDEX_" + name,
                             dbcont_man_.metaGetVariable(name, dbcontent_vars::meta_var_line_id_).dbColumnName());
        if (dbcont_man_.metaCanGetVariable(name, dbcontent_vars::meta_var_utn_))
        {
            indices.emplace_back("UTN_INDEX_" + name,
                                 dbcont_man_.metaGetVariable(name, dbcontent_vars::meta_var_utn_).dbColumnName());
        }
    }

    db_interface_.createTable(dbcontent.dbTableName(), column_infos, indices);

    loginf << "checking " << dbcontent.dbTableName();

    traced_assert(db_interface_.existsTable(dbcontent.dbTableName()));
}

/**
 * Adds the record numbers to the buffer and stores the new max record number.
 */
void DBContentDB::initDBContentBuffer(DBContent& dbcontent, std::shared_ptr<Buffer> buffer)
{
    // create record numbers & and store new max rec num
    traced_assert(dbcontent.hasVariable(dbcontent_vars::meta_var_rec_num_.name()));

    Variable& rec_num_var = dbcontent.variable(dbcontent_vars::meta_var_rec_num_.name());
    traced_assert(rec_num_var.dataType() == PropertyDataType::ULONGINT);

    std::string rec_num_col_str = rec_num_var.dbColumnName();
    traced_assert(!buffer->has<std::uint64_t>(rec_num_col_str));

    buffer->addProperty(rec_num_col_str, PropertyDataType::ULONGINT);

    traced_assert(dbcont_man_.dataEngine().hasMaxRecordNumberWODBContentID());
    std::uint64_t max_rec_num = dbcont_man_.dataEngine().maxRecordNumberWODBContentID();

    NullableVector<std::uint64_t>& rec_num_vec = buffer->get<std::uint64_t>(rec_num_col_str);

    unsigned int buffer_size = buffer->size();
    unsigned int dbcont_id = dbcontent.id();

    for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
    {
        ++max_rec_num;
        rec_num_vec.set(cnt, Number::recNumAddDBContId(max_rec_num, dbcont_id));
    }

    dbcont_man_.dataEngine().maxRecordNumberWODBContentID(max_rec_num);
}

/**
 */
void DBContentDB::insertDBContent(DBContent& dbcontent, std::shared_ptr<Buffer> buffer)
{
    logdbg << "dbcont " << dbcontent.name() << " buffer size " << buffer->size();

    traced_assert(db_interface_.ready());
    traced_assert(buffer);
    traced_assert(!db_interface_.cleanupInProgress());

    // create table if required
    if (!db_interface_.existsTable(dbcontent.dbTableName()))
        createTable(dbcontent);

    initDBContentBuffer(dbcontent, buffer);
    db_interface_.insertBuffer(dbcontent.dbTableName(), buffer);
}

/**
 * Inserts multiple dbcontent buffers at once, possibly utilizing parallelization.
 */
void DBContentDB::insertDBContent(const std::map<std::string, std::shared_ptr<Buffer>>& buffers)
{
    traced_assert(db_interface_.ready());
    traced_assert(!db_interface_.cleanupInProgress());

    std::map<std::string, std::shared_ptr<Buffer>> table_buffers; // table name -> buffer

    for (const auto& [ dbcont_name, buffer ] : buffers)
    {
        traced_assert(buffer);

        auto& dbcontent = dbcont_man_.dbContent(dbcont_name);

        //create table if needed
        if (!db_interface_.existsTable(dbcontent.dbTableName()))
            createTable(dbcontent);

        //init buffer
        initDBContentBuffer(dbcontent, buffer);

        table_buffers[ dbcontent.dbTableName() ] = buffer;
    }

    db_interface_.insertBuffers(table_buffers);
}

/**
 */
void DBContentDB::prepareRead(const DBContent& dbcontent,
                              VariableSet read_list,
                              std::string custom_filter_clause,
                              bool use_order,
                              Variable* order_variable)
{
    logdbg << "dbcont_name " << dbcontent.name() << " read list size " << read_list.getSize();

    traced_assert(db_interface_.ready());
    traced_assert(dbcontent.existsInDB());
    traced_assert(read_list.getSize() != 0);

    //collect needed properties
    PropertyList property_list;
    for (auto var_it : read_list.getSet())
    {
        Variable* variable = var_it;
        property_list.addProperty(variable->dbColumnName(), variable->dataType());
    }

    db_interface_.prepareRead(dbcontent.dbTableName(),
                              property_list,
                              custom_filter_clause,
                              use_order,
                              order_variable ? order_variable->dbColumnName() : "",
                              reportTableJoinClause(dbcontent, read_list));
}

/**
 */
std::pair<std::shared_ptr<Buffer>, bool> DBContentDB::readDataChunk(const DBContent& dbcontent)
{
    auto result = db_interface_.readDataChunk();

    result.first->dbContentName(dbcontent.name());

    return result;
}

/**
 */
void DBContentDB::finalizeReadStatement(const DBContent& dbcontent)
{
    db_interface_.finalizeReadStatement();
}

/**
 * The Report Variables of the read set are read through one LEFT JOIN subquery per Report Table,
 * keyed on the record number. Every subquery column is aliased, so the main table columns stay
 * unqualified in the SELECT list, the WHERE clause and the ORDER BY. A record without a row in
 * the table gets null, which the Views treat like a missing value.
 */
std::string DBContentDB::reportTableJoinClause(const DBContent& dbcontent,
                                               const VariableSet& read_list) const
{
    // join alias -> (table name, key alias, column aliases in read set order)
    struct Join
    {
        std::string                                      table_name;
        std::vector<std::pair<std::string, std::string>> columns; // column name, alias
    };

    std::map<std::string, Join> joins;
    std::vector<std::string>    join_order;

    for (const Variable* var : read_list.getSet())
    {
        if (!var || !var->isReportVariable())
            continue;

        const std::string& alias = var->reportJoinAlias();

        if (!joins.count(alias))
        {
            joins[ alias ].table_name = var->reportTableName();
            join_order.push_back(alias);
        }

        joins[ alias ].columns.emplace_back(var->reportColumnName(), var->dbColumnName());
    }

    if (joins.empty())
        return "";

    //the record number column of the host data content
    if (!dbcont_man_.metaCanGetVariable(dbcontent.name(), dbcontent_vars::meta_var_rec_num_))
    {
        logerr << "dbcontent '" << dbcontent.name() << "' has no record number, report tables skipped";
        return "";
    }

    const std::string rec_num_column =
        dbcont_man_.metaGetVariable(dbcontent.name(), dbcontent_vars::meta_var_rec_num_).dbColumnName();

    std::stringstream ss;

    for (const auto& alias : join_order)
    {
        const auto& join = joins.at(alias);

        const std::string key_alias = alias + "__" + ReportTableDefinition::KeyColumnName;

        ss << " LEFT JOIN (SELECT \"" << ReportTableDefinition::KeyColumnName << "\" AS " << key_alias;

        for (const auto& column : join.columns)
            ss << ", \"" << column.first << "\" AS " << column.second;

        ss << " FROM \"" << join.table_name << "\") AS " << alias
           << " ON " << alias << "." << key_alias << " = " << rec_num_column;
    }

    return ss.str();
}

/**
 * Deletes the rows of the DB content table matching the filter (empty = all rows).
 */
void DBContentDB::deleteRows(const DBContent& dbcontent, const std::string& filter)
{
    traced_assert(db_interface_.ready());

    auto res = db_interface_.deleteRows(dbcontent.dbTableName(), filter);
    traced_assert(res.ok());
}

/**
 * Deletes the data older than the given timestamp (live mode), runs the periodic cleanup if due.
 */
void DBContentDB::deleteBefore(const DBContent& dbcontent, boost::posix_time::ptime before_timestamp)
{
    traced_assert(db_interface_.ready());
    traced_assert(!db_interface_.cleanupInProgress());

    std::stringstream ss;
    ss << dbcont_man_.metaGetVariable(dbcontent.name(), dbcontent_vars::meta_var_timestamp_).dbColumnName();
    ss << " < " << Time::toLong(before_timestamp);

    deleteRows(dbcontent, ss.str());

    db_interface_.liveCleanupIfDue();
}

/**
 */
void DBContentDB::deleteAll(const DBContent& dbcontent)
{
    deleteRows(dbcontent, "");
}

/**
 */
void DBContentDB::deleteContent(const DBContent& dbcontent, unsigned int sac, unsigned int sic)
{
    loginf << "dbcontent " << dbcontent.name() << " sac/sic " << sac << "/" << sic;

    std::stringstream ss;
    ss << dbcont_man_.metaGetVariable(dbcontent.name(), dbcontent_vars::meta_var_sac_id_).dbColumnName() << " = " << sac
       << " AND "
       << dbcont_man_.metaGetVariable(dbcontent.name(), dbcontent_vars::meta_var_sic_id_).dbColumnName() << " = " << sic;

    deleteRows(dbcontent, ss.str());
}

/**
 */
void DBContentDB::deleteContent(const DBContent& dbcontent, unsigned int sac, unsigned int sic, unsigned int line_id)
{
    loginf << "dbcontent " << dbcontent.name() << " sac/sic " << sac << "/" << sic << " line " << line_id;

    std::stringstream ss;
    ss << dbcont_man_.metaGetVariable(dbcontent.name(), dbcontent_vars::meta_var_sac_id_).dbColumnName() << " = " << sac
       << " AND "
       << dbcont_man_.metaGetVariable(dbcontent.name(), dbcontent_vars::meta_var_sic_id_).dbColumnName() << " = " << sic
       << " AND "
       << dbcont_man_.metaGetVariable(dbcontent.name(), dbcontent_vars::meta_var_line_id_).dbColumnName() << " = " << line_id;

    deleteRows(dbcontent, ss.str());
}

/**
 * Sets all UTNs of the DB content to NULL.
 */
void DBContentDB::clearAssociations(const DBContent& dbcontent)
{
    db_interface_.setColumnNull(dbcontent.dbTableName(), dbcontent.variable("UTN").dbColumnName());
}

/**
 */
std::uint64_t DBContentDB::getMaxRecordNumber(DBContent& dbcontent)
{
    traced_assert(db_interface_.ready());
    traced_assert(dbcontent.existsInDB());

    traced_assert(dbcont_man_.existsMetaVariable(dbcontent_vars::meta_var_rec_num_.name()));
    traced_assert(dbcont_man_.metaVariable(
                dbcontent_vars::meta_var_rec_num_.name()).existsIn(dbcontent.name()));

    Variable& rec_num_var = dbcont_man_.metaVariable(
                dbcontent_vars::meta_var_rec_num_.name()).getFor(dbcontent.name());

    traced_assert(dbcontent.hasVariable(rec_num_var.name()));

    auto max_rn = db_interface_.getMaxULongValue(dbcontent.dbTableName(), rec_num_var.dbColumnName());

    if (!max_rn)
    {
        logwrn << "no max record number found";
        return 0;
    }

    return *max_rn;
}

/**
 */
boost::optional<unsigned int> DBContentDB::getMaxRefTrackTrackNum()
{
    traced_assert(db_interface_.ready());

    DBContent& reftraj_content = dbcont_man_.dbContent("RefTraj");

    if (!reftraj_content.existsInDB())
        return {};

    traced_assert(dbcont_man_.existsMetaVariable(dbcontent_vars::meta_var_track_num_.name()));
    traced_assert(dbcont_man_.metaVariable(
                dbcontent_vars::meta_var_track_num_.name()).existsIn("RefTraj"));

    Variable& track_num_var = dbcont_man_.metaVariable(
                dbcontent_vars::meta_var_track_num_.name()).getFor("RefTraj");

    traced_assert(reftraj_content.hasVariable(track_num_var.name()));

    // empty table: no row, or a row with a NULL max
    auto max_tn = db_interface_.getMaxUIntValue(reftraj_content.dbTableName(), track_num_var.dbColumnName());

    if (!max_tn)
    {
        logwrn << "no max track number found";
        return {};
    }

    return (unsigned int)*max_tn;
}

/**
 */
bool DBContentDB::existsTargetsTable()
{
    return db_interface_.existsTable(TABLE_NAME_TARGETS);
}

/**
 */
void DBContentDB::createTargetsTable()
{
    loginf;

    traced_assert(db_interface_.ready());
    traced_assert(!existsTargetsTable());

    //loginf << "sql '" << sql << "'"; // statement is built inside DBInterface::createTable()

    db_interface_.createTable(TABLE_NAME_TARGETS, Target::DBPropertyList, 0);
}

/**
 */
void DBContentDB::clearTargetsTable()
{
    db_interface_.clearTableContent(TABLE_NAME_TARGETS);
}

/**
 */
std::vector<std::unique_ptr<Target>> DBContentDB::loadTargets()
{
    loginf;

    traced_assert(db_interface_.ready());

    std::vector<std::unique_ptr<Target>> targets;

    PropertyList list;
    list.addProperty("utn", PropertyDataType::UINT);
    list.addProperty("json", PropertyDataType::STRING);

    auto result = db_interface_.select(TABLE_NAME_TARGETS, list, "");
    traced_assert(result.ok());

    std::shared_ptr<Buffer> buffer = result.result();

    traced_assert(buffer);
    traced_assert(buffer->has<unsigned int>("utn"));
    traced_assert(buffer->has<std::string>("json"));

    NullableVector<unsigned int>& utn_vec = buffer->get<unsigned int>("utn");
    NullableVector<std::string>& json_vec = buffer->get<std::string>("json");

    unsigned int utn;
    std::string json_str;

    std::set<unsigned int> existing_utns;

    for (size_t cnt = 0; cnt < buffer->size(); ++cnt)
    {
        traced_assert(!utn_vec.isNull(cnt));
        traced_assert(!json_vec.isNull(cnt));

        utn = utn_vec.get(cnt);
        json_str = json_vec.get(cnt);

        traced_assert(!existing_utns.count(utn));
        existing_utns.insert(utn);

        targets.emplace_back(new Target(utn, nlohmann::json::parse(json_str)));
    }

    return targets;
}

/**
 */
void DBContentDB::saveTargets(const std::map<unsigned int, nlohmann::json>& targets_info)
{
    logdbg;

    traced_assert(db_interface_.ready());

    clearTargetsTable();

    {
        //storing all targets at once via a buffer is faster
        std::shared_ptr<Buffer> buffer(new Buffer(Target::DBPropertyList));

        auto& id_vec   = buffer->get<unsigned int>(Target::DBColumnID.name());
        auto& info_vec = buffer->get<nlohmann::json>(Target::DBColumnInfo.name());

        size_t idx = 0;
        for (auto& tgt_it : targets_info)
        {
            id_vec.set(idx, tgt_it.first);
            info_vec.set(idx, tgt_it.second);
            ++idx;
        }

        db_interface_.insertBuffer(TABLE_NAME_TARGETS, buffer);
    }

    logdbg << "done";
}

/**
 */
void DBContentDB::updateTargets(const std::map<unsigned int, nlohmann::json>& targets_info)
{
    loginf << "updating " << targets_info.size() << " utn(s)";

    traced_assert(db_interface_.ready());

    if (targets_info.size() > DBInterface::TableBulkUpdateMinRows)
    {
        //updating many targets at once via a buffer is faster
        std::shared_ptr<Buffer> buffer(new Buffer(Target::DBPropertyList));

        auto& id_vec   = buffer->get<unsigned int>(Target::DBColumnID.name());
        auto& info_vec = buffer->get<nlohmann::json>(Target::DBColumnInfo.name());

        size_t idx = 0;
        for (auto& tgt_it : targets_info)
        {
            id_vec.set(idx, tgt_it.first);
            info_vec.set(idx, tgt_it.second);
            ++idx;
        }

        db_interface_.updateBuffer(TABLE_NAME_TARGETS, Target::DBColumnID.name(), buffer);
    }
    else
    {
        //run individual update for each target, replace with utn as unique key
        for (auto& tgt_it : targets_info)
            db_interface_.replaceRow(TABLE_NAME_TARGETS, { std::to_string(tgt_it.first), tgt_it.second.dump() });
    }

    loginf << "done";
}

/**
 */
std::shared_ptr<DBCommand> DBContentDB::getADSBInfoCommand(DBContent& adsb_obj)
{
    PropertyList list;
    list.addProperty("TARGET_ADDR", PropertyDataType::INT);
    list.addProperty("MOPS_VERSION", PropertyDataType::INT);
    list.addProperty("MIN_NUCP_NIC", PropertyDataType::CHAR);
    list.addProperty("MAX_NUCP_NIC", PropertyDataType::CHAR);
    list.addProperty("MIN_NACP", PropertyDataType::CHAR);
    list.addProperty("MAX_NACP", PropertyDataType::CHAR);

    std::shared_ptr<DBCommand> command = std::make_shared<DBCommand>(DBCommand());

    std::stringstream ss;

    ss << "SELECT TARGET_ADDR, MOPS_VERSION, MIN(NUCP_NIC), MAX(NUCP_NIC), MIN(NAC_P), MAX(NAC_P)";

    ss << " FROM ";

    ss << adsb_obj.dbTableName();

    ss << " group by TARGET_ADDR;";

    command->set(ss.str());
    command->list(list);

    return command;
}
