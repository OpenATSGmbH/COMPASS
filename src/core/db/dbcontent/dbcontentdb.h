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

#include <cstdint>

#include "dbcontent/variable/variableset.h"

#include "json_fwd.hpp"

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

class Buffer;
class DBCommand;
class DBContent;
class DBContentManager;
class DBInterface;

namespace dbContent
{
class Target;
class Variable;
}

/**
 * DB content specific database access, built on the generic DBInterface API.
 * - DB content tables: create, insert with record numbers, chunked read, delete, associations
 * - max record number and RefTraj track number
 * - targets table, created when the database was opened
 * Owned by DBContentManager.
 */
class DBContentDB
{
public:
    DBContentDB(DBInterface& db_interface, DBContentManager& dbcont_man);
    virtual ~DBContentDB() = default;

    DBInterface& dbInterface() { return db_interface_; }
    DBContentManager& dbContentManager() { return dbcont_man_; }

    void createTable(const DBContent& dbcontent);

    // insert data, creates the tables if required and adds the record numbers
    void insertDBContent(DBContent& dbcontent, std::shared_ptr<Buffer> buffer);
    void insertDBContent(const std::map<std::string, std::shared_ptr<Buffer>>& buffers); // dbcontent name -> buffer

    void prepareRead(const DBContent& dbcontent,
                     dbContent::VariableSet read_list,
                     std::string custom_filter_clause,
                     bool use_order = false,
                     dbContent::Variable* order_variable = nullptr);
    std::pair<std::shared_ptr<Buffer>, bool> readDataChunk(const DBContent& dbcontent); // last one flag
    void finalizeReadStatement(const DBContent& dbcontent);

    void deleteBefore(const DBContent& dbcontent, boost::posix_time::ptime before_timestamp);
    void deleteAll(const DBContent& dbcontent);
    void deleteContent(const DBContent& dbcontent, unsigned int sac, unsigned int sic);
    void deleteContent(const DBContent& dbcontent, unsigned int sac, unsigned int sic, unsigned int line_id);

    void clearAssociations(const DBContent& dbcontent);

    std::uint64_t getMaxRecordNumber(DBContent& dbcontent);
    boost::optional<unsigned int> getMaxRefTrackTrackNum(); // none if no RefTraj data

    bool existsTargetsTable();
    void createTargetsTable();
    void clearTargetsTable();
    std::vector<std::unique_ptr<dbContent::Target>> loadTargets();
    void saveTargets(const std::map<unsigned int, nlohmann::json>& targets_info);
    void updateTargets(const std::map<unsigned int, nlohmann::json>& targets_info);

private:
    void initDBContentBuffer(DBContent& dbcontent, std::shared_ptr<Buffer> buffer);
    void deleteRows(const DBContent& dbcontent, const std::string& filter);

    /// LEFT JOIN subqueries of the Report Tables the read set references, one per table,
    /// empty if the read set holds no Report Variable. See readme_dynamic_dbcontent.md 4.4
    std::string reportTableJoinClause(const DBContent& dbcontent,
                                      const dbContent::VariableSet& read_list) const;

    std::shared_ptr<DBCommand> getADSBInfoCommand(DBContent& adsb_obj); // unused, see queryADSBInfo() in dbinterface.cpp

    DBInterface&      db_interface_;
    DBContentManager& dbcont_man_;
};
