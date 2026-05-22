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

#include "configurable.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "idbvariableresolver.h"
#include "traced_assert.h"

#include <QObject>

#include <memory>
#include <string>
#include <vector>

class COMPASS;
class PropertyList;

class DBContentWidget;
class Buffer;
class Job;
class DBContentReadDBJob;
class UpdateBufferDBJob;
class DBContentManager;
class DBContentDeleteDBJob;

namespace dbContent
{
class VariableSet;

//bits 8/7
//    (TRANS)
//    Transversal Acceleration
//    = 00 Constant Course
//    = 01 Right Turn
//    = 10 Left Turn
//    = 11 Undetermined
//      bits 6/5

enum class MOM_TRANS_ACC
{
    ConstantCourse=0,
    RightTurn, // 1
    LeftTurn, // 2
    Undetermined // 3
};

//      (LONG)
//      Longitudinal Acceleration
//    = 00 Constant Groundspeed
//    = 01 Increasing Groundspeed
//    = 10 Decreasing Groundspeed
//    = 11 Undetermined

enum class MOM_LONG_ACC
{
    ConstantGroundspeed=0,
    IncreasingGroundspeed, // 1
    DecreasingGroundspeed, // 2
    Undetermined // 3
};

//      bits 4/3
//      (VERT)
//      Vertical Rate
//    = 00 Level
//    = 01 Climb
//    = 10 Descent
//    = 11 Undetermined

enum class MOM_VERT_RATE
{
    Level=0,
    Climb, // 1
    Descent, // 2
    Undetermined // 3
};

}

/**
 */
class DBContent : public QObject, public Configurable
{
    Q_OBJECT

signals:
    void updateProgressSignal(float percent);
    void updateDoneSignal(DBContent& dbcontent);

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();

    void readJobObsoleteSlot();
    void readJobDoneSlot();

    void updateProgressSlot(float percent);
    void updateDoneSlot();

    void deleteJobDoneSlot();

public:
    DBContent(nlohmann::json& config, DBContentManager* parent);
    virtual ~DBContent();

    bool hasVariable(const std::string& name) const;
    dbContent::Variable& variable(const std::string& name) const;
    void renameVariable(const std::string& name, 
                        const std::string& new_name);
    void deleteVariable(const std::string& name);

    const std::map<std::string, std::unique_ptr<dbContent::Variable>>& variables() const { return variables_; }

    bool hasVariableDBColumnName(const std::string& col_name) const;

    size_t numVariables() const { return variables_.size(); }

    const std::string& name() const { return name_; }
    void name(const std::string& name)
    {
        traced_assert(name.size() > 0);
        name_ = name;
    }

    unsigned int id() const;

    const std::string& info() const { return info_; }
    void info(const std::string& info) { info_ = info; }

    bool loadable() const { return is_loadable_; }

    void quitLoading();

    bool prepareInsert(std::shared_ptr<Buffer>& buffer);
    /// Returns true if a new data source was created in the active context.
    /// Does NOT emit countsChangedSignal/dataSourcesChangedSignal - the
    /// caller (DBContentManager::insertData) coalesces those across all
    /// buffers of one insert and emits once at the end.
    bool updateDataSourcesBeforeInsert(std::shared_ptr<Buffer>& buffer);
    void finalizeInsert(std::shared_ptr<Buffer>& buffer);

    void updateData(dbContent::Variable& key_var, 
                    std::shared_ptr<Buffer> buffer);

    // counts and targets have to be adjusted outside
    void deleteDBContentData(bool cleanup_db = false);
    void deleteDBContentData(unsigned int sac, 
                             unsigned int sic,
                             bool cleanup_db = false);
    void deleteDBContentData(unsigned int sac, 
                             unsigned int sic, 
                             unsigned int line_id,
                             bool cleanup_db = false);

    bool isLoading();
    bool isDeleting();

    bool hasData();
    size_t count();
    size_t loadedCount();

    void refreshCount();

    virtual void generateSubConfigurable(nlohmann::json& child_json) override;

    bool hasKeyVariable();
    dbContent::Variable& getKeyVariable();

    std::string status();

    DBContentWidget* widget();
    void closeWidget();

    bool existsInDB() const;

    std::string dbTableName() const;

    bool containsTargetReports() const;
    bool containsStatusContent() const;
    bool isReferenceContent() const;

    COMPASS& compass() { return compass_; }

private:
    friend class DBContentManager;

    // Sole low-level entry to start a read job. Always called by the manager from
    // DBContentManager::load(LoadRequest) after composing the WHERE clause.
    void loadInternal(dbContent::VariableSet& read_set,
                      std::string custom_filter_clause);

protected:
    void checkStaticVariable(const Property& property);

    COMPASS&          compass_;
    DBContentManager& dbcont_manager_;

    std::string  name_;
    unsigned int id_ {0};
    std::string  info_;
    std::string  db_table_name_;
    std::string  ds_type_;

    bool contains_target_reports_ {false};
    bool contains_status_content_ {false};
    bool is_reftraj_content_ {false};

    bool is_loadable_{false};  // loadable on its own
    size_t count_{0};

    bool insert_active_ = false;

    std::shared_ptr<DBContentReadDBJob> read_job_{nullptr};
    std::shared_ptr<UpdateBufferDBJob> update_job_{nullptr};
    std::shared_ptr<DBContentDeleteDBJob> delete_job_{nullptr};

    /// Container with all variables (variable identifier -> variable pointer)
    std::map<std::string, std::unique_ptr<dbContent::Variable>> variables_;

    std::unique_ptr<DBContentWidget> widget_;
};
