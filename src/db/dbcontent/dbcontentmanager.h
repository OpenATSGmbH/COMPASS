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
#include "buffer.h"
#include "targetmodel.h"
#include "loadrequest.h"
#include "appmode.h"

#include <boost/date_time/posix_time/posix_time_config.hpp>
#include <boost/optional.hpp>

#include <QObject>

#include <vector>
#include <memory>

class COMPASS;
class DBContent;
class DBContentManagerWidget;
class DBSchemaManager;
class DBContentDeleteDBJob;
class DBContentInsertDBJob;
class DBContentDataEngine;
class DBContentDataSet;
class LoadOperation;
class LiveDataFeed;
class ViewableDataConfig;

namespace dbContent
{
    class DBContentEditDialog;
    class Variable;
    class MetaVariable;
    class VariableSet;
    class Target;
    class TargetListWidget;
    class VariableSet;
    class ReconstructorTarget;
}

/**
 */
class DBContentManager : public QObject, public Configurable
{
    Q_OBJECT

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();

    void dbContentEditDialogOKSlot();

    void appModeSwitchSlot(AppMode app_mode_previous, AppMode app_mode_current);

signals:
    void dbContentStatusChanged();
    void dbObjectsChangedSignal();
    void associationStatusChangedSignal();

    // loading bookends moved to ViewManager (loadingStarted/DoneSignal) - the manager's
    // loads may be issuer-private batch loads that must not drive view/UI chrome
    void insertDoneSignal(); // emitted when an insert has finished
    void dataDeletedSignal(); // emitted after data has been deleted and counts adjusted

public:
    DBContentManager(nlohmann::json& config, COMPASS& compass);
    virtual ~DBContentManager();

    COMPASS& compass() { return compass_; }
    const COMPASS& compass() const { return compass_; }

    virtual void generateSubConfigurable(nlohmann::json& child_json) override;

    bool existsDBContent(const std::string& dbcontent_name) const;
    DBContent& dbContent(const std::string& dbcontent_name);
    const DBContent& dbContent(const std::string& dbcontent_name) const;
    void deleteDBContent(const std::string& dbcontent_name);
    bool hasData();

    using DBContentIterator = typename std::map<std::string, DBContent*>::iterator;
    DBContentIterator begin() { return dbcontent_.begin(); }
    DBContentIterator end() { return dbcontent_.end(); }
    size_t size() { return dbcontent_.size(); }

    unsigned int getMaxDBContentID() const;
    bool existsDBContentWithId (unsigned int id) const;
    const std::string& dbContentWithId (unsigned int id) const;
    unsigned int dbContentId(const std::string& dbcont_name) const;

    bool existsMetaVariable(const std::string& var_name);
    dbContent::MetaVariable& metaVariable(const std::string& var_name);
    void renameMetaVariable(const std::string& old_var_name, const std::string& new_var_name);
    void deleteMetaVariable(const std::string& var_name);
    const std::map<std::string, std::unique_ptr<dbContent::MetaVariable>>& metaVariables() const { return meta_variables_; }

    bool usedInMetaVariable(const dbContent::Variable& variable);
    dbContent::DBContentEditDialog* dbContentEditDialog();

    void insertData(std::map<std::string, std::shared_ptr<Buffer>> data);
    bool insertInProgress() const;

    void deleteData(const nlohmann::json& delete_info);
    void deleteDBContentData(boost::posix_time::ptime before_timestamp);

    DBContentManagerWidget* widget();

    bool hasAssociations() const;
    void setAssociationsIdentifier(const std::string& assoc_id);
    std::string associationsID() const;
    void clearAssociationsIdentifier();

    bool canGetVariable (const std::string& dbcont_name, const Property& property);
    dbContent::Variable& getVariable (const std::string& dbcont_name, const Property& property);

    bool metaCanGetVariable (const std::string& dbcont_name, const Property& meta_property);
    dbContent::Variable& metaGetVariable (const std::string& dbcont_name, const Property& meta_property);

    bool hasTargetsInfo() const;
    void deleteAllTargets();
    bool existsTarget(unsigned int utn);
    void createNewTargets(const std::map<unsigned int, dbContent::ReconstructorTarget>& targets);
    dbContent::Target& target(unsigned int utn);
    //void removeDBContentFromTargets(const std::string& dbcont_name);
    void loadTargets();
    void saveTargets();
    unsigned int numTargets() const;

    nlohmann::json targetsInfoAsJSON() const;
    nlohmann::json targetInfoAsJSON(unsigned int utn) const;
    nlohmann::json targetStatsAsJSON() const;
    nlohmann::json utnsAsJSON() const;

    std::set<unsigned int> getIgnoredUTNs() const;

    void resetToStartupConfiguration(); // only resets label generator

    const dbContent::TargetModel* targetModel() const;
    dbContent::TargetListWidget* targetListWidget();
    void resizeTargetListWidget();

    std::string utnComment (unsigned int utn);
    void utnComment (unsigned int utn, std::string value);

    TargetBase::Category emitterCategory(unsigned int utn) const;
    std::string emitterCategoryStr(unsigned int utn) const;

    void autoFilterUTNS();
    void showUTN (unsigned int utn);
    void showUTNs (std::set<unsigned int> utns);

    void showSurroundingData (unsigned int utn);
    void showSurroundingData (std::set<unsigned int> utns);

    dbContent::VariableSet getReadSet(const std::string& dbcontent_name);

    // core meta-vars (rec_num/ds_id/line_id/timestamp), plus optional utn when available
    void addStandardVariables(std::string dbcont_name, dbContent::VariableSet& read_set,
                              bool add_utn_if_available = true);

    // façade over ViewManager's selection carry-over (kept here so callers in lower
    // layers, e.g. FilterManager, don't need a view dependency)
    void storeSelectedRecNums(const std::vector<unsigned long>& selected);

    bool hasMaxLatency() const;
    boost::posix_time::time_duration maxLatency() const;

    DBContentDataEngine& dataEngine() { return *data_engine_; }
    const DBContentDataEngine& dataEngine() const { return *data_engine_; }

    bool showDataCounts() const { return show_data_counts_; }
    void showDataCounts(bool show) { show_data_counts_ = show; }

protected:
    void setViewableDataConfig (const nlohmann::json::object_t& data);

    COMPASS& compass_;

    std::unique_ptr<dbContent::TargetModel> target_model_;
    dbContent::TargetListWidget* target_list_widget_{nullptr}; // deleted by qt

    bool has_associations_{false};
    std::string associations_id_;

    bool show_data_counts_{false};

    /// Container with all DBContent (DBContent name -> dbcont pointer)
    std::map<std::string, DBContent*> dbcontent_;
    std::map<unsigned int, DBContent*> dbcontent_ids_;
    std::map<std::string, std::unique_ptr<dbContent::MetaVariable>> meta_variables_;

    std::unique_ptr<DBContentDataEngine> data_engine_;
    std::unique_ptr<DBContentManagerWidget> widget_;
    std::unique_ptr<dbContent::DBContentEditDialog> db_content_edit_dialog_;
    std::unique_ptr<ViewableDataConfig> viewable_data_cfg_;
};
