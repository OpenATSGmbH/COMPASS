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

#include "task/result/report/report.h"
#include "task/result/report/reportdefs.h"
#include "task/taskdefs.h"

#include "json_fwd.hpp"
#include "property.h"
#include "propertylist.h"

#include "result.h"

#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>

class ResultManager;

namespace ResultReport
{
    class SectionContent;
    class SectionContentFigure;
    class SectionContentTable;
    
    struct SectionContentViewable;
}

class QMenu;

/**
 */
struct TaskResultContentID
{
    TaskResultContentID() = default;
    TaskResultContentID(const std::string& section_id,
                        const std::string& name,
                        ResultReport::SectionContentType type)
    :   content_section_id(section_id),
        content_name      (name),
        content_type      (type) {}

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    static const std::string FieldSectionID;
    static const std::string FieldName;
    static const std::string FieldType;

    std::string                      content_section_id;
    std::string                      content_name;
    ResultReport::SectionContentType content_type;
};

/**
 */
struct TaskResultMetaData
{
    TaskResultMetaData() = default;
    virtual ~TaskResultMetaData() = default;

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    boost::posix_time::ptime ts_created;
    boost::posix_time::ptime ts_refreshed;
    std::string              user;
    std::string              comments;
};

/**
*/
struct TaskResultHeader
{
    typedef task::UpdateState   UpdateState;
    typedef TaskResultContentID ContentID;

    TaskResultHeader() = default;
    virtual ~TaskResultHeader() = default;

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    TaskResultMetaData       metadata;
    UpdateState              update_state = UpdateState::UpToDate;
    std::vector<ContentID>   update_contents;
};

/**
 */
class TaskResult
{
    friend class TaskManager; // to change id if required

public:
    typedef TaskResultContentID                           ContentID;
    typedef std::shared_ptr<ResultReport::SectionContent> ContentPtr;
    typedef task::UpdateState                             UpdateState;

    TaskResult(unsigned int id, 
               TaskManager& task_man);
    virtual ~TaskResult();

    unsigned int id() const;

    std::string name() const;
    void name(const std::string& name);

    const TaskResultMetaData& metadata() const;
    TaskResultHeader header() const;

    const TaskManager& taskManager() const { return task_manager_; }
    TaskManager& taskManager() { return task_manager_; }

    const std::shared_ptr<ResultReport::Report>& report() const;
    std::shared_ptr<ResultReport::Report>& report();

    void configure(const TaskResultHeader& header);

    void setJSONConfiguration(const nlohmann::json& config);
    bool hasJSONConfiguration() const;
    const nlohmann::json& jsonConfiguration() const;

    bool isLocked() const;
    bool updateNeeded() const;
    UpdateState updateState() const;
    void informUpdate(UpdateState state, 
                      const ContentID& cid = ContentID(),
                      bool inform_manager = true);
    Result canUpdate() const;
    Result update(bool restore_section = false,
                  bool inform_manager = true);

    void lock();

    Result prepareResult();
    Result finalizeResult();

    bool loadOnDemandContent(ResultReport::SectionContent* content) const;
    bool loadOnDemandViewable(const ResultReport::SectionContent& content,
                              ResultReport::SectionContentViewable& viewable, 
                              const QVariant& index,
                              unsigned int row) const;
    
    void postprocessTable(ResultReport::SectionContentTable* table);

    bool customContextMenu(QMenu& menu, 
                           ResultReport::SectionContentTable* table, 
                           unsigned int row);
    bool customMenu(QMenu& menu,
                    ResultReport::SectionContent* content);
    
    bool hasCustomTooltip(const ResultReport::SectionContentTable* table, 
                          unsigned int row,
                          unsigned int col) const;
    std::string customTooltip(const ResultReport::SectionContentTable* table, 
                              unsigned int row,
                              unsigned int col) const;

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    std::vector<std::pair<QImage, std::string>> renderFigure(const ResultReport::SectionContentFigure& figure) const;

    virtual task::TaskResultType type() const { return task::TaskResultType::Generic; }

    virtual std::string startSection() const { return ""; }

    static const std::string  DBTableName;
    static const Property     DBColumnID;
    static const Property     DBColumnName;
    static const Property     DBColumnJSONHeader;
    static const Property     DBColumnJSONContent;
    static const Property     DBColumnResultType;
    static const PropertyList DBPropertyList;

    static const std::string FieldID;
    static const std::string FieldName;
    static const std::string FieldType;
    static const std::string FieldMetaDataCreated;
    static const std::string FieldMetaDataRefreshed;
    static const std::string FieldMetaDataUser;
    static const std::string FieldMetaDataComments;
    static const std::string FieldMetaData;
    static const std::string FieldHeaderUpdateState;
    static const std::string FieldHeaderUpdateContents;
    static const std::string FieldReport;
    static const std::string FieldConfig;
    
protected:
    void id(unsigned int id);
    void clearPendingUpdates();

    Result initResult();
    Result updateContents(const std::vector<ContentID>& contents);
    Result updateContent(const ContentID& c);

    void syncContent();

    //reimplement for custom result clearing
    virtual void clear_impl() {}

    //reimplement for recomputation mechanics
    virtual Result update_impl(UpdateState state) { return false; }
    virtual Result canUpdate_impl(UpdateState state) const { return false; }
    virtual Result updateContents_impl(const std::vector<ContentID>& contents);

    //reimplement for custom initialization/preparation/finalization of a result
    virtual Result initResult_impl() { return Result::succeeded(); }
    virtual Result prepareResult_impl() { return Result::succeeded(); }
    virtual Result finalizeResult_impl() { return Result::succeeded(); }

    //reimplement for on-demand generation of contents
    virtual bool loadOnDemandFigure_impl(ResultReport::SectionContentFigure* figure) const { return false; }
    virtual bool loadOnDemandTable_impl(ResultReport::SectionContentTable* table) const { return false; }
    virtual bool loadOnDemandViewable_impl(const ResultReport::SectionContent& content,
                                           ResultReport::SectionContentViewable& viewable, 
                                           const QVariant& index,
                                           unsigned int row) const { return false; }

    //reimplement for postprocessing of contents
    virtual void postprocessTable_impl(ResultReport::SectionContentTable* table) {}
    
    //reimplement for serialization of derived content
    virtual void toJSON_impl(nlohmann::json& root_node) const {};
    virtual bool fromJSON_impl(const nlohmann::json& j) { return true; };

    //reimplement for custom context menus
    virtual bool customContextMenu_impl(QMenu& menu, 
                                        ResultReport::SectionContentTable* table, 
                                        unsigned int row) { return false; }
    virtual bool customMenu_impl(QMenu& menu, 
                                 ResultReport::SectionContent* content) { return false; }
    
    //reimplement for custom tooltips
    virtual bool hasCustomTooltip_impl(const ResultReport::SectionContentTable* table, 
                                       unsigned int row,
                                       unsigned int col) const { return false; }
    virtual std::string customTooltip_impl(const ResultReport::SectionContentTable* table,
                                           unsigned int row,
                                           unsigned int col) const { return ""; }
    TaskManager& task_manager_;

    unsigned int                          id_{0};
    std::string                           name_;
    TaskResultMetaData                    metadata_;
    std::shared_ptr<ResultReport::Report> report_;
    nlohmann::json                        config_;

    //serialized in the task result's header json
    UpdateState            update_state_ = UpdateState::UpToDate;
    std::vector<ContentID> update_contents_;

    bool init_ = false;
};
