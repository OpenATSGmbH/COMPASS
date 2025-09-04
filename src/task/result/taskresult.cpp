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

#include "taskresult.h"
#include "taskmanager.h"

#include "task/result/report/section.h"
#include "task/result/report/sectioncontent.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttable.h"

#include "compass.h"
#include "dbcontentmanager.h"
#include "viewmanager.h"
#include "view.h"

#include "timeconv.h"
#include "logger.h"
#include "files.h"
#include "asynctask.h"

#include <boost/filesystem.hpp>

#include <QMenu>
#include <QApplication>

const std::string  TaskResult::DBTableName          = "task_results";
const Property     TaskResult::DBColumnID           = Property("result_id"    , PropertyDataType::UINT  );
const Property     TaskResult::DBColumnName         = Property("name"         , PropertyDataType::STRING);
const Property     TaskResult::DBColumnJSONHeader   = Property("json_header"  , PropertyDataType::JSON  );
const Property     TaskResult::DBColumnJSONContent  = Property("json_content" , PropertyDataType::JSON  );
const Property     TaskResult::DBColumnResultType   = Property("type"         , PropertyDataType::INT   );
const PropertyList TaskResult::DBPropertyList       = PropertyList({ TaskResult::DBColumnID,
                                                                     TaskResult::DBColumnName,
                                                                     TaskResult::DBColumnJSONHeader,
                                                                     TaskResult::DBColumnJSONContent,
                                                                     TaskResult::DBColumnResultType });
const std::string TaskResult::FieldID                   = "id";
const std::string TaskResult::FieldName                 = "name";
const std::string TaskResult::FieldType                 = "type";
const std::string TaskResult::FieldMetaDataCreated      = "ts_created";
const std::string TaskResult::FieldMetaDataRefreshed    = "ts_refreshed";
const std::string TaskResult::FieldMetaDataUser         = "user";
const std::string TaskResult::FieldMetaDataComments     = "comments";
const std::string TaskResult::FieldMetaData             = "metadata";
const std::string TaskResult::FieldHeaderUpdateState    = "update_state";
const std::string TaskResult::FieldHeaderUpdateContents = "update_contents";
const std::string TaskResult::FieldReport               = "report";
const std::string TaskResult::FieldConfig               = "config";

/************************************************************************************************
 * TaskResultMetaData
 ************************************************************************************************/

const std::string TaskResultContentID::FieldSectionID = "content_section_id";
const std::string TaskResultContentID::FieldName      = "content_name";
const std::string TaskResultContentID::FieldType      = "content_type";

/**
 */
nlohmann::json TaskResultContentID::toJSON() const
{
    nlohmann::json j;

    j[ FieldSectionID ] = content_section_id;
    j[ FieldName      ] = content_name;
    j[ FieldType      ] = ResultReport::SectionContent::contentTypeAsString(content_type);

    return j;
}

/**
 */
bool TaskResultContentID::fromJSON(const nlohmann::json& j)
{
    //legacy support for old content IDs
    if (j.is_array())
    {
        if (j.size() != 2)
            return false;

        content_section_id = j[0].get<std::string>();
        content_name       = j[1].get<std::string>();
        content_type       = ResultReport::SectionContentType::Table; // only type possible in old content IDs

        return true;
    }

    if (!j.is_object() ||
        !j.contains(FieldSectionID) ||
        !j.contains(FieldName)      ||
        !j.contains(FieldType))
        return false;

    content_section_id = j[ FieldSectionID ];
    content_name       = j[ FieldName      ];

    std::string t_str = j[ FieldType ];
    auto t = ResultReport::SectionContent::contentTypeFromString(t_str);
    if (!t.has_value())
        return false;

    content_type = t.value();

    return true;
}

/************************************************************************************************
 * TaskResultMetaData
 ************************************************************************************************/

/**
 */
nlohmann::json TaskResultMetaData::toJSON() const
{
    nlohmann::json j;

    j[ TaskResult::FieldMetaDataCreated   ] = Utils::Time::toString(ts_created);
    j[ TaskResult::FieldMetaDataRefreshed ] = Utils::Time::toString(ts_refreshed);
    j[ TaskResult::FieldMetaDataUser      ] = user;
    j[ TaskResult::FieldMetaDataComments  ] = comments;

    return j;
}

/**
 */
bool TaskResultMetaData::fromJSON(const nlohmann::json& j)
{
    if (!j.is_object() ||
        !j.contains(TaskResult::FieldMetaDataCreated)   ||
        !j.contains(TaskResult::FieldMetaDataRefreshed) ||
        !j.contains(TaskResult::FieldMetaDataUser)      ||
        !j.contains(TaskResult::FieldMetaDataComments))
        return false;

    std::string ts_created_str = j[ TaskResult::FieldMetaDataCreated ];
    ts_created = Utils::Time::fromString(ts_created_str);

    std::string ts_refreshed_str = j[ TaskResult::FieldMetaDataRefreshed ];
    ts_refreshed = Utils::Time::fromString(ts_refreshed_str);

    user     = j[ TaskResult::FieldMetaDataUser     ];
    comments = j[ TaskResult::FieldMetaDataComments ];

    return true;
}

/************************************************************************************************
 * TaskResultHeader
 ************************************************************************************************/

/**
 */
nlohmann::json TaskResultHeader::toJSON() const
{
    nlohmann::json j;

    j[ TaskResult::FieldMetaData             ] = metadata.toJSON();
    j[ TaskResult::FieldHeaderUpdateState    ] = update_state;

    nlohmann::json j_update_contents = nlohmann::json::array();
    for (const auto& c : update_contents)
    {
        nlohmann::json j_content = c.toJSON();
        j_update_contents.push_back(j_content);
    }

    j[ TaskResult::FieldHeaderUpdateContents ] = j_update_contents;

    return j;
}

/**
 */
bool TaskResultHeader::fromJSON(const nlohmann::json& j)
{
    logdbg << "header:\n" << j.dump(4);

    if (!j.is_object() ||
        !j.contains(TaskResult::FieldMetaData)             ||
        !j.contains(TaskResult::FieldHeaderUpdateState)    ||
        !j.contains(TaskResult::FieldHeaderUpdateContents))
        return false;

    if (!metadata.fromJSON(j[ TaskResult::FieldMetaData ]))
        return false;

    update_state    = j[ TaskResult::FieldHeaderUpdateState ];

    if (!j[ TaskResult::FieldHeaderUpdateContents ].is_array())
        return false;

    const auto& j_update_contents = j[ TaskResult::FieldHeaderUpdateContents ]; 

    for(const auto& j_content : j_update_contents)
    {
        TaskResultContentID c;
        if (!c.fromJSON(j_content))
            return false;
        
        update_contents.push_back(c);
    }

    return true;
}

/************************************************************************************************
 * TaskResult
 ************************************************************************************************/

/**
 */
TaskResult::TaskResult(unsigned int id, TaskManager& task_man)
:   task_manager_(task_man)
,   id_          (id)
{
    report_ = std::make_shared<ResultReport::Report> (this);
}

/**
 */
TaskResult::~TaskResult() {}

/**
 */
unsigned int TaskResult::id() const
{
    return id_;
}

/**
 */
void TaskResult::id(unsigned int id)
{
    id_ = id;
}

/**
 */
std::string TaskResult::name() const
{
    return name_;
}

/**
 */
void TaskResult::name(const std::string& name)
{
    name_ = name;
}

/**
 */
const TaskResultMetaData& TaskResult::metadata() const
{
    return metadata_;
}

/**
 */
TaskResultHeader TaskResult::header() const
{
    TaskResultHeader header;
    header.metadata        = metadata_;
    header.update_state    = update_state_;
    header.update_contents = update_contents_;

    return header;
}

/**
 */
const std::shared_ptr<ResultReport::Report>& TaskResult::report() const
{
    traced_assert(report_);
    return report_;
}

/**
 */
std::shared_ptr<ResultReport::Report>& TaskResult::report()
{
    traced_assert(report_);
    return report_;
}

/**
 */
void TaskResult::configure(const TaskResultHeader& header)
{
    //apply update state stored in header
    if (header.update_state == UpdateState::ContentUpdateNeeded)
    {
        for (const auto& c : header.update_contents)
            informUpdate(header.update_state, c, false);
    }
    else
    {
        informUpdate(header.update_state, ContentID(), false);
    }
}

/**
 */
void TaskResult::setJSONConfiguration(const nlohmann::json& config)
{
    config_ = config;
}

/**
 */
bool TaskResult::hasJSONConfiguration() const
{
    return config_.is_null();
}

/**
 */
const nlohmann::json& TaskResult::jsonConfiguration() const
{
    return config_;
}

/**
 */
void TaskResult::informUpdate(UpdateState state, 
                              const ContentID& cid,
                              bool inform_manager)
{
    //"biggest" update wins
    bool state_changed = false;
    if (state > update_state_)
    {
        update_state_ = state;
        state_changed = true;
    }

    if (update_state_ == UpdateState::ContentUpdateNeeded)
    {
        //add content info
        traced_assert(!cid.content_section_id.empty());
        traced_assert(!cid.content_name.empty());

        update_contents_.push_back(cid);
    }
    else
    {
        //content info not needed any more
        update_contents_.clear();
    }

    //inform task manager?
    if (state_changed && inform_manager)
    {
        if (update_state_ == UpdateState::Locked)
        {
            //result is now locked => update contents of report accordingly
            report_->updateContents();
        }

        task_manager_.resultHeaderChanged(*this);
    }
}

/**
 */
bool TaskResult::isLocked() const
{
    return update_state_ == UpdateState::Locked;
}

/**
 */
bool TaskResult::updateNeeded() const
{
    return update_state_ != UpdateState::UpToDate;
}

/**
 */
TaskResult::UpdateState TaskResult::updateState() const
{
    return update_state_;
}

/**
 */
Result TaskResult::canUpdate() const
{
    //we generally assume a config is needed to recompute the result
    if (!config_.is_object())
        return Result::failed("No configuration available");

    if (update_state_ == UpdateState::UpToDate)
        return Result::succeeded();

    //derived custom check
    return canUpdate_impl(update_state_);
}

/**
 */
Result TaskResult::update(bool restore_section,
                          bool inform_manager)
{
    if (!updateNeeded())
        return Result::succeeded();

    //check if we can recompute
    auto r = canUpdate();
    if (!r.ok())
        return r;

    //store current section + settings
    if (restore_section)
        task_manager_.storeBackupSection();

    r = Result::succeeded();
    bool restore_needed = false;

    if (update_state_ == UpdateState::ContentUpdateNeeded)
    {
        loginf << "running content update";

        for (const auto& c : update_contents_)
            loginf << "   " << c.content_section_id << " " << c.content_name;

        //update specific contents
        r = updateContents(update_contents_);
    }
    else if (update_state_ != UpdateState::UpToDate)
    {
        loginf << "running " 
               << (update_state_ == UpdateState::PartialUpdateNeeded ? "partial" : "full") << " update";

        //partial and full update
        r = update_impl(update_state_);

        // these bigger updates might trigger report regeneration
        restore_needed = true;
    }

    //update failed?
    if (!r.ok())
    {
        logerr << "update failed: " << r.error();
        return r;
    }

    //recompute succeeded => reset updates
    clearPendingUpdates();

    //restore current section in widgets if desired
    if (restore_needed && restore_section)
        task_manager_.restoreBackupSection();

    //inform task manager?
    if (inform_manager)
        task_manager_.resultHeaderChanged(*this);

    return Result::succeeded();
}

/**
 */
Result TaskResult::updateContents(const std::vector<ContentID>& contents)
{
    if (contents.empty())
        return Result::succeeded();

    return updateContents_impl(contents);
}

/**
 * Updates a single content in the result.
 */
Result TaskResult::updateContent(const ContentID& c)
{
    if (!report_->hasSection(c.content_section_id))
        return Result::failed("Unknown section '" + c.content_section_id + "'");

    auto& section = report_->getSection(c.content_section_id);
    auto  flags   = section.contentInfo(c.content_name, c.content_type);

    //content must be available in section
    if ((flags & ResultReport::Section::ContentInfoFlag::ContentAvailable) == 0)
        return Result::failed("Unknown table content '" + c.content_name + "' in section '" + c.content_section_id + "'");

    //not yet loaded from db? => no problem, nothing to update
    if ((flags & ResultReport::Section::ContentInfoFlag::ContentLoaded) == 0)
        return Result::succeeded();

    //only on-demand contents allowed
    if ((flags & ResultReport::Section::ContentInfoFlag::ContentOnDemand) == 0)
        return Result::failed("Static table content '" + c.content_name + "' in section '" + c.content_section_id + "'");

    //on-demand content not yet complete? => no problem, nothing to update
    if ((flags & ResultReport::Section::ContentInfoFlag::ContentOnDemandComplete) == 0)
        return Result::succeeded();

    //update content
    auto content_id = section.contentID(c.content_name, c.content_type);
    auto content    = section.retrieveContent(content_id);
    traced_assert(content);
    traced_assert(content->isOnDemand());
    traced_assert(content->isComplete());

    if (!content->forceReload())
        return Result::failed("Could not update content '" + c.content_name + "' in section '" + c.content_section_id + "'");

    return Result::succeeded();
}

/**
 */
Result TaskResult::updateContents_impl(const std::vector<ContentID>& contents)
{
    Result r = Result::succeeded();
    
    //default implementation: update all contents
    for (const auto& c : contents)
    {
        auto rc = updateContent(c);
        if (!rc.ok())
        {
            r = rc;
            continue;
        }
    }

    return r;
}

/**
 * Synchronizes the database result with current content of the result.
 */
void TaskResult::syncContent()
{
    auto cb = [ this ] (const AsyncTaskState& s, AsyncTaskProgressWrapper& p)
    {
        this->taskManager().resultContentChanged(*this);
        return true;
    };

    AsyncFuncTask task(cb, "Updating Report", "Updating report", false);
    task.runAsyncDialog();
}

/**
 */
void TaskResult::clearPendingUpdates()
{
    update_state_ = UpdateState::UpToDate;
    update_contents_.clear();
}

/**
 */
void TaskResult::lock()
{
    informUpdate(UpdateState::Locked);
}

/**
 */
Result TaskResult::initResult()
{
    if (init_)
        return Result::succeeded();

    auto r = initResult_impl();
    if (!r.ok())
        return r;

    init_ = true;

    return Result::succeeded();
}

/**
 * Prepares the result for new content.
 */
Result TaskResult::prepareResult()
{
    //clear report
    report()->clear();

    //custom clear
    clear_impl();

    //reset any pending updates
    clearPendingUpdates();

    //custom init
    return prepareResult_impl();
}

/**
 * Finalizes result after adding content.
 */
Result TaskResult::finalizeResult()
{
    //custom finalization
    auto r = finalizeResult_impl();
    if (!r.ok())
        return r;

    return Result::succeeded();
}

/**
 */
bool TaskResult::loadOnDemandContent(ResultReport::SectionContent* content) const
{
    if (!content)
        return false;

    if (content->isLocked())
        return false;

    if (content->contentType() == ResultReport::SectionContent::ContentType::Figure)
    {
        auto c = dynamic_cast<ResultReport::SectionContentFigure*>(content);
        traced_assert(c);

        return loadOnDemandFigure_impl(c);
    }
    else if (content->contentType() == ResultReport::SectionContent::ContentType::Table)
    {
        auto c = dynamic_cast<ResultReport::SectionContentTable*>(content);
        traced_assert(c);

        return loadOnDemandTable_impl(c);
    }

    return false;
}

/**
 */
bool TaskResult::loadOnDemandViewable(const ResultReport::SectionContent& content,
                                      ResultReport::SectionContentViewable& viewable, 
                                      const QVariant& index,
                                      unsigned int row) const
{
    //on demand viewables cannot be loaded if the result is locked
    if (isLocked())
        return false;

    if (!loadOnDemandViewable_impl(content, viewable, index, row))
        return false;
    
    return viewable.hasCallback();
}

/**
 */
bool TaskResult::customContextMenu(QMenu& menu, 
                                   ResultReport::SectionContentTable* table, 
                                   unsigned int row)
{
    traced_assert(table);

    if (table->isLocked())
        return false;

    bool ok = customContextMenu_impl(menu, table, row);

    return ok && menu.actions().size() > 0;
}

/**
 */
bool TaskResult::customMenu(QMenu& menu,
                            ResultReport::SectionContent* content)
{
    traced_assert(content);

    if (content->isLocked())
        return false;

    return customMenu_impl(menu, content);
}

/**
 */
void TaskResult::postprocessTable(ResultReport::SectionContentTable* table)
{
    //no need to postprocess locked tables
    if (table->isLocked())
        return;

    traced_assert(table);
    postprocessTable_impl(table);
}

/**
 */
bool TaskResult::hasCustomTooltip(const ResultReport::SectionContentTable* table, 
                                  unsigned int row,
                                  unsigned int col) const
{
    traced_assert(table);

    if (table->isLocked())
        return false;

    return hasCustomTooltip_impl(table, row, col);
}

/**
 */
std::string TaskResult::customTooltip(const ResultReport::SectionContentTable* table, 
                                      unsigned int row,
                                      unsigned int col) const
{
    traced_assert(table);
    traced_assert(!table->isLocked());
    return customTooltip_impl(table, row, col);
}

/**
 */
nlohmann::json TaskResult::toJSON() const
{
    nlohmann::json j = nlohmann::json::object();

    j[ FieldType           ] = type();
    j[ FieldID             ] = id_;
    j[ FieldName           ] = name_;
    j[ FieldMetaData       ] = metadata_.toJSON();
    j[ FieldReport         ] = report_->toJSON();
    j[ FieldConfig         ] = config_;

    //derived content
    toJSON_impl(j);

    return j;
}

/**
 */
bool TaskResult::fromJSON(const nlohmann::json& j)
{
    //loginf << j.dump(4);

    if (!j.is_object()                    || 
        !j.contains(FieldType)            ||
        !j.contains(FieldID)              ||
        !j.contains(FieldName)            ||
        !j.contains(FieldMetaData)        ||
        !j.contains(FieldReport)          ||
        !j.contains(FieldConfig))
        return false;

    task::TaskResultType stored_type = j[ FieldType ];
    if (stored_type != type())
    {
        logerr << "stored type " << stored_type
               << " does not match result type " << type();
        return false;
    }

    id_       = j[ FieldID   ];
    name_     = j[ FieldName ];

    if (!metadata_.fromJSON(j[ FieldMetaData ]))
        return false;

    if (!report_->fromJSON(j[ FieldReport ]))
        return false;

    config_ = j[ FieldConfig ];

    //derived content
    if (!fromJSON_impl(j))
        return false;

    //init after reading in data
    auto init_res = initResult();
    if (!init_res.ok())
    {
        logerr << "initializing result failed: " << init_res.error();
        return false;
    }

    return true;
}

/**
 */
std::vector<std::pair<QImage, std::string>> TaskResult::renderFigure(const ResultReport::SectionContentFigure& figure) const
{
    //do net render locked figures
    if (figure.isLocked())
        return std::vector<std::pair<QImage, std::string>>();

    std::vector<std::pair<QImage, std::string>> renderings;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();
    ViewManager&      view_man   = COMPASS::instance().viewManager();

    QCoreApplication::processEvents();

    figure.view();

    while (dbcont_man.loadInProgress())
        QCoreApplication::processEvents();

    QCoreApplication::processEvents();

    //wait a little for e.g. geoimages to warp and render correctly in geographic view
    figure.executeRenderDelay();

    for (auto& view_it : view_man.getViews())
    {
        //skip table views
        if (view_it.second->classId() == "TableView")
            continue;
        
        //skip views which show no content
        if (!view_it.second->hasScreenshotContent())
            continue;

        //render view and collect
        auto img = view_it.second->renderData();
        renderings.emplace_back(img, view_it.second->instanceId());
    }

    return renderings;
}
