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

#include "taskresultdb.h"
#include "taskresult.h"
#include "itaskprovider.h"

#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontent.h"
#include "report/sectioncontentfigure.h"
#include "report/sectioncontenttable.h"
#include "report/sectioncontenttext.h"

#include "dbinterface.h"
#include "dbdefs.h"

#include "buffer.h"
#include "logger.h"
#include "traced_assert.h"

#include <map>

namespace
{
/**
 * Property list used to insert report contents. The JSON text is produced once by the
 * content itself, so the column is written as a plain string here and no second
 * serialization takes place. The database column is VARCHAR either way, and the read
 * path keeps using SectionContent::DBPropertyList, which parses the text back.
 */
const PropertyList& reportContentsInsertProperties()
{
    static const PropertyList props({
        ResultReport::SectionContent::DBColumnContentID,
        ResultReport::SectionContent::DBColumnResultID,
        ResultReport::SectionContent::DBColumnType,
        Property(ResultReport::SectionContent::DBColumnJSONContent.name(), PropertyDataType::STRING) });

    return props;
}

/**
 * Property list used to insert per-target report sections, the JSON text is written as a plain string
 * (see reportContentsInsertProperties()).
 */
const PropertyList& reportSectionsInsertProperties()
{
    static const PropertyList props({
        TaskResult::DBColumnSectionResultID,
        TaskResult::DBColumnSectionIndex,
        Property(TaskResult::DBColumnSectionJSON.name(), PropertyDataType::STRING) });

    return props;
}

/**
 * Replaces each per-target section in the sub section tree of the given section JSON by a
 * reference object, the cut out sections are added as JSON text. The index in the given vector
 * is the reference.
 */
void extractPerTargetSections(nlohmann::json& j_section, std::vector<std::string>& sections)
{
    auto it = j_section.find(ResultReport::Section::FieldSubSections);
    if (it == j_section.end() || !it->is_array())
        return;

    for (auto& j_sub : *it)
    {
        auto it_pt = j_sub.find(ResultReport::Section::FieldPerTarget);

        if (it_pt != j_sub.end() && it_pt->is_boolean() && it_pt->get<bool>())
        {
            sections.push_back(j_sub.dump());

            j_sub = nlohmann::json::object();
            j_sub[ TaskResult::FieldSectionRef ] = sections.size() - 1;
        }
        else
        {
            extractPerTargetSections(j_sub, sections);
        }
    }
}

/**
 * Replaces each section reference in the sub section tree of the given section JSON by the
 * referenced section. Returns false if a referenced section is missing.
 */
bool restorePerTargetSections(nlohmann::json& j_section, std::map<unsigned int, nlohmann::json>& sections)
{
    auto it = j_section.find(ResultReport::Section::FieldSubSections);
    if (it == j_section.end() || !it->is_array())
        return true;

    for (auto& j_sub : *it)
    {
        auto it_ref = j_sub.find(TaskResult::FieldSectionRef);

        if (it_ref != j_sub.end())
        {
            auto it_sec = sections.find(it_ref->get<unsigned int>());
            if (it_sec == sections.end())
                return false;

            j_sub = std::move(it_sec->second);
            sections.erase(it_sec);
        }
        else if (!restorePerTargetSections(j_sub, sections))
        {
            return false;
        }
    }

    return true;
}
}

/**
 */
TaskResultDB::TaskResultDB(DBInterface& db_interface, ITaskProvider& task_provider)
:   db_interface_(db_interface)
,   task_provider_(task_provider)
{
}

/**
 */
bool TaskResultDB::existsTaskResultsTable() const
{
    return db_interface_.existsTable(TaskResult::DBTableName);
}

/**
 */
bool TaskResultDB::existsReportContentsTable() const
{
    return db_interface_.existsTable(ResultReport::SectionContent::DBTableName);
}

/**
 */
void TaskResultDB::createTaskResultsTable()
{
    traced_assert(db_interface_.ready());
    traced_assert(!existsTaskResultsTable());

    // json columns compressed with zstd (large, very repetitive texts)
    db_interface_.createTable(TaskResult::DBTableName, TaskResult::DBPropertyList, 0,
                              { TaskResult::DBColumnJSONContent.name() });
}

/**
 */
void TaskResultDB::createReportContentsTable()
{
    traced_assert(db_interface_.ready());
    traced_assert(!existsReportContentsTable());

    // json column compressed with zstd (large, very repetitive texts)
    db_interface_.createTable(ResultReport::SectionContent::DBTableName, ResultReport::SectionContent::DBPropertyList, 0,
                              { ResultReport::SectionContent::DBColumnJSONContent.name() });
}

/**
 */
bool TaskResultDB::existsReportSectionsTable() const
{
    return db_interface_.existsTable(TaskResult::DBSectionsTableName);
}

/**
 */
void TaskResultDB::createReportSectionsTable()
{
    traced_assert(db_interface_.ready());
    traced_assert(!existsReportSectionsTable());

    // no primary key, a result owns many rows, json column compressed with zstd (large, very repetitive texts)
    db_interface_.createTable(TaskResult::DBSectionsTableName, TaskResult::DBSectionsPropertyList, -1,
                              { TaskResult::DBColumnSectionJSON.name() });
}

/**
 * Returns the content JSON of the given result without its per-target report sections, which
 * are added to the given vector as JSON text and replaced by references in the returned JSON.
 * The per-target sections hold most of an evaluation report, stored apart they are written in
 * batches instead of as one large value.
 */
nlohmann::json TaskResultDB::resultContentWithoutSections(const TaskResult& result,
                                                          std::vector<std::string>& sections) const
{
    nlohmann::json content = result.toJSON();

    auto it_report = content.find(TaskResult::FieldReport);
    if (it_report != content.end())
    {
        auto it_root = it_report->find(ResultReport::Report::FieldRootSection);
        if (it_root != it_report->end())
            extractPerTargetSections(*it_root, sections);
    }

    return content;
}

/**
 * Deletes the per-target report sections of the given result.
 */
void TaskResultDB::deleteReportSections(unsigned int result_id)
{
    if (!existsReportSectionsTable())
        return;

    auto res = db_interface_.deleteRows(TaskResult::DBSectionsTableName,
                                        db::SQLFilter(TaskResult::DBColumnSectionResultID.name(),
                                                      std::to_string(result_id),
                                                      db::SQLFilter::ComparisonOp::Is).statement());
    if (!res.ok())
        throw std::runtime_error(res.error());
}

/**
 * Writes the given per-target report sections of a result. The sections are aggregated into
 * batches of a minimum size, only the last batch may be smaller. The vector is emptied.
 */
void TaskResultDB::writeReportSections(unsigned int result_id,
                                       std::vector<std::string>& sections)
{
    if (!existsReportSectionsTable())
        createReportSectionsTable();

    const size_t min_batch_bytes = 16 * 1024 * 1024;

    std::shared_ptr<Buffer> buffer;

    NullableVector<unsigned int>* result_id_vec = nullptr;
    NullableVector<unsigned int>* index_vec     = nullptr;
    NullableVector<std::string>*  json_vec      = nullptr;

    size_t current_bytes = 0;
    size_t current_row   = 0;

    for (size_t i = 0; i < sections.size(); ++i)
    {
        if (!buffer)
        {
            buffer.reset(new Buffer(reportSectionsInsertProperties()));

            result_id_vec = &buffer->get<unsigned int>(TaskResult::DBColumnSectionResultID.name());
            index_vec     = &buffer->get<unsigned int>(TaskResult::DBColumnSectionIndex.name());
            json_vec      = &buffer->get<std::string>(TaskResult::DBColumnSectionJSON.name());

            current_bytes = 0;
            current_row   = 0;
        }

        current_bytes += sections[ i ].size();

        result_id_vec->set(current_row, result_id);
        index_vec->set(current_row, (unsigned int)i);
        json_vec->set(current_row, std::move(sections[ i ]));

        ++current_row;

        if (current_bytes >= min_batch_bytes)
        {
            db_interface_.insertBuffer(TaskResult::DBSectionsTableName, buffer);
            buffer.reset();
        }
    }

    //insert last batch
    if (buffer && current_row > 0)
        db_interface_.insertBuffer(TaskResult::DBSectionsTableName, buffer);

    loginf << "result " << result_id << ": wrote " << sections.size() << " per-target section(s)";

    sections.clear();
}

/**
 */
Result TaskResultDB::saveResult(const TaskResult& result, bool cleanup_db_if_needed)
{
    traced_assert(db_interface_.ready());

    try
    {
        // removeTable(TaskResult::DBTableName); // UGA TABLEHACK

        //create needed tables
        if (!existsTaskResultsTable())
            createTaskResultsTable();

        if (!existsReportContentsTable())
            createReportContentsTable();
        
        // Materialize all section contents BEFORE the old rows are deleted.
        // For accumulating results (e.g. ASTERIX Import, clear_existing=false)
        // the report carries lazily-loaded contents whose content_ids point
        // at rows deleteResult is about to wipe; loading them after deletion
        // would fail the assert in Section::loadOrGetContent.
        auto report_contents = result.report()->reportContents(true);

        //remove any old result with the same id/name, the report tables of this result stay
        bool result_deleted = false;
        auto del_result = deleteResult(result, false, &result_deleted, true);
        if (!del_result.ok())
            throw std::runtime_error(del_result.error());

        //drop report tables under this result id which the run did not write, left behind by an earlier run
        if (db_interface_.removeReportTables(result.id(), result.reportTableNames()))
            result_deleted = true;

        auto result_id   = result.id();
        auto result_name = result.name();

        //remove per-target sections left behind under this result id by an aborted run
        deleteReportSections(result_id);

        //write result, without the per-target sections
        std::vector<std::string> sections;

        {
            std::shared_ptr<Buffer> buffer(new Buffer(TaskResult::DBPropertyList));

            auto& id_vec      = buffer->get<unsigned int>(TaskResult::DBColumnID.name());
            auto& name_vec    = buffer->get<std::string>(TaskResult::DBColumnName.name());
            auto& header_vec  = buffer->get<nlohmann::json>(TaskResult::DBColumnJSONHeader.name());
            auto& content_vec = buffer->get<nlohmann::json>(TaskResult::DBColumnJSONContent.name());
            auto& type_vec    = buffer->get<int>(TaskResult::DBColumnResultType.name());

            id_vec.set(0, result_id);
            name_vec.set(0, result.name());
            header_vec.set(0, result.header().toJSON());
            content_vec.set(0, resultContentWithoutSections(result, sections));
            type_vec.set(0, (int)result.type());

            db_interface_.insertBuffer(TaskResult::DBTableName, buffer);
        }

        //write per-target sections
        writeReportSections(result_id, sections);

        //write contents
        {
            // Write the accumulated contents out early and often. A JSON tree in
            // memory is several times larger than its serialized form, so a large
            // chunk keeps gigabytes alive until it is flushed.
            const size_t chunk_size_bytes = 16 * 1024 * 1024;
            const size_t chunk_max_rows   = 500;

            size_t current_bytes    = 0;
            size_t current_row      = 0;

            std::shared_ptr<Buffer> buffer;

            NullableVector<unsigned int>*   content_id_vec = nullptr;
            NullableVector<unsigned int>*   result_id_vec  = nullptr;
            NullableVector<int>*            type_vec       = nullptr;
            NullableVector<std::string>*    content_vec    = nullptr;

            for (const auto& c : report_contents)
            {
                if (!buffer || current_bytes > chunk_size_bytes || current_row >= chunk_max_rows)
                {
                    //insert old buffer if available
                    if (buffer)
                    {
                        //loginf << "writing" << buffer->size() << " row(s)";
                        db_interface_.insertBuffer(ResultReport::SectionContent::DBTableName, buffer);
                    }

                    //create new buffer
                    buffer.reset(new Buffer(reportContentsInsertProperties()));

                    content_id_vec = &buffer->get<unsigned int>(ResultReport::SectionContent::DBColumnContentID.name());
                    result_id_vec  = &buffer->get<unsigned int>(ResultReport::SectionContent::DBColumnResultID.name());
                    type_vec       = &buffer->get<int>(ResultReport::SectionContent::DBColumnType.name());
                    content_vec    = &buffer->get<std::string>(ResultReport::SectionContent::DBColumnJSONContent.name());

                    current_bytes = 0;
                    current_row   = 0;
                }

                traced_assert(buffer);

                //!this might trigger recomputations from temporarily generated data,
                //which is immediately thrown away afterwards!
                //serialize once, straight to text, each content into its own string
                std::string c_text;
                c->toJSONText(c_text);

                size_t c_bytes = c_text.size();

                content_id_vec->set(current_row, c->contentID());
                result_id_vec->set(current_row, result_id);
                type_vec->set(current_row, (int)c->contentType());
                content_vec->set(current_row, std::move(c_text));

                current_bytes += c_bytes;
                current_row   += 1;
            }

            //insert remaining buffer content
            if (current_row > 0)
            {
                //loginf << "writing" << buffer->size() << " row(s)";
                db_interface_.insertBuffer(ResultReport::SectionContent::DBTableName, buffer);
            }
        }

        //cleanup db?
        if (result_deleted && cleanup_db_if_needed)
            db_interface_.cleanupDB(false);
    }
    catch(const std::exception& ex)
    {
        logerr << "could not store result: " << ex.what();
        return Result::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not store result: Unknown error";
        return Result::failed("Unknown error");
    }

    return Result::succeeded();
}

/**
 */
Result TaskResultDB::deleteResult(const TaskResult& result,
                                  bool cleanup_db_if_needed,
                                  bool* deleted,
                                  bool keep_own_tables)
{
    if (deleted)
        *deleted = false;

    auto id = result.id();

    if (!existsTaskResultsTable() || 
        !existsReportContentsTable())
    {
        logerr << "result tables do not exist";
        return Result::failed("Result tables do not exist");
    }

    bool results_deleted = false;

    try
    {
        //get old result(s) which resemble(s) the new one (either in id or name)
        auto sel_filter = db::SQLFilter(TaskResult::DBColumnID.name(), std::to_string(id), db::SQLFilter::ComparisonOp::Is)
            .OR(TaskResult::DBColumnName.name(), "'" + result.name() + "'", db::SQLFilter::ComparisonOp::Is).statement();

        PropertyList properties = PropertyList({ TaskResult::DBColumnID });

        auto sel_result = db_interface_.select(TaskResult::DBTableName, properties, sel_filter);
        if (!sel_result.ok())
            throw std::runtime_error("Locating old result failed");

        //collect id(s) of old result(s)
        std::vector<unsigned int> ids_to_remove;

        size_t n = sel_result.result()->size();
        const auto& id_vector = sel_result.result()->get<unsigned int>(TaskResult::DBColumnID.name());
        for (size_t i = 0; i < n; ++i)
            ids_to_remove.push_back(id_vector.get(i));

        loginf << "deleting " << ids_to_remove.size() << " old result(s)";

        //delete old result(s)
        for (auto old_id : ids_to_remove)
        {
            auto result_del_content = db_interface_.deleteRows(ResultReport::SectionContent::DBTableName,
                                                               db::SQLFilter(ResultReport::SectionContent::DBColumnResultID.name(),
                                                                             std::to_string(old_id),
                                                                             db::SQLFilter::ComparisonOp::Is).statement());
            auto result_del_result  = db_interface_.deleteRows(TaskResult::DBTableName,
                                                               db::SQLFilter(TaskResult::DBColumnID.name(),
                                                                             std::to_string(old_id),
                                                                             db::SQLFilter::ComparisonOp::Is).statement());

            if (!result_del_content.ok())
                throw std::runtime_error(result_del_content.error());
            if (!result_del_result.ok())
                throw std::runtime_error(result_del_result.error());

            deleteReportSections(old_id);

            //drop the report tables of the old result, the tables of the result being saved stay
            std::set<std::string> keep;
            if (keep_own_tables && old_id == result.id())
                keep = result.reportTableNames();

            db_interface_.removeReportTables(old_id, keep);

            results_deleted = true;
        }
    }
    catch(const std::exception& ex)
    {
        logerr << "could not delete result: " << ex.what();
        return Result::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not delete result: Unknown error";
        return Result::failed("Unknown error");
    }

    if (deleted)
        *deleted = results_deleted;

    //cleanup db?
    if (results_deleted && cleanup_db_if_needed)
        db_interface_.cleanupDB(false);

    return Result::succeeded();
}

/**
 */
Result TaskResultDB::updateResultHeader(const TaskResult& result)
{
    loginf << "updating header of result '" << result.name() << "'";

    traced_assert(db_interface_.ready());

    try
    {
        // uses replace with utn as unique key
        auto res = db_interface_.updateCell(TaskResult::DBTableName,
                                            TaskResult::DBColumnJSONHeader.name(),
                                            result.header().toJSON(),
                                            TaskResult::DBColumnID.name(),
                                            result.id());
        if (!res.ok())
            return res;
    }
    catch(const std::exception& ex)
    {
        return Result::failed(std::string(ex.what()));
    }
    catch(...)
    {
        return Result::failed("Unknown error");
    }

    loginf << "done";

    return Result::succeeded();
}

/**
 */
Result TaskResultDB::updateResultContent(const TaskResult& result)
{
    loginf << "updating content of result '" << result.name() << "'";

    traced_assert(db_interface_.ready());

    try
    {
        //the per-target sections are written apart, see saveResult()
        std::vector<std::string> sections;

        {
            // uses replace with utn as unique key
            auto res = db_interface_.updateCell(TaskResult::DBTableName,
                                                TaskResult::DBColumnJSONContent.name(),
                                                resultContentWithoutSections(result, sections),
                                                TaskResult::DBColumnID.name(),
                                                result.id());
            if (!res.ok())
                return res;
        }

        deleteReportSections(result.id());
        writeReportSections(result.id(), sections);
    }
    catch(const std::exception& ex)
    {
        return Result::failed(std::string(ex.what()));
    }
    catch(...)
    {
        return Result::failed("Unknown error");
    }

    loginf << "done";

    return Result::succeeded();
}

/**
 */
ResultT<std::vector<std::shared_ptr<TaskResult>>> TaskResultDB::loadResults()
{
    traced_assert(db_interface_.ready());

    std::vector<std::shared_ptr<TaskResult>> results;

    //no results stored yet?
    if (!existsTaskResultsTable())
        return ResultT<std::vector<std::shared_ptr<TaskResult>>>::succeeded(results);

    try
    {
        //remove per-target sections without a result, left behind by aborted runs
        if (existsReportSectionsTable())
        {
            auto res = db_interface_.deleteRows(TaskResult::DBSectionsTableName,
                                                TaskResult::DBColumnSectionResultID.name() +
                                                " NOT IN (SELECT " + TaskResult::DBColumnID.name() + " FROM " + TaskResult::DBTableName + ")");
            if (!res.ok())
                logwrn << "could not remove orphan per-target sections: " << res.error();
        }

        // the content column is left out on purpose, it holds the whole report section tree.
        // TaskResult reads it through loadResultContent() when the result is first used.
        auto result = db_interface_.select(TaskResult::DBTableName, TaskResult::DBHeaderPropertyList, "");
        if (!result.ok())
            throw std::runtime_error("Could not obtain results table");

        auto b = result.result();

        //@TODO: check more tables obtained this way for having the right properties
        if (!b->hasProperty(TaskResult::DBColumnID) ||
            !b->hasProperty(TaskResult::DBColumnName) ||
            !b->hasProperty(TaskResult::DBColumnJSONHeader) ||
            !b->hasProperty(TaskResult::DBColumnResultType))
            throw std::runtime_error("Results table invalid");

        size_t nr = b->size();

        auto& id_vec      = b->get<unsigned int>(TaskResult::DBColumnID.name());
        auto& name_vec    = b->get<std::string>(TaskResult::DBColumnName.name());
        auto& header_vec  = b->get<nlohmann::json>(TaskResult::DBColumnJSONHeader.name());
        auto& type_vec    = b->get<int>(TaskResult::DBColumnResultType.name());

        results.resize(nr);

        for (size_t i = 0; i < nr; ++i)
        {
            const auto& result_name  = name_vec.get(i);
            const auto& result_type  = type_vec.get(i);
            const auto& result_id    = id_vec.get(i);
            const auto& json_header  = header_vec.get(i);

            //create result of given type
            results[ i ] = task_provider_.createResult(result_id, (task::TaskResultType)result_type);
            if (!results[ i ])
                throw std::runtime_error("Could not create result from type");

            //try to read result header
            TaskResultHeader header;
            if (!header.fromJSON(json_header))
                throw std::runtime_error("Could not read header of result '" + result_name + "'");

            //name and type come from the table, the content is not read yet
            results[ i ]->name(result_name);

            //configure result using header information (metadata, update state etc.)
            results[ i ]->configure(header);

            //mark the content as stored but not read
            results[ i ]->setContentStored();

            //final checks
            if (results[ i ]->type() != result_type ||
                results[ i ]->id()   != result_id)
                throw std::runtime_error("Result '" + result_name + "' obtains invalid content");
        }
    }
    catch(const std::exception& ex)
    {
        logerr << "could not load results: " << ex.what();
        return ResultT<std::vector<std::shared_ptr<TaskResult>>>::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not load results: Unknown error";
        return ResultT<std::vector<std::shared_ptr<TaskResult>>>::failed("Unknown error");
    }

    return ResultT<std::vector<std::shared_ptr<TaskResult>>>::succeeded(results);
}

/**
 * Reads the content JSON of a single result. Called by TaskResult when the result is first used.
 */
ResultT<nlohmann::json> TaskResultDB::loadResultContent(unsigned int result_id)
{
    traced_assert(db_interface_.ready());

    if (!existsTaskResultsTable())
        return ResultT<nlohmann::json>::failed("Results table does not exist");

    try
    {
        auto filter = db::SQLFilter(TaskResult::DBColumnID.name(),
                                    std::to_string(result_id),
                                    db::SQLFilter::ComparisonOp::Is).statement();

        auto result = db_interface_.select(TaskResult::DBTableName,
                                           PropertyList({ TaskResult::DBColumnJSONContent }),
                                           filter);

        if (!result.ok())
            throw std::runtime_error("Could not obtain result content");

        auto b = result.result();

        if (!b->hasProperty(TaskResult::DBColumnJSONContent))
            throw std::runtime_error("Results table invalid");

        size_t nr = b->size();
        if (nr == 0)
            throw std::runtime_error("Result id not found in table");
        else if (nr > 1)
            throw std::runtime_error("Multiple result ids found in table");

        auto& content_vec = b->get<nlohmann::json>(TaskResult::DBColumnJSONContent.name());

        if (content_vec.isNull(0))
            throw std::runtime_error("Result content is empty");

        nlohmann::json content = content_vec.get(0);

        //read the per-target sections and put them back into the section tree
        std::map<unsigned int, nlohmann::json> sections;

        if (existsReportSectionsTable())
        {
            auto sec_filter = db::SQLFilter(TaskResult::DBColumnSectionResultID.name(),
                                            std::to_string(result_id),
                                            db::SQLFilter::ComparisonOp::Is).statement();

            auto sec_result = db_interface_.select(TaskResult::DBSectionsTableName,
                                                   TaskResult::DBSectionsPropertyList,
                                                   sec_filter);

            if (!sec_result.ok())
                throw std::runtime_error("Could not obtain per-target sections: " + sec_result.error());

            {
                auto sb = sec_result.result();

                auto& index_vec = sb->get<unsigned int>(TaskResult::DBColumnSectionIndex.name());
                auto& json_vec  = sb->get<nlohmann::json>(TaskResult::DBColumnSectionJSON.name());

                for (size_t i = 0; i < sb->size(); ++i)
                    sections[ index_vec.get(i) ] = json_vec.get(i);
            }
        }

        auto it_report = content.find(TaskResult::FieldReport);
        if (it_report != content.end())
        {
            auto it_root = it_report->find(ResultReport::Report::FieldRootSection);
            if (it_root != it_report->end() && !restorePerTargetSections(*it_root, sections))
                throw std::runtime_error("Per-target section missing");
        }

        return ResultT<nlohmann::json>::succeeded(content);
    }
    catch(const std::exception& ex)
    {
        logerr << "could not load result content: " << ex.what();
        return ResultT<nlohmann::json>::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not load result content: Unknown error";
        return ResultT<nlohmann::json>::failed("Unknown error");
    }
}

/**
 */
ResultT<std::shared_ptr<ResultReport::SectionContent>> TaskResultDB::loadContent(ResultReport::Section* section,
                                                                                 unsigned int content_id)
{
    traced_assert(db_interface_.ready());

    std::shared_ptr<ResultReport::SectionContent> content;

    loginf << "loading content id " << content_id << "...";

    try
    {
        PropertyList properties;
        properties.addProperty(ResultReport::SectionContent::DBColumnContentID);
        properties.addProperty(ResultReport::SectionContent::DBColumnResultID);
        properties.addProperty(ResultReport::SectionContent::DBColumnType);
        properties.addProperty(ResultReport::SectionContent::DBColumnJSONContent);

        auto filter_col = ResultReport::SectionContent::DBColumnContentID.name();

        auto result = db_interface_.select(ResultReport::SectionContent::DBTableName,
                                           ResultReport::SectionContent::DBPropertyList,
                                           db::SQLFilter(filter_col, std::to_string(content_id), db::SQLFilter::ComparisonOp::Is).statement());

        if (!result.ok())
            throw std::runtime_error("Could not obtain content from table");

        auto b = result.result();

        size_t nr = b->size();
        if (nr == 0)
            throw std::runtime_error("Content id not found in table");
        else if (nr > 1)
            throw std::runtime_error("Multiple content ids found in table");

        auto& content_id_vec = b->get<unsigned int>(ResultReport::SectionContent::DBColumnContentID.name());
        auto& type_vec       = b->get<int>(ResultReport::SectionContent::DBColumnType.name());
        auto& content_vec    = b->get<nlohmann::json>(ResultReport::SectionContent::DBColumnJSONContent.name());

        ResultReport::SectionContent::ContentType type = (ResultReport::SectionContent::ContentType)type_vec.get(0);

        //create empty content depending on type
        //@TODO: small factory?
        if (type == ResultReport::SectionContent::ContentType::Figure)
        {
            content.reset(new ResultReport::SectionContentFigure(section));
        }
        else if (type == ResultReport::SectionContent::ContentType::Table)
        {
            content.reset(new ResultReport::SectionContentTable(section));
        }
        else if (type == ResultReport::SectionContent::ContentType::Text)
        {
            content.reset(new ResultReport::SectionContentText(section));
        }
        else
        {
            throw std::runtime_error("Invalid content type");
        }

        //read content
        bool ok = content->fromJSON(content_vec.get(0));
        if (!ok)
            throw std::runtime_error("Could not read content from JSON");

        //check content
        if (content->contentID() != content_id_vec.get(0) ||
            content->contentType() != type)
            throw std::runtime_error("contents invalid");
    }
    catch(const std::exception& ex)
    {
        logerr << "Could not load content: " << ex.what();
        return ResultT<std::shared_ptr<ResultReport::SectionContent>>::failed(ex.what());
    }
    catch(...)
    {
        logerr << "Could not load content: Unknown error";
        return ResultT<std::shared_ptr<ResultReport::SectionContent>>::failed("Unknown error");
    }

    loginf << "loaded";

    return ResultT<std::shared_ptr<ResultReport::SectionContent>>::succeeded(content);
}

/**
 */
boost::optional<std::uint64_t> TaskResultDB::getMaxReportContentID()
{
    traced_assert(db_interface_.ready());

    if (!existsReportContentsTable())
        return boost::optional<std::uint64_t>();

    return db_interface_.getMaxUIntValue(ResultReport::SectionContent::DBTableName,
                                         ResultReport::SectionContent::DBColumnContentID.name());
}
