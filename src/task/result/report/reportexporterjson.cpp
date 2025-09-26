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

#include "task/result/report/reportexporterjson.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontent.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontenttext.h"
#include "task/result/taskresult.h"

#include "files.h"

#include <fstream>

namespace ResultReport
{

/**
 */
ReportExporterJSON::ReportExporterJSON(const ReportExport* report_export,
                                       const std::string& export_fn,
                                       const std::string& export_resource_dir,
                                       bool interaction_mode,
                                       bool write_files)
:   ReportExporter(report_export, export_fn, export_resource_dir, interaction_mode)
,   write_files_  (write_files)
{
}

/**
 */
ReportExporterJSON::~ReportExporterJSON() = default;

/**
 */
Result ReportExporterJSON::initExport_impl(TaskResult& result)
{
    return Result::succeeded();
}

/**
 */
ResultT<nlohmann::json> ReportExporterJSON::finalizeExport_impl(TaskResult& result)
{
    if (!write_files_)
        return ResultT<nlohmann::json>::succeeded(json_data_);

    setStatus("Writing report file");

    std::string fn = exportPath();

    std::ofstream file(fn);
    if (!file.is_open())
        return ResultT<nlohmann::json>::failed("Could not create file '" + fn + "'");

    file << json_data_.dump(4);

    if (!file)
        return ResultT<nlohmann::json>::failed("Could not export report to '" + fn + "'");

    file.close();

    return ResultT<nlohmann::json>::succeeded(nlohmann::json());
}

/**
 */
Result ReportExporterJSON::exportSection_impl(Section& section, 
                                              bool is_root_section,
                                              bool write_subsections,
                                              bool write_contents)
{
    auto res_dir     = exportResourceDir();
    auto res_dir_ptr = exportCreatesResources() ? &res_dir : nullptr;

    //generate section json
    auto res_json = section.toJSONDocument(res_dir_ptr);
    if (!res_json.ok())
        return res_json;

    traced_assert(res_json.result().is_object());

    auto sec_json = res_json.result();

    if (!write_files_ && !write_subsections)
        sec_json.erase(Section::FieldSubSections);
    if (!write_files_ && !write_contents)
        sec_json.erase(Section::FieldDocContents);

    nlohmann::json* jptr = nullptr;

    auto parent = section.parentSection();
    if (parent && !json_sections_.empty())
    {
        //section obtains parent => try to find json section of parent
        auto it = json_sections_.find(parent);
        if (it == json_sections_.end())
            return Result::failed("Section '" + section.name() + "' obtains no parent section in report");

        traced_assert(it->second);
        traced_assert(it->second->is_object());

        auto& parent_json = *it->second;

        traced_assert(parent_json.contains(Section::FieldSubSections));
        auto& subsections = parent_json[ Section::FieldSubSections ];
        traced_assert(subsections.is_array());

        //add to parents subsections
        subsections.push_back(sec_json);

        jptr = &subsections.back();
    }
    else
    {
        traced_assert(json_sections_.empty());

        //root section => init json data
        json_data_ = sec_json;

        jptr = &json_data_;
    }

    traced_assert(jptr);

    //remember my section's json object
    json_sections_[ &section ] = jptr;

    return Result::succeeded();
}

/**
 */
Result ReportExporterJSON::exportContentToJSON(SectionContent& content, 
                                               bool is_root_section)
{
    //loginf;

    auto res_dir     = exportResourceDir();
    auto res_dir_ptr = exportCreatesResources() ? &res_dir : nullptr;

    //generate content json
    auto res_json = content.toJSONDocument(res_dir_ptr);
    if (!res_json.ok())
        return res_json;

    //contents might yield a single or multiple json content objects
    traced_assert(res_json.result().is_object() || res_json.result().is_array());

    auto parent = content.parentSection();
    traced_assert(parent);

    // try to find json section of parent
    auto it = json_sections_.find(parent);
    if (it == json_sections_.end())
        return Result::failed("Content '" + content.name() + "' obtains no parent section in report");

    traced_assert(it->second);
    traced_assert(it->second->is_object());

    auto& parent_json = *it->second;

    traced_assert(parent_json.contains(Section::FieldDocContents));
    auto& contents = parent_json[ Section::FieldDocContents ];
    traced_assert(contents.is_array());

    //add to parent section's contents
    if (res_json.result().is_object())
    {
        //loginf << "adding content to parent " << parent->name();

        //add single json content
        contents.push_back(res_json.result());
    }
    else // array
    {
        //loginf << "adding " << res_json.result().size() << " content(s) to parent " << parent->name();

        //add multiple json contents
        for (const auto& it : res_json.result())
        {
            traced_assert(it.is_object());
            contents.push_back(it);
        }
    }

    return Result::succeeded();
}

/**
 */
Result ReportExporterJSON::exportFigure_impl(SectionContentFigure& figure, 
                                             bool is_root_section)
{
    return exportContentToJSON(figure, is_root_section);
}

/**
 */
Result ReportExporterJSON::exportTable_impl(SectionContentTable& table, 
                                            bool is_root_section)
{
    return exportContentToJSON(table, is_root_section);
}

/**
 */
Result ReportExporterJSON::exportText_impl(SectionContentText& text, 
                                           bool is_root_section)
{
    return exportContentToJSON(text, is_root_section);
}

}
