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

#include "task/result/report/reportexporter.h"
#include "task/result/report/reportexport.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontent.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontenttext.h"

#include "task/result/taskresult.h"

#include "files.h"
#include "global.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "geographicview.h"
#endif

#include <boost/filesystem.hpp>

namespace ResultReport
{

const int ReportExporter::TableMaxRows    = 0;
const int ReportExporter::TableMaxColumns = 0;

const std::string ReportExporter::ResourceFolderScreenshots = "screenshots";
const std::string ReportExporter::ResourceFolderTables      = "tables";
const std::string ReportExporter::ResourceFolderIcons       = "icons";

const std::string ReportExporter::ExportImageFormat = ".jpg";
const std::string ReportExporter::ExportTableFormat = ".json";
const std::string ReportExporter::ExportTextFormat  = ".txt";

/**
 */
ReportExporter::ReportExporter(const ReportExport* report_export,
                               const std::string& export_fn,
                               const std::string& export_resource_dir,
                               bool interaction_mode)
:   report_export_      (report_export      )
,   export_fn_          (export_fn          )
,   export_resource_dir_(export_resource_dir)
,   interaction_mode_   (interaction_mode   )
{
    traced_assert(report_export_);
}

/**
 */
ReportExporter::~ReportExporter()
{
}

/**
 */
const ReportExportSettings& ReportExporter::settings() const
{
    return report_export_->settings();
}

/**
 */
std::string ReportExporter::exportPath() const
{
    return exportResourceDir() + "/" + exportFilename();
}

/**
 */
std::string ReportExporter::resourceSubDir(ResourceDir dir)
{
    switch (dir)
    {
        case ResourceDir::Root:
            return "";
        case ResourceDir::Screenshots:
            return ResourceFolderScreenshots;
        case ResourceDir::Tables:
            return ResourceFolderTables;
        case ResourceDir::Icons:
            return ResourceFolderIcons;
    }

    return "";
}

/**
 */
Result ReportExporter::initExport(TaskResult& result)
{
    return initExport_impl(result);
}

/**
 */
ResultT<nlohmann::json> ReportExporter::finalizeExport(TaskResult& result)
{
    return finalizeExport_impl(result);
}

/**
 */
ResultT<nlohmann::json> ReportExporter::exportReport(TaskResult& result,
                                                     const std::string& section,
                                                     const Content& content)
{
    loginf << "exporting result '" << result.name() << "'";

    num_sections_exported_ = 0;
    done_                  = false;

    ResultT<nlohmann::json> res_final;

    try
    {
        //check stuff
        if (exportCreatesFile() && export_fn_.empty())
            return Result::failed("Filename not provided");
        if (exportCreatesResources() && export_resource_dir_.empty())
            return Result::failed("Resource directory not provided");

        // do not delete, user error
        // if (exportCreatesResources() && Utils::Files::directoryExists(export_resource_dir_))
        // {
        //     Utils::Files::deleteFolder(export_resource_dir_);
        //     if (Utils::Files::directoryExists(export_resource_dir_))
        //         return Result::failed("Existing report resources could not be removed");
        // }
        if (exportCreatesResources() && !Utils::Files::createMissingDirectories(export_resource_dir_))
            return Result::failed("Resource directory could not be created");

        Section*                      section_ptr = nullptr;
        boost::optional<unsigned int> content_id;

        //retrieve initial section and content id
        if (!section.empty())
        {
            if (!result.report()->hasSection(section))
                return Result::failed("Section '" + section + "' not found in report");

            section_ptr = &result.report()->getSection(section);

            if (!content.first.empty())
            {
                if (!section_ptr->hasContent(content.first, content.second))
                    return Result::failed("Content '" + content.first + "' not found in report section '" + section + "'");

                content_id = section_ptr->contentID(content.first, content.second);
            }
        }

        if (!section_ptr)
        {
            if (exportNeedsRootSection())
                return Result::failed("No root section specified");

            section_ptr = result.report()->rootSection().get();
        }

        //check if section is deactivated for export
        if (!section_ptr->exportEnabled(exportMode()))
            return Result::failed("Parent section '" + section_ptr->name() + "' is disabled for export");

        traced_assert(section_ptr);

        //pre-compute total number of exported sections
        auto func = [ this ] (const Section& s)
        {
            return s.exportEnabled(this->exportMode());
        };
        num_sections_total_ = section_ptr->totalNumSections(func);
        num_contents_total_ = section_ptr->totalNumContents(func);

        //for immediate rendering of geographic view during image generation
#if USE_EXPERIMENTAL_SOURCE == true
        GeographicView::instant_display_ = true;
#endif

        //export
        res_final = exportReport_impl(result, section_ptr, content_id);
        if (!res_final.ok())
            return res_final;

#if USE_EXPERIMENTAL_SOURCE == true
        GeographicView::instant_display_ = false;
#endif

        //check on created in-mem data
        if (exportCreatesInMemoryData() && res_final.result().is_null())
            return Result::failed("Creating in-memory data failed");

        //check on created file
        if (exportCreatesFile() && !Utils::Files::fileExists(exportPath()))
            return Result::failed("Creating report file failed");
    }
    catch (const std::exception& ex)
    {
        return Result::failed("Exporting report failed: " + std::string(ex.what())); 
    }

    loginf << "exporting result '" << result.name() << "' succeeded";

    done_ = true;
    emit progressChanged();

    return res_final;
}

/**
 */
ResultT<nlohmann::json> ReportExporter::exportReport_impl(TaskResult& result,
                                                          Section* section,
                                                          const boost::optional<unsigned int>& content_id)
{
    traced_assert(section);

    auto res = initExport(result);
    if (!res.ok())
        return res;

    loginf << "start section = " << section->name();

    //visit start section
    res = visitSection(*section, true, true, true);
    if (!res.ok())
        return res;

    return finalizeExport(result);
}

/**
 */
Result ReportExporter::visitSection(Section& section, 
                                    bool is_root_section,
                                    bool write_subsections,
                                    bool write_contents)
{
    //skip section?
    if (!section.exportEnabled(exportMode()))
    {
        loginf << "skipping, section '" << section.id() << "' disabled for export";
        return Result::succeeded();
    }

    //loginf << "exporting section '" << section.id() << "'";

    setStatus("exporting section '" + section.name() + "'");

    //export section first
    auto res = exportSection_impl(section, 
                                  is_root_section,
                                  write_subsections,
                                  write_contents);
    if (!res.ok())
        return res;

    current_content_section_ = &section;

    //then its contents
    if (write_contents)
    {
        auto contents = section.sectionContent();
        for (auto& c : contents)
        {
            if (!c)
                return Result::failed("Contents could not be loaded in section '" + section.name() + "'");

            res = visitContent(*c, is_root_section);

            num_contents_exported_[ c->contentType() ] += 1;
            emit progressChanged();

            if (!res.ok())
                return res;
        }
    }

    current_content_section_ = nullptr;

    ++num_sections_exported_;
    emit progressChanged();

    //then its subsections
    if (write_subsections)
    {
        for (const auto& sec_it : section.subSections(false))
        {
            bool export_more = !exportNeedsRootSection();

            //subsections are never a root section
            res = visitSection(*sec_it, false, export_more, export_more);
            if (!res.ok())
                return res;
        }
    }

    return Result::succeeded();
}

/**
 */
Result ReportExporter::visitContent(SectionContent& content, bool is_root_section)
{
    //skip content?
    if (!content.exportEnabled(exportMode()))
    {
        loginf << "skipping, content '" << content.id() << "' disabled for export";
        return Result::succeeded();
    }

    if (content.isLocked())
    {
        loginf << "skipping, content '" << content.id() << "' locked";
        return Result::succeeded();
    }

    //loginf << "exporting content '" << content.id() << "'";

    //load content?
    if (content.isOnDemand() && !content.loadOnDemandIfNeeded())
        return Result::failed("Content '" + content.name() + "' could not be loaded on demand");

    //check which content
    if (content.contentType() == SectionContent::ContentType::Figure)
    {
        auto c = dynamic_cast<SectionContentFigure*>(&content);
        if (c) 
            return exportFigure(*c, is_root_section);
    }
    else if (content.contentType() == SectionContent::ContentType::Table)
    {
        auto c = dynamic_cast<SectionContentTable*>(&content);
        if (c) 
            return exportTable(*c, is_root_section);
    }
    else if (content.contentType() == SectionContent::ContentType::Text)
    {
        auto c = dynamic_cast<SectionContentText*>(&content);
        if (c) 
            return exportText(*c, is_root_section);
    }

    return Result::failed("Content '" + content.name() + "' is of illegal type");
}

/**
 */
Result ReportExporter::exportFigure(SectionContentFigure& figure, bool is_root_section)
{
    return exportFigure_impl(figure, is_root_section);
}

/**
 */
Result ReportExporter::exportTable(SectionContentTable& table, bool is_root_section)
{
    return exportTable_impl(table, is_root_section);
}

/**
 */
Result ReportExporter::exportText(SectionContentText& text, bool is_root_section)
{
    return exportText_impl(text, is_root_section);
}

/**
 */
void ReportExporter::setStatus(const std::string& status)
{
    status_ = status;

    emit progressChanged();
}

/**
 */
std::string ReportExporter::storeFile(ResourceDir dir, const std::string& fn) const
{
    auto basename = Utils::Files::getFilenameFromPath(fn);
    auto dst_dir  = exportResourceDir() + "/" + resourceSubDir(dir);
    auto dst_path = dst_dir + "/" + basename;
    auto rel_path = resourceSubDir(dir) + "/" + basename;

    if (Utils::Files::fileExists(dst_path))
        return rel_path;

    Utils::Files::createMissingDirectories(dst_dir);

    if (!boost::filesystem::copy_file(fn, dst_path, boost::filesystem::copy_options::overwrite_existing))
        return "";

    return rel_path;
}

/**
 */
double ReportExporter::progress() const
{
    double total    = 0.0;
    double finished = 0.0;

    const double FactorSection = 0.1;
    const double FactorText    = 0.1;
    const double FactorTable   = 0.2;
    const double FactorFigure  = 10.0;

    total    += num_sections_total_    * FactorSection;
    finished += num_sections_exported_ * FactorSection;

    auto getFactor = [ & ] (SectionContentType t)
    {
        if (t == SectionContentType::Figure)
            return FactorFigure;
        if (t == SectionContentType::Table)
            return FactorTable;
        if (t == SectionContentType::Text)
            return FactorText;

        return 1.0;
    };

    for (const auto& it : num_contents_total_)
    {
        total += it.second * getFactor(it.first);

        auto it2 = num_contents_exported_.find(it.first);
        if (it2 != num_contents_exported_.end())
            finished += it2->second * getFactor(it.first);
    }

    double part0_progress = total == 0.0 ? 0.0 : finished / total;
    double part1_progress = isDone() ? 1.0 : 0.0;

    const double part1_factor = finalizeFactor();
    const double part0_factor = 1.0 - part1_factor;

    return part0_factor * part0_progress + part1_factor * part1_progress;
}

}
