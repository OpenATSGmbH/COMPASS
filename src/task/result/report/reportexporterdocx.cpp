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

#include "task/result/report/reportexporterdocx.h"

#include "task/result/report/section.h"
#include "task/result/report/sectioncontent.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontenttext.h"
#include "task/result/taskresult.h"

#include "docxdocument.h"
#include "docxsection.h"
#include "docximage.h"
#include "docxtable.h"

#include "compass.h"
#include "files.h"
#include "stringconv.h"
#include "logger.h"

#include <boost/filesystem.hpp>

#include <QDesktopServices>
#include <QUrl>

namespace ResultReport
{

/**
 */
ReportExporterDocx::ReportExporterDocx(const ReportExport* report_export,
                                       const std::string& export_fn,
                                       const std::string& export_resource_dir,
                                       bool interaction_mode)
:   ReportExporter(report_export, export_fn, export_resource_dir, interaction_mode)
{
}

/**
 */
ReportExporterDocx::~ReportExporterDocx()
{
}

/**
 */
Result ReportExporterDocx::initExport_impl(TaskResult& result)
{
    std::string report_fn = boost::filesystem::path(exportFilename()).stem().string() + ".docx";

    docx_doc_.reset(new DocxDocument(exportResourceDir(), report_fn));

    docx_doc_->title(result.name() + " Report");

    const auto& s = settings();

    if (s.author.size())
        docx_doc_->author(s.author);

    if (s.abstract.size())
        docx_doc_->abstract(s.abstract);

    // footer: licensee (left), page number (center), logo (right)
    auto licensee = COMPASS::instance().licenseeString(true);
    if (!licensee.empty())
        docx_doc_->footerLeft(licensee);

    auto logo_path = Utils::Files::getImageFilepath("logo.png");
    if (boost::filesystem::exists(logo_path))
        docx_doc_->setLogo(logo_path);

    return Result::succeeded();
}

/**
 */
ResultT<nlohmann::json> ReportExporterDocx::finalizeExport_impl(TaskResult& result)
{
    setStatus("Writing DOCX file");

    try
    {
        docx_doc_->write();
    }
    catch (const std::exception& e)
    {
        return Result::failed("DOCX write failed: " + std::string(e.what()));
    }

    if (settings().open_created_file && hasInteraction())
    {
        std::string fullpath = (boost::filesystem::path(exportResourceDir()) /
                                boost::filesystem::path(exportFilename())).string();

        // fix extension if needed
        if (Utils::String::hasEnding(fullpath, ".tex") ||
            Utils::String::hasEnding(fullpath, ".pdf"))
        {
            auto stem = boost::filesystem::path(fullpath).stem().string();
            auto dir  = boost::filesystem::path(fullpath).parent_path().string();
            fullpath  = (boost::filesystem::path(dir) / (stem + ".docx")).string();
        }
        else if (!Utils::String::hasEnding(fullpath, ".docx"))
        {
            fullpath = boost::filesystem::path(fullpath).stem().string() + ".docx";
        }

        loginf << "ReportExporterDocx: opening '" << fullpath << "'";

        if (!QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(fullpath))))
            logerr << "ReportExporterDocx: could not open created file";
    }

    return ResultT<nlohmann::json>::succeeded(nlohmann::json());
}

/**
 */
Result ReportExporterDocx::exportSection_impl(Section& section,
                                              bool is_root_section,
                                              bool write_subsections,
                                              bool write_contents)
{
    auto heading = section.compoundResultsHeading();

    // ignore empty "Results" root section
    if (heading.empty())
        return Result::succeeded();

    DocxSection& docx_section = docx_doc_->getSection(heading);
    docx_sections_[&section] = heading;

    return Result::succeeded();
}

/**
 */
Result ReportExporterDocx::exportFigure_impl(SectionContentFigure& figure,
                                             bool is_root_section)
{
    auto it = docx_sections_.find(figure.parentSection());
    if (it == docx_sections_.end())
        return Result::failed("Content '" + figure.name() + "' obtains no parent section in report");

    // obtain rendered images
    auto images = figure.obtainImages(&exportResourceDir());
    if (!images.ok())
        return images;

    auto& docx_section = docx_doc_->getSection(it->second);

    for (const auto& img : images.result())
    {
        // register image with the document (reads dimensions, assigns rId)
        std::string rel_id = docx_doc_->addImageFile(img.path);

        // find the image entry to get dimensions
        int w_emu = 5760000;  // default 16cm
        int h_emu = 3240000;  // default 9cm

        // look up the registered entry for actual dimensions
        // (addImageFile may have computed them from the actual image)
        // The simplest approach: re-read. But addImageFile caches, so use that.
        // We need access to the entry — for now use defaults from addImageFile.
        // DocxDocument stores entries internally; the section just needs rel_id + dims.
        // Let's get them from a second call (cached, returns same rId).
        // Actually, let's just pass through addImageFile which already computed dims.

        // For proper dim forwarding, we add a lookup method or pass dims separately.
        // Quick approach: read the image ourselves
        QImage qimg(QString::fromStdString(img.path));
        if (!qimg.isNull())
        {
            const int max_width_emu = 5760000;
            double dpi_x = qimg.dotsPerMeterX() > 0 ? qimg.dotsPerMeterX() / 39.3701 : 96.0;
            double dpi_y = qimg.dotsPerMeterY() > 0 ? qimg.dotsPerMeterY() / 39.3701 : 96.0;

            w_emu = (int)(qimg.width()  / dpi_x * 914400.0);
            h_emu = (int)(qimg.height() / dpi_y * 914400.0);

            if (w_emu > max_width_emu)
            {
                double scale = (double)max_width_emu / w_emu;
                w_emu = max_width_emu;
                h_emu = (int)(h_emu * scale);
            }
        }

        docx_section.addImage(img.path, img.name, rel_id, w_emu, h_emu);
    }

    return Result::succeeded();
}

/**
 */
Result ReportExporterDocx::exportTable_impl(SectionContentTable& table,
                                            bool is_root_section)
{
    auto it = docx_sections_.find(table.parentSection());
    if (it == docx_sections_.end())
        return Result::failed("Content '" + table.name() + "' obtains no parent section in report");

    const auto& s = settings();

    auto& docx_section = docx_doc_->getSection(it->second);

    std::vector<std::string> headings = table.proxyHeadings();
    unsigned int num_cols = headings.size();

    traced_assert(num_cols);

    // configure wide table
    bool wide_table = false;
    if (s.latex_table_min_cols_wide >= 0 && headings.size() >= (size_t)s.latex_table_min_cols_wide)
        wide_table = true;

    // determine max row count
    bool has_max_row_override = table.maxRowCount().has_value();
    int max_row_count;
    if (has_max_row_override)
        max_row_count = table.maxRowCount().value();
    else
        max_row_count = s.latex_table_max_rows;

    unsigned int num_rows = table.numProxyRows();
    nlohmann::json row_data;

    bool split_tables = has_max_row_override;
    const int HardLimit = 500;
    max_row_count = split_tables ? std::max(max_row_count, HardLimit) : max_row_count;

    DocxTable* current_table = nullptr;
    unsigned int current_rows = 0;
    unsigned int num_tables = 0;

    for (unsigned int row = 0; row < num_rows; ++row, ++current_rows)
    {
        if (!current_table || (split_tables && current_rows > (unsigned int)max_row_count))
        {
            std::string table_name_cur = table.name();
            if (split_tables)
            {
                current_rows = 0;
                ++num_tables;
                table_name_cur += std::to_string(num_tables);
            }

            traced_assert(!docx_section.hasTable(table_name_cur));

            docx_section.addTable(table_name_cur, num_cols, headings);
            current_table = &docx_section.getTable(table_name_cur);

            current_table->setMaxRowCount(max_row_count);
            current_table->setWideTable(wide_table);

            if (docx_doc_->hasFooter())
                current_table->setFooterRefId(docx_doc_->footerRelId());
        }

        // export row content using DOCX mode
        row_data = table.exportProxyContent(row, ReportExportMode::DOCX);
        if (!row_data.is_array() || row_data.size() != num_cols)
            return Result::failed("Content '" + table.name() + "' could not be prepared for export @row" + std::to_string(row));

        std::vector<std::string> row_strings(num_cols);
        std::vector<unsigned int> cell_styles(num_cols, 0);

        for (unsigned int cnt = 0; cnt < num_cols; ++cnt)
        {
            traced_assert(row_data.at(cnt).is_string());
            row_strings[cnt] = row_data.at(cnt).get<std::string>();

            // get cell style from the table model
            cell_styles[cnt] = table.cellStyle((int)row, (int)cnt);

            // apply "Passed"/"Failed" hack same as LaTeX exporter
            if (row_strings[cnt] == "Passed")
                cell_styles[cnt] |= CellStyleTextColorGreen;
            else if (row_strings[cnt] == "Failed")
                cell_styles[cnt] |= CellStyleTextColorRed;
        }

        current_table->addRow(std::move(row_strings), std::move(cell_styles));
    }

    return Result::succeeded();
}

/**
 */
Result ReportExporterDocx::exportText_impl(SectionContentText& text,
                                           bool is_root_section)
{
    auto it = docx_sections_.find(text.parentSection());
    if (it == docx_sections_.end())
        return Result::failed("Content '" + text.name() + "' obtains no parent section in report");

    auto& docx_section = docx_doc_->getSection(it->second);

    for (const auto& txt_it : text.texts())
        docx_section.addText(txt_it);

    return Result::succeeded();
}

} // namespace ResultReport
