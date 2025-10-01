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

#include "task/result/report/reportexporterlatex.h"

#include "task/result/report/section.h"
#include "task/result/report/sectioncontent.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontenttext.h"
#include "task/result/taskresult.h"

#include "latexdocument.h"
#include "latexsection.h"
#include "lateximage.h"
#include "latextable.h"

#include "files.h"
#include "stringconv.h"
#include "util/system.h"
#include "asynctask.h"

#include <fstream>

#include <boost/filesystem.hpp>

#include <QApplication>
#include <QDesktopServices>
#include <QUrl>

namespace ResultReport
{

/**
 */
ReportExporterLatex::ReportExporterLatex(const ReportExport* report_export,
                                         const std::string& export_fn,
                                         const std::string& export_resource_dir,
                                         bool interaction_mode,
                                         bool write_pdf)
:   ReportExporter(report_export, export_fn, export_resource_dir, interaction_mode)
,   write_pdf_    (write_pdf)
{
}

/**
 */
ReportExporterLatex::~ReportExporterLatex()
{
}

/**
 */
Result ReportExporterLatex::initExport_impl(TaskResult& result)
{
    bool pdflatex_found = Utils::System::exec("which pdflatex").size();
    if (write_pdf_ && !pdflatex_found)
        return Result::failed("Cannot generate PDF: pdflatex not installed");

    std::string report_fn = boost::filesystem::path(exportFilename()).stem().string() + ".tex";

    latex_doc_.reset(new LatexDocument(exportResourceDir(), report_fn));

    latex_doc_->title("OpenATS COMPASS " + result.name() + " Report");

    const auto& s = settings();

    if (s.author.size())
        latex_doc_->author(s.author);

    if (s.abstract.size())
        latex_doc_->abstract(s.abstract);

    return Result::succeeded();
}

/**
 */
ResultT<nlohmann::json> ReportExporterLatex::finalizeExport_impl(TaskResult& result)
{
    setStatus("Writing report file");

    latex_doc_->write();

    if (write_pdf_)
    {
        setStatus("Converting to PDF");

        auto res_pdf = writePDF();
        if (!res_pdf.ok())
            return res_pdf;
    }

    return ResultT<nlohmann::json>::succeeded(nlohmann::json());
}

/**
 */
Result ReportExporterLatex::exportSection_impl(Section& section, 
                                               bool is_root_section,
                                               bool write_subsections,
                                               bool write_contents)
{
    auto heading      = section.compoundResultsHeading();
    auto section_name = Utils::String::latexString(heading);

    // ignore if top "Results"
    if (section_name.empty())
        return Result::succeeded();

    LatexSection& latex_section = latex_doc_->getSection(section_name);
    latex_section.label("sec:" + heading);

    latex_sections_[ &section ] = section_name;

    return Result::succeeded();
}

/**
 */
Result ReportExporterLatex::exportFigure_impl(SectionContentFigure& figure, 
                                              bool is_root_section)
{
    auto it = latex_sections_.find(figure.parentSection());
    if (it == latex_sections_.end())
        return Result::failed("Content '" + figure.name() + "' obtains no parent section in report"); 

    //obtain image data
    auto images = figure.obtainImages(&exportResourceDir());
    if (!images.ok())
        return images;

    auto& latex_section = latex_doc_->getSection(it->second);

    for (const auto& img : images.result())
        latex_section.addImage(img.path, img.name);

    return Result::succeeded();
}

/**
 */
Result ReportExporterLatex::exportTable_impl(SectionContentTable& table, 
                                             bool is_root_section)
{
    auto it = latex_sections_.find(table.parentSection());
    if (it == latex_sections_.end())
        return Result::failed("Content '" + table.name() + "' obtains no parent section in report"); 

    const auto& s = settings();

    auto& latex_section = latex_doc_->getSection(it->second);

    std::vector<std::string> headings = table.proxyHeadings();
    unsigned int num_cols = headings.size();

    for (unsigned int cnt=0; cnt < num_cols; ++cnt) // latexify headings
        headings[ cnt ] = Utils::String::latexString(headings[ cnt ]);

    traced_assert(num_cols);

    //configure wide table settings
    bool wide_table = false;
    if (s.latex_table_min_cols_wide >= 0 && headings.size() >= (size_t)s.latex_table_min_cols_wide)
        wide_table = true;

    //determine max row count
    //table > settings
    bool has_max_row_override = table.maxRowCount().has_value();

    int max_row_count;
    if (has_max_row_override) // from internal override
        max_row_count = table.maxRowCount().value();
    else // from settings
        max_row_count = s.latex_table_max_rows;

    unsigned int num_rows = table.numProxyRows();
    nlohmann::json row_data;
    std::string ref;

    // split all too long tables with internal max row override into subtables
    bool split_tables = has_max_row_override; 

    //apply a hard limit to maximum rows when splitting tables
    const int HardLimit = 500;
    max_row_count = split_tables ? std::max(max_row_count, HardLimit) : max_row_count;

    LatexTable*  current_table    = nullptr;
    unsigned int current_rows     = 0;
    unsigned int num_tables       = 0;

    for (unsigned int row=0; row < num_rows; ++row, ++current_rows)
    {
        //create initial table or change table on hitting max row count when in split mode
        if (!current_table || (split_tables && current_rows > (unsigned int)max_row_count))
        {
            std::string table_name_cur = table.name();

            //split mode => keep track of tables
            if (split_tables)
            {
                current_rows = 0;
                ++num_tables;
                table_name_cur += std::to_string(num_tables);
            }

            //create new table
            traced_assert(!latex_section.hasTable(table_name_cur));

            latex_section.addTable(table_name_cur, num_cols, headings, "", false);
            current_table = &latex_section.getTable(table_name_cur);

            current_table->setMaxRowCount(max_row_count);
            current_table->setWideTable(wide_table);
        }

        row_data = table.exportProxyContent(row, ReportExportMode::Latex);
        if (!row_data.is_array() || row_data.size() != num_cols)
            return Result::failed("Content '" + table.name() + "' could not be prepared for export @row" + std::to_string(row)); 

        traced_assert(row_data.is_array());
        traced_assert(row_data.size() == num_cols);

        std::vector<std::string> row_strings(num_cols);

        for (unsigned int cnt=0; cnt < num_cols; ++cnt)
        {
            traced_assert(row_data.at(cnt).is_string());
            row_strings[cnt] = row_data.at(cnt).get<std::string>();

            auto& str = row_strings[cnt];

            bool command_handled = false;

            //handle special commands
            if (str.size() > 0 && str[ 0 ] == '\\')
            {
                auto res = handleLatexCommand(str);
                if (!res.ok())
                    return Result::failed("Content '" + table.name() + "' obtains invalid command: " + res.error()); 

                command_handled = res.result() == LatexCmdResult::Handled;
            }

            //limit too long columns
            if (wide_table &&
                !command_handled &&
                cnt > 2 && 
                s.latex_table_max_col_width >= 0 &&  
                str.size() > (size_t)s.latex_table_max_col_width)
            {
                std::string::size_type space_pos = str.rfind(' ', (size_t)s.latex_table_max_col_width);
                std::string::size_type comma_pos = str.rfind(',', (size_t)s.latex_table_max_col_width);

                if (space_pos == std::string::npos)
                {
                    if (comma_pos == std::string::npos)
                    {
                        // no 64-bit-or-less substring
                    }
                    else
                    {
                        str = str.substr(0, comma_pos) + "...";
                    }
                }
                else
                {
                    str = str.substr(0, space_pos) + "...";
                }
            }
        }

        //add row to current table
        current_table->addRow(std::move(row_strings));
    }

    return Result::succeeded();
}

/**
 */
ResultT<ReportExporterLatex::LatexCmdResult> ReportExporterLatex::handleLatexCommand(std::string& cmd) const
{
    const std::string IncludeGraphics = "\\includegraphics";

    //handle graphics
    auto gfx_idx = cmd.find(IncludeGraphics);
    if (gfx_idx != std::string::npos)
    {
        auto gfx_fn0 = cmd.find('{', gfx_idx + IncludeGraphics.size());
        auto gfx_fn1 = cmd.find('}', gfx_idx + IncludeGraphics.size());

        if (gfx_fn0 == std::string::npos ||
            gfx_fn1 == std::string::npos ||
            gfx_fn0 >= gfx_fn1)
            return Result::failed("Command obtains badly configured icons");

        gfx_fn0 += 1;
        auto len = gfx_fn1 - gfx_fn0;

        std::string fn   = cmd.substr(gfx_fn0, len);
        std::string path = Utils::Files::getIconFilepath(fn, false);

        if (fn.empty() || !Utils::Files::fileExists(path))
            return Result::failed("Command obtains missing icons");

        std::string icon_path_rel = storeFile(ResourceDir::Icons, path);
        if (icon_path_rel.empty())
            return Result::failed("Could not store icon");

        std::string part0 = cmd.substr(0, gfx_fn0);
        std::string part1 = cmd.substr(gfx_fn1, std::string::npos);

        cmd = part0 + icon_path_rel + part1;

        return ResultT<LatexCmdResult>::succeeded(LatexCmdResult::Handled);
    }

    return ResultT<LatexCmdResult>::succeeded(LatexCmdResult::Skipped);
}

/**
 */
Result ReportExporterLatex::exportText_impl(SectionContentText& text, 
                                            bool is_root_section)
{
    auto it = latex_sections_.find(text.parentSection());
    if (it == latex_sections_.end())
        return Result::failed("Content '" + text.name() + "' obtains no parent section in report"); 

    auto& latex_section = latex_doc_->getSection(it->second);

    for (const auto& txt_it : text.texts())
        latex_section.addText(txt_it);

    return Result::succeeded();
}

/**
 */
Result ReportExporterLatex::writePDF() const
{
    traced_assert(latex_doc_);

    std::string command_out;
    std::string command = "cd \"" + latex_doc_->path() + "\" && pdflatex --interaction=nonstopmode \"" + latex_doc_->filename()
            + "\" | awk 'BEGIN{IGNORECASE = 1}/warning|!/,/^$/;'";

    loginf << "running pdflatex";

    auto runPDFLatex = [ & ] ()
    {
        auto cb = [ & ] (const AsyncTaskState& state, AsyncTaskProgressWrapper& progress)
        {
            command_out = Utils::System::exec(command);

            return Result::succeeded();
        };

        AsyncFuncTask task(cb, "Finalizing Report", "Finalizing report...", false);
        task.runAsync();
    };

    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

    loginf << "cmd '" << command << "'";

    //initial run
    runPDFLatex();

    logdbg << "cmd done";

    const auto& s = settings();

    unsigned int max_runs = s.latex_pdf_max_reruns;
    unsigned int run_cnt  = 0;

    while (run_cnt < max_runs || 
           (command_out.find("Rerun to get outlines right"        ) != std::string::npos) || 
           (command_out.find("Rerun to get cross-references right") != std::string::npos))
    {
        loginf << "re-running pdflatex";
        
        //re-run
        runPDFLatex();

        logdbg << "re-run done";

        ++run_cnt;
    }

    loginf << "result '" << command_out << "'";

    if (command_out.size())
        return Result::failed("PDF Latex failed with warnings:\n\n" + std::string(command_out.c_str()));

    if (settings().open_created_file && hasInteraction())
    {
        std::string fullpath = (boost::filesystem::path(exportResourceDir()) / boost::filesystem::path(exportFilename())).string();
        if (Utils::String::hasEnding(fullpath, ".tex"))
            Utils::String::replace(fullpath, ".tex", ".pdf");

        loginf << "opening '" << fullpath << "'";

        if (!QDesktopServices::openUrl(QUrl(fullpath.c_str())))
            logerr << "could not open created file";
    }

    return Result::succeeded();
}

}
