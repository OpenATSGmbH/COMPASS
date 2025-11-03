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

#include "task/result/report/reportexport.h"
#include "task/result/report/reportexporter.h"
#include "task/result/report/reportexporterjson.h"
#include "task/result/report/reportexporterlatex.h"

#include "taskmanager.h"

#include "system.h"
#include "logger.h"

#include <QApplication>

namespace ResultReport
{

/**
 */
ReportExport::ReportExport(const std::string& class_id, 
                           const std::string& instance_id, 
                           TaskManager* task_manager)
:   Configurable(class_id, instance_id, task_manager)
{
    registerParameter("author"           , &settings_.author           , ReportExportSettings().author           );
    registerParameter("open_created_file", &settings_.open_created_file, ReportExportSettings().open_created_file);

    registerParameter("latex_table_max_rows"     , &settings_.latex_table_max_rows     , ReportExportSettings().latex_table_max_rows     );
    registerParameter("latex_table_max_col_width", &settings_.latex_table_max_col_width, ReportExportSettings().latex_table_max_col_width);
    registerParameter("latex_table_min_cols_wide", &settings_.latex_table_min_cols_wide, ReportExportSettings().latex_table_min_cols_wide);
    registerParameter("latex_pdf_max_reruns"     , &settings_.latex_pdf_max_reruns     , ReportExportSettings().latex_pdf_max_reruns     );

    registerParameter("json_table_max_rows_inline", &settings_.json_table_max_rows_inline, ReportExportSettings().json_table_max_rows_inline);
    registerParameter("json_table_max_cols_inline", &settings_.json_table_max_cols_inline, ReportExportSettings().json_table_max_cols_inline);

    //fill in some default values if missing
    if (!settings_.author.size())
        settings_.author = Utils::System::getUserName();
    if (!settings_.author.size())
        settings_.author = "User";
}

/**
 */
ReportExport::~ReportExport()
{
}

/**
 */
ResultT<nlohmann::json> ReportExport::exportReport(TaskResult& result,
                                                   ReportExportMode mode,
                                                   const std::string& fn,
                                                   const std::string& resource_dir,
                                                   const std::string& section,
                                                   bool interaction_mode)
{
    //create exporter
    auto exporter = createExporter(mode, fn, resource_dir, interaction_mode);
    if (!exporter)
        return Result::failed("Exporter could not be created for export type ''");

    auto exporter_ptr = exporter.get();

    connect(exporter.get(), &ReportExporter::progressChanged, 
        [ this, exporter_ptr ] () { this->updateProgress(exporter_ptr); });

    //export using exporter
    auto res = exporter->exportReport(result, section);

    return res;
}

/**
 */
std::unique_ptr<ReportExporter> ReportExport::createExporter(ReportExportMode mode,
                                                             const std::string& fn,
                                                             const std::string& resource_dir,
                                                             bool interaction_mode) const
{
    if (mode == ReportExportMode::JSONFile)
    {
        return std::unique_ptr<ReportExporter>(new ReportExporterJSONFile(this, fn, resource_dir, interaction_mode));
    }
    else if (mode == ReportExportMode::JSONBlob)
    {
        return std::unique_ptr<ReportExporter>(new ReportExporterJSONBlob(this, fn, resource_dir, interaction_mode));
    }
    else if (mode == ReportExportMode::Latex)
    {
        return std::unique_ptr<ReportExporter>(new ReportExporterLatexFiles(this, fn, resource_dir, interaction_mode));
    }
    else if (mode == ReportExportMode::LatexPDF)
    {
        return std::unique_ptr<ReportExporter>(new ReportExporterLatexPDF(this, fn, resource_dir, interaction_mode));
    }

    return std::unique_ptr<ReportExporter>();
}

/**
 */
void ReportExport::updateProgress(ReportExporter* exporter)
{
    traced_assert(exporter);

    //loginf << "num exported: " << exporter->numSectionsExported() << ", num total: " << exporter->numSectionsTotal();

    status_   = exporter->status();
    progress_ = exporter->progress();

    emit progressChanged();
}

}
