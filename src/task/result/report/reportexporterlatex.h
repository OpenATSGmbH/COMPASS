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

#include "reportexporter.h"

#include <memory>

class LatexDocument;

namespace ResultReport
{

/**
 */
class ReportExporterLatex : public ReportExporter
{
public:
    ReportExporterLatex(const ReportExport* report_export,
                        const std::string& export_fn,
                        const std::string& export_resource_dir,
                        bool interaction_mode,
                        bool write_pdf);
    virtual ~ReportExporterLatex();

protected:
    Result initExport_impl(TaskResult& result) override final;
    ResultT<nlohmann::json> finalizeExport_impl(TaskResult& result) override final;

    Result exportSection_impl(Section& section, 
                              bool is_root_section,
                              bool write_subsections,
                              bool write_contents) override final;
    Result exportFigure_impl(SectionContentFigure& figure, bool is_root_section) override final;
    Result exportTable_impl(SectionContentTable& table, bool is_root_section) override final;
    Result exportText_impl(SectionContentText& text, bool is_root_section) override final;

    bool exportCreatesFile() const override final { return false; }
    bool exportCreatesResources() const override final { return true; }
    bool exportCreatesInMemoryData() const override final { return false; }
    bool exportNeedsRootSection() const override final { return false; }

    double finalizeFactor() const override final { return write_pdf_ ? 0.1 : 0.01; }

private:
    enum class LatexCmdResult
    {
        Handled = 0,
        Skipped
    };

    Result writePDF() const;

    ResultT<LatexCmdResult> handleLatexCommand(std::string& cmd) const;

    bool write_pdf_ = false;

    std::unique_ptr<LatexDocument>  latex_doc_;
    std::map<Section*, std::string> latex_sections_;
};

/**
 */
class ReportExporterLatexFiles : public ReportExporterLatex
{
public:
    ReportExporterLatexFiles(const ReportExport* report_export,
                             const std::string& export_fn,
                             const std::string& export_resource_dir,
                             bool interaction_mode)
    :   ReportExporterLatex(report_export, export_fn, export_resource_dir, interaction_mode, false) {}

    virtual ~ReportExporterLatexFiles() = default;

    ReportExportMode exportMode() const override final { return ReportExportMode::Latex; }
};

/**
 */
class ReportExporterLatexPDF : public ReportExporterLatex
{
public:
    ReportExporterLatexPDF(const ReportExport* report_export,
                           const std::string& export_fn,
                           const std::string& export_resource_dir,
                           bool interaction_mode)
    :   ReportExporterLatex(report_export, export_fn, export_resource_dir, interaction_mode, true) {}

    virtual ~ReportExporterLatexPDF() = default;

    ReportExportMode exportMode() const override final { return ReportExportMode::LatexPDF; }
};

} // namespace ResultReport
