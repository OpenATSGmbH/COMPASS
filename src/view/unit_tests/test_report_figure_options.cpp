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

#include "catch.hpp"
#include "reportfigure.h"
#include "reportfigureoptions.h"
#include "viewpointgenerator.h"

#include "json.hpp"

#include <QImage>

namespace
{
    nlohmann::json histogramViewPoint()
    {
        RawHistogram h;
        h.addBin(RawHistogramBin(3, "0-10"));
        h.addBin(RawHistogramBin(12, "10-20"));

        ViewPointGenVP vp("Histogram", 0, "Histogram");
        vp.annotations().getOrCreateAnnotation("Histogram")->addFeature(
            new ViewPointGenFeatureHistogram(h, "", Qt::blue, true, PlotMetadata("Histogram", "", "Value", "Count")));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }
}

TEST_CASE("report figure options JSON", "[view][reportfigure]")
{
    const ReportFigureOptions defaults;

    CHECK(defaults.histogram_log_scale);
    CHECK(defaults.export_layout_size == QSize(1200, 800));
    CHECK(defaults.export_layout_size == report_figure::ExportLayoutSize);

    SECTION("round trip")
    {
        ReportFigureOptions options;
        options.histogram_log_scale = false;
        options.export_layout_size  = QSize(600, 500);

        const nlohmann::json j = options.toJSON();

        CHECK(j.at("histogram_log_scale") == false);
        CHECK(j.at("export_width") == 600);
        CHECK(j.at("export_height") == 500);

        ReportFigureOptions read;
        REQUIRE(read.fromJSON(j));
        CHECK(read == options);
    }

    SECTION("null and missing fields give the defaults")
    {
        ReportFigureOptions read;
        read.histogram_log_scale = false;

        REQUIRE(read.fromJSON(nlohmann::json()));
        CHECK(read == defaults);

        REQUIRE(read.fromJSON(nlohmann::json::object({ { "export_width", 700 } })));
        CHECK(read.histogram_log_scale);
        CHECK(read.export_layout_size == QSize(700, 800));
    }

    SECTION("a value that is not readable keeps the values")
    {
        ReportFigureOptions read;
        read.export_layout_size = QSize(900, 900);

        const ReportFigureOptions before = read;

        CHECK(!read.fromJSON("small"));
        CHECK(!read.fromJSON(nlohmann::json::object({ { "histogram_log_scale", "yes" } })));
        CHECK(!read.fromJSON(nlohmann::json::object({ { "export_width", "1200" } })));
        CHECK(!read.fromJSON(nlohmann::json::object({ { "export_width", ReportFigureOptions::MinExportLayoutSize - 1 } })));
        CHECK(!read.fromJSON(nlohmann::json::object({ { "export_height", ReportFigureOptions::MaxExportLayoutSize + 1 } })));

        CHECK(read == before);
    }

    SECTION("parameter check")
    {
        CHECK(ReportFigureOptions::ParameterName == "report_figure_options");

        CHECK(ReportFigureOptions::checkParameter(nlohmann::json::object()).empty());
        CHECK(ReportFigureOptions::checkParameter(nlohmann::json::object({ { "report_figure_options", nlohmann::json() } })).empty());
        CHECK(ReportFigureOptions::checkParameter(
            nlohmann::json::object({ { "report_figure_options", { { "histogram_log_scale", false } } } })).empty());
        CHECK(!ReportFigureOptions::checkParameter(
            nlohmann::json::object({ { "report_figure_options", { { "histogram_log_scale", "no" } } } })).empty());
        CHECK(!ReportFigureOptions::checkParameter(
            nlohmann::json::object({ { "report_figure_options", { { "export_width", 10 } } } })).empty());
        CHECK(!ReportFigureOptions::checkParameter(nlohmann::json::object({ { "report_figure_options", 5 } })).empty());
    }
}

TEST_CASE("report figure export layout size of a view point", "[view][reportfigure]")
{
    nlohmann::json view_point = histogramViewPoint();

    SECTION("without the field: the default size")
    {
        const auto entries = report_figure::selectRenderEntries(view_point);
        REQUIRE(entries.size() == 1);
        CHECK(entries[ 0 ].export_layout_size == report_figure::ExportLayoutSize);

        QImage image;
        REQUIRE(report_figure::renderExportImage(entries[ 0 ], image));
        CHECK(image.size() == report_figure::ExportLayoutSize * report_figure::ExportScale);
    }

    SECTION("with the field: an export image of that size times the export scale")
    {
        ReportFigureOptions options;
        options.export_layout_size = QSize(600, 450);

        view_point[ ReportFigureOptions::ViewPointFieldExportLayoutSize ] = options.exportLayoutSizeJSON();

        CHECK(ReportFigureOptions::exportLayoutSizeFromViewPoint(view_point) == QSize(600, 450));

        const auto entries = report_figure::selectRenderEntries(view_point);
        REQUIRE(entries.size() == 1);
        CHECK(entries[ 0 ].export_layout_size == QSize(600, 450));

        QImage image;
        REQUIRE(report_figure::renderExportImage(entries[ 0 ], image));
        CHECK(image.size() == QSize(600, 450) * report_figure::ExportScale);
    }

    SECTION("a field that is not readable: the default size")
    {
        view_point[ ReportFigureOptions::ViewPointFieldExportLayoutSize ] = { { "width", 600 } };
        CHECK(ReportFigureOptions::exportLayoutSizeFromViewPoint(view_point) == report_figure::ExportLayoutSize);

        view_point[ ReportFigureOptions::ViewPointFieldExportLayoutSize ] = { { "width", 600 }, { "height", 5 } };
        CHECK(ReportFigureOptions::exportLayoutSizeFromViewPoint(view_point) == report_figure::ExportLayoutSize);
    }
}
