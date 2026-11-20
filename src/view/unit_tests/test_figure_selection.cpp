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
#include "variableview.h"
#include "viewpointgenerator.h"
#include "grid2dlayer.h"

#include "json.hpp"

#include <memory>
#include <string>
#include <vector>

namespace
{
    RawHistogram histogram(uint32_t count)
    {
        RawHistogram h;
        h.addBin(RawHistogramBin(count, "0-10"));
        h.addBin(RawHistogramBin(count + 1, "10-20"));

        return h;
    }

    /// adds an annotation with one histogram feature, the series name is the name of the annotation
    void addHistogram(ViewPointGenVP& vp, const std::string& name, const std::string& plot_group = "")
    {
        vp.annotations().getOrCreateAnnotation(name)->addFeature(
            new ViewPointGenFeatureHistogram(histogram(1), name, Qt::blue, boost::optional<bool>(),
                                             PlotMetadata(name + " Title", "", "Value", "Count", plot_group)));
    }

    /// adds an annotation with one scatter series feature, the series name is the name of the annotation
    void addScatter(ViewPointGenVP& vp, const std::string& name, const std::string& plot_group = "")
    {
        ScatterSeries series;
        series.points = { { 1.0, 2.0 }, { 3.0, 4.0 } };

        vp.annotations().getOrCreateAnnotation(name)->addFeature(
            new ViewPointGenFeatureScatterSeries(series, name, Qt::red, 8.0, boost::optional<bool>(),
                                                 PlotMetadata(name + " Title", "", "X", "Y", plot_group)));
    }

    /// adds an annotation with one grid feature. srs "wgs84": longitude / latitude grid, "": other axes
    void addGrid(ViewPointGenVP& vp, const std::string& name, const std::string& srs, const std::string& plot_group = "")
    {
        Grid2DLayer layer;
        layer.name = name;
        layer.data.resize(1, 2);
        layer.data << 0.0, 1.0;

        layer.ref.srs              = srs;
        layer.ref.img_origin_x     = 16.0;
        layer.ref.img_origin_y     = 2000.0;
        layer.ref.img_pixel_size_x = 0.1;
        layer.ref.img_pixel_size_y = 1000.0;
        layer.ref.is_north_up      = true;

        vp.annotations().getOrCreateAnnotation(name)->addFeature(
            new ViewPointGenFeatureGrid(layer, boost::optional<Grid2DRenderSettings>(),
                                        PlotMetadata(name + " Title", "", "X", "Y", plot_group)));
    }

    void addText(ViewPointGenVP& vp, const std::string& name)
    {
        vp.annotations().getOrCreateAnnotation(name)->addFeature(new ViewPointGenFeatureText("text", 1.0, 2.0));
    }

    nlohmann::json toJSON(const ViewPointGenVP& vp)
    {
        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    /// view point that loads no data
    std::unique_ptr<ViewPointGenVP> viewPoint()
    {
        auto vp = std::make_unique<ViewPointGenVP>("Figure", 0, "Histogram");

        return vp;
    }

    std::vector<std::string> titles(const std::vector<report_figure::RenderEntry>& entries)
    {
        std::vector<std::string> t;
        for (const auto& e : entries)
            t.push_back(e.metadata.title_);

        return t;
    }
}

TEST_CASE("report figure selection", "[view][reportfigure]")
{
    SECTION("histogram only gives one render widget")
    {
        auto vp = viewPoint();
        addHistogram(*vp, "Errors");

        const auto entries = report_figure::selectRenderEntries(toJSON(*vp));

        REQUIRE(entries.size() == 1);

        CHECK(entries[ 0 ].kind == report_figure::Kind::Histogram);
        CHECK(entries[ 0 ].annotation_index == 0);
        CHECK(entries[ 0 ].image_name == "histogram_0");
        CHECK(entries[ 0 ].metadata.title_ == "Errors Title");

        REQUIRE(entries[ 0 ].feature);
        CHECK(entries[ 0 ].feature->at("type") == "histogram");
        CHECK(entries[ 0 ].feature->contains("histogram"));
    }

    SECTION("a view point with load_data gives no render widget")
    {
        ViewPointGenVP vp("Figure", 0, "Histogram");
        addHistogram(vp, "Errors");
        vp.enableDataLoading();

        CHECK(report_figure::selectRenderEntries(toJSON(vp)).empty());
    }

    SECTION("a view point with a null load_data loads no data")
    {
        ViewPointGenVP vp("Figure", 0, "Histogram");
        addHistogram(vp, "Errors");

        nlohmann::json j = toJSON(vp);
        j[ "load_data" ] = nullptr;

        CHECK(report_figure::selectRenderEntries(j).size() == 1);
    }

    SECTION("another feature gives no render widget")
    {
        auto vp = viewPoint();
        addText(*vp, "Label");

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
    }

    SECTION("a histogram plus another feature gives no render widget")
    {
        auto vp = viewPoint();
        addHistogram(*vp, "Errors");
        addText(*vp, "Label");

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
    }

    SECTION("scatter series only gives one render widget")
    {
        auto vp = viewPoint();
        addScatter(*vp, "Bias");

        const auto entries = report_figure::selectRenderEntries(toJSON(*vp));

        REQUIRE(entries.size() == 1);

        CHECK(entries[ 0 ].kind == report_figure::Kind::Scatterplot);
        CHECK(entries[ 0 ].annotation_index == 0);
        CHECK(entries[ 0 ].image_name == "scatterplot_0");
        CHECK(entries[ 0 ].metadata.title_ == "Bias Title");

        REQUIRE(entries[ 0 ].feature);
        CHECK(entries[ 0 ].feature->at("type") == "scatterseries");
        CHECK(entries[ 0 ].feature->contains("scatterseries"));
    }

    SECTION("a scatter series in a view point that loads data gives no render widget")
    {
        ViewPointGenVP vp("Figure", 0, "Scatterplot");
        addScatter(vp, "Bias");
        vp.enableDataLoading();

        CHECK(report_figure::selectRenderEntries(toJSON(vp)).empty());
    }

    SECTION("a scatter series plus another feature gives no render widget")
    {
        auto vp = viewPoint();
        addScatter(*vp, "Bias");
        addText(*vp, "Label");

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
    }

    SECTION("several scatter annotations give the entries in the order of the Scatterplot View")
    {
        auto vp = viewPoint();
        addScatter(*vp, "A", "Group 2");
        addScatter(*vp, "B", "Group 1");
        addScatter(*vp, "C", "Group 2");

        const nlohmann::json j = toJSON(*vp);

        const auto entries = report_figure::selectRenderEntries(j);

        REQUIRE(entries.size() == 3);

        for (unsigned int i = 0; i < 3; ++i)
        {
            CHECK(entries[ i ].kind == report_figure::Kind::Scatterplot);
            CHECK(entries[ i ].annotation_index == i);
            CHECK(entries[ i ].image_name == "scatterplot_" + std::to_string(i));
        }

        std::vector<std::string> view_titles;
        for (const auto& group : VariableView::scanForAnnotations(j, { "scatterseries" }))
            for (const auto& annotation : group.annotations)
                view_titles.push_back(annotation.metadata.title_);

        CHECK(titles(entries) == view_titles);

        REQUIRE(view_titles.size() == 3);
        CHECK(view_titles[ 0 ] == "B Title");
    }

    SECTION("histograms and scatter series give the render widgets of both kinds, the histograms first")
    {
        auto vp = viewPoint();
        addScatter(*vp, "S1");
        addHistogram(*vp, "H1");
        addScatter(*vp, "S2");
        addHistogram(*vp, "H2");

        const auto entries = report_figure::selectRenderEntries(toJSON(*vp));

        REQUIRE(entries.size() == 4);

        CHECK(entries[ 0 ].kind == report_figure::Kind::Histogram);
        CHECK(entries[ 1 ].kind == report_figure::Kind::Histogram);
        CHECK(entries[ 2 ].kind == report_figure::Kind::Scatterplot);
        CHECK(entries[ 3 ].kind == report_figure::Kind::Scatterplot);

        //the index counts per kind, so the names are unique inside the figure
        CHECK(entries[ 0 ].image_name == "histogram_0");
        CHECK(entries[ 1 ].image_name == "histogram_1");
        CHECK(entries[ 2 ].image_name == "scatterplot_0");
        CHECK(entries[ 3 ].image_name == "scatterplot_1");

        CHECK(entries[ 2 ].annotation_index == 0);
        CHECK(entries[ 3 ].annotation_index == 1);

        SECTION("with another feature the figure gives no render widget")
        {
            addText(*vp, "Label");

            CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
        }
    }

    SECTION("a view point without annotations gives no render widget")
    {
        CHECK(report_figure::selectRenderEntries(toJSON(*viewPoint())).empty());
        CHECK(report_figure::selectRenderEntries(nlohmann::json()).empty());
        CHECK(report_figure::selectRenderEntries(nlohmann::json::array()).empty());
    }

    SECTION("three histograms of one annotation give three entries in the stored order")
    {
        auto vp = viewPoint();

        auto anno = vp->annotations().getOrCreateAnnotation("Errors");
        for (const std::string name : { "A", "B", "C" })
            anno->addFeature(new ViewPointGenFeatureHistogram(histogram(1), name, Qt::blue, boost::optional<bool>(),
                                                              PlotMetadata(name, "", "Value", "Count")));

        const auto entries = report_figure::selectRenderEntries(toJSON(*vp));

        REQUIRE(entries.size() == 3);

        for (unsigned int i = 0; i < 3; ++i)
        {
            CHECK(entries[ i ].annotation_index == i);
            CHECK(entries[ i ].image_name == "histogram_" + std::to_string(i));
        }

        CHECK(titles(entries) == std::vector<std::string>({ "A", "B", "C" }));
    }

    SECTION("three histogram annotations give three entries in the order of the views")
    {
        auto vp = viewPoint();
        addHistogram(*vp, "A", "Group 2");
        addHistogram(*vp, "B", "Group 1");
        addHistogram(*vp, "C", "Group 2");

        const nlohmann::json j = toJSON(*vp);

        const auto entries = report_figure::selectRenderEntries(j);

        REQUIRE(entries.size() == 3);

        for (unsigned int i = 0; i < 3; ++i)
        {
            CHECK(entries[ i ].annotation_index == i);
            CHECK(entries[ i ].image_name == "histogram_" + std::to_string(i));
        }

        //the views sort the groups by name and keep the found order inside a group
        std::vector<std::string> view_titles;
        for (const auto& group : VariableView::scanForAnnotations(j, { "histogram" }))
            for (const auto& annotation : group.annotations)
                view_titles.push_back(annotation.metadata.title_);

        CHECK(titles(entries) == view_titles);

        REQUIRE(view_titles.size() == 3);
        CHECK(view_titles[ 0 ] == "B Title");
    }

    SECTION("a grid over other axes gives one render widget")
    {
        auto vp = viewPoint();
        addGrid(*vp, "PD - Altitude/Longitude", "");

        const auto entries = report_figure::selectRenderEntries(toJSON(*vp));

        REQUIRE(entries.size() == 1);

        CHECK(entries[ 0 ].kind == report_figure::Kind::Grid);
        CHECK(entries[ 0 ].annotation_index == 0);
        CHECK(entries[ 0 ].image_name == "grid_0");
        CHECK(entries[ 0 ].metadata.title_ == "PD - Altitude/Longitude Title");

        REQUIRE(entries[ 0 ].feature);
        CHECK(entries[ 0 ].feature->at("type") == "grid");
    }

    SECTION("a geographic grid gives one render widget, without labels and background set")
    {
        auto vp = viewPoint();
        addGrid(*vp, "PD - Horizontal", "wgs84");

        const auto entries = report_figure::selectRenderEntries(toJSON(*vp));

        REQUIRE(entries.size() == 1);
        CHECK(entries[ 0 ].kind == report_figure::Kind::Grid);
        CHECK(entries[ 0 ].image_name == "grid_0");
        CHECK(entries[ 0 ].labels.empty());
        CHECK(!entries[ 0 ].background.has_value());
    }

    SECTION("a geographic grid next to a grid over other axes gives two render widgets")
    {
        auto vp = viewPoint();
        addGrid(*vp, "PD - Altitude/Longitude", "");
        addGrid(*vp, "PD - Horizontal", "wgs84");

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).size() == 2);
    }

    SECTION("a geographic grid plus a text gives the render widget with the label")
    {
        auto vp = viewPoint();
        addGrid(*vp, "PD - Horizontal", "wgs84");
        addText(*vp, "RU position");

        const auto entries = report_figure::selectRenderEntries(toJSON(*vp));

        REQUIRE(entries.size() == 1);
        REQUIRE(entries[ 0 ].labels.size() == 1);
        CHECK(entries[ 0 ].labels[ 0 ].text == "text");
    }

    SECTION("a geographic grid plus points and circles gives the render widget without them")
    {
        auto vp = viewPoint();
        addGrid(*vp, "PD - Horizontal", "wgs84");

        auto marker = vp->annotations().getOrCreateAnnotation("RU position");
        marker->addFeature(new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Circle, 6.0f,
                                                         { Eigen::Vector2d(48.0, 16.0) }));
        marker->addFeature(new ViewPointGenFeatureErrEllipses(3.0f, 32, { Eigen::Vector2d(48.0, 16.0) }, {},
                                                              { Eigen::Vector3d(1000.0, 1000.0, 0.0) }));

        const auto entries = report_figure::selectRenderEntries(toJSON(*vp));

        REQUIRE(entries.size() == 1);
        CHECK(entries[ 0 ].kind == report_figure::Kind::Grid);
        CHECK(entries[ 0 ].labels.empty());
    }

    SECTION("points only give no render widget")
    {
        auto vp = viewPoint();
        vp->annotations().getOrCreateAnnotation("Unassociated Reports")->addFeature(
            new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Square, 6.0f,
                                          { Eigen::Vector2d(48.0, 16.0) }));

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
    }

    SECTION("a geographic image is no overlay")
    {
        auto vp = viewPoint();
        addGrid(*vp, "PD - Horizontal", "wgs84");

        RasterReference ref;
        vp->annotations().getOrCreateAnnotation("Image")->addFeature(
            new ViewPointGenFeatureGeoImage(QImage(2, 2, QImage::Format_ARGB32), ref));

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
    }

    SECTION("the background set of the view point goes to the grid entries")
    {
        auto vp = viewPoint();
        addHistogram(*vp, "H1");
        addGrid(*vp, "PD - Horizontal", "wgs84");

        auto j = toJSON(*vp);

        CHECK(report_figure::hasGeographicGrid(j.at(ViewPointGenVP::ViewPointFieldAnnotations)));

        GridBackgroundSettings background = GridBackgroundSettings::reportDefaults();
        background.hidden_sector_layers = { "TMA" };

        j[ ViewPointGenFeatureGrid::ViewPointFieldNameBackground ] = background.toJSON();

        const auto entries = report_figure::selectRenderEntries(j);

        REQUIRE(entries.size() == 2);
        CHECK(!entries[ 0 ].background.has_value());
        REQUIRE(entries[ 1 ].background.has_value());
        CHECK(*entries[ 1 ].background == background);
    }

    SECTION("only a longitude / latitude grid is a geographic grid")
    {
        auto vp = viewPoint();
        addGrid(*vp, "PD - Altitude/Longitude", "");
        addHistogram(*vp, "H1");

        CHECK(!report_figure::hasGeographicGrid(toJSON(*vp).at(ViewPointGenVP::ViewPointFieldAnnotations)));
    }

    SECTION("a grid plus another feature gives no render widget")
    {
        auto vp = viewPoint();
        addGrid(*vp, "PD - Altitude/Longitude", "");
        addText(*vp, "Label");

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
    }

    SECTION("several grids give the entries in the order of the Grid View, after the other kinds")
    {
        auto vp = viewPoint();
        addGrid(*vp, "G1", "", "Group 2");
        addHistogram(*vp, "H1");
        addGrid(*vp, "G2", "", "Group 1");
        addScatter(*vp, "S1");

        const nlohmann::json j = toJSON(*vp);

        const auto entries = report_figure::selectRenderEntries(j);

        REQUIRE(entries.size() == 4);

        CHECK(entries[ 0 ].kind == report_figure::Kind::Histogram);
        CHECK(entries[ 1 ].kind == report_figure::Kind::Scatterplot);
        CHECK(entries[ 2 ].kind == report_figure::Kind::Grid);
        CHECK(entries[ 3 ].kind == report_figure::Kind::Grid);

        CHECK(entries[ 0 ].image_name == "histogram_0");
        CHECK(entries[ 1 ].image_name == "scatterplot_0");
        CHECK(entries[ 2 ].image_name == "grid_0");
        CHECK(entries[ 3 ].image_name == "grid_1");

        std::vector<std::string> view_titles;
        for (const auto& group : VariableView::scanForAnnotations(j, { "grid" }))
            for (const auto& annotation : group.annotations)
                view_titles.push_back(annotation.metadata.title_);

        REQUIRE(view_titles.size() == 2);
        CHECK(view_titles[ 0 ] == "G2 Title");

        CHECK(entries[ 2 ].metadata.title_ == view_titles[ 0 ]);
        CHECK(entries[ 3 ].metadata.title_ == view_titles[ 1 ]);
    }
}
