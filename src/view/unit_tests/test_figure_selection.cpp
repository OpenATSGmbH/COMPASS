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
        vp->noDataLoaded(true);

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

    SECTION("a view point that loads data uses the view path")
    {
        ViewPointGenVP vp("Figure", 0, "Histogram");
        addHistogram(vp, "Errors");

        CHECK(report_figure::selectRenderEntries(toJSON(vp)).empty());
    }

    SECTION("a view point with an empty data source list loads no data")
    {
        ViewPointGenVP vp("Figure", 0, "Histogram");
        addHistogram(vp, "Errors");

        nlohmann::json j = toJSON(vp);
        REQUIRE(!report_figure::loadsNoData(j));

        j[ "data_sources" ] = nlohmann::json::array();

        CHECK(report_figure::loadsNoData(j));
        CHECK(report_figure::selectRenderEntries(j).size() == 1);
    }

    SECTION("another feature uses the view path")
    {
        auto vp = viewPoint();
        addText(*vp, "Label");

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
    }

    SECTION("a histogram plus another feature uses the view path")
    {
        auto vp = viewPoint();
        addHistogram(*vp, "Errors");
        addText(*vp, "Label");

        CHECK(report_figure::selectRenderEntries(toJSON(*vp)).empty());
    }

    SECTION("a view point without annotations uses the view path")
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
}
