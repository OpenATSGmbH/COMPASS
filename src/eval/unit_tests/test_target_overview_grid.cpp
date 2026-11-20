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

#include "results/reporttablecontent.h"
#include "reporttable.h"
#include "reportfigure.h"
#include "viewpointkeys.h"
#include "viewpointgenerator.h"
#include "grid2d.h"
#include "grid2dlayer.h"
#include "buffer.h"

#include "json.hpp"

#include <memory>
#include <tuple>
#include <vector>

namespace
{
    // latitude, longitude, passed
    typedef std::tuple<double, double, bool> Row;

    ReportTableDefinition positionTable()
    {
        auto def = ReportTableDefinition::record("test_position", "Test Position", { "CAT062" });

        def.addColumn(ReportTableContent::ColumnTstLatitude , PropertyDataType::DOUBLE, "Latitude");
        def.addColumn(ReportTableContent::ColumnTstLongitude, PropertyDataType::DOUBLE, "Longitude");
        def.addColumn("check_passed", PropertyDataType::BOOL, "Check Passed");

        return def;
    }

    std::shared_ptr<Buffer> rows(const ReportTableDefinition& def, const std::vector<Row>& values)
    {
        auto buffer = std::make_shared<Buffer>(def.propertyList(), "test_position");

        for (unsigned int i = 0; i < values.size(); ++i)
        {
            buffer->get<double>(ReportTableContent::ColumnTstLatitude ).set(i, std::get<0>(values[ i ]));
            buffer->get<double>(ReportTableContent::ColumnTstLongitude).set(i, std::get<1>(values[ i ]));
            buffer->get<bool>("check_passed").set(i, std::get<2>(values[ i ]));
        }

        return buffer;
    }

    ReportTableContent::OverviewFigureSettings settings()
    {
        ReportTableContent::OverviewFigureSettings s;
        s.annotation_name = "Evaluation:Test:UTN1";
        s.title           = "Test";
        s.min_region_size = 1.0;
        s.num_cells_x     = 10;
        s.num_cells_y     = 10;

        return s;
    }

    unsigned int countCells(const Grid2DLayer& layer, double value)
    {
        unsigned int n = 0;

        for (Eigen::Index r = 0; r < layer.data.rows(); ++r)
            for (Eigen::Index c = 0; c < layer.data.cols(); ++c)
                if (layer.data(r, c) == value)
                    ++n;

        return n;
    }
}

TEST_CASE("Target overview region - failed rows, at least the minimum size", "[eval][target_overview]")
{
    auto def = positionTable();

    SECTION("failed rows give the region, passed rows are not part of it")
    {
        auto buffer = rows(def, { Row(47.0, 16.0, false), Row(49.0, 19.0, false), Row(40.0, 10.0, true) });

        auto region = ReportTableContent::overviewRegion(def, *buffer, 1.0);

        // x = longitude, y = latitude
        CHECK(region.center().x() == Approx(17.5));
        CHECK(region.center().y() == Approx(48.0));
        CHECK(region.width()      == Approx(3.0));
        CHECK(region.height()     == Approx(2.0));
    }

    SECTION("a single failed row gives the minimum size around it")
    {
        auto buffer = rows(def, { Row(47.0, 16.0, false), Row(47.2, 16.2, true) });

        auto region = ReportTableContent::overviewRegion(def, *buffer, 1.0);

        CHECK(region.center().x() == Approx(16.0));
        CHECK(region.center().y() == Approx(47.0));
        CHECK(region.width()      == Approx(1.0));
        CHECK(region.height()     == Approx(1.0));
    }

    SECTION("all rows passed: all rows give the region")
    {
        auto buffer = rows(def, { Row(47.0, 16.0, true), Row(47.0, 18.0, true) });

        auto region = ReportTableContent::overviewRegion(def, *buffer, 1.0);

        CHECK(region.center().x() == Approx(17.0));
        CHECK(region.width()      == Approx(2.0));
        CHECK(region.height()     == Approx(1.0));
    }

    SECTION("no row with a position: empty region")
    {
        auto buffer = std::make_shared<Buffer>(def.propertyList(), "test_position");

        CHECK(ReportTableContent::overviewRegion(def, *buffer, 1.0).isEmpty());
    }
}

TEST_CASE("Target overview view point - simple figure with a binary grid", "[eval][target_overview]")
{
    auto def = positionTable();

    // cell of (47.0, 16.0): a failed and a passed row, cell of (47.25, 16.25): two passed rows,
    // (50.0, 20.0) is outside the region
    auto buffer = rows(def, { Row(47.0 , 16.0 , false),
                              Row(47.0 , 16.0 , true ),
                              Row(47.25, 16.25, true ),
                              Row(47.27, 16.27, true ),
                              Row(50.0 , 20.0 , true ) });

    auto view_point = ReportTableContent::createOverviewViewPoint(def, *buffer, settings());

    CHECK_FALSE(view_point_load::loadsData(view_point));
    CHECK(view_point.contains(ViewPointGenFeatureGrid::ViewPointFieldNameBackground));

    CHECK(view_point.at(ViewPointKeys::VP_POS_LAT_KEY).get<double>()     == Approx(47.0));
    CHECK(view_point.at(ViewPointKeys::VP_POS_LON_KEY).get<double>()     == Approx(16.0));
    CHECK(view_point.at(ViewPointKeys::VP_POS_WIN_LAT_KEY).get<double>() == Approx(1.0));
    CHECK(view_point.at(ViewPointKeys::VP_POS_WIN_LON_KEY).get<double>() == Approx(1.0));

    auto entries = report_figure::selectRenderEntries(view_point);

    REQUIRE(entries.size() == 1);
    CHECK(entries[ 0 ].kind == report_figure::Kind::Grid);
    CHECK(entries[ 0 ].geographic);

    Grid2DLayer layer;
    REQUIRE(layer.fromJSON(entries[ 0 ].feature->at(ViewPointGenFeatureGrid::FeatureGridFieldNameGrid)));

    CHECK(layer.data.rows() * layer.data.cols() == 100);
    CHECK(countCells(layer, 0.0) == 1); // fail
    CHECK(countCells(layer, 1.0) == 1); // pass
    CHECK(countCells(layer, Grid2D::InvalidValue) == 98);
}

TEST_CASE("Target overview view point - keeps the point annotations of the rows", "[eval][target_overview]")
{
    auto def    = positionTable();
    auto buffer = rows(def, { Row(47.0, 16.0, false), Row(47.3, 16.3, true) });

    auto view_point = ReportTableContent::createOverviewViewPoint(def, *buffer, settings());

    const auto& children = view_point.at(ViewPointKeys::VP_ANNOTATION_KEY).at(0)
                                     .at(ViewPointGenAnnotation::AnnotationFieldAnnotations);

    std::vector<std::string> names;
    for (const auto& c : children)
        names.push_back(c.at(ViewPointGenAnnotation::AnnotationFieldName).get<std::string>());

    // grid first, then the rows in the order of their type
    CHECK(names == std::vector<std::string>({ "Test", "OK", "Errors" }));
}
