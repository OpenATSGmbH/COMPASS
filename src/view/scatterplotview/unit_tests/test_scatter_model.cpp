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
#include "scatterplotviewmodel.h"
#include "plotmetadata.h"
#include "timeconv.h"

#include "json.hpp"

using nlohmann::json;

namespace
{
    /// a scatter series feature as a view point holds it
    json scatterFeature()
    {
        return json::parse(R"({
            "type": "scatterseries",
            "scatterseries": {
                "dataseries": [
                    {
                        "name": "Radar A",
                        "color": "#ff8000",
                        "marker_size": 6.0,
                        "data_type_x": 0,
                        "data_type_y": 0,
                        "data": [ [ 1.0, 10.0 ], [ 2.0, 30.0 ], [ 4.0, 20.0 ] ]
                    },
                    {
                        "name": "Radar B",
                        "color": "#0080ff",
                        "marker_size": 9.0,
                        "data_type_x": 0,
                        "data_type_y": 0,
                        "data": [ [ -3.0, 5.0 ], [ 0.5, 50.0 ] ]
                    }
                ]
            }
        })");
    }

    PlotMetadata metadata()
    {
        return PlotMetadata("Range Bias", "", "Range [m]", "Bias [m]");
    }

    /// milliseconds since epoch of 2023-04-22 05:00:00 UTC
    const double TimestampMs = 1682139600000.0;

    void setAxisDataTypes(json& feature, int data_type_x, int data_type_y)
    {
        for (auto& series : feature[ "scatterseries" ][ "dataseries" ])
        {
            series[ "data_type_x" ] = data_type_x;
            series[ "data_type_y" ] = data_type_y;
        }
    }
}

TEST_CASE("scatter annotation gives the display data", "[view][scatterplot][scattermodel]")
{
    //no model object and no manager
    std::string error;
    auto data = ScatterPlotViewModel::displayDataFromAnnotation(scatterFeature(), metadata(), &error);

    REQUIRE(data);
    CHECK(error.empty());

    CHECK(data->valid);
    CHECK(data->title == "Range Bias");
    CHECK(data->x_axis_label == "Range [m]");
    CHECK(data->y_axis_label == "Bias [m]");

    CHECK(!data->x_axis_is_datetime);
    CHECK(!data->y_axis_is_datetime);

    for (int axis_id = 0; axis_id < 2; ++axis_id)
    {
        CHECK(data->axis_data_type[ axis_id ] == PropertyDataType::DOUBLE);
        CHECK(data->axis_repr[ axis_id ] == dbContent::Representation::STANDARD);
    }

    //no connection lines in the annotation, the render option decides
    CHECK(!data->series.useConnectionLines().has_value());

    CHECK(!data->domain_id.empty());

    const auto& series = data->series.dataSeries();
    REQUIRE(series.size() == 2);
    REQUIRE(series.count("Radar A") == 1);
    REQUIRE(series.count("Radar B") == 1);

    const auto& series_a = series.at("Radar A");
    const auto& series_b = series.at("Radar B");

    CHECK(series_a.name == "Radar A");
    CHECK(series_a.color == QColor("#ff8000"));
    CHECK(series_a.marker_size == Approx(6.0));
    CHECK(series_a.visible);

    CHECK(series_b.color == QColor("#0080ff"));
    CHECK(series_b.marker_size == Approx(9.0));
    CHECK(series_b.visible);

    REQUIRE(series_a.scatter_series.points.size() == 3);
    REQUIRE(series_b.scatter_series.points.size() == 2);

    CHECK(series_a.scatter_series.points[ 1 ].x() == Approx(2.0));
    CHECK(series_a.scatter_series.points[ 1 ].y() == Approx(30.0));
    CHECK(series_b.scatter_series.points[ 0 ].x() == Approx(-3.0));
    CHECK(series_b.scatter_series.points[ 0 ].y() == Approx(5.0));

    SECTION("the bounds cover the points of all series")
    {
        REQUIRE(data->bounds.has_value());

        CHECK(data->bounds->left()   == Approx(-3.0));
        CHECK(data->bounds->right()  == Approx(4.0));
        CHECK(data->bounds->top()    == Approx(5.0));
        CHECK(data->bounds->bottom() == Approx(50.0));
    }

    SECTION("the same annotation gives the same domain")
    {
        auto data_again = ScatterPlotViewModel::displayDataFromAnnotation(scatterFeature(), metadata());

        REQUIRE(data_again);
        CHECK(data_again->domain_id == data->domain_id);
    }
}

TEST_CASE("scatter annotation carries its connection lines", "[view][scatterplot][scattermodel]")
{
    for (bool connection_lines : { true, false })
    {
        INFO("connection lines " << connection_lines);

        auto feature = scatterFeature();
        feature[ "scatterseries" ][ "connection_lines" ] = connection_lines;

        auto data = ScatterPlotViewModel::displayDataFromAnnotation(feature, metadata());

        REQUIRE(data);
        REQUIRE(data->series.useConnectionLines().has_value());
        CHECK(data->series.useConnectionLines().value() == connection_lines);
    }
}

TEST_CASE("scatter annotation with timestamps gives date time axes", "[view][scatterplot][scattermodel]")
{
    auto feature = scatterFeature();

    auto& points = feature[ "scatterseries" ][ "dataseries" ][ 0 ][ "data" ];
    points = json::array({ { TimestampMs, 10.0 }, { TimestampMs + 60000.0, 30.0 } });

    feature[ "scatterseries" ][ "dataseries" ][ 1 ][ "data" ] = json::array({ { TimestampMs + 1000.0, 5.0 } });

    SECTION("timestamps on the x axis")
    {
        setAxisDataTypes(feature, ScatterSeries::DataTypeTimestamp, ScatterSeries::DataTypeFloatingPoint);

        auto data = ScatterPlotViewModel::displayDataFromAnnotation(feature, metadata());
        REQUIRE(data);

        CHECK(data->x_axis_is_datetime);
        CHECK(!data->y_axis_is_datetime);
        CHECK(data->axis_data_type[ 0 ] == PropertyDataType::TIMESTAMP);
        CHECK(data->axis_data_type[ 1 ] == PropertyDataType::DOUBLE);

        //a date time axis of Qt shows local time, the points are corrected for that
        const double expected = (double)Utils::Time::correctLongQtUTC((std::int64_t)TimestampMs);

        const auto& points_a = data->series.dataSeries().at("Radar A").scatter_series.points;
        REQUIRE(points_a.size() == 2);

        CHECK(points_a[ 0 ].x() == Approx(expected));
        CHECK(points_a[ 1 ].x() - points_a[ 0 ].x() == Approx(60000.0));
        CHECK(points_a[ 0 ].y() == Approx(10.0));

        REQUIRE(data->bounds.has_value());
        CHECK(data->bounds->left() == Approx(expected));
        CHECK(data->bounds->width() == Approx(60000.0));
    }

    SECTION("timestamps on the y axis")
    {
        setAxisDataTypes(feature, ScatterSeries::DataTypeFloatingPoint, ScatterSeries::DataTypeTimestamp);

        auto data = ScatterPlotViewModel::displayDataFromAnnotation(feature, metadata());
        REQUIRE(data);

        CHECK(!data->x_axis_is_datetime);
        CHECK(data->y_axis_is_datetime);
        CHECK(data->axis_data_type[ 1 ] == PropertyDataType::TIMESTAMP);
    }

    SECTION("series with different data types give a value axis")
    {
        setAxisDataTypes(feature, ScatterSeries::DataTypeTimestamp, ScatterSeries::DataTypeFloatingPoint);
        feature[ "scatterseries" ][ "dataseries" ][ 1 ][ "data_type_x" ] = ScatterSeries::DataTypeFloatingPoint;

        auto data = ScatterPlotViewModel::displayDataFromAnnotation(feature, metadata());
        REQUIRE(data);

        CHECK(!data->x_axis_is_datetime);
        CHECK(data->series.dataSeries().at("Radar A").scatter_series.points[ 0 ].x() == Approx(TimestampMs));
    }
}

TEST_CASE("scatter annotation in the binary form gives the same points", "[view][scatterplot][scattermodel]")
{
    ScatterSeries points;
    points.points = { { 1.5, 2.5 }, { -4.0, 8.0 }, { 3.0, 0.25 } };

    ScatterSeriesCollection collection;
    collection.addDataSeries(points, "Track", QColor("#00ff00"), 7.0);

    json feature;
    feature[ "type"          ] = "scatterseries";
    feature[ "scatterseries" ] = collection.toJSON(true);

    REQUIRE(feature[ "scatterseries" ][ "dataseries" ][ 0 ].contains("data_raw"));

    auto data = ScatterPlotViewModel::displayDataFromAnnotation(feature, metadata());
    REQUIRE(data);

    const auto& read_points = data->series.dataSeries().at("Track").scatter_series.points;
    REQUIRE(read_points.size() == 3);

    CHECK(read_points[ 1 ].x() == Approx(-4.0));
    CHECK(read_points[ 1 ].y() == Approx(8.0));
    CHECK(read_points[ 2 ].y() == Approx(0.25));
}

TEST_CASE("scatter annotation without axis labels gets the default labels", "[view][scatterplot][scattermodel]")
{
    auto data = ScatterPlotViewModel::displayDataFromAnnotation(scatterFeature(), PlotMetadata());

    REQUIRE(data);
    CHECK(data->x_axis_label == PlotMetadata::DefaultXAxisLabel);
    CHECK(data->y_axis_label == PlotMetadata::DefaultYAxisLabel);
}

TEST_CASE("scatter annotation without points is no valid display", "[view][scatterplot][scattermodel]")
{
    auto feature = scatterFeature();

    for (auto& series : feature[ "scatterseries" ][ "dataseries" ])
        series[ "data" ] = json::array();

    auto data = ScatterPlotViewModel::displayDataFromAnnotation(feature, metadata());

    REQUIRE(data);
    CHECK(!data->valid);
    CHECK(!data->bounds.has_value());
    CHECK(data->title == "Range Bias");
}

TEST_CASE("malformed scatter annotation gives no display data", "[view][scatterplot][scattermodel]")
{
    auto check = [ ] (const json& feature)
    {
        std::string error;
        auto data = ScatterPlotViewModel::displayDataFromAnnotation(feature, metadata(), &error);

        CHECK(!data);
        CHECK(!error.empty());
    };

    SECTION("the feature is no object")
    {
        check(json::array());
        check(json("scatterseries"));
        check(json());
    }

    SECTION("the series field is missing")
    {
        check(json::object({ { "type", "scatterseries" } }));
    }

    SECTION("the series field is no object")
    {
        auto feature = scatterFeature();
        feature[ "scatterseries" ] = 5;

        check(feature);
    }

    SECTION("the series list is missing")
    {
        auto feature = scatterFeature();
        feature[ "scatterseries" ].erase("dataseries");

        check(feature);
    }

    SECTION("a series without data")
    {
        auto feature = scatterFeature();
        feature[ "scatterseries" ][ "dataseries" ][ 1 ].erase("data");

        check(feature);
    }

    SECTION("a point without two values")
    {
        auto feature = scatterFeature();
        feature[ "scatterseries" ][ "dataseries" ][ 0 ][ "data" ][ 1 ] = json::array({ 1.0, 2.0, 3.0 });

        check(feature);
    }

    SECTION("a point that is no number")
    {
        auto feature = scatterFeature();
        feature[ "scatterseries" ][ "dataseries" ][ 0 ][ "data" ][ 1 ] = json::array({ "a", "b" });

        check(feature);
    }

    SECTION("no error text is necessary")
    {
        CHECK(!ScatterPlotViewModel::displayDataFromAnnotation(json::array(), metadata()));
    }
}
