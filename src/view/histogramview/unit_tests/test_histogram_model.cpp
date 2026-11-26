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
#include "histogramviewmodel.h"
#include "histogramviewrenderwidget.h"
#include "plotmetadata.h"

#include "json.hpp"

#include <QtCharts/QChart>

#include "chartview.h"

using nlohmann::json;

namespace
{
    /// a histogram feature as a view point holds it
    json histogramFeature()
    {
        return json::parse(R"({
            "type": "histogram",
            "histogram": {
                "dataseries": [
                    {
                        "name": "Radar A",
                        "color": "#ff8000",
                        "histogram": {
                            "bins": [
                                { "count": 3,  "label": "0-10"  },
                                { "count": 12, "label": "10-20" },
                                { "count": 0,  "label": "20-30" }
                            ]
                        }
                    },
                    {
                        "name": "Radar B",
                        "color": "#0080ff",
                        "histogram": {
                            "bins": [
                                { "count": 1, "label": "0-10"  },
                                { "count": 2, "label": "10-20" },
                                { "count": 7, "label": "20-30" }
                            ]
                        }
                    }
                ]
            }
        })");
    }

    PlotMetadata metadata()
    {
        return PlotMetadata("Range Error", "", "Error [m]", "");
    }
}

TEST_CASE("histogram annotation gives the display data", "[view][histogram][histogrammodel]")
{
    //no model object and no manager
    std::string error;
    auto data = HistogramViewModel::displayDataFromAnnotation(histogramFeature(), metadata(), &error);

    REQUIRE(data);
    CHECK(error.empty());

    CHECK(data->valid);
    CHECK(data->title == "Range Error");
    CHECK(data->x_axis_label == "Error [m]");

    //no log scale in the annotation, the render option decides
    CHECK(!data->histogram.useLogScale().has_value());

    const auto& series = data->histogram.dataSeries();
    REQUIRE(series.size() == 2);

    CHECK(series[ 0 ].name == "Radar A");
    CHECK(series[ 0 ].color == QColor("#ff8000"));
    CHECK(series[ 1 ].name == "Radar B");
    CHECK(series[ 1 ].color == QColor("#0080ff"));

    CHECK(data->histogram.labels() == std::vector<std::string>{ "0-10", "10-20", "20-30" });

    const auto& bins0 = series[ 0 ].histogram.getBins();
    const auto& bins1 = series[ 1 ].histogram.getBins();
    REQUIRE(bins0.size() == 3);
    REQUIRE(bins1.size() == 3);

    CHECK(bins0[ 0 ].count == 3);
    CHECK(bins0[ 1 ].count == 12);
    CHECK(bins0[ 2 ].count == 0);
    CHECK(bins1[ 2 ].count == 7);
}

TEST_CASE("histogram annotation carries its log scale", "[view][histogram][histogrammodel]")
{
    for (bool log_scale : { true, false })
    {
        INFO("log scale " << log_scale);

        auto feature = histogramFeature();
        feature[ "histogram" ][ "log_scale" ] = log_scale;

        auto data = HistogramViewModel::displayDataFromAnnotation(feature, metadata());

        REQUIRE(data);
        REQUIRE(data->histogram.useLogScale().has_value());
        CHECK(data->histogram.useLogScale().value() == log_scale);
    }
}

TEST_CASE("histogram annotation without axis label gets the default label", "[view][histogram][histogrammodel]")
{
    auto data = HistogramViewModel::displayDataFromAnnotation(histogramFeature(), PlotMetadata());

    REQUIRE(data);
    CHECK(data->x_axis_label == PlotMetadata::DefaultXAxisLabel);
}

TEST_CASE("histogram annotation without series is no valid display", "[view][histogram][histogrammodel]")
{
    auto feature = histogramFeature();
    feature[ "histogram" ][ "dataseries" ] = json::array();

    auto data = HistogramViewModel::displayDataFromAnnotation(feature, metadata());

    //well formed, but the chart stays empty
    REQUIRE(data);
    CHECK(!data->valid);
    CHECK(data->title == "Range Error");
}

TEST_CASE("malformed histogram annotation gives no display data", "[view][histogram][histogrammodel]")
{
    auto checkMalformed = [ ] (const json& feature)
    {
        std::string error;
        auto data = HistogramViewModel::displayDataFromAnnotation(feature, metadata(), &error);

        CHECK(!data);
        CHECK(!error.empty());

        //the reason is optional
        CHECK(!HistogramViewModel::displayDataFromAnnotation(feature, metadata()));
    };

    SECTION("feature is not an object")
    {
        checkMalformed(json::array());
        checkMalformed(json("histogram"));
        checkMalformed(json());
    }

    SECTION("no histogram field")
    {
        checkMalformed(json::parse(R"({ "type": "histogram" })"));
    }

    SECTION("histogram is not an object")
    {
        checkMalformed(json::parse(R"({ "type": "histogram", "histogram": [ 1, 2, 3 ] })"));
    }

    SECTION("no data series field")
    {
        checkMalformed(json::parse(R"({ "type": "histogram", "histogram": { "not_dataseries": [] } })"));
    }

    SECTION("data series is not an array")
    {
        checkMalformed(json::parse(R"({ "type": "histogram", "histogram": { "dataseries": "not_an_array" } })"));
    }

    SECTION("bins without count or label")
    {
        auto feature = histogramFeature();
        feature[ "histogram" ][ "dataseries" ][ 0 ][ "histogram" ][ "bins" ] =
            json::parse(R"([ { "label": "0-10" }, { "count": 5 } ])");

        checkMalformed(feature);
    }

    SECTION("a value of the wrong type")
    {
        auto feature = histogramFeature();
        feature[ "histogram" ][ "log_scale" ] = "yes";

        checkMalformed(feature);
    }
}

TEST_CASE("histogram annotation draws without a model object", "[view][histogram][histogrammodel]")
{
    //the path of the report: static function, then the render widget
    auto feature = histogramFeature();
    feature[ "histogram" ][ "log_scale" ] = true;

    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(HistogramViewModel::displayDataFromAnnotation(feature, metadata()), {});

    CHECK(render_widget.hasContent());
    CHECK(render_widget.usesLogScale());

    REQUIRE(render_widget.chartView());
    CHECK(render_widget.chartView()->chart()->title() == "Range Error");
}
