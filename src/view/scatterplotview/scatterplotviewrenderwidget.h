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

#include "chartrenderwidget.h"
#include "scatterplotdisplaydata.h"

#include <QRectF>

#include <boost/optional.hpp>

#include <memory>
#include <string>

/// tool ids of the Scatterplot View, the render widget registers the navigate tool and offers the zoom tools
enum ScatterPlotViewTool
{
    SP_NAVIGATE_TOOL = 0,
    SP_ZOOM_RECT_TOOL,
    SP_SELECT_TOOL,
    SP_INVERT_SELECTION_TOOL,
    SP_DELETE_SELECTION_TOOL,
    SP_ZOOM_HOME_TOOL
};

/**
 * Draws a scatterplot chart from a ScatterPlotDisplayData and ScatterPlotRenderOptions.
 * Knows no model and no view.
 *
 * The axis ranges (zoom) are a state of the render widget. It keeps them over a new chart while the
 * domain of the display data (ScatterPlotDisplayData::domain_id) stays the same and the last chart had
 * content. Else the new chart shows the data bounds.
 */
class ScatterPlotViewRenderWidget : public ChartRenderWidget
{
public:
    explicit ScatterPlotViewRenderWidget(QWidget* parent = nullptr);
    virtual ~ScatterPlotViewRenderWidget();

    /// Data and options of the display, one call gives one redraw. An empty pointer gives the empty chart.
    void setDisplayData(std::shared_ptr<const ScatterPlotDisplayData> data, const ScatterPlotRenderOptions& options);

    /// the connection lines of the display data if they are set, else the render option
    bool usesConnectionLines() const;

    void redraw() override;
    /// the image shows the axis ranges of the display, or the data bounds if the widget is not drawn
    bool render(QImage& image, const QSize& size, double scale = 1.0) override;

    // The zoom tools change the axis ranges only, so they are tools of the render widget. Its host
    // registers them where its toolbar shows them, the toolbar lists the tools in the order of
    // registration. A host without a toolbar (report) registers none and gets no shortcuts.

    /// mode tool: the next rectangle of the user becomes the axis ranges
    void registerZoomRectangleTool();
    /// trigger tool: zoomToHome()
    void registerZoomHomeTool();

    /// the rectangle (chart coordinates) becomes the axis ranges
    void zoomToRectangle(const QRectF& chart_rect);
    /// the axis ranges show the data bounds with a border
    void zoomToHome();

    /// The axis ranges of the chart, as a rectangle in chart coordinates. No value without a chart.
    boost::optional<QRectF> axisRanges() const;

    /// adds the axis ranges and the content of the chart as "chart"
    void viewInfoJSON(nlohmann::json& info) const override;

protected:
    std::unique_ptr<ChartView> createChartView(bool& has_content) override;
    void finishChartView(ChartView& chart_view) override;

private:
    /// decides if the chart that is built next keeps the axis ranges of the chart in the widget
    void captureAxisRanges();

    std::shared_ptr<const ScatterPlotDisplayData> data_;
    ScatterPlotRenderOptions                      options_;

    boost::optional<QRectF> kept_axis_ranges_; // for the chart that is built next, no value: data bounds
    std::string             drawn_domain_id_;  // domain of the chart in the widget
};
