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
#include "histogramdisplaydata.h"

#include <boost/optional.hpp>

#include <memory>
#include <utility>

/// tool ids of the Histogram View, the render widget registers the navigate tool
enum HistogramViewTool
{
    HG_NAVIGATE_TOOL = 0,
    HG_SELECT_TOOL,
    HG_ZOOM_TOOL,
    HG_INVERT_SELECTION_TOOL,
    HG_DELETE_SELECTION_TOOL,
    HG_ZOOM_HOME_TOOL
};

/**
 * Draws a histogram chart from a HistogramDisplayData and HistogramRenderOptions.
 * Knows no model and no view.
 */
class HistogramViewRenderWidget : public ChartRenderWidget
{
public:
    explicit HistogramViewRenderWidget(QWidget* parent = nullptr);
    virtual ~HistogramViewRenderWidget();

    /// Data and options of the display, one call gives one redraw. An empty pointer gives the empty chart.
    void setDisplayData(std::shared_ptr<const HistogramDisplayData> data, const HistogramRenderOptions& options);

    /// the log scale of the display data if it is set, else the render option
    bool usesLogScale() const;

    /// The bins with their center inside the rectangle (chart coordinates), first and last index.
    /// The index after the last standard bin stands for the bin of the null values.
    /// No value if the rectangle covers no bin center or the chart has no bins.
    boost::optional<std::pair<unsigned int, unsigned int>> binRange(const QRectF& chart_rect) const;

    /// adds the content of the chart as "chart"
    void viewInfoJSON(nlohmann::json& info) const override;

    static const QString  YAxisLabel;
    static constexpr int  LabelAngleX      = 85;
    static constexpr int  MaxLegendEntries = 8;

protected:
    std::unique_ptr<ChartView> createChartView(bool& has_content) override;

private:
    unsigned int numStandardBins() const;

    std::shared_ptr<const HistogramDisplayData> data_;
    HistogramRenderOptions                      options_;
};
