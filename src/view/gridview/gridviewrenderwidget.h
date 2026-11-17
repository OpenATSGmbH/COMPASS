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
#include "griddisplaydata.h"
#include "gridbackgroundrenderer.h"
#include "colorlegend.h"

#include <QImage>
#include <QRectF>

#include <boost/optional.hpp>

#include <memory>
#include <string>

class ColorLegendWidget;

/// tool ids of the Grid View, the render widget registers the navigate tool and offers the zoom tools
enum GridViewTool
{
    GV_NAVIGATE_TOOL = 0,
    GV_ZOOM_RECT_TOOL,
    GV_SELECT_TOOL,
    GV_ZOOM_HOME_TOOL
};

/**
 * Draws a grid chart from a GridDisplayData and GridRenderOptions: the grid image, the background of a
 * longitude / latitude grid and the color legend right of the chart. Knows no model and no view.
 *
 * The zoom and position the user set (user view) is a state of the render widget. It is kept over a new
 * chart while the domain of the display data (GridDisplayData::domain_id) stays the same, also over a
 * chart without content (e.g. during a load). A chart with content of another domain drops it.
 */
class GridViewRenderWidget : public ChartRenderWidget
{
    Q_OBJECT
public:
    explicit GridViewRenderWidget(QWidget* parent = nullptr);
    virtual ~GridViewRenderWidget();

    /// Data and options of the display, one call gives one redraw. An empty pointer gives the empty chart.
    /// With the data and the color settings of before, only the background and the grid opacity change.
    void setDisplayData(std::shared_ptr<const GridDisplayData> data, const GridRenderOptions& options);

    void redraw() override;
    /// the image shows the chart and the color legend right of it
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
    /// the axis ranges show the grid, or the background extent if the background is active
    void zoomToHome();

    // state of the drawing, read by the chart view

    GridBackgroundRenderer& background() { return background_; }
    const GridBackgroundRenderer& background() const { return background_; }
    /// the background is available (longitude / latitude grid) and switched on
    bool backgroundActive() const;
    /// area for zoom to home and the zoom limits: the grid bounds, without a grid the database extent
    QRectF backgroundExtent() const;

    const QImage& gridRendering() const { return grid_rendering_; }
    const QRectF& gridBounds() const { return grid_roi_; }
    bool gridIsNorthUp() const { return grid_north_up_; }
    /// width of a grid cell in x, 0 if there is no grid
    double gridCellWidth() const;
    unsigned int gridOpacity() const { return options_.grid_opacity; }

    /// zoom / position changed by the user (axis ranges), kept over a new chart of the same domain
    void setUserView(const QRectF& view);
    /// back to the data bounds (zoom to home, database closed)
    void clearUserView();
    bool hasUserView() const { return user_view_.has_value(); }

    // the current rendering

    /// the minimum / maximum of the options are no valid range
    bool customRangeInvalid() const { return custom_range_invalid_; }
    boost::optional<std::pair<QImage, RasterReference>> currentGeoImage() const;
    const ColorLegend& currentLegend() const { return legend_; }

    /// The shown grid layer with the render settings of the current rendering, e.g. to send it to
    /// the Geographic View as a grid feature. The legend fields give the shown legend texts: a
    /// number of decimals (the last digit may differ by rounding), for other texts (e.g. times)
    /// the shown legend as fixed entries.
    boost::optional<std::pair<Grid2DLayer, Grid2DRenderSettings>> currentGrid() const;

    /// The axis ranges of the chart, as a rectangle in chart coordinates. No value without axes.
    boost::optional<QRectF> axisRanges() const;
    /// true if the axis ranges show less than the data bounds
    static bool zoomActive(const QRectF& data_bounds, const QRectF& axis_ranges);

    /// adds the background state, the axis ranges, the content of the chart and the legend
    void viewInfoJSON(nlohmann::json& info) const override;

    /// number of decimals of the legend numbers if no other value is known
    static const int LegendDecimalsDefault;

protected:
    std::unique_ptr<ChartView> createChartView(bool& has_content) override;
    void finishChartView(ChartView& chart_view) override;
    void chartImageLaidOut(ChartView& chart_view) override;

private:
    /// grid image, color map and legend from the display data and the render options
    void updateRendering();
    void updateLegend();
    /// the renderer gets the background of the display data and the options
    void updateBackground();
    /// the shown chart follows the background and the opacity
    void updateBackgroundDisplay();

    std::shared_ptr<const GridDisplayData> data_;
    GridRenderOptions                      options_;

    GridBackgroundRenderer background_;
    ColorLegendWidget*     legend_widget_ = nullptr;

    QImage                    grid_rendering_;
    QRectF                    grid_roi_;
    bool                      grid_north_up_ = true;
    RasterReference           ref_;
    boost::optional<ColorMap> colormap_;
    Grid2DRenderSettings      render_settings_;      // of the current rendering
    ColorLegend               legend_;
    int                       legend_decimals_ = -1; // of the shown legend numbers, -1 if not known
    bool                      custom_range_invalid_ = false;

    boost::optional<QRectF> user_view_;        // see setUserView()
    std::string             user_view_domain_; // domain of the display data the user view belongs to
};
