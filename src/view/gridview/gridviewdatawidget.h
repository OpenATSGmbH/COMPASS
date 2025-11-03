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

#include "variableviewstashdatawidget.h"
#include "grid2dlayer.h"
#include "colormap.h"

#include <memory>

#include <QImage>
#include <QRectF>

#include <boost/optional.hpp>

class GridView;
class GridViewWidget;
class Grid2D;
class ColorLegendWidget;

namespace QtCharts
{
    class GridViewChart;
    class QChart;
}

class QHBoxLayout;
class QPixmap;

enum GridViewDataTool
{
    GV_NAVIGATE_TOOL = 0,
    GV_ZOOM_RECT_TOOL,
    GV_SELECT_TOOL
};

/**
 * @brief Widget with tab containing BufferTableWidgets in ScatterPlotView
 *
 */
class GridViewDataWidget : public VariableViewStashDataWidget
{
    Q_OBJECT
public:
    /// @brief Constructor
    GridViewDataWidget(GridViewWidget* view_widget,
                       QWidget* parent = nullptr, 
                       Qt::WindowFlags f = Qt::WindowFlags());
    /// @brief Destructor
    virtual ~GridViewDataWidget();

    GridViewDataTool selectedTool() const;

    bool hasValidGrid() const;

    boost::optional<QRectF> getXYVariableBounds(bool fix_small_ranges) const;
    boost::optional<std::pair<double, double>> getZVariableBounds(bool fix_small_ranges) const;

    QPixmap renderPixmap();

    const QImage& gridRendering() const { return grid_rendering_; }
    const QRectF& gridBounds() const { return grid_roi_; }
    bool gridIsNorthUp() const { return grid_north_up_; }

    const boost::optional<double>& getGridValueMin() const { return grid_value_min_; }
    const boost::optional<double>& getGridValueMax() const { return grid_value_max_; }

    bool customRangeInvalid() const { return custom_range_invalid_; }

    const GridView* getView() const { return view_; }

    boost::optional<std::pair<QImage, RasterReference>> currentGeoImage() const;
    const ColorLegend& currentLegend() const;

public slots:
    void rectangleSelectedSlot(QPointF p1, QPointF p2);

    void invertSelectionSlot();
    void clearSelectionSlot();

    void resetZoomSlot();

protected:
    virtual void mouseMoveEvent(QMouseEvent* event) override;

    virtual void toolChanged_impl(int mode) override;

    virtual bool postLoadTrigger() override final;
    virtual void resetVariableDisplay() override final;
    virtual DrawState updateVariableDisplay() override final;
    virtual bool updateFromAnnotations() override final;

    virtual void processStash(const VariableViewStash<double>& stash) override final;
    virtual void resetStashDependentData() override final;

    void viewInfoJSON_impl(nlohmann::json& info) const override;

private:
    void resetGrid();
    void resetGridChart();
    void resetGridLayers();

    DrawState updateGridChart();
    void updateRendering();
    DrawState updateChart(QtCharts::QChart* chart);

    GridView* view_   = nullptr;
    
    GridViewDataTool selected_tool_ = GV_NAVIGATE_TOOL;

    QHBoxLayout* main_layout_ = nullptr;

    std::unique_ptr<QtCharts::GridViewChart> grid_chart_;
    ColorLegendWidget* legend_ = nullptr;

    std::unique_ptr<Grid2D>   grid_;
    QImage                    grid_rendering_;
    QRectF                    grid_roi_;
    bool                      grid_north_up_;
    RasterReference           ref_;
    boost::optional<ColorMap> colormap_;
    boost::optional<double>   grid_value_min_;
    boost::optional<double>   grid_value_max_;
    bool                      custom_range_invalid_ = false;

    Grid2DLayers grid_layers_;
    std::string  x_axis_name_;
    std::string  y_axis_name_;
    std::string  title_;
};
