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
#include <set>
#include <string>
#include <vector>

#include <QImage>
#include <QRectF>

#include <boost/optional.hpp>

class GridView;
class GridViewWidget;
class Grid2D;
class ColorLegendWidget;
class DBContentRootItem;
class LayerTreeModel;
class GridLeafPayload;

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

    /// Called by GridViewConfigWidget once the LayerPanelWidget is built.
    /// Provides the DBContent root item (owned by the panel's model) and the
    /// layer tree model used for hidden-state round-tripping and recompute
    /// triggering.
    void attachLayerPanel(DBContentRootItem* root, LayerTreeModel* layer_model);

signals:
    /// Emitted after rebuildLayerTree() has replaced the DBContent subtree.
    /// The config widget uses this to re-apply default expansion.
    void layerTreeRebuiltSignal();

public slots:
    void rectangleSelectedSlot(QPointF p1, QPointF p2);

    void invertSelectionSlot();
    void clearSelectionSlot();

    void resetZoomSlot();

    /// Connected to LayerTreeModel::hiddenChangedSignal. Captures the new set
    /// of hidden series and triggers a full recompute of the grid so that only
    /// checked layers contribute.
    void layersChangedSlot();

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

    /// Rebuild grid_ / grid_layers_ / value ranges by iterating the current
    /// stash and accumulating only groups that are NOT in hidden_series_.
    /// Safe to call on a layer toggle - touches neither the stash nor the
    /// layer tree, so DBContentLeafItem payload pointers stay valid.
    void buildGridFromStash();

    /// Rebuild the DBContent subtree in the layer panel from payloads_.
    /// Re-applies hidden_series_ on the new tree. Emits layerTreeRebuiltSignal.
    void rebuildLayerTree();

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

    DBContentRootItem* db_content_root_{nullptr};   // owned by layer panel model
    LayerTreeModel*    layer_model_    {nullptr};   // owned by LayerPanelWidget

    std::vector<std::unique_ptr<GridLeafPayload>> payloads_;

    /// Set of series keys currently hidden by the user. Consulted by
    /// processStash() to skip aggregation of unchecked layers.
    std::set<std::string> hidden_series_;

    /// Re-entry guard for layersChangedSlot. rebuildLayerTree() eventually
    /// calls applyPersistedHiddenIds() which re-emits hiddenChangedSignal -
    /// without this guard, the post-load tree rebuild would recurse through
    /// the slot.
    bool in_layer_recompute_{false};
};
