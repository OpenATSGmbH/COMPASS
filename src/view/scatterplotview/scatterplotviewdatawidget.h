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

#include <memory>
#include <set>
#include <vector>

#include "scatterseries.h"
#include "scatterplotviewchartview.h"

class ScatterPlotView;
class ScatterPlotViewWidget;
class ScatterPlotViewDataSource;

class Buffer;
class DBContentRootItem;
class LayerTreeModel;
class ScatterLeafPayload;

namespace QtCharts 
{
    class QChart;
    class QAbstractAxis;
    //class ScatterPlotViewChartView;
}

class QHBoxLayout;

enum ScatterPlotViewDataTool
{
    SP_NAVIGATE_TOOL = 0,
    SP_ZOOM_RECT_TOOL,
    SP_SELECT_TOOL
};

/**
 * @brief Widget with tab containing BufferTableWidgets in ScatterPlotView
 *
 */
class ScatterPlotViewDataWidget : public VariableViewStashDataWidget
{
    Q_OBJECT

public:
    /// @brief Constructor
    ScatterPlotViewDataWidget(ScatterPlotViewWidget* view_widget,
                              QWidget* parent = nullptr, 
                              Qt::WindowFlags f = Qt::WindowFlags());
    /// @brief Destructor
    virtual ~ScatterPlotViewDataWidget();

    ScatterPlotViewDataTool selectedTool() const;

    QPixmap renderPixmap();

    static const int ConnectLinesDataCountMax;

    /// Called by ScatterPlotViewConfigWidget once the LayerPanelWidget is
    /// built. Provides the DBContent root item (owned by the panel's model)
    /// and the layer tree model used for hidden-state round-tripping.
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

    void updateChartSlot();

protected:
    typedef std::unique_ptr<QtCharts::ScatterPlotViewChartView> ChartViewPtr;

    virtual void mouseMoveEvent(QMouseEvent* event) override;

    virtual void toolChanged_impl(int mode) override;

    virtual bool postLoadTrigger() override final;
    virtual void resetVariableDisplay() override final;
    virtual DrawState updateVariableDisplay() override final;
    virtual bool updateFromAnnotations() override final;

    virtual void processStash(const VariableViewStash<double>& stash) override final;
    virtual void resetStashDependentData() override final;

    virtual boost::optional<QRectF> getViewBounds() const override final;

    void viewInfoJSON_impl(nlohmann::json& info) const override;

private:
    void updateDateTimeInfoFromVariables();

    DrawState updateChart();
    DrawState updateDataSeries(QtCharts::QChart* chart);

    void resetSeries();
    void correctSeriesDateTime(ScatterSeriesCollection& collection);
    void setAxisRange(QtCharts::QAbstractAxis* axis, double vmin, double vmax);
    boost::optional<std::pair<double, double>> getAxisRange(QtCharts::QAbstractAxis* axis) const;

    /// Rebuild payloads_ from scatter_series_ and repopulate the DBContent
    /// subtree. Re-applies hidden_series_. Emits layerTreeRebuiltSignal.
    void rebuildLayerTree();

    ScatterPlotView*           view_       {nullptr};
    ScatterPlotViewDataSource* data_source_{nullptr};

    ScatterPlotViewDataTool selected_tool_{SP_NAVIGATE_TOOL};

    QHBoxLayout* main_layout_ {nullptr};
    ChartViewPtr chart_view_  {nullptr};

    ScatterSeriesCollection scatter_series_;
    std::string             x_axis_name_;
    std::string             y_axis_name_;
    std::string             title_;

    bool x_axis_is_datetime_ = false;
    bool y_axis_is_datetime_ = false;

    DBContentRootItem* db_content_root_{nullptr};   // owned by layer panel model
    LayerTreeModel*    layer_model_    {nullptr};   // owned by LayerPanelWidget

    std::vector<std::unique_ptr<ScatterLeafPayload>> payloads_;

    boost::optional<QRectF> bounds_;

    // True iff the last updateChart() drew real content (i.e. axes reflect
    // real data bounds). Gates zoom preservation in updateVariableDisplay() so
    // the initial render - where the prior "chart" has no data and axes are
    // default/empty - does not carry a meaningless range into the first real
    // draw.
    bool prior_draw_had_content_{false};

    std::set<std::string> hidden_series_;  // transient: remember unchecked series across reloads
};
