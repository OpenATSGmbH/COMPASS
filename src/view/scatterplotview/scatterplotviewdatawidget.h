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
#include "scatterseries.h"
#include "scatterseriesmodel.h"
#include "scatterplotviewchartview.h"

class ScatterPlotView;
class ScatterPlotViewWidget;
class ScatterPlotViewDataSource;

class Buffer;

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

    static const int ConnectLinesDataCountMax = 100000;

    ScatterSeriesModel& dataModel();

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

    ScatterSeriesModel data_model_;

    boost::optional<QRectF> bounds_;
};
