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

//#include "global.h"
//#include "nullablevector.h"
//#include "dbcontent/variable/variable.h"
#include "histogramviewchartview.h"
#include "variableviewdatawidget.h"
#include "histogram_raw.h"
//#include "histogram.h"
//#include "results/base.h"

#include <QVariant>

#include <memory>

class HistogramView;
class HistogramViewWidget;
class HistogramViewDataSource;
class QTabWidget;
class QHBoxLayout;
class Buffer;
class DBContent;
class HistogramGenerator;

enum HistogramViewDataTool
{
    HG_DEFAULT_TOOL = 0,
    HG_SELECT_TOOL,
    HG_ZOOM_TOOL
};

/**
 * @brief Widget with tab containing BufferTableWidgets in HistogramView
 *
 */
class HistogramViewDataWidget : public VariableViewDataWidget
{
    Q_OBJECT
public:
    /// @brief Constructor
    HistogramViewDataWidget(HistogramViewWidget* view_widget,
                            QWidget* parent = nullptr, 
                            Qt::WindowFlags f = Qt::WindowFlags());
    /// @brief Destructor
    virtual ~HistogramViewDataWidget();

    unsigned int numBins() const;

    HistogramViewDataTool selectedTool() const;
    QCursor currentCursor() const;

    QPixmap renderPixmap();

    struct ViewInfo
    {
        QString  min;
        QString  max;
        uint32_t out_of_range = 0;
        bool     has_result   = false;
        bool     zoom_active  = false;
    };

    ViewInfo getViewInfo() const;

signals:
    void exportDoneSignal(bool cancelled);

public slots:
    void exportDataSlot(bool overwrite);
    void exportDoneSlot(bool cancelled);

    void resetZoomSlot();

    void rectangleSelectedSlot (unsigned int index1, unsigned int index2);

    void invertSelectionSlot();
    void clearSelectionSlot();

protected:
    virtual void updateDataEvent(bool requires_reset) override final;
    virtual void resetVariableData() override final;
    virtual void resetIntermediateVariableData() override final;
    virtual void resetVariableDisplay() override final;
    virtual void preUpdateVariableDataEvent() override final;
    virtual void postUpdateVariableDataEvent() override final;
    virtual DrawState updateVariableDisplay() override final;
    virtual void updateFromVariables() override final;
    virtual bool updateFromAnnotations() override final;

    void toolChanged_impl(int mode) override;
    void viewInfoJSON_impl(nlohmann::json& info) const override;

    void resetHistogram();
    void compileRawDataFromGenerator();

    DrawState updateChart();
    bool updateChartFromVariable();

    void selectData(unsigned int index1, unsigned int index2);
    void zoomToSubrange(unsigned int index1, unsigned int index2);

    static const unsigned int NumBins     = 20;
    static const int          LabelAngleX = 85;

    QHBoxLayout*             main_layout_{nullptr};
    HistogramView*           view_       {nullptr};
    HistogramViewDataSource* data_source_{nullptr};

    QCursor               current_cursor_{Qt::CrossCursor};
    HistogramViewDataTool selected_tool_ {HG_DEFAULT_TOOL};

    std::unique_ptr<QtCharts::HistogramViewChartView> chart_view_;
    std::unique_ptr<HistogramGenerator>               histogram_generator_;
    RawHistogramCollection                            histogram_raw_;
    std::string                                       x_axis_name_;
    std::string                                       title_;
};
