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
#include "viewlayerscan.h"
#include "histogram_raw.h"
//#include "histogram.h"
//#include "results/base.h"

#include <set>

#include <QVariant>

#include <boost/optional.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
    void exportDoneSignal(bool canceled);

public slots:
    void exportDataSlot(bool overwrite);
    void exportDoneSlot(bool canceled);

    void resetZoomSlot();

    void rectangleSelectedSlot (unsigned int index1, unsigned int index2);

protected:
    // load-time asynchronous recompute: the generator update runs on a worker (it
    // competes with the other views' workers and can exceed the ping budget on the
    // main thread otherwise), the chart update commits on the main thread
    virtual bool postLoadTrigger() override final;

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

    /// layer snapshots from the (cached) layer aggregation of the loaded buffers
    virtual void rebuildLayerTree() override final;
    /// hidden layers are left out => full redraw
    virtual void layerVisibilityChanged() override final;
    /// other layer colors => snapshots and chart
    virtual void colorModeChanged() override final;

    void resetHistogram();
    /// drops the current chart view via deleteLater - see the implementation
    void releaseChartView();
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

    /// Current zoom bin range [first, second], valid only while zoom is
    /// active. Captured on zoomToSubrange and re-applied after a generator
    /// regeneration (e.g. selection-triggered redrawData(true)) so the
    /// user's zoom survives. Cleared on data reload and on resetZoomSlot.
    boost::optional<std::pair<unsigned int, unsigned int>> saved_zoom_range_;

    /// Per-row layer id as a pool index, computed once per data change (on a worker for
    /// asynchronous loads, see updateDataEvent). Two bytes per row instead of a string -
    /// with millions of rows the string form costs one heap allocation each, and made
    /// every generator row do string work. Referenced via closures passed as the
    /// HistogramGeneratorBuffer row filter + group lookup, so this must outlive the
    /// generator.
    view_layer_scan::RowLayerIndex row_layer_index_;

    /// row_layer_ids_ matches the current data (invalidated on data change/clear,
    /// NOT per redraw - the ids only depend on the loaded buffers; the base's layer
    /// aggregation cache follows the same validity)
    bool row_layer_ids_valid_ {false};

private:
    /// layer snapshots + data subtree from layer aggregates (the scan may run on a worker)
    void applyLayerTree(const LayerAggregates& agg);

    /// Populate row_layer_ids_ from current viewData(). Empty string for
    /// rows without ds_id/line_id (unmappable).
    void computeRowLayerIds();
};
