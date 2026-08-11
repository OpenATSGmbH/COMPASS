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

#include "histogramviewdatawidget.h"
#include "histogramviewwidget.h"
#include "histogramview.h"
#include "compass.h"
#include "buffer.h"
#include "color_provider.h"
#include "data_source.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontentlayer.h"
#include "histogramleafpayload.h"
#include "histogramviewdatasource.h"
#include "histogramviewchartview.h"
#include "layertreemodel.h"
#include "viewlayertreemodel.h"
#include "annotationsrootitem.h"
#include "logger.h"
#include "number.h"
#include "stringconv.h"
#include "evaluationmanager.h"
#include "histogramgenerator.h"
#include "histogramgeneratorbuffer.h"
#include "viewasyncprocessor.h"
#include "viewvariable.h"
#include "property_templates.h"
#include "viewpointgenerator.h"

#include <QHBoxLayout>
#include <QMessageBox>
#include <QTabWidget>

#include <QtCharts/QChartView>
#include <QtCharts/QAbstractBarSeries>
#include <QtCharts/QStackedBarSeries>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QLegend>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLogValueAxis>

#include <QGraphicsLayout>
#include <QShortcut>
#include <QApplication>

#include <algorithm>

QT_CHARTS_USE_NAMESPACE

using namespace EvaluationRequirementResult;
using namespace std;

namespace
{
    /// Group key derived from a full layer id ("<ds_type>:<ds_name>:L<n>:
    /// <dbcontent>") for the given color mode - this is what the chart
    /// actually stacks on (one bar set per group). Matches the semantics of
    /// COMPASS::colorMode():
    ///   0 DSType, 1 DBContent, 2 Data Source, 3 Data Source + Line.
    std::string groupKeyFor(const std::string& layer_id, unsigned int color_mode)
    {
        std::vector<std::string> parts;
        size_t start = 0;
        for (size_t i = 0; i <= layer_id.size(); ++i)
        {
            if (i == layer_id.size() || layer_id[i] == ':')
            {
                parts.push_back(layer_id.substr(start, i - start));
                start = i + 1;
            }
        }
        if (parts.size() != 4)
            return layer_id;

        switch (color_mode)
        {
            case 0: return parts[0];                                        // DSType
            case 1: return parts[3];                                        // DBContent
            case 2: return parts[0] + ":" + parts[1];                       // Data Source
            case 3: return parts[0] + ":" + parts[1] + ":" + parts[2];      // Data Source + Line
            default: return layer_id;
        }
    }
}

/**
 */
HistogramViewDataWidget::HistogramViewDataWidget(HistogramViewWidget* view_widget, 
                                                 QWidget* parent, 
                                                 Qt::WindowFlags f)
:   VariableViewDataWidget(view_widget, view_widget->getView(), parent, f)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    data_source_ = view_->getDataSource();
    traced_assert(data_source_);

    main_layout_ = new QHBoxLayout();
    main_layout_->setMargin(0);

    setLayout(main_layout_);

    x_axis_name_ = view_->variable(0).description();

    updateChart();

    // Annotations subtree of the layer panel mirrors view_->annotations(); the
    // panel is wired via attachLayerPanel and may not yet be present at
    // construction time.
    connect(view_, &VariableView::annotationsChangedSignal, this, [this]()
    {
        if (annotations_root_)
            annotations_root_->update(view_->annotations(),
                                      view_->currentAnnotationGroupIdx(),
                                      view_->currentAnnotationIdx(),
                                      view_);
    });
}

/**
 */
HistogramViewDataWidget::~HistogramViewDataWidget() = default;

/**
 */
void HistogramViewDataWidget::attachLayerPanel(DBContentRootItem* root,
                                               LayerTreeModel* layer_model)
{
    db_content_root_ = root;
    layer_model_     = layer_model;

    if (auto* vlm = dynamic_cast<ViewLayerTreeModel*>(layer_model))
        annotations_root_ = vlm->annotationsRootItem();

    // Visibility toggle -> trigger a full redraw so the histogram is rebuilt
    // without the hidden layers (the hidden set itself is captured/restored
    // by LayerTreeModel).
    connect(layer_model_, &LayerTreeModel::hiddenChangedSignal,
            this, [this]() { redrawData(true); });

    // Color-mode change -> resolveSeriesColor outputs differ -> rebuild
    // payloads (leaf icon + panel group colors) AND redraw the chart so
    // the stacked bar colors match. The histogram values are unchanged -
    // redrawData(true) does recompute, but rebuilding the histogram on a
    // color-mode toggle is cheap compared to a buffer reload.
    connect(&view_->compass(), &COMPASS::colorModeChangedSignal,
            this, [this](unsigned int) {
                rebuildLayerTree();
                redrawData(true);
            });

    rebuildLayerTree();

    // Same for annotations: VariableView may have already populated them via
    // a view point load that fired before attachLayerPanel.
    if (annotations_root_)
        annotations_root_->update(view_->annotations(),
                                  view_->currentAnnotationGroupIdx(),
                                  view_->currentAnnotationIdx(),
                                  view_);
}

/**
 */
void HistogramViewDataWidget::resetHistogram()
{
    histogram_generator_.reset();
    histogram_raw_.clear();

    x_axis_name_ = "";
    title_       = "";
}

/**
 */
void HistogramViewDataWidget::updateDataEvent(bool requires_reset)
{
    //current generator makes no sense any more
    resetHistogram();

    //new data -> old zoom bin indices are meaningless
    saved_zoom_range_.reset();

    //new data -> per-row layer ids are stale
    row_layer_ids_valid_ = false;

    // Offline the per-row scans - the layer aggregation for the panel and the row
    // layer ids the generator consumes - run on a worker; the commit installs them and
    // rebuilds the panel, and the base defers the load-done redraw until then (the
    // redraw then only runs the generator + chart with the ids already computed).
    // Live stays synchronous: the feed mutates the same buffers every tick.
    bool async = view_->compass().appMode() != AppMode::LiveRunning
              && !viewData().empty()
              && db_content_root_ && layer_model_;

    if (async)
    {
        auto scan_input = std::make_shared<view_layer_scan::ScanInput>(
            view_layer_scan::makeScanInput(viewData(), view_->compass()));

        auto agg     = std::make_shared<std::map<std::string, view_layer_scan::LayerAgg>>();
        auto row_ids = std::make_shared<std::map<std::string, std::vector<std::string>>>();

        asyncProcessor().launch("histogram view layer data",
            [ scan_input, agg, row_ids ] ()
            {
                *agg     = view_layer_scan::aggregateLayers(*scan_input);
                *row_ids = view_layer_scan::computeRowLayerIds(*scan_input);
            },
            [ this, agg, row_ids ] ()
            {
                row_layer_ids_       = std::move(*row_ids);
                row_layer_ids_valid_ = true;

                applyLayerTree(*agg);
            });

        return;
    }

    // Buffers have changed; repopulate the layer panel (empty if no data).
    rebuildLayerTree();
}

/**
 */
void HistogramViewDataWidget::resetVariableData()
{
    resetHistogram();

    row_layer_ids_valid_ = false;
}

/**
 * Load-time asynchronous recompute (offline): the generator update runs on a worker,
 * the chart update commits on the main thread once done. Launched after the layer
 * data job of updateDataEvent completed (same serialized processor; its commit
 * installed row_layer_ids_ and the layer panel this reads).
 */
bool HistogramViewDataWidget::postLoadTrigger()
{
    //annotations are display work, live mutates the buffers per tick - default redraw
    if (view_->showsAnnotation())
        return false;

    if (view_->compass().appMode() == AppMode::LiveRunning)
        return false;

    if (viewData().empty())
        return false;

    asyncProcessor().launch("histogram view generator update",
        [ this ] ()
        {
            //the compute phase of redrawData(true)
            clearIntermediateRedrawData();
            preUpdateVariableDataEvent();
            updateFromVariables();
            postUpdateVariableDataEvent();
        },
        [ this ] ()
        {
            setDrawState(updateVariableDisplay());

            emit displayChanged();
        });

    //no default redraw; the base defers dataLoaded until the commit ran
    return true;
}

/**
 */
void HistogramViewDataWidget::resetIntermediateVariableData()
{
    resetHistogram();
}

/**
 */
void HistogramViewDataWidget::resetVariableDisplay()
{
    chart_view_.reset();
}

/**
 */
void HistogramViewDataWidget::preUpdateVariableDataEvent()
{
    //nothing to do
}

/**
 */
void HistogramViewDataWidget::postUpdateVariableDataEvent()
{
    //nothing to do - panel is refreshed from updateDataEvent on data arrival,
    //and the per-redraw filter is rebuilt inside updateFromVariables().
}

/**
 */
ViewDataWidget::DrawState HistogramViewDataWidget::updateVariableDisplay()
{
    return updateChart();
}

/**
 */
bool HistogramViewDataWidget::updateFromAnnotations()
{
    loginf;

    if (!view_->hasCurrentAnnotation())
        return false;

    try
    {
        const auto& anno = view_->currentAnnotation();

        title_       = anno.metadata.title_;
        x_axis_name_ = anno.metadata.xAxisLabel();

        const auto& feature = anno.feature_json;

        if (!feature.is_object())
            throw std::runtime_error("histogram annotation feature is not an object");

        if (!feature.contains(ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram))
            throw std::runtime_error("histogram annotation feature missing '"
                + ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram + "' field");

        if (!histogram_raw_.fromJSON(feature[ ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram ]))
        {
            histogram_raw_.clear();
            throw std::runtime_error("could not read histogram from annotation");
        }

        if (histogram_raw_.useLogScale().has_value())
        {
            view_->useLogScale(histogram_raw_.useLogScale().value(), false);
            view_->updateComponents();
        }
    }
    catch (const std::exception& e)
    {
        histogram_raw_.clear();
        if (view_->hasViewPoint())
            view_->viewPoint().reportError(view_->getName(),
                std::string("annotation error: ") + e.what());
        return false;
    }

    loginf << "done";

    return true;
}

/**
 * Override default workflow, since all dbcontents are handled inside the histogram generator.
 */
void HistogramViewDataWidget::updateFromVariables()
{
    loginf;

    traced_assert(view_->numVariables() == 1);
    traced_assert(view_->variable(0).hasVariable());

    auto& variable = view_->variable(0);

    title_       = "";
    x_axis_name_ = variable.description();

    if (viewData().empty())
        return;

    dbContent::Variable*     data_var = variable.variablePtr();
    dbContent::MetaVariable* meta_var = variable.metaVariablePtr();

    traced_assert(meta_var || data_var);

    auto data_type = meta_var ? meta_var->dataType() : data_var->dataType();

    #define UpdateFunc(PDType, DType, Suffix) \
        histogram_generator_.reset(new HistogramGeneratorBufferT<DType>(&viewData(), data_var, meta_var));

    #define NotFoundFunc                                                                                                                      \
        const std::string msg = "HistogramViewDataWidget: updateVariableData: impossible for property type " + Property::asString(data_type); \
        logerr << msg;                                                                                                                        \
        throw std::runtime_error(msg);

    #define UnsupportedFunc(PDType, DType, Suffix) traced_assert(true);

    SwitchPropertyDataTypeNumeric(data_type, UpdateFunc, UnsupportedFunc, UnsupportedFunc, NotFoundFunc)

    traced_assert(histogram_generator_);

    // Build per-buffer row-layer ids once per data change (covers every row; already
    // computed on a worker for asynchronous loads). The row filter uses the full layer
    // id (panel visibility is per-layer). The layer lookup fed to the generator
    // returns the color-mode *group key* - that way the generator produces one bucket
    // per visible group (e.g. 5 in DSType mode) rather than per full-layer, and no
    // merge step is needed downstream.
    if (!row_layer_ids_valid_)
    {
        computeRowLayerIds();
        row_layer_ids_valid_ = true;
    }

    HistogramGeneratorBuffer* buf_gen =
        dynamic_cast<HistogramGeneratorBuffer*>(histogram_generator_.get());
    traced_assert(buf_gen);

    const unsigned int color_mode = view_->compass().colorMode();

    buf_gen->setRowLayerLookup(
        [this, color_mode](const std::string& dbc, unsigned int i) -> std::string
        {
            auto it = row_layer_ids_.find(dbc);
            if (it == row_layer_ids_.end() || i >= it->second.size())
                return {};
            const std::string& full_id = it->second[i];
            if (full_id.empty())
                return {};
            return groupKeyFor(full_id, color_mode);
        });

    const std::set<std::string> hidden_layer_ids =
        layer_model_ ? layer_model_->storedHiddenIds() : std::set<std::string>{};

    if (!hidden_layer_ids.empty())
    {
        buf_gen->setRowFilter(
            [this, hidden_layer_ids](const std::string& dbc, unsigned int i) -> bool
            {
                auto it = row_layer_ids_.find(dbc);
                if (it == row_layer_ids_.end() || i >= it->second.size())
                    return true;   // no lookup -> allow (can't match a hidden id)
                const std::string& lid = it->second[i];
                if (lid.empty())
                    return true;   // unmappable rows still contribute to scan nulls
                return hidden_layer_ids.count(lid) == 0;
            });
    }

    histogram_generator_->update();
    //histogram_generator_->print();

    // Re-apply any zoom that was active before this regeneration, so a
    // selection-triggered redrawData(true) doesn't reset the user's zoom.
    // Cleared on data reload and on explicit resetZoomSlot, so this only
    // fires for same-data redraws.
    if (saved_zoom_range_.has_value())
        histogram_generator_->zoom(saved_zoom_range_->first,
                                   saved_zoom_range_->second);

    HistogramGeneratorBuffer* generator = dynamic_cast<HistogramGeneratorBuffer*>(histogram_generator_.get());
    traced_assert(generator);

    //variable missing from buffer?
    if (generator->dataNotInBuffer())
        setVariableState(0, VariableState::MissingFromBuffer);

    compileRawDataFromGenerator();

    //add to standard counts
    addNullCount(generator->getResults().buffer_null_count);
    addNanCount (generator->getResults().buffer_nan_count );

    loginf << "done";
}

/**
 * Creates raw histogram data from the current generator's results.
 */
void HistogramViewDataWidget::compileRawDataFromGenerator()
{
    histogram_raw_.clear();

    if (!histogram_generator_ || !variablesOk() || !histogram_generator_->hasValidResult())
        return;

    // Color map keyed by the generator's group key (DSType / DBContent /
    // Data Source / Data Source + Line - determined by color mode). All
    // full-layer payloads that collapse to the same group share the same
    // color by construction, so "first wins" is safe.
    const unsigned int color_mode = view_->compass().colorMode();

    std::map<std::string, QColor> group_colors;
    for (const auto& p : payloads_)
    {
        if (!p) continue;
        group_colors.try_emplace(groupKeyFor(p->persistenceId(), color_mode),
                                 p->color());
    }

    histogram_generator_->getResults().toRaw(histogram_raw_, group_colors, ColorSelected);
}

/**
 */
void HistogramViewDataWidget::toolChanged_impl(int mode)
{
    selected_tool_ = (HistogramViewDataTool)mode;

    if (chart_view_)
        chart_view_->onToolChanged();
}

/**
 */
unsigned int HistogramViewDataWidget::numBins() const
{
    if (histogram_generator_)
        return histogram_generator_->currentBins();

    return 0;
}

/**
 */
HistogramViewDataTool HistogramViewDataWidget::selectedTool() const
{
    return selected_tool_;
}

/**
 */
QCursor HistogramViewDataWidget::currentCursor() const
{
    return current_cursor_;
}

/**
 */
QPixmap HistogramViewDataWidget::renderPixmap()
{
    traced_assert(chart_view_);
    return chart_view_->grab();
}

/**
 */
ViewDataWidget::DrawState HistogramViewDataWidget::updateChart()
{
    loginf;

    //check if data is present/valid
    bool has_data = histogram_raw_.hasData() && (variablesOk() || view_->showsAnnotation());

    chart_view_.reset(nullptr);

    ViewDataWidget::DrawState draw_state = ViewDataWidget::DrawState::NotDrawn;

    //create chart
    QChart* chart = new QChart();
    chart->setBackgroundRoundness(0);
    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setTitle(QString::fromStdString(title_));

    bool use_log_scale = view_->useLogScale();

    QString x_axis_name = QString::fromStdString(x_axis_name_);
    QString y_axis_name = "Count";

    // Legend visibility is decided later based on series count (see
    // MaxLegendEntries below) - at a few layers it's still useful, but with
    // many layers the layer panel is the better reference.
    chart->legend()->setAlignment(Qt::AlignBottom);

    //create stacked bar series (per-layer segments share each bin)
    QStackedBarSeries* chart_series = new QStackedBarSeries();
    chart->addSeries(chart_series);

    //create x axis; use setTitleText(" ") to reserve layout space for the axis
    // title, then render the actual label via ChartView::setXAxisLabel() as a
    // QGraphicsSimpleTextItem - Qt Charts truncates the real title to "..." when
    // tick labels are rotated
    QBarCategoryAxis* chart_x_axis = new QBarCategoryAxis;
    chart_x_axis->setLabelsAngle(LabelAngleX);
    chart_x_axis->setTitleText(" ");
    chart_x_axis->setTitleBrush(Qt::transparent);

    chart->addAxis(chart_x_axis, Qt::AlignBottom);
    chart_series->attachAxis(chart_x_axis);

    //create y axis
    QAbstractAxis* chart_y_axis = nullptr;

    auto generateYAxis = [ & ] (bool log_scale, double max_count)
    {
        if (log_scale)
        {
            QLogValueAxis* tmp_chart_y_axis = new QLogValueAxis;
            tmp_chart_y_axis->setLabelFormat("%g");
            tmp_chart_y_axis->setBase(10.0);
            //tmp_chart_y_axis->setMinorTickCount(10);
            //tmp_chart_y_axis->setMinorTickCount(-1);
            tmp_chart_y_axis-> setRange(10e-2, std::pow(10.0, 1 + std::ceil(std::log10(max_count))));

            chart_y_axis = tmp_chart_y_axis;
        }
        else
        {
            int max_i = std::max(1, (int)std::ceil(max_count));

            // pick a "nice" tick step from {1, 2, 5} x 10^n targeting ~8 ticks
            double raw_step = max_i / 8.0;
            double pow10    = std::pow(10.0, std::floor(std::log10(raw_step)));
            double n        = raw_step / pow10;
            double nice     = (n <= 1.0) ? 1.0 : (n <= 2.0) ? 2.0 : (n <= 5.0) ? 5.0 : 10.0;
            int step        = std::max(1, (int)(nice * pow10));
            int upper       = (max_i / step + 1) * step;

            QValueAxis* tmp_chart_y_axis = new QValueAxis;
            tmp_chart_y_axis->setRange(0, upper);
            tmp_chart_y_axis->setLabelFormat("%d");
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
            tmp_chart_y_axis->setTickType(QValueAxis::TicksDynamic);
            tmp_chart_y_axis->setTickAnchor(0.0);
            tmp_chart_y_axis->setTickInterval(step);
#else
            // Qt < 5.12 lacks dynamic ticks; with range [0, upper] aligned to step,
            // (upper/step)+1 evenly spaced ticks reproduces the desired anchoring.
            tmp_chart_y_axis->setTickCount(upper / step + 1);
#endif
            chart_y_axis = tmp_chart_y_axis;
        }
        traced_assert(chart_y_axis);

        chart_y_axis->setTitleText(y_axis_name);

        chart->addAxis(chart_y_axis, Qt::AlignLeft);
        chart_series->attachAxis(chart_y_axis);
    };

    if (has_data)
    {
        unsigned int max_count = 0;

        // Log scale disallows zero, so zero-count bins need a tiny positive
        // fallback to silence QLogValueAxis warnings. With stacked bars this
        // fallback accumulates across all layers in a bin - at 10e-3 per
        // layer the cumulative (N * 0.01) crossed the log axis min (0.1) and
        // produced a visible "floor". 1e-12 keeps the cumulative well below
        // any realistic axis min even for thousands of layers.
        constexpr double LogZeroFallback = 1e-12;

        auto addCount = [ & ] (QBarSet* set, unsigned int count)
        {
            if (use_log_scale && count == 0)
                *set << LogZeroFallback;
            else
                *set << count;
        };

        //generate a bar set for each layer; track per-bin stack totals so the
        //y axis upper bound covers the summed bar height, not the per-layer max

        std::vector<unsigned int> bin_totals;

        for (const auto& data_series : histogram_raw_.dataSeries())
        {
            const auto& histogram = data_series.histogram;

            const QString bar_legend_name = QString::fromStdString(data_series.name);

            QBarSet* set = new QBarSet(bar_legend_name);

            const auto& bins = histogram.getBins();
            if (bin_totals.size() < bins.size())
                bin_totals.resize(bins.size(), 0);

            for (size_t i = 0; i < bins.size(); ++i)
            {
                addCount(set, bins[i].count);
                bin_totals[i] += bins[i].count;
            }

            set->setColor(data_series.color);
            // Remove the bar outline so adjacent same-colored stack segments
            // (same DSType / DBContent under a coarse color mode) read as one
            // solid block instead of a striped column.
            QPen pen(data_series.color);
            pen.setWidth(0);
            set->setPen(pen);
            chart_series->append(set);
        }

        //create categories
        QStringList categories;
        for (const auto& l : histogram_raw_.labels())
            categories << QString::fromStdString(l);

        chart_x_axis->append(categories);

        for (unsigned int v : bin_totals)
            if (v > max_count)
                max_count = v;

        //to generate a safe range we set max count to 1
        max_count = std::max(max_count, (unsigned)1);

        generateYAxis(use_log_scale, max_count);

        draw_state = ViewDataWidget::DrawState::DrawnContent;
    }
    else
    {
        //no data, generate empty display

        //we need some bogus category in order to make the bar plot work
        chart_x_axis->append("Category"); 

        chart_x_axis->setLabelsVisible(false);
        chart_x_axis->setGridLineVisible(false);
        chart_x_axis->setMinorGridLineVisible(false);

        //just generate linear axis
        generateYAxis(false, 1);
        
        chart_y_axis->setLabelsVisible(false);
        chart_y_axis->setGridLineVisible(false);
        chart_y_axis->setMinorGridLineVisible(false);

        draw_state = ViewDataWidget::DrawState::Drawn;
    }

    //update chart
    chart->update();

    //create new chart view
    chart_view_.reset(new HistogramViewChartView(this, chart));
    chart_view_->setObjectName("chart_view");
    chart_view_->setXAxisLabel(x_axis_name);

    // Must run AFTER chart-view construction: the ChartView base constructor
    // auto-sets legend visibility from marker labels (non-empty -> visible),
    // which would otherwise override whatever we set earlier. Hide the
    // legend when there are too many entries to be useful - the layer panel
    // carries per-layer identity in that case.
    {
        constexpr int MaxLegendEntries = 8;
        const bool show_legend = has_data
                              && chart_series->count() > 0
                              && chart_series->count() <= MaxLegendEntries;
        chart->legend()->setVisible(show_legend);
    }

    //    connect (chart_series_, &QBarSeries::clicked,
    //             chart_view_, &HistogramViewChartView::seriesPressedSlot);
    //    connect (chart_series_, &QBarSeries::released,
    //             chart_view_, &HistogramViewChartView::seriesReleasedSlot);

    connect (chart_view_.get(), &HistogramViewChartView::rectangleSelectedSignal,
            this, &HistogramViewDataWidget::rectangleSelectedSlot, Qt::ConnectionType::QueuedConnection);

    main_layout_->addWidget(chart_view_.get());

    loginf << "done";

    return draw_state;
}

/**
 */
void HistogramViewDataWidget::exportDataSlot(bool overwrite)
{
    logdbg;

}

/**
 */
void HistogramViewDataWidget::exportDoneSlot(bool canceled)
{
    emit exportDoneSignal(canceled);
}

/**
 */
void HistogramViewDataWidget::selectData(unsigned int index1, unsigned int index2)
{
    loginf << "index1 " << index1 << " index2 " << index2;

    if (histogram_generator_)
        histogram_generator_->select(index1, index2);

    //note: triggers a view update, otherwise a manual update would be needed
    emit view_->selectionChangedSignal();
}

/**
 */
void HistogramViewDataWidget::zoomToSubrange(unsigned int index1, unsigned int index2)
{
    if (histogram_generator_)
    {
        //zoom to bin range and refill with data
        histogram_generator_->zoom(index1, index2);

        //remember the zoom so a subsequent recompute (e.g. selection-driven
        //redrawData(true)) can re-apply it after the generator is rebuilt
        saved_zoom_range_ = std::make_pair(index1, index2);

        //update raw data and chart
        compileRawDataFromGenerator();

        redrawData(false);
    }
}

/**
 */
void HistogramViewDataWidget::rectangleSelectedSlot(unsigned int index1, unsigned int index2)
{
    if (selected_tool_ == HG_SELECT_TOOL)
    {
        selectData(index1, index2);
    }
    else if (selected_tool_ == HG_ZOOM_TOOL)
    {
        zoomToSubrange(index1, index2);
    }
    endTool();
}

/**
 */
void HistogramViewDataWidget::invertSelectionSlot()
{
    loginf;

    for (auto& buf_it : viewData())
    {
        traced_assert(buf_it.second->has<bool>(dbcontent_vars::selected_var_.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(dbcontent_vars::selected_var_.name());

        for (unsigned int cnt=0; cnt < buf_it.second->size(); ++cnt)
        {
            if (selected_vec.isNull(cnt))
                selected_vec.set(cnt, true);
            else
                selected_vec.set(cnt, !selected_vec.get(cnt));
        }
    }

    emit view_->selectionChangedSignal();
}

/**
 */
void HistogramViewDataWidget::clearSelectionSlot()
{
    loginf;

    for (auto& buf_it : viewData())
    {
        traced_assert(buf_it.second->has<bool>(dbcontent_vars::selected_var_.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(dbcontent_vars::selected_var_.name());

        for (unsigned int cnt=0; cnt < buf_it.second->size(); ++cnt)
            selected_vec.set(cnt, false);
    }

    emit view_->selectionChangedSignal();
}

/**
 */
void HistogramViewDataWidget::resetZoomSlot()
{
    loginf;

    //reset the preserved zoom so the upcoming redraw uses the full range
    saved_zoom_range_.reset();

    if (histogram_generator_ && histogram_generator_->subRangeActive())
    {
        redrawData(true); //@TODO: maybe redrawData(false) may suffice?
    }
    else if (chart_view_ && chart_view_->chart())
    {
        //no bin zoom active, just reset the chart view
        //@TODO: actually not needed any more
        chart_view_->chart()->zoomReset();
    }
}

/**
 */
HistogramViewDataWidget::ViewInfo HistogramViewDataWidget::getViewInfo() const
{
    if (!histogram_generator_ || !histogram_generator_->hasValidResult())
        return {};

    auto range       = histogram_generator_->currentRangeAsLabels();
    auto zoom_active = histogram_generator_->subRangeActive();

    const auto& results = histogram_generator_->getResults();
    
    ViewInfo vi;
    vi.min          = QString::fromStdString(range.first);
    vi.max          = QString::fromStdString(range.second);
    vi.out_of_range = results.not_inserted_count;
    vi.has_result   = true;
    vi.zoom_active  = zoom_active;

    return vi;
}

/**
 */
void HistogramViewDataWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    bool valid = histogram_generator_ && histogram_generator_->hasValidResult();

    info[ "result_valid" ] = valid;
    
    if (valid)
    {
        auto range       = histogram_generator_->currentRangeAsLabels();
        bool zoom_active = histogram_generator_->subRangeActive();

        const auto& results     = histogram_generator_->getResults();
        const auto& dbc_results = results.content_results;

        auto obtainRanges = [ & ] ()
        {
            if (dbc_results.size() == 0)
                return std::vector<std::string>();
            
            const auto& bins = dbc_results.begin()->second.bins;
            if (bins.size() == 0)
                return std::vector<std::string>();

            std::vector<std::string> ranges;
            for (const auto& bin : bins)
                ranges.push_back(bin.labels.label_min);
            ranges.push_back(bins.rbegin()->labels.label_max);

            return ranges;
        };
        UNUSED_VARIABLE(obtainRanges);

        info[ "result_range_min"      ] = range.first;
        info[ "result_range_max"      ] = range.second;
        info[ "result_zoom_active"    ] = zoom_active;
        info[ "result_num_bins"       ] = histogram_generator_->currentBins();
        info[ "result_oor_count"      ] = results.not_inserted_count;
        info[ "result_null_count"     ] = results.null_count;
        info[ "result_null_sel_count" ] = results.null_selected_count;
        info[ "result_sel_count"      ] = results.selected_count;
        info[ "result_valid_count"    ] = results.valid_count;
        //info[ "result_counts"         ] = results.valid_counts;
        //info[ "result_sel_counts"     ] = results.selected_counts;
        info[ "result_max_count"      ] = results.max_count;
        info[ "result_discrete"       ] = dbc_results.size() > 0 ? dbc_results.begin()->second.bins_are_categories : false;
        //info[ "result_ranges"         ] = obtainRanges();

        if (chart_view_)
        {
            nlohmann::json chart_info;

            bool y_axis_log = dynamic_cast<QLogValueAxis*>(chart_view_->chart()->axes(Qt::Vertical).first()) != nullptr;

            chart_info[ "x_axis_label" ] = x_axis_name_;
            chart_info[ "y_axis_label" ] = chart_view_->chart()->axes(Qt::Vertical).first()->titleText().toStdString();
            chart_info[ "y_axis_log"   ] = y_axis_log;
            chart_info[ "num_series"   ] = chart_view_->chart()->series().count();

            nlohmann::json series_infos = nlohmann::json::array();

            //std::vector<size_t> total_counts(histogram_generator_->currentBins(), 0);

            auto series = chart_view_->chart()->series();
            for (auto s : series)
            {
                QAbstractBarSeries* bar_series = dynamic_cast<QAbstractBarSeries*>(s);
                traced_assert(bar_series);

                nlohmann::json series_info;
                series_info[ "name"     ] = bar_series->name().toStdString();
                series_info[ "num_sets" ] = bar_series->count();

                nlohmann::json bset_infos = nlohmann::json::array();
                for (auto bset : bar_series->barSets())
                {
                    std::vector<int> counts(bset->count());
                    for (int i = 0; i < bset->count(); ++i)
                    {
                        counts[ i ] = (int)(*bset)[ i ];
                        //total_counts.at(i) += counts[ i ];
                    }

                    nlohmann::json bset_info;
                    bset_info[ "name"       ] = bset->label().toStdString();
                    bset_info[ "num_counts" ] = counts.size();
                    bset_info[ "counts"     ] = counts;
                    bset_info[ "color"      ] = bset->color().name().toStdString();

                    bset_infos.push_back(bset_info);
                }

                series_info[ "sets" ] = bset_infos;

                series_infos.push_back(series_info);
            }

            // size_t total_count = 0;
            // for (auto c : total_counts)
            //     total_count += c;

            chart_info[ "series"       ] = series_infos;
            //chart_info[ "total_counts" ] = total_counts;
            //chart_info[ "total_count"  ] = total_count;

            info[ "chart" ] = chart_info;
        }
    }
}

void HistogramViewDataWidget::rebuildLayerTree()
{
    if (!db_content_root_ || !layer_model_)
        return;

    // shared scan helper; on the asynchronous load path the scan runs on a worker
    // instead, see updateDataEvent
    applyLayerTree(view_layer_scan::aggregateLayers(
        view_layer_scan::makeScanInput(viewData(), view_->compass())));
}

/**
 * The tree/payload part of rebuildLayerTree, from precomputed layer aggregates.
 */
void HistogramViewDataWidget::applyLayerTree(const std::map<std::string, view_layer_scan::LayerAgg>& agg)
{
    if (!db_content_root_ || !layer_model_)
        return;

    auto& compass = view_->compass();

    std::vector<std::unique_ptr<HistogramLeafPayload>> new_payloads;
    std::vector<DBContentRootItem::LeafEntry>          entries;
    new_payloads.reserve(agg.size());
    entries.reserve(agg.size());

    for (const auto& kv : agg)
    {
        const std::string&               full_key = kv.first;
        const view_layer_scan::LayerAgg& a        = kv.second;

        QColor color = context::resolveSeriesColor(
            a.ds_type, a.ds_name, a.line_index, a.dbcontent, compass);

        new_payloads.emplace_back(std::make_unique<HistogramLeafPayload>(
            full_key, a.count, color));

        entries.push_back({a.ds_type, a.ds_name, a.line, a.dbcontent,
                           new_payloads.back().get()});
    }

    // refreshSubtree re-applies the stored hidden state to the fresh leaves.
    layer_model_->refreshSubtree(db_content_root_, [&]() {
        payloads_ = std::move(new_payloads);
        return db_content_root_->buildChildrenFrom(entries);
    });
    db_content_root_->recomputeColorsRecursive();

    emit layerTreeRebuiltSignal();
}

void HistogramViewDataWidget::computeRowLayerIds()
{
    // shared scan helper (worker version runs in updateDataEvent's task)
    row_layer_ids_ = view_layer_scan::computeRowLayerIds(
        view_layer_scan::makeScanInput(viewData(), view_->compass()));
}

