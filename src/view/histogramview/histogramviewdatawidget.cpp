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
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "histogramviewdatasource.h"
#include "histogramviewchartview.h"
#include "logger.h"
#include "evaluationmanager.h"
#include "histogramgenerator.h"
#include "histogramgeneratorbuffer.h"
#include "viewvariable.h"
#include "property_templates.h"
#include "viewpointgenerator.h"

#include <QHBoxLayout>
#include <QMessageBox>
#include <QTabWidget>

#include <QtCharts/QChartView>
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
}

/**
 */
HistogramViewDataWidget::~HistogramViewDataWidget() = default;

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
}

/**
 */
void HistogramViewDataWidget::resetVariableData()
{
    resetHistogram();
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
    //nothing to do
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

    const auto& anno = view_->currentAnnotation();

    title_       = anno.metadata.title_;
    x_axis_name_ = anno.metadata.xAxisLabel();

    const auto& feature = anno.feature_json;

    if (!feature.is_object() || !feature.contains(ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram))
        return false;

    if (!histogram_raw_.fromJSON(feature[ ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram ]))
    {
        histogram_raw_.clear();
        return false;
    }

    if (histogram_raw_.useLogScale().has_value())
    {
        view_->useLogScale(histogram_raw_.useLogScale().value(), false);
        view_->updateComponents();
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
    
    histogram_generator_->update();
    //histogram_generator_->print();

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

    //convert results to raw data
    histogram_generator_->getResults().toRaw(histogram_raw_, dbContentColors(), ColorSelected);
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

    chart->legend()->setAlignment(Qt::AlignBottom);

    //create bar series
    QBarSeries* chart_series = new QBarSeries();
    chart->addSeries(chart_series);

    //create x axis
    QBarCategoryAxis* chart_x_axis = new QBarCategoryAxis;
    chart_x_axis->setLabelsAngle(LabelAngleX);
    chart_x_axis->setTitleText(x_axis_name);

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
            chart_y_axis = new QValueAxis;
            chart_y_axis->setRange(0, (int)max_count);
        }
        traced_assert(chart_y_axis);

        chart_y_axis->setTitleText(y_axis_name);

        chart->addAxis(chart_y_axis, Qt::AlignLeft);
        chart_series->attachAxis(chart_y_axis);
    };

    if (has_data)
    {
        //data available
        chart->legend()->setVisible(true);

        unsigned int max_count = 0;

        auto addCount = [ & ] (QBarSet* set, unsigned int count) 
        {
            if (count > max_count)
                max_count = count;

            if (use_log_scale && count == 0)
                *set << 10e-3; // Logarithms of zero and negative values are undefined.
            else
                *set << count;
        };

        //generate a bar set for each DBContent

        for (const auto& data_series : histogram_raw_.dataSeries())
        {
            const auto& histogram = data_series.histogram;

            const QString bar_legend_name = QString::fromStdString(data_series.name);

            QBarSet* set = new QBarSet(bar_legend_name);

            for (const auto& bin : histogram.getBins())
                addCount(set, bin.count);
            
            set->setColor(data_series.color);
            chart_series->append(set);
        }

        //create categories
        QStringList categories;
        for (const auto& l : histogram_raw_.labels())
            categories << QString::fromStdString(l);

        chart_x_axis->append(categories);

        //to generate a safe range we set max count to 1
        max_count = std::max(max_count, (unsigned)1);

        generateYAxis(use_log_scale, max_count);

        draw_state = ViewDataWidget::DrawState::DrawnContent;
    }
    else 
    {
        //no data, generate empty display

        chart->legend()->setVisible(false);

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
void HistogramViewDataWidget::exportDoneSlot(bool cancelled)
{
    emit exportDoneSignal(cancelled);
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
        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

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
        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

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

            chart_info[ "x_axis_label" ] = chart_view_->chart()->axes(Qt::Horizontal).first()->titleText().toStdString();
            chart_info[ "y_axis_label" ] = chart_view_->chart()->axes(Qt::Vertical).first()->titleText().toStdString();
            chart_info[ "y_axis_log"   ] = y_axis_log;
            chart_info[ "num_series"   ] = chart_view_->chart()->series().count();

            nlohmann::json series_infos = nlohmann::json::array();

            //std::vector<size_t> total_counts(histogram_generator_->currentBins(), 0);

            auto series = chart_view_->chart()->series();
            for (auto s : series)
            {
                QBarSeries* bar_series = dynamic_cast<QBarSeries*>(s);
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
