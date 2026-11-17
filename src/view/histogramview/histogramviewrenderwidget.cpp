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

#include "histogramviewrenderwidget.h"
#include "histogramviewchartview.h"
#include "axisticks.h"
#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

#include <QtCharts/QChartView>
#include <QtCharts/QAbstractBarSeries>
#include <QtCharts/QStackedBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QLegend>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLogValueAxis>

#include <QGraphicsLayout>

#include <algorithm>
#include <cmath>

QT_CHARTS_USE_NAMESPACE

const QString HistogramViewRenderWidget::YAxisLabel = "Count";

/**
 */
HistogramViewRenderWidget::HistogramViewRenderWidget(QWidget* parent)
:   ChartRenderWidget(parent)
{
    ViewTool navigate;
    navigate.id       = HG_NAVIGATE_TOOL;
    navigate.cursor   = Qt::ArrowCursor;
    navigate.kind     = ViewTool::Kind::Mode;
    navigate.callback = [ this ] (bool on) { setDragEnabled(on); };

    registerTool(navigate);
}

/**
 */
HistogramViewRenderWidget::~HistogramViewRenderWidget() = default;

/**
 */
void HistogramViewRenderWidget::setDisplayData(std::shared_ptr<const HistogramDisplayData> data,
                                               const HistogramRenderOptions& options)
{
    data_    = std::move(data);
    options_ = options;

    redraw();
}

/**
 */
bool HistogramViewRenderWidget::usesLogScale() const
{
    if (data_ && data_->histogram.useLogScale().has_value())
        return data_->histogram.useLogScale().value();

    return options_.use_log_scale;
}

/**
 */
unsigned int HistogramViewRenderWidget::numStandardBins() const
{
    if (!data_ || !data_->valid || !data_->histogram.hasData())
        return 0;

    const auto& bins = data_->histogram.dataSeries().front().histogram.getBins();

    return (unsigned int)std::count_if(bins.begin(), bins.end(),
                                       [ ] (const RawHistogramBin& bin) { return bin.isStandardBin(); });
}

/**
 */
boost::optional<std::pair<unsigned int, unsigned int>> HistogramViewRenderWidget::binRange(const QRectF& chart_rect) const
{
    const int bins = (int)numStandardBins();

    if (bins <= 0)
        return {};

    auto clampBinToRange = [ & ] (int& bin)
    {
        //bin == binmax is the zero value bin, so clamp to [0, numbins]
        bin = std::max(0, std::min(bin, bins));
    };

    //determine which bin centers are inside the numeric range (bin centers are at full integers, 0, 1, 2, and so on)
    int bin_min = (int)std::ceil(chart_rect.left());
    int bin_max = (int)std::floor(chart_rect.right());

    //clamp bin ids for safety
    clampBinToRange(bin_min);
    clampBinToRange(bin_max);

    logdbg << "bin range [" << bin_min << "," << bin_max << "] bins " << bins;

    //only a valid range
    if (bin_min > bin_max)
        return {};

    return std::make_pair((unsigned int)bin_min, (unsigned int)bin_max);
}

/**
 */
std::unique_ptr<ChartView> HistogramViewRenderWidget::createChartView(bool& has_content)
{
    //check if data is present/valid
    const bool has_data = data_ && data_->valid && data_->histogram.hasData();

    //create chart
    QChart* chart = new QChart();
    chart->setBackgroundRoundness(0);
    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setTitle(data_ ? QString::fromStdString(data_->title) : QString());

    bool use_log_scale = usesLogScale();

    QString x_axis_name = data_ ? QString::fromStdString(data_->x_axis_label) : QString();

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
            //ceil already rounds up to the next decade, a further step would
            //leave a full decade of empty space above the tallest bar
            tmp_chart_y_axis->setRange(0.1, std::pow(10.0, std::ceil(std::log10(max_count))));

            chart_y_axis = tmp_chart_y_axis;
        }
        else
        {
            int max_i = std::max(1, (int)std::ceil(max_count));

            // pick a "nice" tick step from {1, 2, 5} x 10^n targeting ~8 ticks
            int step  = (int)axis_ticks::niceStep(max_i / (double)axis_ticks::TargetTickCountDefault, true);
            int upper = (max_i / step + 1) * step;

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

        chart_y_axis->setTitleText(YAxisLabel);

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

        for (const auto& data_series : data_->histogram.dataSeries())
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
        for (const auto& l : data_->histogram.labels())
            categories << QString::fromStdString(l);

        chart_x_axis->append(categories);

        for (unsigned int v : bin_totals)
            if (v > max_count)
                max_count = v;

        //to generate a safe range we set max count to 1
        max_count = std::max(max_count, (unsigned)1);

        generateYAxis(use_log_scale, max_count);
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
    }

    //update chart
    chart->update();

    //create new chart view
    auto chart_view = std::make_unique<HistogramViewChartView>(this, chart);
    chart_view->setXAxisLabel(x_axis_name);

    // Must run AFTER chart-view construction: the ChartView base constructor
    // auto-sets legend visibility from marker labels (non-empty -> visible),
    // which would otherwise override whatever we set earlier. Hide the
    // legend when there are too many entries to be useful - the layer panel
    // carries per-layer identity in that case.
    {
        const bool show_legend = has_data
                              && chart_series->count() > 0
                              && chart_series->count() <= MaxLegendEntries;
        chart->legend()->setVisible(show_legend);
    }

    //    connect (chart_series_, &QBarSeries::clicked,
    //             chart_view_, &HistogramViewChartView::seriesPressedSlot);
    //    connect (chart_series_, &QBarSeries::released,
    //             chart_view_, &HistogramViewChartView::seriesReleasedSlot);

    has_content = has_data;

    return chart_view;
}

/**
 */
void HistogramViewRenderWidget::viewInfoJSON(nlohmann::json& info) const
{
    auto chart_view = chartView();
    if (!chart_view)
        return;

    nlohmann::json chart_info;

    bool y_axis_log = dynamic_cast<QLogValueAxis*>(chart_view->chart()->axes(Qt::Vertical).first()) != nullptr;

    chart_info[ "x_axis_label" ] = data_ ? data_->x_axis_label : std::string();
    chart_info[ "y_axis_label" ] = chart_view->chart()->axes(Qt::Vertical).first()->titleText().toStdString();
    chart_info[ "y_axis_log"   ] = y_axis_log;
    chart_info[ "num_series"   ] = chart_view->chart()->series().count();

    //the bin labels as the axis really shows them
    nlohmann::json tick_labels = nlohmann::json::array();

    auto axes_x = chart_view->chart()->axes(Qt::Horizontal);
    if (!axes_x.empty())
        if (auto axis_cat = dynamic_cast<QBarCategoryAxis*>(axes_x.first()))
            for (const auto& l : axis_cat->categories())
                tick_labels.push_back(l.toStdString());

    chart_info[ "x_axis_tick_labels" ] = tick_labels;

    nlohmann::json series_infos = nlohmann::json::array();

    //std::vector<size_t> total_counts(histogram_generator_->currentBins(), 0);

    auto series = chart_view->chart()->series();
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
