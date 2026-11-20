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

#include "gridviewrenderwidget.h"
#include "gridviewchart.h"
#include "grid2dlayerrenderer.h"
#include "colormapwidget.h"
#include "axisticks.h"
#include "property_templates.h"
#include "files.h"
#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

#include <QtCharts/QChart>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLegend>
#include <QtCharts/QValueAxis>
#include <QtCharts/QCategoryAxis>

#include <QAction>
#include <QBoxLayout>
#include <QFontInfo>
#include <QFontMetricsF>
#include <QGraphicsLayout>
#include <QPainter>

#include <algorithm>
#include <cmath>
#include <set>

const int GridViewRenderWidget::LegendDecimalsDefault = 3;

namespace
{
    /**
     * Fills the axis with tick positions and labels for its current range. Called once on creation
     * and on every range change, since a zoom needs ticks of its own.
     */
    void updateAxisTicks(QtCharts::QCategoryAxis* axis, PropertyDataType dtype, dbContent::Representation repr)
    {
        traced_assert(axis);

        //drop the ticks of the previous range, categories are keyed by their label
        const auto old_labels = axis->categoriesLabels();
        for (const auto& l : old_labels)
            axis->remove(l);

        const double vmin = axis->min();
        const double vmax = axis->max();

        auto ticks       = axis_ticks::generate(vmin, vmax, dtype, repr);
        auto tick_labels = axis_ticks::labels(ticks, dtype, repr);

        traced_assert(ticks.values.size() == tick_labels.size());

        //a category must end above the start value, so keep the start just below
        const double span  = std::max(std::fabs(vmax - vmin), 1e-9);
        const double start = vmin - span * 1e-9;

        axis->setStartValue(start);

        std::set<QString> used;

        for (size_t i = 0; i < ticks.values.size(); ++i)
        {
            if (ticks.values[ i ] <= start)
                continue;

            QString label = QString::fromStdString(tick_labels[ i ]);

            //Qt keys categories by label and drops a duplicate without a word
            if (used.count(label))
                continue;

            used.insert(label);

            axis->append(label, ticks.values[ i ]);
        }
    }

    /**
     * The axes of the chart, with the value range of its series, which the grid image is positioned against.
     */
    void createAxes(QtCharts::QChart* chart, const GridDisplayData* data)
    {
        //createDefaultAxes puts the value range on the axes, which the grid
        //image is positioned against. Replacing an axis afterwards would lose
        //that range, so the ranges are carried over by hand.
        chart->createDefaultAxes();

        traced_assert(chart->axes(Qt::Horizontal).size() == 1);
        traced_assert(chart->axes(Qt::Vertical).size() == 1);

        auto replaceByCategoryAxis = [ & ] (int axis_id, Qt::Alignment alignment)
        {
            const auto orientation = (alignment == Qt::AlignBottom) ? Qt::Horizontal : Qt::Vertical;

            auto axis_old = dynamic_cast<QtCharts::QValueAxis*>(chart->axes(orientation).at(0));
            if (!axis_old)
                return;

            const auto dtype = data ? data->axis_data_type[ axis_id ] : PropertyDataType::DOUBLE;
            const auto repr  = data ? data->axis_repr[ axis_id ]      : dbContent::Representation::STANDARD;

            //a plain value axis is right for a plain floating point value
            if (!axis_ticks::needsCustomLabels(dtype, repr))
                return;

            const double vmin = axis_old->min();
            const double vmax = axis_old->max();

            //QCategoryAxis derives from QValueAxis, so the value space the grid
            //image is drawn against stays exactly the same
            auto axis = new QtCharts::QCategoryAxis;
            axis->setLabelsPosition(QtCharts::QCategoryAxis::AxisLabelsPositionOnValue);

            chart->removeAxis(axis_old);
            chart->addAxis(axis, alignment);

            for (auto series : chart->series())
                series->attachAxis(axis);

            axis->setRange(vmin, vmax);

            QObject::connect(axis, &QtCharts::QCategoryAxis::rangeChanged, axis,
                             [ chart, axis, dtype, repr ] (qreal, qreal)
                             {
                                 updateAxisTicks(axis, dtype, repr);

                                 //the labels changed, so the room they need changed too
                                 ::ChartView::reserveHorizontalLabelMargins(chart);
                             });

            updateAxisTicks(axis, dtype, repr);
        };

        replaceByCategoryAxis(0, Qt::AlignBottom);
        replaceByCategoryAxis(1, Qt::AlignLeft);

        chart->axes(Qt::Horizontal).at(0)->setTitleText(data ? QString::fromStdString(data->x_axis_label) : QString());
        chart->axes(Qt::Vertical  ).at(0)->setTitleText(data ? QString::fromStdString(data->y_axis_label) : QString());

        ::ChartView::reserveHorizontalLabelMargins(chart);
    }

    /// a series with points at the corners of the area, gives the axes their value range
    QtCharts::QScatterSeries* areaSeries(const QRectF& area)
    {
        auto series = new QtCharts::QScatterSeries;

        if (!area.isEmpty())
        {
            series->setMarkerSize(0.1);
            series->setColor(Qt::white);

            series->append(area.topLeft());
            series->append(area.topRight());
            series->append(area.bottomRight());
            series->append(area.bottomLeft());
        }

        return series;
    }

    /// narrow: the narrow chart margin on each side, else the margins of the style (-1)
    void setLegendMargins(ColorLegendWidget& legend, bool narrow)
    {
        const int margin = narrow ? ChartView::NarrowMargin : -1;

        legend.layout()->setContentsMargins(margin, margin, margin, margin);
    }
}

/**
 */
GridViewRenderWidget::GridViewRenderWidget(QWidget* parent)
:   ChartRenderWidget(parent)
{
    legend_widget_ = new ColorLegendWidget(this);
    legend_widget_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    legend_widget_->setVisible(false);

    //right of the chart view, which is inserted before it
    contentLayout()->addWidget(legend_widget_);

    //e.g. a map image arrived
    connect(&background_, &GridBackgroundRenderer::updateSignal, this, [ this ] ()
    {
        if (chartView())
            chartView()->viewport()->update();
    });

    ViewTool navigate;
    navigate.id       = GV_NAVIGATE_TOOL;
    navigate.name     = "Navigate";
    navigate.cursor   = Qt::ArrowCursor;
    navigate.kind     = ViewTool::Kind::Mode;
    navigate.callback = [ this ] (bool on) { setDragEnabled(on); };

    registerTool(navigate);
}

/**
 */
GridViewRenderWidget::~GridViewRenderWidget() = default;

/**
 */
void GridViewRenderWidget::registerZoomRectangleTool()
{
    ViewTool zoom;
    zoom.id       = GV_ZOOM_RECT_TOOL;
    zoom.name     = "Zoom to Rectangle";
    zoom.key      = Qt::Key_R;
    zoom.icon     = Utils::Files::IconProvider::getIcon("zoom_select_action.png");
    zoom.cursor   = Qt::CrossCursor;
    zoom.kind     = ViewTool::Kind::Mode;
    zoom.update   = [ this ] (QAction* a) { a->setEnabled(isDrawn()); };
    zoom.callback = [ this ] (bool on)
    {
        if (on)
            requestRectangle([ this ] (const QRectF& rect) { zoomToRectangle(rect); });
    };

    registerTool(zoom);
}

/**
 */
void GridViewRenderWidget::registerZoomHomeTool()
{
    ViewTool zoom_home;
    zoom_home.id       = GV_ZOOM_HOME_TOOL;
    zoom_home.name     = "Zoom to Home";
    zoom_home.key      = Qt::Key_Space;
    zoom_home.icon     = Utils::Files::IconProvider::getIcon("zoom_home.png");
    zoom_home.kind     = ViewTool::Kind::Trigger;
    zoom_home.callback = [ this ] (bool) { zoomToHome(); };

    registerTool(zoom_home);
}

/**
 * Background and grid opacity change no grid image and no axis, so they only repaint the chart, as the
 * user changes them often and the chart keeps its look meanwhile.
 */
void GridViewRenderWidget::setDisplayData(std::shared_ptr<const GridDisplayData> data, const GridRenderOptions& options)
{
    const bool same_rendering = isDrawn() && data == data_ &&
                                options.render_settings.toJSON() == options_.render_settings.toJSON();

    data_    = std::move(data);
    options_ = options;

    updateBackground();

    if (same_rendering)
    {
        updateBackgroundDisplay();
        return;
    }

    updateRendering();
    redraw();
}

/**
 */
void GridViewRenderWidget::redraw()
{
    ChartRenderWidget::redraw();

    if (imageOnly())
        return;

    //the legend of the color map, hidden without
    legend_widget_->setLegend(legend_);
    legend_widget_->setVisible(colormap_.has_value());

    setLegendMargins(*legend_widget_, narrowMargins());
    contentLayout()->setSpacing(narrowMargins() ? 0 : -1);
}

/**
 */
void GridViewRenderWidget::updateBackground()
{
    background_.setAvailable(data_ && data_->is_geographic);
    background_.setSettings(data_ && data_->background.has_value() ? data_->background.value() : options_.background);

    //shares the lifetime of the display data
    background_.setSectors(data_ ? std::shared_ptr<const std::vector<GridSectorLine>>(data_, &data_->sectors)
                                 : std::shared_ptr<const std::vector<GridSectorLine>>());
}

/**
 */
void GridViewRenderWidget::updateBackgroundDisplay()
{
    auto chart_view = dynamic_cast<QtCharts::GridViewChart*>(chartView());
    if (!chart_view)
        return;

    //aspect ratio and limits depend on the background
    chart_view->applyViewLimits();
    chart_view->viewport()->update();
}

/**
 */
bool GridViewRenderWidget::backgroundActive() const
{
    return background_.active();
}

/**
 */
QRectF GridViewRenderWidget::backgroundExtent() const
{
    if (!grid_roi_.isEmpty())
        return grid_roi_;

    return data_ ? data_->database_extent : QRectF();
}

/**
 */
double GridViewRenderWidget::gridCellWidth() const
{
    if (!data_ || !data_->layer.has_value() || grid_roi_.isEmpty())
        return 0.0;

    const auto cols = data_->layer->data.cols();

    return cols > 0 ? grid_roi_.width() / cols : 0.0;
}

/**
 * Fonts with a pixel size and without hinting, so the label looks the same at every display scaling.
 */
void GridViewRenderWidget::paintLabels(QPainter& painter, const QRectF& plot_rect, const QRectF& lonlat) const
{
    if (!data_ || data_->labels.empty() || plot_rect.isEmpty() || lonlat.width() <= 0.0 || lonlat.height() <= 0.0)
        return;

    //distance of the text from its position
    const double offset = 3.0;

    painter.save();
    painter.setClipRect(plot_rect);

    for (const auto& label : data_->labels)
    {
        if (!lonlat.contains(label.position))
            continue;

        const QPointF p(plot_rect.left() + (label.position.x() - lonlat.left()) / lonlat.width() * plot_rect.width(),
                        plot_rect.top() + (lonlat.bottom() - label.position.y()) / lonlat.height() * plot_rect.height());

        QFont font = painter.font();
        font.setPixelSize(std::max(1, (int)std::lround(label.font_size)));
        font.setHintingPreference(QFont::PreferNoHinting);

        painter.setFont(font);
        painter.setPen(label.color);

        const QString text = QString::fromStdString(label.text);
        const QSizeF  size = QFontMetricsF(font, painter.device()).boundingRect(text).size();

        const bool left = label.direction == GridTextLabel::Direction::LeftUp ||
                          label.direction == GridTextLabel::Direction::LeftDown;
        const bool down = label.direction == GridTextLabel::Direction::RightDown ||
                          label.direction == GridTextLabel::Direction::LeftDown;

        const QRectF rect(left ? p.x() - offset - size.width() : p.x() + offset,
                          down ? p.y() + offset : p.y() - offset - size.height(),
                          size.width(),
                          size.height());

        painter.drawText(rect, Qt::AlignLeft | Qt::AlignVCenter | Qt::TextDontClip, text);
    }

    painter.restore();
}

/**
 */
void GridViewRenderWidget::setUserView(const QRectF& view)
{
    user_view_        = view;
    user_view_domain_ = data_ ? data_->domain_id : std::string();
}

/**
 */
void GridViewRenderWidget::clearUserView()
{
    user_view_.reset();
    user_view_domain_.clear();
}

/**
 */
void GridViewRenderWidget::zoomToRectangle(const QRectF& chart_rect)
{
    loginf;

    auto chart_view = dynamic_cast<QtCharts::GridViewChart*>(chartView());
    if (!chart_view || !chart_view->chart())
        return;

    //TODO: prevent from going nuts when zero rect is passed!
    chart_view->zoom(chart_rect.topLeft(), chart_rect.bottomRight());
}

/**
 */
void GridViewRenderWidget::zoomToHome()
{
    loginf;

    if (auto chart_view = dynamic_cast<QtCharts::GridViewChart*>(chartView()))
        chart_view->resetZoom();
}

/**
 */
void GridViewRenderWidget::updateRendering()
{
    loginf << "rendering";

    custom_range_invalid_ = false;
    colormap_.reset();
    render_settings_ = {};
    grid_rendering_  = QImage();
    grid_roi_        = QRectF();
    grid_north_up_   = true;
    ref_             = RasterReference();

    //no valid grid no rendering
    if (!data_ || !data_->layer.has_value() || data_->layer->data.cols() < 1 || data_->layer->data.rows() < 1)
    {
        updateLegend();
        return;
    }

    const auto& layer      = data_->layer.value();
    const auto& anno_rs    = data_->render_settings;
    const auto& options_rs = options_.render_settings;
    const auto  data_range = layer.range();

    loginf << "input range "
           << "min = "
           << (data_range.has_value() ? std::to_string(data_range->first) : "undef") << " "
           << "max = "
           << (data_range.has_value() ? std::to_string(data_range->second) : "undef");

    Grid2DRenderSettings render_settings;

    //if there is a valid range for the layer values we can do a little extra stuff
    if (data_range.has_value())
    {
        //combine data range with user override range
        std::pair<double, double> range = data_range.value();

        //annotation-provided min/max take priority over the min/max of the options
        boost::optional<double> ann_min, ann_max;
        if (anno_rs.has_value())
        {
            ann_min = anno_rs->min_value;
            ann_max = anno_rs->max_value;
        }

        auto vmin = ann_min.has_value() ? ann_min : options_rs.min_value;
        auto vmax = ann_max.has_value() ? ann_max : options_rs.max_value;

        if (vmin.has_value() || vmax.has_value())
        {
            if (vmin.has_value() && vmax.has_value() && vmin.value() <= vmax.value())
            {
                range = std::pair<double, double>(vmin.value(), vmax.value());
            }
            else if (vmin.has_value() && !vmax.has_value())
            {
                range.first = vmin.value();

                if (vmin.value() > data_range->second)
                    range.second = vmin.value();
            }
            else if (vmax.has_value() && !vmin.has_value())
            {
                range.second = vmax.value();

                if (vmax.value() < data_range->first)
                    range.first = vmax.value();
            }
            else
            {
                custom_range_invalid_ = true;
            }
        }

        loginf << "combined range min = " << range.first << " max = " << range.second;

        const bool annotation_color_map = anno_rs.has_value() && anno_rs->color_map.valid();

        if (annotation_color_map)
        {
            //annotation: its color map as suggested by the producer (colors, type, log scale), with the
            //combined range
            render_settings.color_map = anno_rs->color_map;
            render_settings.color_map.setValueRange(range);
        }
        else
        {
            //color scale and steps from the options
            const auto   color_scale = options_rs.color_map.colorScale();
            const size_t num_steps   = property_templates::suggestedNumColorSteps(
                data_->legend_data_type, range.first, range.second, options_rs.color_map.colorSteps());

            loginf << "suggested color steps = " << num_steps;

            //create color map
            if (num_steps == 1)
            {
                //single value
                render_settings.color_map.create(color_scale, 1, ColorMap::Type::LinearSamples, range);
            }
            else if (num_steps == 2)
            {
                //binary
                render_settings.color_map.create(color_scale, 2, ColorMap::Type::Binary, range);
            }
            else if (num_steps > 0)
            {
                //multi-value
                render_settings.color_map.create(color_scale, num_steps, ColorMap::Type::LinearSamples, range);
            }

            render_settings.color_map.setLogScale(options_rs.color_map.logScale());
        }

        colormap_ = render_settings.color_map;
    }

    render_settings_ = render_settings;

    //render grid
    auto rendering = Grid2DLayerRenderer::render(layer, render_settings);

    grid_rendering_ = rendering.first;
    grid_roi_       = rendering.second.getROI(grid_rendering_.width(), grid_rendering_.height());
    grid_north_up_  = rendering.second.is_north_up;
    ref_            = rendering.second;

    updateLegend();
}

/**
 * The legend labels read like an axis: the representation decides the form, the value range decides how
 * many decimals carry information.
 */
void GridViewRenderWidget::updateLegend()
{
    legend_.clear();
    legend_decimals_ = -1;

    if (!colormap_.has_value())
        return;

    const auto dtype = data_->legend_data_type;
    const auto repr  = data_->legend_repr;

    axis_ticks::LabelStyle style;
    style.decimals = LegendDecimalsDefault;

    const auto& range = colormap_->valueRange();
    if (range.has_value() && range->second > range->first)
        style.decimals = axis_ticks::generate(range->first,
                                              range->second,
                                              dtype,
                                              repr,
                                              (int)std::max((size_t)2, colormap_->numColors())).style.decimals;

    legend_decimals_ = style.decimals;

    auto decoratorFunc = [ = ] (double v)
    {
        return axis_ticks::label(v, dtype, repr, style);
    };

    //legend from the render settings: color map with the active range, legend fields of the
    //annotation if set (order, precision, unit)
    Grid2DRenderSettings legend_settings;
    legend_settings.color_map = colormap_.value();

    if (data_->render_settings.has_value())
    {
        legend_settings.legend_order     = data_->render_settings->legend_order;
        legend_settings.legend_precision = data_->render_settings->legend_precision;
        legend_settings.legend_unit      = data_->render_settings->legend_unit;
        legend_settings.legend_entries   = data_->render_settings->legend_entries;
    }

    legend_ = legend_settings.legend(false, false, decoratorFunc);
}

/**
 */
std::unique_ptr<ChartView> GridViewRenderWidget::createChartView(bool& has_content)
{
    const bool has_valid_grid_data = data_ && data_->layer.has_value() &&
                                     !grid_rendering_.isNull() &&
                                     !grid_roi_.isEmpty();

    const bool has_data = has_valid_grid_data && data_->valid;

    QtCharts::QChart* chart = new QtCharts::QChart();
    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setBackgroundRoundness(0);
    chart->setDropShadowEnabled(false);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->legend()->setVisible(false);
    chart->setTitle(data_ ? QString::fromStdString(data_->title) : QString());

    if (has_data)
    {
        chart->addSeries(areaSeries(grid_roi_));

        createAxes(chart, data_.get());
    }
    else
    {
        //no data -> generate default empty layout. Longitude / latitude => the background shows the
        //position extent of the database
        const QRectF db_extent = data_ ? data_->database_extent : QRectF();

        chart->addSeries(areaSeries(db_extent));

        createAxes(chart, data_.get());

        //with the database extent the axes keep their labels
        if (db_extent.isEmpty())
        {
            chart->axes(Qt::Horizontal).at(0)->setLabelsVisible(false);
            chart->axes(Qt::Horizontal).at(0)->setGridLineVisible(false);
            chart->axes(Qt::Horizontal).at(0)->setMinorGridLineVisible(false);

            chart->axes(Qt::Vertical).at(0)->setLabelsVisible(false);
            chart->axes(Qt::Vertical).at(0)->setGridLineVisible(false);
            chart->axes(Qt::Vertical).at(0)->setMinorGridLineVisible(false);
        }
    }

    chart->update();

    auto chart_view = std::make_unique<QtCharts::GridViewChart>(this, chart);
    chart_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    //the zoom / position of the user belongs to a domain, another domain with content drops it
    const std::string domain = data_ ? data_->domain_id : std::string();

    if (user_view_.has_value() && domain != user_view_domain_ && has_data)
        clearUserView();

    if (user_view_.has_value() && domain == user_view_domain_)
        chart_view->setView(user_view_.value());

    has_content = has_data;

    return chart_view;
}

/**
 * The margins for the outermost tick labels follow the label widths, so they are computed again
 * with the final fonts.
 */
void GridViewRenderWidget::finishChartView(ChartView& chart_view)
{
    ChartView::reserveHorizontalLabelMargins(chart_view.chart());
}

/**
 * On the screen the limits follow a change of the plot area later, an image is drawn at once. The map
 * tiles of the viewport of the image are loaded before.
 */
void GridViewRenderWidget::chartImageLaidOut(ChartView& chart_view)
{
    auto grid_chart = dynamic_cast<QtCharts::GridViewChart*>(&chart_view);
    if (!grid_chart || !backgroundActive())
        return;

    grid_chart->applyViewLimits();

    background_.prepareExport(grid_chart->viewLonLat(), grid_chart->chart()->plotArea().size().toSize());
}

/**
 * The legend is a ColorLegendWidget as next to the chart on the screen, with fonts and palette that do
 * not depend on the screen.
 */
bool GridViewRenderWidget::render(QImage& image, const QSize& size, double scale)
{
    if (size.isEmpty() || scale <= 0.0)
        return false;

    std::unique_ptr<ColorLegendWidget> legend;
    int legend_width = 0;

    if (colormap_.has_value())
    {
        legend.reset(new ColorLegendWidget);

        QFont font = legend->font();
        font.setPixelSize(QFontInfo(font).pixelSize());
        font.setHintingPreference(QFont::PreferNoHinting);

        legend->setFont(font);

        if (fixedPalette())
            legend->setPalette(*fixedPalette());

        legend->setLegend(legend_);

        setLegendMargins(*legend, narrowMargins());

        legend_width = std::min(legend->sizeHint().width(), size.width() / 2);
    }

    image = createImage(size, scale);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);

    if (!drawChartImage(painter, QSize(size.width() - legend_width, size.height())))
        return false;

    if (legend)
    {
        legend->resize(legend_width, size.height());
        legend->render(&painter, QPoint(size.width() - legend_width, 0));
    }

    return true;
}

/**
*/
boost::optional<std::pair<QImage, RasterReference>> GridViewRenderWidget::currentGeoImage() const
{
    return std::make_pair(grid_rendering_, ref_);
}

/**
 * The legend fields of the annotation if one is shown. Else (or without its own precision) the
 * number of decimals of the shown legend, if it gives the same texts (plain numbers). Otherwise
 * (e.g. times) the shown legend as fixed entries.
 */
boost::optional<std::pair<Grid2DLayer, Grid2DRenderSettings>> GridViewRenderWidget::currentGrid() const
{
    if (!data_ || !data_->layer.has_value() || !colormap_.has_value() || grid_rendering_.isNull())
        return {};

    Grid2DRenderSettings settings = render_settings_;

    //the JSON of the render settings keeps min / max only, not the value range of the color map
    if ((!settings.min_value.has_value() || !settings.max_value.has_value()) && settings.color_map.canSampleValues())
    {
        settings.min_value = settings.color_map.valueRange()->first;
        settings.max_value = settings.color_map.valueRange()->second;
    }

    if (data_->render_settings.has_value())
    {
        settings.legend_order     = data_->render_settings->legend_order;
        settings.legend_precision = data_->render_settings->legend_precision;
        settings.legend_unit      = data_->render_settings->legend_unit;
        settings.legend_entries   = data_->render_settings->legend_entries;
    }

    if (settings.legend_entries.empty())
    {
        const ColorLegend& shown = legend_;

        auto texts = [] (const ColorLegend& legend)
        {
            std::vector<std::string> t;
            for (const auto& e : legend.entries())
                t.push_back(e.second);
            return t;
        };

        //number of decimals giving the shown texts: the one of the shown legend first, whole numbers
        //(integer data types) ignore it
        std::vector<int> candidates;
        if (settings.legend_precision.has_value())
            candidates.push_back(settings.legend_precision.value());
        else
        {
            if (legend_decimals_ >= 0)
                candidates.push_back(legend_decimals_);
            for (int d = 0; d <= 6; ++d)
                if (d != legend_decimals_)
                    candidates.push_back(d);
        }

        bool found = false;
        for (int d : candidates)
        {
            Grid2DRenderSettings with_decimals = settings;
            with_decimals.legend_precision = d;

            if (texts(with_decimals.legend()) == texts(shown))
            {
                settings = with_decimals;
                found    = true;
                break;
            }
        }

        if (!found)
        {
            //numbers (e.g. whole numbers cut instead of rounded): their number of decimals, the last
            //digit may differ by the rounding; other texts (e.g. times): fixed entries, the coloring
            //cannot be changed then
            bool numeric = true;
            int  decimals = 0;

            for (const auto& t : texts(shown))
            {
                const QString s = QString::fromStdString(t).trimmed();

                bool ok;
                s.toDouble(&ok);
                if (!ok)
                {
                    numeric = false;
                    break;
                }

                const int dot = s.indexOf('.');
                if (dot >= 0)
                    decimals = std::max(decimals, (int)s.size() - dot - 1);
            }

            if (numeric)
            {
                settings.legend_precision = decimals;
            }
            else
            {
                loginf << "legend texts are no numbers, sent as fixed entries";
                settings.legend_entries = shown;
            }
        }
    }

    return std::make_pair(data_->layer.value(), settings);
}

/**
 */
boost::optional<QRectF> GridViewRenderWidget::axisRanges() const
{
    auto chart_view = chartView();

    if (!chart_view || !chart_view->chart() ||
        chart_view->chart()->axes(Qt::Horizontal).empty() ||
        chart_view->chart()->axes(Qt::Vertical).empty())
        return {};

    auto axis_x = dynamic_cast<QtCharts::QValueAxis*>(chart_view->chart()->axes(Qt::Horizontal).first());
    auto axis_y = dynamic_cast<QtCharts::QValueAxis*>(chart_view->chart()->axes(Qt::Vertical).first());

    if (!axis_x || !axis_y)
        return QRectF();

    return QRectF(axis_x->min(), axis_y->min(), axis_x->max() - axis_x->min(), axis_y->max() - axis_y->min());
}

/**
 */
bool GridViewRenderWidget::zoomActive(const QRectF& data_bounds, const QRectF& axis_ranges)
{
    if (!data_bounds.isValid() || !axis_ranges.isValid())
        return false;

    return (axis_ranges.left()   > data_bounds.left()  ||
            axis_ranges.right()  < data_bounds.right() ||
            axis_ranges.top()    > data_bounds.top()   ||
            axis_ranges.bottom() < data_bounds.bottom());
}

/**
 */
void GridViewRenderWidget::viewInfoJSON(nlohmann::json& info) const
{
    //background map: available for longitude / latitude grids, active if also switched on
    info[ "background_available" ] = background_.available();
    info[ "background_active"    ] = background_.active();
    info[ "background_settings"  ] = background_.settings().toJSON();
    info[ "num_labels"           ] = data_ ? data_->labels.size() : (size_t)0;

    const auto axis_ranges = axisRanges();
    const bool has_axes    = axis_ranges.has_value();

    if (has_axes)
    {
        auto axis_x = chartView()->chart()->axes(Qt::Horizontal).first();
        auto axis_y = chartView()->chart()->axes(Qt::Vertical).first();

        //Qt keeps the tick text of a plain value axis to itself, only a category
        //axis can report what it actually shows
        auto axisType = [ ] (QtCharts::QAbstractAxis* axis)
        {
            if (dynamic_cast<QtCharts::QCategoryAxis*>(axis))
                return std::string("category");

            return std::string("value");
        };

        auto axisTickLabels = [ ] (QtCharts::QAbstractAxis* axis)
        {
            nlohmann::json labels = nlohmann::json::array();

            if (auto axis_cat = dynamic_cast<QtCharts::QCategoryAxis*>(axis))
                for (const auto& l : axis_cat->categoriesLabels())
                    labels.push_back(l.toStdString());

            return labels;
        };

        nlohmann::json chart_info;

        chart_info[ "x_axis_label"       ] = axis_x->titleText().toStdString();
        chart_info[ "y_axis_label"       ] = axis_y->titleText().toStdString();
        chart_info[ "x_axis_type"        ] = axisType(axis_x);
        chart_info[ "y_axis_type"        ] = axisType(axis_y);
        chart_info[ "x_axis_tick_labels" ] = axisTickLabels(axis_x);
        chart_info[ "y_axis_tick_labels" ] = axisTickLabels(axis_y);

        info[ "chart" ] = chart_info;
    }

    const QRectF axis_bounds = has_axes ? axis_ranges.value() : QRectF();

    info[ "axis_bounds_valid" ] = has_axes && axis_bounds.isValid();
    info[ "axis_bounds_xmin"  ] = has_axes ? axis_bounds.left()   : 0.0;
    info[ "axis_bounds_ymin"  ] = has_axes ? axis_bounds.top()    : 0.0;
    info[ "axis_bounds_xmax"  ] = has_axes ? axis_bounds.right()  : 0.0;
    info[ "axis_bounds_ymax"  ] = has_axes ? axis_bounds.bottom() : 0.0;

    //the legend as the user reads it next to the grid
    nlohmann::json legend_entries = nlohmann::json::array();

    for (const auto& entry : legend_.entries())
        legend_entries.push_back(entry.second);

    info[ "legend_entries" ] = legend_entries;
}
