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

#include "chartview.h"
#include <logger.h>

#include <QRubberBand>
#include <QChart>
#include <QAreaSeries>
#include <QLineSeries>
#include <QXYSeries>
#include <QLegendMarker>

const QColor ChartView::SelectionColor = Qt::red;

/**
 */
ChartView::ChartView(QtCharts::QChart* chart, SelectionStyle sel_style, QWidget* parent)
:   QChartView(chart, parent)
,   sel_style_(sel_style)
{
    //rescale axis font to obtain title font
    QFont title_font;
    for (auto axis : chart->axes())
    {
        if (axis->orientation() == Qt::Horizontal)
        {
            title_font = axis->titleFont();
            break;
        }
    }
    title_font.setPointSize(title_font.pointSize() * 1.2);
    chart->setTitleFont(title_font);

    //only show legend if at least one marker has a meaningful text
    if (chart->legend())
    {
        bool has_nonempty_legend_marker = false;
        for (const auto& m : chart->legend()->markers())
            if (!m->label().isEmpty())
                has_nonempty_legend_marker = true;

        chart->legend()->setVisible(has_nonempty_legend_marker);
    }

    createDisplayElements(chart);

    setRenderHint(QPainter::Antialiasing);
}

/**
 */
ChartView::~ChartView() = default;

/**
 * Convert from local widget coordinates to chart coordinates.
 */
QPointF ChartView::widgetToChart(const QPointF& pos) const
{
    // view->mapToScene: widget (view) coordinates -> scene coordinates
    QPointF scene_pos = mapToScene(QPoint(static_cast<int>(pos.x()), static_cast<int>(pos.y())));
    // chart->mapFromScene: scene coordinates -> chart item coordinates
    QPointF chart_item_pos = chart()->mapFromScene(scene_pos);
    // chart->mapToValue: chart item coordinates -> value in a given series.
    QPointF p = chart()->mapToValue(chart_item_pos);

    return p;
}

/**
 * Convert from chart coordinates to local widget coordinates.
 */
QPointF ChartView::widgetFromChart(const QPointF& pos) const
{
    QPointF p = chart()->mapToPosition(pos); // widget pos

    return p;
}

/**
 * Creates the needed display elements.
 */
void ChartView::createDisplayElements(QtCharts::QChart* chart)
{
    if (sel_style_ == SelectionStyle::SeriesArea)
    {
        //add a selection box
        auto selection_box_lower = new QtCharts::QLineSeries;
        auto selection_box_upper = new QtCharts::QLineSeries;

        QBrush b;
        b.setColor(SelectionColor);
        b.setStyle(Qt::BrushStyle::SolidPattern);
        
        QPen p(Qt::red);
        p.setStyle(Qt::PenStyle::DashLine);

        selection_box_ = new QtCharts::QAreaSeries(selection_box_upper, selection_box_lower);
        selection_box_->setBrush(b);
        selection_box_->setPen(p);

        updateSelectionBox(QRectF());

        chart->addSeries(selection_box_);

        //note: attaching the axis should be called after adding the series to the chart
        for (auto axis : chart->axes(Qt::Horizontal))
            selection_box_->attachAxis(axis);
        for (auto axis : chart->axes(Qt::Vertical))
            selection_box_->attachAxis(axis);
    }
    else if (sel_style_ == SelectionStyle::SeriesLines)
    {
        QPen p(Qt::red);
        p.setStyle(Qt::PenStyle::DashLine);

        selection_lines_ = new QtCharts::QLineSeries;
        selection_lines_->setUseOpenGL(true);
        selection_lines_->setPen(p);
        
        updateSelectionLines(QRectF());

        //note: attaching the axis should be called after adding the series to the chart
        for (auto axis : chart->axes(Qt::Horizontal))
            selection_lines_->attachAxis(axis);
        for (auto axis : chart->axes(Qt::Vertical))
            selection_lines_->attachAxis(axis);

    }
    else //SelectionStyle::RubberBand
    {
        rubber_band_.reset(new QRubberBand(QRubberBand::Rectangle, viewport()));
    
        QPalette pal;
        pal.setBrush(QPalette::Highlight, QBrush(SelectionColor));
        rubber_band_->setPalette(pal);
        
        updateRubberBand(QRectF());
    }
}

/**
 * Updates the QtCharts based selection.
 */
void ChartView::updateSelectionBox(const QRectF& region)
{
    if (!selection_box_)
        return;

    selection_box_->lowerSeries()->clear();
    selection_box_->upperSeries()->clear();

    if (region.isEmpty() || !enable_selection_)
    {
        if (!data_bounds_.isEmpty())
        {
            selection_box_->lowerSeries()->append(data_bounds_.left() , data_bounds_.top()   );
            selection_box_->lowerSeries()->append(data_bounds_.right(), data_bounds_.top()   );
            selection_box_->upperSeries()->append(data_bounds_.left() , data_bounds_.bottom());
            selection_box_->upperSeries()->append(data_bounds_.right(), data_bounds_.bottom());
        }

        selection_box_->hide();

        return;
    }

    selection_box_->lowerSeries()->append(region.left() , region.top()   );
    selection_box_->lowerSeries()->append(region.right(), region.top()   );
    selection_box_->upperSeries()->append(region.left() , region.bottom());
    selection_box_->upperSeries()->append(region.right(), region.bottom());

    //@TODO: implement SelectionStyle

    selection_box_->show();
}

/**
 * 
 */
void ChartView::updateSelectionLines(const QRectF& region)
{
    if (!selection_lines_)
        return;

    selection_lines_->clear();

    auto configLines =  [ & ] (const QRectF& r) 
    {
        selection_lines_->append(r.left() , r.top());
        selection_lines_->append(r.right(), r.top()   );
        selection_lines_->append(r.right(), r.bottom());
        selection_lines_->append(r.left() , r.bottom());
        selection_lines_->append(r.left() , r.top());
    };

    if (region.isEmpty() || !enable_selection_)
    {
        if (!data_bounds_.isEmpty())
            configLines(data_bounds_);

        //std::cout << "HIDE selection box" << std::endl;

        selection_lines_->hide();

        return;
    }

    //std::cout << "SHOW selection box" << std::endl;

    configLines(region);

    selection_lines_->show();
}

/**
 * Updates the QRubberBand based selection.
 */
void ChartView::updateRubberBand(const QRectF& region)
{
    if (!rubber_band_)
        return;

    if (region.isEmpty() || !enable_selection_)
    {
        rubber_band_->hide();
        return;
    }

    if (selection_axes_ == SelectionAxes::XY || !chart())
    {
        rubber_band_->setGeometry(region.toRect());
    }
    else 
    {
        auto x0 = region.left();
        auto x1 = region.right();
        auto y0 = this->chart()->plotArea().top();
        auto y1 = this->chart()->plotArea().bottom();
        rubber_band_->setGeometry(QRectF(x0, y0, x1 - x0, y1 - y0).toRect());
    }
    
    rubber_band_->show();
}

/**
 * Reacts on tool changes.
 * Override for more specific behaviour.
 */
void ChartView::onToolChanged()
{
    //end the selection if the tool has been changed
    endSelection();
}

/**
 * Sets the data bounds (used for QtCharts based selection box)
 */
void ChartView::setDataBounds(const QRectF& r)
{
    data_bounds_ = r;
}

/**
 * Starts a new selection (will also enable the selection for display).
 */
void ChartView::beginSelection(const QRectF& r_widget,
                               const QRectF& r_chart)
{
    enable_selection_ = true;
    updateSelection(r_widget, r_chart);
}

/**
 * Starts a new selection using ANY two points (will also enable the selection for display).
 */
void ChartView::beginSelection(const QPointF& p1_widget, 
                               const QPointF& p2_widget, 
                               const QPointF& p1_chart, 
                               const QPointF& p2_chart)
{
    enable_selection_ = true;
    updateSelection(p1_widget, p2_widget, p1_chart, p2_chart);
}

/**
 * Updates the current selection region.
 */
void ChartView::updateSelection(const QRectF& r_widget,
                                const QRectF& r_chart)
{
    selected_region_widget_ = r_widget;
    selected_region_chart_  = r_chart;

    update();
    this->viewport()->repaint();
}

/**
 * Updates the current selection region using ANY two points.
 */
void ChartView::updateSelection(const QPointF& p1_widget, 
                                const QPointF& p2_widget, 
                                const QPointF& p1_chart, 
                                const QPointF& p2_chart)
{
    auto rectFromPoints = [ & ] (const QPointF& p1, const QPointF& p2) 
    {
        QPointF p_min {std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y())};
        QPointF p_max {std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y())};

        return QRectF(p_min, p_max);
    };

    updateSelection(rectFromPoints(p1_widget, p2_widget), 
                    rectFromPoints(p1_chart , p2_chart ));
}

/**
 * Empties the current selection region.
 */
void ChartView::clearSelection()
{
    updateSelection(QRectF(), QRectF());
}

/**
 * Ends the current selection (will also disable the selection for display)
 */
void ChartView::endSelection()
{
    enable_selection_ = false;
    clearSelection();
}

/**
 */
bool ChartView::isSelectionEnabled() const
{
    return enable_selection_;
}

/**
 * Can be used to react on series mouse clicks.
 * By default the click will be forwarded to the mouse press handler.
 */
void ChartView::seriesPressedSlot(const QPointF& point)
{
    QPointF p = widgetFromChart(point); // widget pos

    logdbg << "x " << point.x() << " y " << point.y();

    //forward to handler method
    handleMousePress(Qt::LeftButton, p);
}

/**
 * Can be used to react on series mouse clicks.
 * By default the click will be forwarded to the mouse release handler.
 */
void ChartView::seriesReleasedSlot(const QPointF& point)
{
    QPointF p = widgetFromChart(point); // widget pos

    logdbg << "x " << point.x() << " y " << point.y();

    //forward to handler method
    handleMouseRelease(Qt::LeftButton, p, false);
}

/**
 */
void ChartView::mousePressEvent(QMouseEvent* event)
{
    logdbg << "button = " << event->button() << " buttons = " << event->buttons();

    //forward to handler method
    if (handleMousePress(event->button(), event->pos()))
    {
        event->accept();
        return;
    }
    QChartView::mousePressEvent(event);
}

/**
 */
void ChartView::mouseMoveEvent(QMouseEvent* event)
{
    logdbg << "button = " << event->button() << " buttons = " << event->buttons();

    //forward to handler method
    if (handleMouseMove(event->buttons(), event->pos()))
    {
        event->accept();
        return;
    }
    QChartView::mouseMoveEvent(event);
}

/**
 */
void ChartView::mouseReleaseEvent(QMouseEvent* event)
{
    logdbg << "button = " << event->button() << " buttons = " << event->buttons();

    //forward to handler method
    if (handleMouseRelease(event->button(), event->pos(), true))
    {
        event->accept();
        return;
    }
    QChartView::mouseReleaseEvent(event);
}

/**
 */
void ChartView::paintEvent(QPaintEvent *e)
{
    //call base
    QChartView::paintEvent(e);

    {
        QPainter painter(this->viewport());

        //paint any other items
        paintCustomItems(e, painter);
    }

    //paint selection items
    if (sel_style_ == SelectionStyle::SeriesArea)
        updateSelectionBox(selected_region_chart_);
    else if (sel_style_ == SelectionStyle::SeriesLines)
        updateSelectionLines(selected_region_chart_);
    else //SelectionStyle::RubberBand
        updateRubberBand(selected_region_widget_);
}

/**
 */
void ChartView::addLegendOnlyItem(const QString& name, const QColor& color)
{
    if (!chart())
        return;

    QtCharts::QLineSeries* series = new QtCharts::QLineSeries;
    series->setName(name);
    series->setColor(color);
    series->show();

    chart()->addSeries(series);
}
