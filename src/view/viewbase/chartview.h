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

#include <QChartView>
#include <QGraphicsSimpleTextItem>

#include <memory>

class QRubberBand;

namespace QtCharts
{
    class QChart;
    class QAreaSeries;
    class QLineSeries;
}

/**
 * Base chart view, encapsulating common features such as rectangle selection and reaction on tool changes.
 */
class ChartView : public QtCharts::QChartView
{
    Q_OBJECT
public:
    enum class SelectionStyle
    {
        RubberBand = 0,
        SeriesArea, 
        SeriesLines //gl accelerated lines
    };
    enum class SelectionAxes
    {
        X = 0,
        XY
    };

    ChartView(QtCharts::QChart* chart, SelectionStyle sel_style, QWidget* parent = nullptr);
    virtual ~ChartView();

    void setDataBounds(const QRectF& r);
    void setSelectionAxes(SelectionAxes axes) { selection_axes_ = axes; }

    void addLegendOnlyItem(const QString& name, const QColor& color);
    void setXAxisLabel(const QString& label);

    virtual void onToolChanged();

public slots:
    virtual void seriesPressedSlot(const QPointF& point);
    virtual void seriesReleasedSlot(const QPointF& point);

protected:
    virtual void resizeEvent(QResizeEvent* event) override;
    virtual void paintEvent(QPaintEvent* e) override final;
    virtual void mousePressEvent(QMouseEvent* event) override final;
    virtual void mouseMoveEvent(QMouseEvent* event) override final;
    virtual void mouseReleaseEvent(QMouseEvent* event) override final;
    virtual void changeEvent(QEvent* event) override;

    virtual bool handleMousePress(Qt::MouseButtons buttons, const QPointF& widget_pos) = 0;
    virtual bool handleMouseRelease(Qt::MouseButtons buttons, const QPointF& widget_pos, bool update_pos) = 0;
    virtual bool handleMouseMove(Qt::MouseButtons buttons, const QPointF& widget_pos) = 0;

    virtual void paintCustomItems(QPaintEvent* e, QPainter& painter) {}

    QPointF widgetToChart(const QPointF& pos) const;
    QPointF widgetFromChart(const QPointF& pos) const;

    const QRectF& selectedChartRegion() const { return selected_region_chart_; }
    void beginSelection(const QRectF& r_widget,
                        const QRectF& r_chart);
    void beginSelection(const QPointF& p1_widget, 
                        const QPointF& p2_widget, 
                        const QPointF& p1_chart, 
                        const QPointF& p2_chart);
    void updateSelection(const QRectF& r_widget,
                         const QRectF& r_chart);
    void updateSelection(const QPointF& p1_widget, 
                         const QPointF& p2_widget, 
                         const QPointF& p1_chart, 
                         const QPointF& p2_chart);
    void endSelection();

    bool isSelectionEnabled() const;

private:
    void createDisplayElements(QtCharts::QChart* chart);
    void updateXAxisLabelPosition();
    void applyPaletteTheme();

    void clearSelection();
    void updateSelectionBox(const QRectF& region);
    void updateSelectionLines(const QRectF& region);
    void updateRubberBand(const QRectF& region);

    static const QColor SelectionColor;

    SelectionStyle               sel_style_;
    QRectF                       data_bounds_;
    QRectF                       selected_region_widget_;
    QRectF                       selected_region_chart_;
    std::unique_ptr<QRubberBand> rubber_band_;
    QtCharts::QAreaSeries*       selection_box_    = nullptr;
    QtCharts::QLineSeries*       selection_lines_  = nullptr;
    bool                         enable_selection_ = false;
    SelectionStyle               selection_style_  = SelectionStyle::RubberBand;
    SelectionAxes                selection_axes_   = SelectionAxes::XY;

    // Custom x-axis label rendered as a QGraphicsSimpleTextItem, used instead of
    // QAbstractAxis::setTitleText() which gets truncated by Qt Charts' internal
    // layout when tick labels are rotated (e.g. 85 degrees).
    QGraphicsSimpleTextItem*     x_axis_label_item_ = nullptr;
};
