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
#include <QFont>
#include <QGraphicsSimpleTextItem>
#include <QPalette>

#include <boost/optional.hpp>

#include <memory>
#include <set>

class QRubberBand;

namespace QtCharts
{
    class QChart;
    class QAbstractSeries;
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

    /// Shows the legend at the right side of the chart, as a vertical list with one entry per series.
    /// The entries that do not fit in the height are hidden, the last entry then tells their count
    /// ("... (n more)"). Qt Charts alone cuts off such entries without a mark.
    void showLegendAtRightSide();

    /// The series gets no entry in the legend at the right side, e.g. a series that is no data.
    /// To call before showLegendAtRightSide().
    void excludeFromLegend(const QtCharts::QAbstractSeries* series);

    /// The chart is drawn with this palette and does not follow the application palette any more.
    void setFixedPalette(const QPalette& palette);

    /// Gives all texts of the chart a font with a pixel size of the same height, and the given hinting.
    /// With a fixed hinting (not the default) the drawn text does not depend on the display scaling,
    /// as an image for a report needs it. The default hinting keeps the look of the screen.
    void usePixelSizeFonts(QFont::HintingPreference hinting = QFont::PreferDefaultHinting);

    /// The chart does not use the mouse wheel, so a scroll area around it scrolls.
    void setWheelIgnored(bool ignored) { wheel_ignored_ = ignored; }

    /// Chart margins of NarrowMargin in place of the default of Qt Charts, also as the lower limit of
    /// reserveHorizontalLabelMargins(), and a legend without margins except for the gap to the plot.
    void useNarrowMargins();

    static constexpr int NarrowMargin = 6;

    /// Lays out the chart and the items of the view for the given size, for a view that is never
    /// shown. A hidden widget gets no resize event, which does this for a view on the screen.
    void layoutForSize(const QSize& size);

    /// Paints the items the view paints above its chart (paintCustomItems()) into an image of the
    /// chart. The painter has the coordinates of the chart.
    void paintImageItems(QPainter& painter) { paintCustomItems(nullptr, painter); }

    /// Widens the chart margins so the outermost labels of a horizontal
    /// category axis are not cut off. Qt centers such a label on its tick, so
    /// half of it reaches past the plot area, and the default margin is
    /// narrower than half a wide label such as a time of day.
    /// Recomputed from the current labels, so a zoom never widens it for good.
    /// Never below the default margin of Qt Charts, or NarrowMargin after useNarrowMargins().
    static void reserveHorizontalLabelMargins(QtCharts::QChart* chart);

    /// The series the selection is drawn with, null unless the view uses
    /// SelectionStyle::SeriesLines. It belongs to the chart like any other
    /// series, so callers walking the series have to skip it.
    const QtCharts::QAbstractSeries* selectionSeries() const;

    virtual void onToolChanged();

public slots:
    virtual void seriesPressedSlot(const QPointF& point);
    virtual void seriesReleasedSlot(const QPointF& point);

protected:
    virtual void resizeEvent(QResizeEvent* event) override;
    virtual bool viewportEvent(QEvent* event) override;
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

    /// Geometry the rubber band should get for a selection region given in
    /// viewport coordinates. The rubber band is a child of the view, so the
    /// region has to be mapped out of the viewport.


private:
    void createDisplayElements(QtCharts::QChart* chart);
    void updateXAxisLabelPosition();
    void applyPaletteTheme(const QPalette& palette);
    QPalette themePalette() const;
    void updateLegendOverflow();
    void layoutLegendNow();
    QFont pixelSizeFont(const QFont& font) const;

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

    QtCharts::QLineSeries*       legend_more_series_ = nullptr; // carries the legend entry "... (n more)"
    std::set<const QtCharts::QAbstractSeries*> legend_excluded_; // see excludeFromLegend()

    boost::optional<QPalette>    fixed_palette_;
    bool                         pixel_size_fonts_ = false;
    QFont::HintingPreference     font_hinting_     = QFont::PreferDefaultHinting;
    bool                         wheel_ignored_    = false;
};
