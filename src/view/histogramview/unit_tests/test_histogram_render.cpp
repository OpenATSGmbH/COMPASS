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

#include "catch.hpp"
#include "histogramviewrenderwidget.h"
#include "histogramdisplaydata.h"
#include "chartview.h"
#include "viewtoolswitcher.h"
#include "apppalette.h"

#include "json.hpp"

#include <QApplication>
#include <QGraphicsSimpleTextItem>
#include <QLabel>
#include <QMouseEvent>
#include <QImage>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QStackedWidget>
#include <QTest>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <QtCharts/QChart>
#include <QtCharts/QAbstractBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QLegend>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QLogValueAxis>
#include <QtCharts/QValueAxis>

#include <cmath>
#include <cstdlib>
#include <memory>
#include <set>
#include <vector>

namespace
{
    const std::vector<QColor> SeriesColors = { QColor("#ff0000"), QColor("#00ff00"), QColor("#0000ff") };

    /**
     * Display data with the given number of series and bins. Series s has the count (s + 1) * (b + 1)
     * in bin b, the name "series<s>" and a color of SeriesColors. The bins have the labels "bin<b>".
     */
    std::shared_ptr<HistogramDisplayData> makeData(size_t num_series, size_t num_bins, bool null_bin = false)
    {
        auto data = std::make_shared<HistogramDisplayData>();

        for (size_t s = 0; s < num_series; ++s)
        {
            RawHistogram h;
            for (size_t b = 0; b < num_bins; ++b)
                h.addBin(RawHistogramBin((uint32_t)((s + 1) * (b + 1)), "bin" + std::to_string(b)));

            if (null_bin)
                h.addBin(RawHistogramBin(1, "NULL", RawHistogramBin::Tag::NullValues));

            data->histogram.addDataSeries(h, "series" + std::to_string(s), SeriesColors[ s % SeriesColors.size() ]);
        }

        data->title        = "Test Title";
        data->x_axis_label = "Test Axis";
        data->valid        = true;

        return data;
    }

    QtCharts::QChart* chartOf(HistogramViewRenderWidget& render_widget)
    {
        REQUIRE(render_widget.chartView());
        REQUIRE(render_widget.chartView()->chart());

        return render_widget.chartView()->chart();
    }

    QList<QtCharts::QBarSet*> barSetsOf(HistogramViewRenderWidget& render_widget)
    {
        auto series = chartOf(render_widget)->series();
        REQUIRE(series.size() == 1);

        auto bar_series = dynamic_cast<QtCharts::QAbstractBarSeries*>(series.first());
        REQUIRE(bar_series);

        return bar_series->barSets();
    }

    bool hasLogAxis(HistogramViewRenderWidget& render_widget)
    {
        auto axes = chartOf(render_widget)->axes(Qt::Vertical);
        REQUIRE(axes.size() == 1);

        const bool is_log    = dynamic_cast<QtCharts::QLogValueAxis*>(axes.first()) != nullptr;
        const bool is_linear = dynamic_cast<QtCharts::QValueAxis*>(axes.first()) != nullptr;

        REQUIRE(is_log != is_linear);

        return is_log;
    }

    /// the texts the chart shows in addition to the Qt Charts items, the x axis label is one of them
    QStringList extraTexts(HistogramViewRenderWidget& render_widget)
    {
        QStringList texts;

        for (auto item : chartOf(render_widget)->childItems())
            if (auto text_item = dynamic_cast<QGraphicsSimpleTextItem*>(item))
                texts << text_item->text();

        return texts;
    }

    size_t numColors(const QImage& image)
    {
        std::set<QRgb> colors;

        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                colors.insert(image.pixel(x, y));

        return colors.size();
    }

    /// bounding rectangle of the pixels that differ, empty for equal images
    QRect differingRect(const QImage& image0, const QImage& image1)
    {
        REQUIRE(image0.size() == image1.size());

        QRect rect;

        for (int y = 0; y < image0.height(); ++y)
            for (int x = 0; x < image0.width(); ++x)
                if (image0.pixel(x, y) != image1.pixel(x, y))
                    rect = rect.united(QRect(x, y, 1, 1));

        return rect;
    }

    HistogramRenderOptions options(bool use_log_scale)
    {
        HistogramRenderOptions o;
        o.use_log_scale = use_log_scale;

        return o;
    }

    HistogramRenderOptions legendAtRightSide()
    {
        HistogramRenderOptions o;
        o.legend_mode = HistogramLegendMode::RightSide;

        return o;
    }

    /// the labels of the legend entries that are shown, in the order of the legend
    QStringList shownLegendEntries(HistogramViewRenderWidget& render_widget)
    {
        QStringList labels;

        for (auto marker : chartOf(render_widget)->legend()->markers())
            if (marker->isVisible())
                labels << marker->label();

        return labels;
    }

    /// bounding rectangle of the pixels that differ from the background (the pixel at the top left)
    QRect contentRect(const QImage& image)
    {
        const QRgb background = image.pixel(0, 0);

        QRect rect;

        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                if (image.pixel(x, y) != background)
                    rect = rect.united(QRect(x, y, 1, 1));

        return rect;
    }

    QPalette darkPalette()
    {
        QPalette palette;
        palette.setColor(QPalette::Window    , QColor(40, 40, 40));
        palette.setColor(QPalette::WindowText, Qt::white);
        palette.setColor(QPalette::Base      , QColor(20, 20, 20));
        palette.setColor(QPalette::Text      , Qt::white);
        palette.setColor(QPalette::Mid       , QColor(120, 120, 120));

        return palette;
    }

    /// sets the application palette, the destructor sets the palette from before again
    struct ApplicationPaletteGuard
    {
        explicit ApplicationPaletteGuard(const QPalette& palette)
        :   palette_before(QApplication::palette())
        {
            QApplication::setPalette(palette);
        }

        ~ApplicationPaletteGuard()
        {
            QApplication::setPalette(palette_before);
        }

        QPalette palette_before;
    };
}

TEST_CASE("histogram chart shows the series of the display data", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(3, 4), {});

    CHECK(render_widget.isDrawn());
    CHECK(render_widget.hasContent());

    auto sets = barSetsOf(render_widget);
    REQUIRE(sets.size() == 3);

    for (int s = 0; s < sets.size(); ++s)
    {
        INFO("series " << s);

        CHECK(sets[ s ]->label() == QString("series%1").arg(s));
        CHECK(sets[ s ]->color() == SeriesColors[ s ]);

        REQUIRE(sets[ s ]->count() == 4);
        for (int b = 0; b < 4; ++b)
            CHECK(sets[ s ]->at(b) == Approx((s + 1) * (b + 1)));
    }
}

TEST_CASE("histogram chart shows bin labels, title and axis labels", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(2, 4), {});

    auto chart = chartOf(render_widget);

    auto axes_x = chart->axes(Qt::Horizontal);
    REQUIRE(axes_x.size() == 1);

    auto axis_x = dynamic_cast<QtCharts::QBarCategoryAxis*>(axes_x.first());
    REQUIRE(axis_x);

    CHECK(axis_x->categories() == QStringList{ "bin0", "bin1", "bin2", "bin3" });
    CHECK(axis_x->labelsAngle() == HistogramViewRenderWidget::LabelAngleX);

    CHECK(chart->title() == "Test Title");
    CHECK(extraTexts(render_widget).contains("Test Axis"));

    auto axes_y = chart->axes(Qt::Vertical);
    REQUIRE(axes_y.size() == 1);
    CHECK(axes_y.first()->titleText() == HistogramViewRenderWidget::YAxisLabel);
    CHECK(axes_y.first()->titleText() == "Count");
}

TEST_CASE("histogram y axis follows the log scale option", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    auto data = makeData(2, 4);

    render_widget.setDisplayData(data, options(false));
    CHECK(!hasLogAxis(render_widget));
    CHECK(!render_widget.usesLogScale());

    //a change of the options calls again with the same data
    render_widget.setDisplayData(data, options(true));
    CHECK(hasLogAxis(render_widget));
    CHECK(render_widget.usesLogScale());

    //the bars are the same
    CHECK(barSetsOf(render_widget).size() == 2);
}

TEST_CASE("the log scale of the display data wins over the option", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    SECTION("data linear, option logarithmic")
    {
        auto data = makeData(2, 4);
        data->histogram.setUseLogScale(false);

        render_widget.setDisplayData(data, options(true));

        CHECK(!hasLogAxis(render_widget));
    }

    SECTION("data logarithmic, option linear")
    {
        auto data = makeData(2, 4);
        data->histogram.setUseLogScale(true);

        render_widget.setDisplayData(data, options(false));

        CHECK(hasLogAxis(render_widget));
    }
}

TEST_CASE("histogram legend is shown up to 8 series", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    REQUIRE(HistogramViewRenderWidget::MaxLegendEntries == 8);

    for (size_t num_series : { 1, 8 })
    {
        INFO("series " << num_series);

        render_widget.setDisplayData(makeData(num_series, 3), {});

        CHECK(barSetsOf(render_widget).size() == (int)num_series);
        CHECK(chartOf(render_widget)->legend()->isVisible());
    }

    for (size_t num_series : { 9, 30 })
    {
        INFO("series " << num_series);

        render_widget.setDisplayData(makeData(num_series, 3), {});

        CHECK(barSetsOf(render_widget).size() == (int)num_series);
        CHECK(!chartOf(render_widget)->legend()->isVisible());
    }
}

TEST_CASE("histogram legend at the right side lists the series", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    const QSize layout_size(1200, 800);

    SECTION("few series, all entries are shown")
    {
        render_widget.setDisplayData(makeData(4, 3), legendAtRightSide());
        render_widget.chartView()->layoutForSize(layout_size);

        auto legend = chartOf(render_widget)->legend();

        CHECK(legend->isVisible());
        CHECK(legend->alignment() == Qt::AlignRight);
        CHECK(shownLegendEntries(render_widget) == QStringList({ "series0", "series1", "series2", "series3" }));
    }

    SECTION("more than 8 series are shown too")
    {
        render_widget.setDisplayData(makeData(12, 3), legendAtRightSide());
        render_widget.chartView()->layoutForSize(layout_size);

        CHECK(chartOf(render_widget)->legend()->isVisible());
        CHECK(shownLegendEntries(render_widget).size() == 12);
    }

    SECTION("many series, the last entry tells the count of the series that do not fit")
    {
        const int num_series = 100;

        render_widget.setDisplayData(makeData(num_series, 3), legendAtRightSide());
        render_widget.chartView()->layoutForSize(layout_size);

        const QStringList entries = shownLegendEntries(render_widget);

        REQUIRE(entries.size() > 2);
        REQUIRE(entries.size() < num_series);

        const int num_shown = entries.size() - 1;

        for (int i = 0; i < num_shown; ++i)
            CHECK(entries[ i ] == QString("series%1").arg(i));

        CHECK(entries.last() == QString("... (%1 more)").arg(num_series - num_shown));
    }

    SECTION("the number of entries follows the height")
    {
        render_widget.setDisplayData(makeData(100, 3), legendAtRightSide());

        render_widget.chartView()->layoutForSize(layout_size);
        const QStringList entries = shownLegendEntries(render_widget);

        render_widget.chartView()->layoutForSize(QSize(1200, 400));
        const QStringList entries_low = shownLegendEntries(render_widget);

        CHECK(entries_low.size() < entries.size());
        CHECK(entries_low.size() > 1);

        render_widget.chartView()->layoutForSize(layout_size);
        CHECK(shownLegendEntries(render_widget) == entries);
    }

    SECTION("the chart without data has no legend")
    {
        render_widget.setDisplayData({}, legendAtRightSide());

        CHECK(!chartOf(render_widget)->legend()->isVisible());
    }

    SECTION("the view info counts the data series only")
    {
        render_widget.setDisplayData(makeData(100, 3), legendAtRightSide());

        nlohmann::json info;
        render_widget.viewInfoJSON(info);

        CHECK(info[ "chart" ][ "num_series" ] == 1);
        CHECK(info[ "chart" ][ "series" ].size() == 1);
        CHECK(info[ "chart" ][ "series" ][ 0 ][ "num_sets" ] == 100);
    }
}

TEST_CASE("histogram render draws the legend at the right side", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    /// number of pixels with a series color in the columns right of the plot area
    auto numLegendPixels = [ & ] (const HistogramRenderOptions& render_options, size_t num_series)
    {
        render_widget.setDisplayData(makeData(num_series, 5), render_options);

        QImage image;
        REQUIRE(render_widget.render(image, QSize(1200, 800)));

        render_widget.chartView()->layoutForSize(QSize(1200, 800));
        const int plot_right = (int)std::ceil(chartOf(render_widget)->plotArea().right());

        REQUIRE(plot_right < image.width() - 1);

        size_t num_pixels = 0;

        for (int y = 0; y < image.height(); ++y)
            for (int x = plot_right + 1; x < image.width(); ++x)
                for (const auto& c : SeriesColors)
                    if (image.pixel(x, y) == c.rgb())
                        ++num_pixels;

        return num_pixels;
    };

    SECTION("few series")
    {
        CHECK(numLegendPixels(legendAtRightSide(), 3) > 0);
    }

    SECTION("many series")
    {
        CHECK(numLegendPixels(legendAtRightSide(), 100) > 0);
    }

    SECTION("the legend of the view is not at the right side")
    {
        CHECK(numLegendPixels({}, 3) == 0);
    }
}

TEST_CASE("histogram without valid data gives the empty chart", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    auto checkEmptyChart = [ & ] ()
    {
        CHECK(render_widget.isDrawn());
        CHECK(!render_widget.hasContent());

        CHECK(barSetsOf(render_widget).isEmpty());
        CHECK(!chartOf(render_widget)->legend()->isVisible());

        for (auto axis : chartOf(render_widget)->axes())
            CHECK(!axis->labelsVisible());
    };

    SECTION("no display data")
    {
        render_widget.setDisplayData({}, {});

        checkEmptyChart();
        CHECK(chartOf(render_widget)->title().isEmpty());
    }

    SECTION("display data flagged invalid")
    {
        auto data = makeData(2, 4);
        data->valid = false;

        render_widget.setDisplayData(data, {});

        checkEmptyChart();

        //title and axis label stay
        CHECK(chartOf(render_widget)->title() == "Test Title");
        CHECK(extraTexts(render_widget).contains("Test Axis"));
    }

    SECTION("display data without series")
    {
        auto data = std::make_shared<HistogramDisplayData>();
        data->valid = true;

        render_widget.setDisplayData(data, {});

        checkEmptyChart();
    }

    SECTION("the empty chart is linear also with the log scale option")
    {
        render_widget.setDisplayData({}, options(true));

        CHECK(!hasLogAxis(render_widget));
    }

    SECTION("data after the empty chart")
    {
        render_widget.setDisplayData({}, {});
        render_widget.setDisplayData(makeData(2, 4), {});

        CHECK(render_widget.hasContent());
        CHECK(barSetsOf(render_widget).size() == 2);
    }
}

TEST_CASE("histogram render widget has no chart before the first display data", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    CHECK(!render_widget.isDrawn());
    CHECK(!render_widget.hasContent());
}

TEST_CASE("histogram render gives the requested image", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(3, 5), {});

    QImage image;

    SECTION("requested size")
    {
        REQUIRE(render_widget.render(image, QSize(1200, 800)));
        CHECK(image.size() == QSize(1200, 800));

        REQUIRE(render_widget.render(image, QSize(640, 480)));
        CHECK(image.size() == QSize(640, 480));
    }

    SECTION("not blank and with the colors of the series")
    {
        REQUIRE(render_widget.render(image, QSize(1200, 800)));

        CHECK(numColors(image) > 3);

        //the bars are stacked in the last bin, each series has an area of its color there
        std::set<QRgb> colors;
        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                colors.insert(image.pixel(x, y));

        for (const auto& c : SeriesColors)
        {
            INFO("color " << c.name().toStdString());
            CHECK(colors.count(c.rgb()) == 1);
        }
    }

    SECTION("the image shows the x axis label below the plot area")
    {
        auto data_without_label = makeData(3, 5);
        data_without_label->x_axis_label = "";

        QImage image_without_label;

        REQUIRE(render_widget.render(image, QSize(1200, 800)));

        render_widget.setDisplayData(data_without_label, {});
        REQUIRE(render_widget.render(image_without_label, QSize(1200, 800)));

        //the label is the only difference, and it is inside the image
        const QRect diff = differingRect(image, image_without_label);

        REQUIRE(!diff.isEmpty());
        CHECK(diff.top() > image.height() / 2);
        CHECK(diff.bottom() < image.height() - 1);
        CHECK(diff.left() > 0);
        CHECK(diff.right() < image.width() - 1);
    }

    SECTION("the image shows the title above the plot area")
    {
        auto data_without_title = makeData(3, 5);
        data_without_title->title = "";

        QImage image_without_title;

        REQUIRE(render_widget.render(image, QSize(1200, 800)));

        render_widget.setDisplayData(data_without_title, {});
        REQUIRE(render_widget.render(image_without_title, QSize(1200, 800)));

        //without a title the plot area grows, so the images differ from the top on
        const QRect diff = differingRect(image, image_without_title);

        REQUIRE(!diff.isEmpty());
        CHECK(diff.top() < image.height() / 4);
    }

    SECTION("the same input gives the same image twice")
    {
        QImage image2;

        REQUIRE(render_widget.render(image, QSize(1200, 800)));
        REQUIRE(render_widget.render(image2, QSize(1200, 800)));

        CHECK(image == image2);
    }

    SECTION("another input gives another image")
    {
        QImage image2;

        REQUIRE(render_widget.render(image, QSize(1200, 800)));

        render_widget.setDisplayData(makeData(3, 5), options(true));
        REQUIRE(render_widget.render(image2, QSize(1200, 800)));

        CHECK(image != image2);
    }

    SECTION("the image does not depend on the size of the widget")
    {
        QImage image2;

        render_widget.resize(300, 200);
        REQUIRE(render_widget.render(image, QSize(1200, 800)));

        render_widget.resize(900, 700);
        REQUIRE(render_widget.render(image2, QSize(1200, 800)));

        CHECK(image == image2);
    }

    SECTION("the empty chart renders too")
    {
        render_widget.setDisplayData({}, {});

        REQUIRE(render_widget.render(image, QSize(1200, 800)));
        CHECK(image.size() == QSize(1200, 800));
    }
}

TEST_CASE("histogram render scales the pixels and keeps the layout", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(3, 5), {});

    QImage image;
    REQUIRE(render_widget.render(image, QSize(1200, 800), 2.0));

    SECTION("size 1200 x 800 and scale 2 give 2400 x 1600 pixels")
    {
        CHECK(image.size() == QSize(2400, 1600));
        CHECK(image.devicePixelRatio() == Approx(2.0));
    }

    SECTION("the resolution of the image is 96 dpi")
    {
        CHECK(image.dotsPerMeterX() == 3780);
        CHECK(image.dotsPerMeterY() == 3780);
    }

    SECTION("the content has the same place as without scale")
    {
        QImage image_unscaled;
        REQUIRE(render_widget.render(image_unscaled, QSize(1200, 800)));

        const QRect rect          = contentRect(image);
        const QRect rect_unscaled = contentRect(image_unscaled);

        REQUIRE(!rect.isEmpty());
        REQUIRE(!rect_unscaled.isEmpty());

        const int tolerance = 3;

        CHECK(std::abs(rect.left()   - 2 * rect_unscaled.left()  ) <= tolerance);
        CHECK(std::abs(rect.top()    - 2 * rect_unscaled.top()   ) <= tolerance);
        CHECK(std::abs(rect.right()  - 2 * rect_unscaled.right() ) <= tolerance);
        CHECK(std::abs(rect.bottom() - 2 * rect_unscaled.bottom()) <= tolerance);
    }
}

TEST_CASE("histogram render uses the fixed palette", "[view][histogram][histogramrender]")
{
    const QPalette dark  = darkPalette();
    const QPalette light = app_palette::light();

    REQUIRE(light.color(QPalette::Window).rgb() != dark.color(QPalette::Window).rgb());

    ApplicationPaletteGuard guard(dark);

    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(3, 5), {});

    QImage image;

    SECTION("without a fixed palette the image follows the application palette")
    {
        REQUIRE(render_widget.render(image, QSize(1200, 800)));

        CHECK(image.pixel(1, 1) == dark.color(QPalette::Window).rgb());
    }

    SECTION("with the light palette the background is light while the application is dark")
    {
        render_widget.setFixedPalette(light);

        REQUIRE(render_widget.render(image, QSize(1200, 800)));

        CHECK(image.pixel(1, 1) == light.color(QPalette::Window).rgb());
    }

    SECTION("the chart in the widget uses the fixed palette too")
    {
        CHECK(chartOf(render_widget)->backgroundBrush().color() == dark.color(QPalette::Window));

        render_widget.setFixedPalette(light);

        CHECK(chartOf(render_widget)->backgroundBrush().color() == light.color(QPalette::Window));

        //also after new display data
        render_widget.setDisplayData(makeData(2, 4), {});

        CHECK(chartOf(render_widget)->backgroundBrush().color() == light.color(QPalette::Window));
    }
}

TEST_CASE("histogram chart texts have fonts with a pixel size", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(3, 5), {});

    auto chart = chartOf(render_widget);

    CHECK(chart->titleFont().pixelSize() > 0);
    CHECK(chart->legend()->font().pixelSize() > 0);

    for (auto axis : chart->axes())
    {
        CHECK(axis->labelsFont().pixelSize() > 0);
        CHECK(axis->titleFont().pixelSize() > 0);
    }

    bool has_axis_label_item = false;

    for (auto item : chart->childItems())
    {
        if (auto text_item = dynamic_cast<QGraphicsSimpleTextItem*>(item))
        {
            has_axis_label_item = true;
            CHECK(text_item->font().pixelSize() > 0);
        }
    }

    CHECK(has_axis_label_item);
}

TEST_CASE("histogram chart fonts take the hinting of an image", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(3, 5), {});

    auto chart = chartOf(render_widget);

    auto checkHinting = [ & ] (QFont::HintingPreference hinting)
    {
        CHECK(chart->titleFont().hintingPreference() == hinting);
        CHECK(chart->legend()->font().hintingPreference() == hinting);

        for (auto axis : chart->axes())
        {
            CHECK(axis->labelsFont().hintingPreference() == hinting);
            CHECK(axis->titleFont().hintingPreference() == hinting);
        }

        for (auto item : chart->childItems())
            if (auto text_item = dynamic_cast<QGraphicsSimpleTextItem*>(item))
                CHECK(text_item->font().hintingPreference() == hinting);
    };

    //the chart on the screen keeps the hinting of the screen
    checkHinting(QFont::PreferDefaultHinting);

    render_widget.chartView()->usePixelSizeFonts(QFont::PreferNoHinting);

    checkHinting(QFont::PreferNoHinting);
}

TEST_CASE("render widget height follows the width inside the limits", "[view][histogram][histogramrender]")
{
    const double ratio      = 2.0 / 3.0;
    const int    min_height = 400;

    SECTION("the height rule")
    {
        //below the minimum
        CHECK(ViewRenderWidget::limitedHeightForWidth(300, ratio, min_height, 700) == 400);
        //inside the range
        CHECK(ViewRenderWidget::limitedHeightForWidth(900, ratio, min_height, 700) == 600);
        //above the maximum
        CHECK(ViewRenderWidget::limitedHeightForWidth(1800, ratio, min_height, 700) == 700);
        //the minimum wins over the maximum
        CHECK(ViewRenderWidget::limitedHeightForWidth(900, ratio, min_height, 300) == 400);
    }

    SECTION("a render widget of a view reports no height for a width")
    {
        HistogramViewRenderWidget render_widget;
        render_widget.setDisplayData(makeData(3, 5), {});

        CHECK(!render_widget.hasHeightForWidth());
    }

    SECTION("the widget reports the height of the rule, without a scroll area there is no maximum")
    {
        HistogramViewRenderWidget render_widget;
        render_widget.setDisplayData(makeData(3, 5), {});
        render_widget.setHeightForWidth(ratio, min_height);

        CHECK(render_widget.hasHeightForWidth());
        CHECK(render_widget.heightForWidth(300) == 400);
        CHECK(render_widget.heightForWidth(900) == 600);
        CHECK(render_widget.heightForWidth(3000) == 2000);
    }
}

namespace
{
    /**
     * A render widget in a scroll area, with the widget structure of a report section:
     * scroll area, stack of the sections, section with label, render widget, button and stretch.
     */
    struct FigureInScrollArea
    {
        FigureInScrollArea()
        {
            render_widget = new HistogramViewRenderWidget;
            render_widget->setDisplayData(makeData(3, 5), {});
            render_widget->setHeightForWidth(2.0 / 3.0, 400);
            render_widget->setWheelIgnored(true);

            section = new QWidget;

            auto layout = new QVBoxLayout;
            layout->addWidget(new QLabel("Figure: Test"));
            layout->addWidget(render_widget);
            layout->addWidget(new QPushButton("View"));
            layout->addStretch();

            section->setLayout(layout);

            auto stack = new QStackedWidget;
            stack->addWidget(section);

            scroll_area.setWidgetResizable(true);
            scroll_area.setWidget(stack);
        }

        void showWithSize(const QSize& size)
        {
            scroll_area.resize(size);
            scroll_area.show();

            REQUIRE(QTest::qWaitForWindowExposed(&scroll_area));

            for (int i = 0; i < 10; ++i)
                QApplication::processEvents();
        }

        int expectedHeight() const
        {
            return ViewRenderWidget::limitedHeightForWidth(render_widget->width(), 2.0 / 3.0, 400,
                                                           scroll_area.viewport()->height());
        }

        QScrollArea                scroll_area;
        QWidget*                   section       = nullptr;
        HistogramViewRenderWidget* render_widget = nullptr;
    };
}

TEST_CASE("render widget in a scroll area has the full width and the height of the rule", "[view][histogram][histogramrender]")
{
    FigureInScrollArea figure;

    auto checkSize = [ & ] (const QSize& size)
    {
        INFO("scroll area " << size.width() << " x " << size.height());

        figure.showWithSize(size);

        CHECK(figure.render_widget->width() == figure.section->layout()->contentsRect().width());
        CHECK(figure.render_widget->height() == figure.expectedHeight());
    };

    SECTION("narrow, the height is the minimum")
    {
        checkSize(QSize(500, 900));
        CHECK(figure.render_widget->height() == 400);
    }

    SECTION("the height is the width * 2 / 3")
    {
        checkSize(QSize(900, 900));
        CHECK(figure.render_widget->height() > 400);
        CHECK(figure.render_widget->height() < figure.scroll_area.viewport()->height());
    }

    SECTION("wide, the height is the visible height of the scroll area")
    {
        checkSize(QSize(1800, 600));
        CHECK(figure.render_widget->height() == figure.scroll_area.viewport()->height());
    }

    SECTION("the height follows a new height of the scroll area")
    {
        checkSize(QSize(1800, 600));
        checkSize(QSize(1800, 800));
        checkSize(QSize(1800, 500));
    }

    SECTION("the chart fills the widget")
    {
        checkSize(QSize(900, 900));
        CHECK(figure.render_widget->chartView()->size() == figure.render_widget->size());
    }
}

TEST_CASE("mouse wheel over a render widget that ignores it scrolls the scroll area", "[view][histogram][histogramrender]")
{
    FigureInScrollArea figure;

    auto wheelOverChart = [ & ] ()
    {
        figure.showWithSize(QSize(900, 300));

        auto scroll_bar = figure.scroll_area.verticalScrollBar();
        REQUIRE(scroll_bar->maximum() > 0);
        REQUIRE(scroll_bar->value() == 0);

        auto viewport = figure.render_widget->chartView()->viewport();

        const QPointF pos(viewport->width() / 2.0, 20.0);

        QWheelEvent event(pos, viewport->mapToGlobal(pos.toPoint()), QPoint(), QPoint(0, -120),
                          Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);

        //as a wheel event of the user: Qt gives only these on to the parent widgets
        QSpontaneKeyEvent::setSpontaneous(&event);
        qApp->notify(viewport, &event);

        return scroll_bar->value();
    };

    CHECK(wheelOverChart() > 0);
}

TEST_CASE("histogram rectangle gives the bins with their center inside", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    typedef std::pair<unsigned int, unsigned int> Bins;

    //bin centers are at 0, 1, 2, 3
    render_widget.setDisplayData(makeData(2, 4), {});

    auto rect = [ ] (double x0, double x1) { return QRectF(QPointF(x0, 0.0), QPointF(x1, 10.0)); };

    SECTION("inner bins")
    {
        auto bins = render_widget.binRange(rect(0.6, 2.4));

        REQUIRE(bins.has_value());
        CHECK(bins.value() == Bins(1, 2));
    }

    SECTION("a single bin")
    {
        auto bins = render_widget.binRange(rect(1.9, 2.1));

        REQUIRE(bins.has_value());
        CHECK(bins.value() == Bins(2, 2));
    }

    SECTION("no bin center inside")
    {
        CHECK(!render_widget.binRange(rect(1.1, 1.9)).has_value());
    }

    SECTION("a rectangle past the left end starts at the first bin")
    {
        auto bins = render_widget.binRange(rect(-5.0, 1.2));

        REQUIRE(bins.has_value());
        CHECK(bins.value() == Bins(0, 1));
    }

    SECTION("a rectangle past the right end stops at the index after the last bin")
    {
        auto bins = render_widget.binRange(rect(2.5, 20.0));

        REQUIRE(bins.has_value());
        CHECK(bins.value() == Bins(3, 4));
    }

    SECTION("the bin of the null values has the index after the last standard bin")
    {
        render_widget.setDisplayData(makeData(2, 4, true), {});

        auto bins = render_widget.binRange(rect(2.5, 20.0));

        REQUIRE(bins.has_value());
        CHECK(bins.value() == Bins(3, 4));
    }

    SECTION("the empty chart has no bins")
    {
        render_widget.setDisplayData({}, {});

        CHECK(!render_widget.binRange(rect(0.0, 3.0)).has_value());
    }
}

TEST_CASE("histogram rectangle drawn with the mouse reaches the requester", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;
    render_widget.resize(800, 600);
    render_widget.setDisplayData(makeData(1, 4), {});
    render_widget.show();

    REQUIRE(QTest::qWaitForWindowExposed(&render_widget));
    QCoreApplication::processEvents();

    auto chart_view = render_widget.chartView();
    REQUIRE(chart_view);

    const QRectF plot_area = chart_view->chart()->plotArea();
    REQUIRE(plot_area.width() > 100.0);

    const int y = (int)plot_area.center().y();

    //one bin is a quarter of the plot area wide, the bin centers are in the middle of each quarter
    auto xOfBin = [ & ] (double bin) { return (int)(plot_area.left() + (bin + 0.5) * plot_area.width() / 4.0); };

    auto drag = [ & ] (double bin_from, double bin_to)
    {
        const QPoint p0(xOfBin(bin_from), y);
        const QPoint p1(xOfBin(bin_to), y);

        QTest::mousePress(chart_view->viewport(), Qt::LeftButton, Qt::NoModifier, p0);

        QMouseEvent move(QEvent::MouseMove, p1, Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(chart_view->viewport(), &move);

        QTest::mouseRelease(chart_view->viewport(), Qt::LeftButton, Qt::NoModifier, p1);
    };

    std::vector<QRectF> rectangles;

    SECTION("without a request the mouse draws no rectangle")
    {
        drag(0.6, 2.4);
        QCoreApplication::processEvents();

        CHECK(rectangles.empty());
    }

    SECTION("a requested rectangle arrives in chart coordinates")
    {
        render_widget.requestRectangle([ & ] (const QRectF& r) { rectangles.push_back(r); });

        drag(0.6, 2.4);

        //queued
        CHECK(rectangles.empty());

        QCoreApplication::processEvents();

        REQUIRE(rectangles.size() == 1);

        //the x axis of the chart counts in bins
        CHECK(rectangles[ 0 ].left()  == Approx(0.6).margin(0.05));
        CHECK(rectangles[ 0 ].right() == Approx(2.4).margin(0.05));

        auto bins = render_widget.binRange(rectangles[ 0 ]);
        REQUIRE(bins.has_value());
        CHECK(bins->first  == 1);
        CHECK(bins->second == 2);

        CHECK(!render_widget.rectangleRequested());
    }

    SECTION("a rectangle without a bin center is not reported, the request stays")
    {
        render_widget.requestRectangle([ & ] (const QRectF& r) { rectangles.push_back(r); });

        drag(1.1, 1.9);
        QCoreApplication::processEvents();

        CHECK(rectangles.empty());
        CHECK(render_widget.rectangleRequested());

        drag(0.6, 2.4);
        QCoreApplication::processEvents();

        CHECK(rectangles.size() == 1);
    }
}

TEST_CASE("histogram render widget registers the navigate tool", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;

    auto& switcher = render_widget.toolSwitcher();

    REQUIRE(switcher.registeredTools().size() == 1);
    CHECK(switcher.registeredTools().front().kind == ViewTool::Kind::Mode);
    CHECK(switcher.currentTool() == HG_NAVIGATE_TOOL);

    //it is the default tool, the tool after the end of another one
    ViewTool select;
    select.id   = HG_SELECT_TOOL;
    select.kind = ViewTool::Kind::Mode;

    render_widget.registerTool(select);

    switcher.setCurrentTool(HG_SELECT_TOOL);
    switcher.endCurrentTool();

    CHECK(switcher.currentTool() == HG_NAVIGATE_TOOL);
}

TEST_CASE("histogram view info lists the chart content", "[view][histogram][histogramrender]")
{
    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(2, 3), options(true));

    nlohmann::json info;
    render_widget.viewInfoJSON(info);

    REQUIRE(info.contains("chart"));

    const auto& chart = info[ "chart" ];

    CHECK(chart[ "x_axis_label" ] == "Test Axis");
    CHECK(chart[ "y_axis_label" ] == "Count");
    CHECK(chart[ "y_axis_log"   ] == true);
    CHECK(chart[ "num_series"   ] == 1);
    CHECK(chart[ "x_axis_tick_labels" ] == nlohmann::json({ "bin0", "bin1", "bin2" }));

    REQUIRE(chart[ "series" ].size() == 1);
    REQUIRE(chart[ "series" ][ 0 ][ "sets" ].size() == 2);

    const auto& set1 = chart[ "series" ][ 0 ][ "sets" ][ 1 ];

    CHECK(set1[ "name"   ] == "series1");
    CHECK(set1[ "counts" ] == nlohmann::json({ 2, 4, 6 }));
    CHECK(set1[ "color"  ] == "#00ff00");
}

// Not run by default. Writes the image of a histogram chart to the file in COMPASS_TEST_IMAGE_FILE.
// scripts/check_image_display_scaling.py runs it with the display scaling 1 and 2 and compares the images.
// The check needs a platform with real font settings (wayland, xcb): on the offscreen platform of the
// unit tests the images are equal also with fonts that depend on the display scaling.
TEST_CASE("histogram image for the display scaling check", "[.][displayscaling]")
{
    const QString file = qEnvironmentVariable("COMPASS_TEST_IMAGE_FILE");
    REQUIRE(!file.isEmpty());

    HistogramViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(4, 6), legendAtRightSide());

    QImage image;
    REQUIRE(render_widget.render(image, QSize(1200, 800), 2.0));
    REQUIRE(image.save(file));
}
