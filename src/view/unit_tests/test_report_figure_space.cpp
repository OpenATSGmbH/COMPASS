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
#include "reportfigure.h"
#include "chartrenderwidget.h"
#include "chartview.h"
#include "histograminitializer.h"
#include "grid2d.h"
#include "gridviewrenderwidget.h"
#include "histogramviewmodel.h"
#include "histogramviewrenderwidget.h"
#include "scatterplotviewmodel.h"
#include "viewpointgenerator.h"
#include "apppalette.h"

#include "json.hpp"

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsTextItem>
#include <QImage>
#include <QLayout>
#include <QTextStream>

#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QChart>

#include <algorithm>
#include <cmath>

namespace
{
    /// size of a report figure in the Reports tool
    const QSize GuiSize(664, 443);

    RawHistogram errorHistogram()
    {
        std::vector<double> values;

        for (int i = 0; i < 126000; ++i)
            values.push_back(190.0 * i / 126000.0);

        const std::vector<int> tail_counts = { 60, 25, 12, 0, 5, 3, 0, 1, 2 };

        for (size_t k = 0; k < tail_counts.size(); ++k)
            for (int i = 0; i < tail_counts[ k ]; ++i)
                values.push_back(385.0 * (k + 1) + 100.0);

        values.push_back(3850.0);

        RawHistogram h;
        HistogramInitializerT<double> init;
        REQUIRE(init.createRAW(h, values, false, 10));

        return h;
    }

    RawHistogram errorCountHistogram()
    {
        RawHistogram h;
        HistogramInitializerT<std::string> init;
        REQUIRE(init.createRAW(h, { "#CF", "#CP" }, { 217, 126608 }, false));

        return h;
    }

    nlohmann::json toJSON(ViewPointGenVP& vp)
    {
        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    nlohmann::json histogramViewPoint(const RawHistogram& h, const std::string& x_label)
    {
        RawHistogramCollection collection;
        collection.addDataSeries(h, "", QColor(0, 0, 255));

        ViewPointGenVP vp("Sector Overview", 0, "Histogram");
        vp.annotations().getOrCreateAnnotation(x_label)->addFeature(
            new ViewPointGenFeatureHistogram(collection, PlotMetadata("Position Error", x_label, x_label, "")));

        return toJSON(vp);
    }

    nlohmann::json scatterViewPoint()
    {
        ScatterSeries points_a;
        ScatterSeries points_b;

        for (int i = 0; i <= 50; ++i)
        {
            points_a.points.push_back({ 2000.0 * i, 40.0 * std::sin(i / 5.0) });
            points_b.points.push_back({ 2000.0 * i + 1000.0, 20.0 * std::cos(i / 7.0) - 10.0 });
        }

        ScatterSeriesCollection collection;
        collection.addDataSeries(points_a, "Radar A", QColor("#ff8000"), 6.0);
        collection.addDataSeries(points_b, "Radar B", QColor("#0080ff"), 6.0);

        ViewPointGenVP vp("Range Bias", 0, "Scatterplot");
        vp.annotations().getOrCreateAnnotation("Range Bias")->addFeature(
            new ViewPointGenFeatureScatterSeries(collection, PlotMetadata("Range Bias", "", "Range [m]", "Bias [m]")));

        return toJSON(vp);
    }

    /// longitude / latitude grid over Austria with a legend of two colors
    nlohmann::json geoGridViewPoint()
    {
        Grid2DLayer layer;
        layer.name = "pe";
        layer.data.resize(6, 16);

        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 16; ++c)
                layer.data(r, c) = (r + c) % 5 == 0 ? 0.0 : 1.0;

        layer.ref.srs              = "wgs84";
        layer.ref.img_origin_x     = 9.5;
        layer.ref.img_origin_y     = 49.0;
        layer.ref.img_pixel_size_x = 0.5;
        layer.ref.img_pixel_size_y = 0.5;
        layer.ref.is_north_up      = true;

        Grid2DRenderSettings rs;
        rs.color_map.create(QColor("#ff0000"), QColor("#00ff00"), 2, ColorMap::Type::Binary);
        rs.min_value = 0.0;
        rs.max_value = 1.0;

        ViewPointGenVP vp("Sector Overview", 0, "Grid");
        vp.annotations().getOrCreateAnnotation("Position Error")->addFeature(
            new ViewPointGenFeatureGrid(layer, rs, PlotMetadata("Position Error", "", "Longitude (deg)", "Latitude (deg)")));

        return toJSON(vp);
    }

    report_figure::RenderEntry onlyEntry(const nlohmann::json& view_point)
    {
        const auto entries = report_figure::selectRenderEntries(view_point);
        REQUIRE(entries.size() == 1);

        return entries[ 0 ];
    }

    /// The space a figure gives its content, in layout pixels.
    struct Space
    {
        QSize  size;
        QRect  content; // bounding box of the pixels that are not the window background
        QRectF plot;    // plot area of the chart

        int emptyLeft()   const { return content.left(); }
        int emptyRight()  const { return size.width() - 1 - content.right(); }
        int emptyTop()    const { return content.top(); }
        int emptyBottom() const { return size.height() - 1 - content.bottom(); }

        double outsideLeft()   const { return plot.left() / size.width(); }
        double outsideRight()  const { return (size.width() - plot.right()) / size.width(); }
        double outsideTop()    const { return plot.top() / size.height(); }
        double outsideBottom() const { return (size.height() - plot.bottom()) / size.height(); }

        double plotFraction() const { return plot.width() * plot.height() / (size.width() * size.height()); }
    };

    bool isBackground(QRgb pixel, const QColor& background)
    {
        const int MaxDiff = 8;

        return std::abs(qRed(pixel)   - background.red())   <= MaxDiff &&
               std::abs(qGreen(pixel) - background.green()) <= MaxDiff &&
               std::abs(qBlue(pixel)  - background.blue())  <= MaxDiff;
    }

    /// in layout pixels
    QRect contentBox(const QImage& image, const QColor& background)
    {
        const QImage img   = image.convertToFormat(QImage::Format_ARGB32);
        const double scale = image.devicePixelRatio();

        int x0 = img.width(), y0 = img.height(), x1 = -1, y1 = -1;

        for (int y = 0; y < img.height(); ++y)
        {
            const QRgb* line = reinterpret_cast<const QRgb*>(img.constScanLine(y));

            for (int x = 0; x < img.width(); ++x)
            {
                if (isBackground(line[ x ], background))
                    continue;

                x0 = std::min(x0, x);
                x1 = std::max(x1, x);
                y0 = std::min(y0, y);
                y1 = std::max(y1, y);
            }
        }

        REQUIRE(x1 >= 0);

        return QRect(QPoint((int)std::floor(x0 / scale), (int)std::floor(y0 / scale)),
                     QPoint((int)std::floor(x1 / scale), (int)std::floor(y1 / scale)));
    }

    /// The render widget of the report GUI, laid out for the size as in the Reports tool.
    std::unique_ptr<ViewRenderWidget> layoutWidget(const report_figure::RenderEntry& entry, const QSize& size)
    {
        auto widget = report_figure::createRenderWidget(entry);
        REQUIRE(widget);

        widget->resize(size);
        widget->layout()->activate();

        auto chart_widget = dynamic_cast<ChartRenderWidget*>(widget.get());
        REQUIRE(chart_widget);
        REQUIRE(chart_widget->chartView());

        chart_widget->chartView()->layoutForSize(chart_widget->chartView()->size());

        return widget;
    }

    ChartView& chartViewOf(ViewRenderWidget& widget)
    {
        return *dynamic_cast<ChartRenderWidget&>(widget).chartView();
    }

    /// Every text of the chart is shown in full (not elided) and inside the chart. Histogram bins all have
    /// their label.
    void checkTextsReadable(ViewRenderWidget& widget)
    {
        auto& chart_view = chartViewOf(widget);
        auto  chart      = chart_view.chart();

        const QRectF chart_rect = chart->geometry().adjusted(-1, -1, 1, 1);

        QStringList texts;

        for (auto item : chart->scene()->items())
        {
            if (!item->isVisible())
                continue;

            QString text;

            if (auto text_item = dynamic_cast<QGraphicsTextItem*>(item))
                text = text_item->toPlainText();
            else if (auto simple_item = dynamic_cast<QGraphicsSimpleTextItem*>(item))
                text = simple_item->text();
            else
                continue;

            if (text.trimmed().isEmpty())
                continue;

            texts << text;

            INFO("text '" << text.toStdString() << "'");
            CHECK(!text.endsWith("..."));
            CHECK(chart_rect.contains(item->sceneBoundingRect()));
        }

        CHECK(texts.contains(chart->title()));

        for (auto axis : chart->axes(Qt::Horizontal))
            if (auto axis_cat = dynamic_cast<QtCharts::QBarCategoryAxis*>(axis))
                for (const auto& label : axis_cat->categories())
                    CHECK(texts.contains(label));
    }

    Space guiSpace(const report_figure::RenderEntry& entry, QImage& image)
    {
        auto widget = layoutWidget(entry, GuiSize);

        checkTextsReadable(*widget);

        auto& chart_view = chartViewOf(*widget);

        Space space;
        space.size = GuiSize;
        space.plot = chart_view.chart()->plotArea().translated(chart_view.pos());

        REQUIRE(widget->render(image, GuiSize, 1.0));

        space.content = contentBox(image, QApplication::palette().color(QPalette::Window));

        return space;
    }

    Space exportSpace(const report_figure::RenderEntry& entry, QImage& image)
    {
        const QSize size = entry.export_layout_size;

        //the image has the light palette, the layout does not depend on the palette
        auto widget = layoutWidget(entry, size);

        checkTextsReadable(*widget);

        auto& chart_view = chartViewOf(*widget);

        Space space;
        space.size = size;
        space.plot = chart_view.chart()->plotArea().translated(chart_view.pos());

        REQUIRE(report_figure::renderExportImage(entry, image));

        space.content = contentBox(image, app_palette::light().color(QPalette::Window));

        return space;
    }

    /// Saves the image and appends the space to space.txt, if COMPASS_TEST_IMAGE_DIR is set.
    void record(const QString& name, const QImage& image, const Space& space)
    {
        const QString dir = qEnvironmentVariable("COMPASS_TEST_IMAGE_DIR");
        if (dir.isEmpty())
            return;

        REQUIRE(image.save(QDir(dir).filePath(name + ".png")));

        QFile file(QDir(dir).filePath("space.txt"));
        REQUIRE(file.open(QIODevice::Append | QIODevice::Text));

        QTextStream out(&file);
        out << name.leftJustified(28)
            << " size " << space.size.width() << "x" << space.size.height()
            << " empty l/r/t/b " << space.emptyLeft() << "/" << space.emptyRight() << "/"
                                 << space.emptyTop() << "/" << space.emptyBottom()
            << " outside plot l/r/t/b " << QString::number(space.outsideLeft()  , 'f', 3) << "/"
                                        << QString::number(space.outsideRight() , 'f', 3) << "/"
                                        << QString::number(space.outsideTop()   , 'f', 3) << "/"
                                        << QString::number(space.outsideBottom(), 'f', 3)
            << " plot " << QString::number(space.plotFraction(), 'f', 3) << "\n";
    }
}

TEST_CASE("report figures use the space of the image, all texts readable", "[view][reportfigure]")
{
    //Before the narrow margins (Qt's default chart margins of 20 px) the plot took 0.51 / 0.52 / 0.41 / 0.46 of
    //the image of the Reports tool and 0.71 / 0.72 / 0.64 / 0.68 of the exported image, and the empty border
    //was 20 to 33 px wide on the left, right and top.
    struct Case
    {
        QString                    name;
        report_figure::RenderEntry entry;
        double                     min_plot_gui;
        double                     min_plot_export;
    };

    const std::vector<Case> cases =
    {
        { "histogram_log"       , onlyEntry(histogramViewPoint(errorHistogram()     , "Full distribution [m]")), 0.55 , 0.735 },
        { "histogram_categories", onlyEntry(histogramViewPoint(errorCountHistogram(), "Error Count"))          , 0.55 , 0.735 },
        { "scatterplot"         , onlyEntry(scatterViewPoint())                                                , 0.455, 0.67  },
        { "grid_geographic"     , onlyEntry(geoGridViewPoint())                                                , 0.505, 0.705 }
    };

    //the texts keep some air to the edge of the image
    const int MinEmpty = 3;
    const int MaxEmpty = 18;

    auto checkSpace = [ & ] (const Space& space, double min_plot)
    {
        CHECK(space.plotFraction() >= min_plot);

        for (int empty : { space.emptyLeft(), space.emptyRight(), space.emptyTop(), space.emptyBottom() })
        {
            CHECK(empty >= MinEmpty);
            CHECK(empty <= MaxEmpty);
        }
    };

    for (const auto& c : cases)
    {
        QImage gui_image, export_image;

        const Space gui        = guiSpace(c.entry, gui_image);
        const Space export_img = exportSpace(c.entry, export_image);

        record(c.name + "_gui"   , gui_image   , gui);
        record(c.name + "_export", export_image, export_img);

        {
            INFO(c.name.toStdString() << " in the Reports tool");
            checkSpace(gui, c.min_plot_gui);
        }
        {
            INFO(c.name.toStdString() << " exported");
            checkSpace(export_img, c.min_plot_export);
        }
    }
}

TEST_CASE("the render widget of a view keeps the default chart margins", "[view][reportfigure]")
{
    const auto entry = onlyEntry(histogramViewPoint(errorHistogram(), "Full distribution [m]"));

    auto data = HistogramViewModel::displayDataFromAnnotation(*entry.feature, entry.metadata, nullptr);
    REQUIRE(data);

    HistogramViewRenderWidget widget;
    widget.setDisplayData(data, HistogramRenderOptions());

    QImage image;
    REQUIRE(widget.render(image, GuiSize, 1.0));

    Space space;
    space.size    = GuiSize;
    space.content = contentBox(image, QApplication::palette().color(QPalette::Window));

    CHECK(space.emptyLeft()   >= 20);
    CHECK(space.emptyRight()  >= 20);
    CHECK(space.emptyTop()    >= 20);
    CHECK(space.emptyBottom() >= 20);
}

