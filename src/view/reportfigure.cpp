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

#include "reportfigure.h"
#include "histogramviewmodel.h"
#include "histogramviewrenderwidget.h"
#include "variableview.h"
#include "viewpointgenerator.h"
#include "viewpointkeys.h"
#include "apppalette.h"
#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

#include <QImage>

namespace report_figure
{

const QSize  ExportLayoutSize = QSize(1200, 800);
const double ExportScale      = 2.0;

namespace
{
    const std::string ImageNameHistogram = "histogram";

    //in a narrow window the chart stays readable
    const int WidgetMinHeight = 400;

    /**
     * The settings of a report figure. They are set before the display data, so the widget draws once.
     * palette: set for an image, empty pointer for the report GUI (application palette).
     */
    void configure(ViewRenderWidget& widget, const QPalette* palette)
    {
        widget.setWheelIgnored(true);
        widget.setHeightForWidth(ExportLayoutSize.height() / (double)ExportLayoutSize.width(), WidgetMinHeight);

        if (palette)
            widget.setFixedPalette(*palette);
    }

    /**
     */
    std::unique_ptr<ViewRenderWidget> createHistogramWidget(const RenderEntry& entry,
                                                            QWidget* parent,
                                                            const QPalette* palette)
    {
        std::string error;

        auto data = HistogramViewModel::displayDataFromAnnotation(*entry.feature, entry.metadata, &error);
        if (!data)
            logwrn << "malformed histogram annotation of figure image '" << entry.image_name << "': " << error;

        HistogramRenderOptions options;
        options.legend_mode = HistogramLegendMode::RightSide;

        auto widget = std::make_unique<HistogramViewRenderWidget>(parent);

        configure(*widget, palette);
        widget->setDisplayData(data, options);

        return widget;
    }

    /**
     */
    std::unique_ptr<ViewRenderWidget> createWidget(const RenderEntry& entry, QWidget* parent, const QPalette* palette)
    {
        traced_assert(entry.feature);

        switch (entry.kind)
        {
            case Kind::Histogram:
                return createHistogramWidget(entry, parent, palette);
        }

        traced_assert(false);
        return {};
    }
}

/**
 * A view point loads no data if it lists no data source type or no data source. A view point without
 * these keys loads all of them (FilterManager::showViewPointSlot).
 */
bool loadsNoData(const nlohmann::json& view_point)
{
    if (!view_point.is_object())
        return false;

    for (const auto& key : { ViewPointKeys::VP_DS_TYPES_KEY, ViewPointKeys::VP_DS_KEY })
    {
        auto it = view_point.find(key);

        if (it != view_point.end() && (it->is_array() || it->is_object()) && it->empty())
            return true;
    }

    return false;
}

/**
 */
std::vector<RenderEntry> selectRenderEntries(const nlohmann::json& view_point)
{
    if (!loadsNoData(view_point))
        return {};

    //histogram annotations only
    const auto features = ViewPointGenVP::scanForFeatures(view_point);
    if (features.empty())
        return {};

    for (const auto& f : features)
        if (f.feature_json.at(ViewPointGenFeature::FeatureTypeFieldType) != ViewPointGenFeatureHistogram::FeatureName)
            return {};

    const auto groups = VariableView::scanForAnnotations(view_point, { ViewPointGenFeatureHistogram::FeatureName });

    std::vector<RenderEntry> entries;

    for (const auto& group : groups)
    {
        for (const auto& annotation : group.annotations)
        {
            RenderEntry entry;
            entry.kind             = Kind::Histogram;
            entry.annotation_index = (unsigned int)entries.size();
            entry.image_name       = ImageNameHistogram + "_" + std::to_string(entry.annotation_index);
            entry.feature          = std::make_shared<const nlohmann::json>(annotation.feature_json);
            entry.metadata         = annotation.metadata;

            entries.push_back(entry);
        }
    }

    return entries;
}

/**
 */
std::unique_ptr<ViewRenderWidget> createRenderWidget(const RenderEntry& entry, QWidget* parent)
{
    return createWidget(entry, parent, nullptr);
}

/**
 */
bool renderExportImage(const RenderEntry& entry, QImage& image)
{
    const QPalette palette = app_palette::light();

    return createWidget(entry, nullptr, &palette)->render(image, ExportLayoutSize, ExportScale);
}

}
