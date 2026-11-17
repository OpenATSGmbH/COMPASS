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
#include "scatterplotviewmodel.h"
#include "scatterplotviewrenderwidget.h"
#include "gridviewmodel.h"
#include "gridviewrenderwidget.h"
#include "grid2dlayer.h"
#include "variableview.h"
#include "viewpointgenerator.h"
#include "viewpointkeys.h"
#include "apppalette.h"
#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

#include <QImage>

#include <algorithm>

namespace report_figure
{

const QSize  ExportLayoutSize = QSize(1200, 800);
const double ExportScale      = 2.0;

namespace
{
    /// The kinds in the order of their render widgets in a figure, with the feature type of their
    /// annotations and the prefix of their image names.
    struct KindInfo
    {
        Kind        kind;
        std::string feature_type;
        std::string image_name;
    };

    const std::vector<KindInfo>& kindInfos()
    {
        static const std::vector<KindInfo> infos =
        {
            { Kind::Histogram  , ViewPointGenFeatureHistogram::FeatureName    , "histogram"   },
            { Kind::Scatterplot, ViewPointGenFeatureScatterSeries::FeatureName, "scatterplot" },
            { Kind::Grid       , ViewPointGenFeatureGrid::FeatureName         , "grid"        }
        };

        return infos;
    }

    /**
     * True for a grid feature over longitude / latitude. Reads the reference only, not the values.
     * A grid without a readable reference is no geographic grid, its render widget shows the empty chart.
     */
    bool isGeographicGrid(const nlohmann::json& feature)
    {
        const auto grid_it = feature.find(ViewPointGenFeatureGrid::FeatureGridFieldNameGrid);
        if (grid_it == feature.end() || !grid_it->is_object())
            return false;

        const auto ref_it = grid_it->find(Grid2DLayer::TagRef);
        if (ref_it == grid_it->end())
            return false;

        RasterReference ref;
        if (!ref.fromJSON(*ref_it))
            return false;

        return Grid2DLayer::isGeographicSRS(ref.srs);
    }

    //in a narrow window the chart stays readable
    const int WidgetMinHeight = 400;

    /**
     * The settings of a report figure. They are set before the display data, so the widget draws once.
     * image_palette: set for a widget that draws an image of the export. It then builds no display.
     * Empty pointer: widget of the report GUI, with the application palette.
     */
    void configure(ViewRenderWidget& widget, const QPalette* image_palette)
    {
        widget.setWheelIgnored(true);
        widget.setHeightForWidth(ExportLayoutSize.height() / (double)ExportLayoutSize.width(), WidgetMinHeight);

        if (image_palette)
        {
            widget.setFixedPalette(*image_palette);
            widget.setImageOnly(true);
        }
    }

    /**
     */
    std::unique_ptr<ViewRenderWidget> createHistogramWidget(const RenderEntry& entry,
                                                            QWidget* parent,
                                                            const QPalette* image_palette)
    {
        std::string error;

        auto data = HistogramViewModel::displayDataFromAnnotation(*entry.feature, entry.metadata, &error);
        if (!data)
            logwrn << "malformed histogram annotation of figure image '" << entry.image_name << "': " << error;

        HistogramRenderOptions options;
        options.legend_mode = HistogramLegendMode::RightSide;

        auto widget = std::make_unique<HistogramViewRenderWidget>(parent);

        configure(*widget, image_palette);
        widget->setDisplayData(data, options);

        return widget;
    }

    /**
     */
    std::unique_ptr<ViewRenderWidget> createScatterplotWidget(const RenderEntry& entry,
                                                              QWidget* parent,
                                                              const QPalette* image_palette)
    {
        std::string error;

        auto data = ScatterPlotViewModel::displayDataFromAnnotation(*entry.feature, entry.metadata, &error);
        if (!data)
            logwrn << "malformed scatter series annotation of figure image '" << entry.image_name << "': " << error;

        //a report draws with the CPU only, a section with several figures creates no OpenGL surfaces
        ScatterPlotRenderOptions options;
        options.use_opengl  = false;
        options.legend_mode = ScatterPlotLegendMode::RightSide;

        auto widget = std::make_unique<ScatterPlotViewRenderWidget>(parent);

        configure(*widget, image_palette);

        //the navigate tool of the widget moves the chart with the mouse, a report figure has no interaction
        widget->setDragEnabled(false);

        widget->setDisplayData(data, options);

        return widget;
    }

    /**
     */
    std::unique_ptr<ViewRenderWidget> createGridWidget(const RenderEntry& entry,
                                                       QWidget* parent,
                                                       const QPalette* image_palette)
    {
        std::string error;

        //grids over other axes have no sectors
        auto data = GridViewModel::displayDataFromAnnotation(*entry.feature, entry.metadata, {}, &error);
        if (!data)
            logwrn << "malformed grid annotation of figure image '" << entry.image_name << "': " << error;

        auto widget = std::make_unique<GridViewRenderWidget>(parent);

        configure(*widget, image_palette);

        //the navigate tool of the widget moves the chart with the mouse, a report figure has no interaction
        widget->setDragEnabled(false);

        widget->setDisplayData(data, GridRenderOptions());

        return widget;
    }

    /**
     */
    std::unique_ptr<ViewRenderWidget> createWidget(const RenderEntry& entry, QWidget* parent, const QPalette* image_palette)
    {
        traced_assert(entry.feature);

        switch (entry.kind)
        {
            case Kind::Histogram:
                return createHistogramWidget(entry, parent, image_palette);
            case Kind::Scatterplot:
                return createScatterplotWidget(entry, parent, image_palette);
            case Kind::Grid:
                return createGridWidget(entry, parent, image_palette);
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

    //only annotations of the kinds that have a render widget
    const auto features = ViewPointGenVP::scanForFeatures(view_point);
    if (features.empty())
        return {};

    for (const auto& f : features)
    {
        const auto& type = f.feature_json.at(ViewPointGenFeature::FeatureTypeFieldType);

        const bool has_render_widget = std::any_of(kindInfos().begin(), kindInfos().end(),
            [ & ] (const KindInfo& info) { return type == info.feature_type; });

        if (!has_render_widget)
            return {};

        //the background of geographic grids is not in the report figures yet, the views show them
        if (type == ViewPointGenFeatureGrid::FeatureName && isGeographicGrid(f.feature_json))
            return {};
    }

    std::vector<RenderEntry> entries;

    for (const auto& info : kindInfos())
    {
        //the order of the view of this kind
        const auto groups = VariableView::scanForAnnotations(view_point, { info.feature_type });

        unsigned int annotation_index = 0;

        for (const auto& group : groups)
        {
            for (const auto& annotation : group.annotations)
            {
                RenderEntry entry;
                entry.kind             = info.kind;
                entry.annotation_index = annotation_index++;
                entry.image_name       = info.image_name + "_" + std::to_string(entry.annotation_index);
                entry.feature          = std::make_shared<const nlohmann::json>(annotation.feature_json);
                entry.metadata         = annotation.metadata;

                entries.push_back(entry);
            }
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
