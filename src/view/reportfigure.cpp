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
#include <set>

namespace report_figure
{

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

    /// Features a geographic grid can have next to it. Only the texts are drawn (labels), the Geographic View
    /// shows the others.
    bool isGeographicOverlay(const std::string& type)
    {
        static const std::set<std::string> types =
        {
            ViewPointGenFeaturePoints::FeatureName,
            ViewPointGenFeatureLineString::FeatureName,
            ViewPointGenFeatureLineString::FeatureNameInterp,
            ViewPointGenFeatureLines::FeatureName,
            ViewPointGenFeatureErrEllipses::FeatureName,
            ViewPointGenFeatureText::FeatureName
        };

        return types.count(type) > 0;
    }

    /// walks the annotation tree without copying the features
    bool annotationHasGeographicGrid(const nlohmann::json& annotation)
    {
        if (!annotation.is_object())
            return false;

        auto features_it = annotation.find(ViewPointGenAnnotation::AnnotationFieldFeatures);
        if (features_it != annotation.end() && features_it->is_array())
        {
            for (const auto& f : *features_it)
            {
                if (!f.is_object())
                    continue;

                auto type_it = f.find(ViewPointGenFeature::FeatureTypeFieldType);
                if (type_it != f.end() && *type_it == ViewPointGenFeatureGrid::FeatureName && isGeographicGrid(f))
                    return true;
            }
        }

        auto children_it = annotation.find(ViewPointGenAnnotation::AnnotationFieldAnnotations);
        if (children_it != annotation.end() && children_it->is_array())
            for (const auto& c : *children_it)
                if (annotationHasGeographicGrid(c))
                    return true;

        return false;
    }

    void collectFeatureTypes(const nlohmann::json& annotation, std::set<std::string>& types)
    {
        if (!annotation.is_object())
        {
            types.insert(std::string());
            return;
        }

        auto features_it = annotation.find(ViewPointGenAnnotation::AnnotationFieldFeatures);
        if (features_it != annotation.end() && features_it->is_array())
        {
            for (const auto& f : *features_it)
            {
                auto type_it = f.is_object() ? f.find(ViewPointGenFeature::FeatureTypeFieldType) : f.end();

                //a feature without a type is no known feature
                types.insert(f.is_object() && type_it != f.end() && type_it->is_string() ? type_it->get<std::string>()
                                                                                         : std::string());
            }
        }

        auto children_it = annotation.find(ViewPointGenAnnotation::AnnotationFieldAnnotations);
        if (children_it != annotation.end() && children_it->is_array())
            for (const auto& c : *children_it)
                collectFeatureTypes(c, types);
    }

    /// the feature types of the view point, without copying the features
    std::set<std::string> featureTypes(const nlohmann::json& view_point)
    {
        std::set<std::string> types;

        auto annotations_it = view_point.find(ViewPointGenVP::ViewPointFieldAnnotations);
        if (annotations_it == view_point.end() || !annotations_it->is_array())
            return types;

        for (const auto& a : *annotations_it)
            collectFeatureTypes(a, types);

        return types;
    }

    //in a narrow window the chart stays readable
    const int WidgetMinHeight = 400;

    /**
     * The settings of a report figure. They are set before the display data, so the widget draws once.
     * image_palette: set for a widget that draws an image of the export. It then builds no display.
     * Empty pointer: widget of the report GUI, with the application palette.
     */
    void configure(ViewRenderWidget& widget, const RenderEntry& entry, const QPalette* image_palette)
    {
        const QSize& layout_size = entry.export_layout_size;

        widget.setWheelIgnored(true);
        widget.setHeightForWidth(layout_size.height() / (double)layout_size.width(), WidgetMinHeight);

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

        //the series are stacked, on a log axis the upper segments would look much smaller than they are
        HistogramRenderOptions options;
        options.legend_mode   = HistogramLegendMode::RightSide;
        options.use_log_scale = data && data->histogram.numDataSeries() <= 1;

        auto widget = std::make_unique<HistogramViewRenderWidget>(parent);

        configure(*widget, entry, image_palette);
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

        configure(*widget, entry, image_palette);

        //the navigate tool of the widget moves the chart with the mouse, a report figure has no interaction
        widget->setDragEnabled(false);

        widget->setDisplayData(data, options);

        return widget;
    }

    /**
     */
    std::unique_ptr<ViewRenderWidget> createGridWidget(const RenderEntry& entry,
                                                       const context::DBContextManager* ctx_man,
                                                       QWidget* parent,
                                                       const QPalette* image_palette)
    {
        std::string error;

        //only a geographic grid uses the sectors, the labels and the background
        const auto sectors = entry.geographic && ctx_man ? GridViewModel::sectorLines(*ctx_man)
                                                         : std::vector<GridSectorLine>();

        auto data = GridViewModel::displayDataFromAnnotation(*entry.feature, entry.metadata, sectors,
                                                             entry.labels, entry.background, &error);
        if (!data)
            logwrn << "malformed grid annotation of figure image '" << entry.image_name << "': " << error;

        //the background set of the view point wins over these
        GridRenderOptions options;
        options.background = GridBackgroundSettings::reportDefaults();

        auto widget = std::make_unique<GridViewRenderWidget>(parent);

        configure(*widget, entry, image_palette);

        //the navigate tool of the widget moves the chart with the mouse, a report figure has no interaction
        widget->setDragEnabled(false);

        widget->setDisplayData(data, options);

        return widget;
    }

    /**
     */
    std::unique_ptr<ViewRenderWidget> createWidget(const RenderEntry& entry,
                                                   const context::DBContextManager* ctx_man,
                                                   QWidget* parent,
                                                   const QPalette* image_palette)
    {
        traced_assert(entry.feature);

        switch (entry.kind)
        {
            case Kind::Histogram:
                return createHistogramWidget(entry, parent, image_palette);
            case Kind::Scatterplot:
                return createScatterplotWidget(entry, parent, image_palette);
            case Kind::Grid:
                return createGridWidget(entry, ctx_man, parent, image_palette);
        }

        traced_assert(false);
        return {};
    }
}

/**
 */
std::vector<RenderEntry> selectRenderEntries(const nlohmann::json& view_point)
{
    if (view_point_load::loadsData(view_point))
        return {};

    auto annotations_it = view_point.find(ViewPointGenVP::ViewPointFieldAnnotations);

    const bool geographic = annotations_it != view_point.end() && hasGeographicGrid(*annotations_it);

    //only annotations of the kinds that have a render widget, and the overlays of a geographic grid
    bool has_render_kind = false;

    for (const auto& type : featureTypes(view_point))
    {
        const bool has_render_widget = std::any_of(kindInfos().begin(), kindInfos().end(),
            [ & ] (const KindInfo& info) { return type == info.feature_type; });

        has_render_kind |= has_render_widget;

        if (!has_render_widget && !(geographic && isGeographicOverlay(type)))
            return {};
    }

    if (!has_render_kind)
        return {};

    std::vector<GridTextLabel>              labels;
    boost::optional<GridBackgroundSettings> background;

    if (geographic)
    {
        labels     = GridViewModel::textLabels(view_point);
        background = GridViewModel::backgroundFromViewPoint(view_point);
    }

    const QSize export_layout_size = ReportFigureOptions::exportLayoutSizeFromViewPoint(view_point);

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
                entry.kind               = info.kind;
                entry.annotation_index   = annotation_index++;
                entry.image_name         = info.image_name + "_" + std::to_string(entry.annotation_index);
                entry.feature            = std::make_shared<const nlohmann::json>(annotation.feature_json);
                entry.metadata           = annotation.metadata;
                entry.export_layout_size = export_layout_size;

                if (info.kind == Kind::Grid && isGeographicGrid(annotation.feature_json))
                {
                    entry.geographic = true;
                    entry.labels     = labels;
                    entry.background = background;
                }

                entries.push_back(entry);
            }
        }
    }

    return entries;
}

/**
 */
std::unique_ptr<ViewRenderWidget> createRenderWidget(const RenderEntry& entry,
                                                     const context::DBContextManager* ctx_man,
                                                     QWidget* parent)
{
    return createWidget(entry, ctx_man, parent, nullptr);
}

/**
 */
bool renderExportImage(const RenderEntry& entry, QImage& image, const context::DBContextManager* ctx_man)
{
    const QPalette palette = app_palette::light();

    return createWidget(entry, ctx_man, nullptr, &palette)->render(image, entry.export_layout_size, ExportScale);
}

/**
 */
bool hasGeographicGrid(const nlohmann::json& annotations)
{
    if (!annotations.is_array())
        return false;

    for (const auto& a : annotations)
        if (annotationHasGeographicGrid(a))
            return true;

    return false;
}

}
