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

#include "plotmetadata.h"
#include "griddisplaydata.h"
#include "json_fwd.hpp"

#include <QSize>

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <vector>

class ViewRenderWidget;

class QImage;
class QWidget;

namespace context
{
    class DBContextManager;
}

/**
 * Report figures that are drawn by render widgets, without a view.
 * The report GUI shows the render widgets, the report export renders them into images.
 */
namespace report_figure
{

enum class Kind
{
    Histogram,
    Scatterplot,
    Grid
};

/**
 * One render widget of a report figure. It shows one annotation of the view point of the figure.
 */
struct RenderEntry
{
    Kind                                  kind             = Kind::Histogram;
    unsigned int                          annotation_index = 0; // position in the order of the view of this kind
    std::string                           image_name;           // "<kind>_<annotation index>", unique inside one figure
    std::shared_ptr<const nlohmann::json> feature;              // annotation feature, never empty
    PlotMetadata                          metadata;

    // a grid over longitude / latitude draws them, other grids ignore them
    bool                                    geographic = false; // a grid over longitude / latitude
    std::vector<GridTextLabel>              labels;     // the text features of the view point
    boost::optional<GridBackgroundSettings> background; // the background set of the view point
};

/// An exported image has the layout of this size, with ExportScale * ExportScale pixels per layout pixel.
/// More pixels alone make the chart texts smaller, their font size is fixed.
extern const QSize ExportLayoutSize;
extern const double ExportScale;

/// true: the view point loads no data, it shows its annotations only
bool loadsNoData(const nlohmann::json& view_point);

/// true if the annotations (the annotation array of a view point) hold a grid over longitude / latitude.
/// The tasks write the background set only into such view points.
bool hasGeographicGrid(const nlohmann::json& annotations);

/**
 * Decides how a report figure with this view point is drawn.
 * Entries: one render widget per entry. The histograms first, then the scatterplots, then the grids, each
 * kind in the order of its view.
 * Empty: the figure is shown by the views ("View" button, image of each view with content).
 *
 * A figure gets render widgets if its view point loads no data and holds histogram, scatter series and grid
 * annotations only. With a longitude / latitude grid the view point can also hold overlay features (points,
 * lines, error ellipses, text): the texts are the labels of the geographic grids, the other overlays are not
 * drawn. The caller leaves out the figures that are locked or built on demand.
 */
std::vector<RenderEntry> selectRenderEntries(const nlohmann::json& view_point);

/// The render widget of an entry, with the display of a report figure: legend at the right side, no
/// interaction (mouse wheel ignored, no tools, no drag), no OpenGL, and the height follows the width
/// with the ratio of the export image. A geographic grid without a background set in its view point gets
/// GridBackgroundSettings::reportDefaults().
/// Created without a model object or a view. ctx_man: gives the sector lines of geographic grids, none
/// without it.
std::unique_ptr<ViewRenderWidget> createRenderWidget(const RenderEntry& entry,
                                                     const context::DBContextManager* ctx_man = nullptr,
                                                     QWidget* parent = nullptr);

/// The image of an entry for the report export, with the light palette.
bool renderExportImage(const RenderEntry& entry, QImage& image, const context::DBContextManager* ctx_man = nullptr);

}
