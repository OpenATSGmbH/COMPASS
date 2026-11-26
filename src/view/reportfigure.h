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
#include "reportfigureoptions.h"
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
    Grid,
    Map // the overlays of a view point without grid, on the background over longitude / latitude
};

/**
 * One render widget of a report figure. It shows one annotation of the view point of the figure, a map shows
 * all overlays of it.
 */
struct RenderEntry
{
    Kind                                  kind             = Kind::Histogram;
    unsigned int                          annotation_index = 0; // position in the order of the view of this kind
    std::string                           image_name;           // "<kind>_<annotation index>", unique inside one figure
    std::shared_ptr<const nlohmann::json> feature;              // annotation feature, empty for a map
    PlotMetadata                          metadata;

    // a grid over longitude / latitude and a map draw them, other grids ignore them
    bool                                    geographic = false; // a grid over longitude / latitude, or a map
    std::vector<GridTextLabel>              labels;     // the text features of the view point
    std::vector<GridOverlay>                overlays;   // the points, lines and line strings of the view point
    boost::optional<GridBackgroundSettings> background; // the background set of the view point

    QSize export_layout_size = ExportLayoutSize; // of the view point, also gives the height for width in the GUI
};

/// true if the annotations (the annotation array of a view point) hold a grid over longitude / latitude.
/// The tasks write the background set only into such view points.
bool hasGeographicGrid(const nlohmann::json& annotations);

/**
 * Decides how a report figure with this view point is drawn.
 * Entries: one render widget per entry. The histograms first, then the scatterplots, then the grids, each
 * kind in the order of its view.
 * Empty: a view figure (the view point loads data), or a figure that cannot be drawn.
 *
 * A figure gets render widgets if its view point loads no data and holds histogram, scatter series and grid
 * annotations only. With a longitude / latitude grid the view point can also hold overlay features (points,
 * lines, line strings, error ellipses, text): the geographic grids draw the points, lines and line strings,
 * with the texts as labels on top. The error ellipses are not drawn.
 * A view point with overlay features only, at least one of them points, lines or a line string, gives one map
 * entry ("map_0"): the overlays on the background, over the area they cover.
 * The caller leaves out the locked figures and loads the content of on-demand figures first.
 */
std::vector<RenderEntry> selectRenderEntries(const nlohmann::json& view_point);

/// The render widget of an entry, with the display of a report figure: legend at the right side, narrow
/// margins, no interaction (mouse wheel ignored, no tools, no drag), no OpenGL, and the height follows the width
/// with the ratio of the export layout size of the entry. A histogram of one series has a log y axis, a histogram of several
/// (stacked) series a linear one, unless the annotation sets the scale. A geographic grid without a
/// background set in its view point gets GridBackgroundSettings::reportDefaults(), a map too.
/// Created without a model object or a view. ctx_man: gives the sector lines of geographic grids, none
/// without it.
std::unique_ptr<ViewRenderWidget> createRenderWidget(const RenderEntry& entry,
                                                     const context::DBContextManager* ctx_man = nullptr,
                                                     QWidget* parent = nullptr);

/// The image of an entry for the report export, with the light palette: the export layout size of the entry
/// times ExportScale.
bool renderExportImage(const RenderEntry& entry, QImage& image, const context::DBContextManager* ctx_man = nullptr);

}
