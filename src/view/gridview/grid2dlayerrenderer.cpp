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

#include "grid2dlayerrenderer.h"
#include "grid2dlayer.h"
#include "grid2drendersettings.h"
#include "rasterreference.h"
#include "grid2d_defs.h"
#include "grid2d.h"
#include "colormap.h"

#include <QImage>
#include <QColor>

/**
*/
std::pair<QImage,RasterReference> Grid2DLayerRenderer::render(const Grid2DLayer& layer,
                                                              const Grid2DRenderSettings& settings)
{
    if (!layer.hasData())
        return {};

    const auto& data  = layer.data;
    const auto& flags = layer.flags;

    bool has_flags = layer.hasFlags();

    //create empty (transparent) image
    int pixels_per_cell = std::max(1, settings.pixels_per_cell);

    size_t nrows  = layer.data.rows();
    size_t ncols  = layer.data.cols();

    size_t nrows_out = nrows * pixels_per_cell;
    size_t ncols_out = ncols * pixels_per_cell;

    QImage img(ncols_out, nrows_out, QImage::Format_ARGB32);
        img.fill(QColor(0, 0, 0, 0));

    //create reference corrected for oversampling
    RasterReference ref = layer.ref;

    ref.img_pixel_size_x /= pixels_per_cell;
    ref.img_pixel_size_y /= pixels_per_cell;

    //determine range
    auto range = layer.range();

    double vmin, vmax;
    bool use_colormap_range = false;

    if (settings.min_value.has_value() && settings.max_value.has_value())
    {
        //use min max values specified in settings first
        vmin = settings.min_value.value();
        vmax = settings.max_value.value();
    }
    else if (settings.color_map.canSampleValues())
    {
        //otherwise use range in colormap if available
        vmin = settings.color_map.valueRange()->first;
        vmax = settings.color_map.valueRange()->second;

        use_colormap_range = true;
    }
    else if (range.has_value())
    {
        //otherwise use data range if valid
        vmin = range.value().first;
        vmax = range.value().second;
    }
    else
    {
        //no range => cannot render
        return std::make_pair(img, ref);
    }

    //std::cout << "vmin: " << vmin << ", vmax: " << vmax << std::endl;

    double vrange = vmax - vmin;
    
    auto getColor = [ & ] (size_t cx, size_t cy)
    {
        //handle cell selection
        if (settings.show_selected && has_flags && (flags(cy, cx) & grid2d::CellFlags::CellSelected))
            return settings.color_map.specialColor(ColorMap::SpecialColorSelected);

        //flip y-index if north is up
        if (layer.ref.is_north_up)
            cy = nrows - 1 - cy;

        if (data(cy, cx) == Grid2D::InvalidValue)
            return QColor(0, 0, 0, 0);

        double v = data(cy, cx);
        if (use_colormap_range)
            return settings.color_map.sampleValue(v);

        double t = (v - vmin) / vrange;

        return settings.color_map.sample(t);
    };

    for (size_t y = 0, y0 = 0; y < nrows; ++y, y0 += pixels_per_cell)
    {
        for (size_t x = 0, x0 = 0; x < ncols; ++x, x0 += pixels_per_cell)
        {
            QColor col = getColor(x, y);

            for (int y2 = 0; y2 < pixels_per_cell; ++y2)
                for (int x2 = 0; x2 < pixels_per_cell; ++x2)
                    img.setPixelColor(x0 + x2, y0 + y2, col);
        }
    }

    return std::make_pair(img, ref);
}
