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

#include "scatterseries.h"
#include "property.h"
#include "representation.h"

#include <QRectF>

#include <boost/optional.hpp>

#include <string>
#include <vector>

/**
 * Output of ScatterPlotViewModel, the only data the scatterplot render widget draws from.
 * Colors, marker sizes and hidden layers are inside the series.
 */
struct ScatterPlotDisplayData
{
    /// Name of the series that pools the selected points. It is drawn on top and gets no legend entry.
    /// The name is outside the "<ds_type>:<ds_name>:L<n>:<dbcontent>" scheme of the other series, so
    /// the layer tree can show it as a dedicated top-level leaf.
    inline static const std::string SelectedSeriesName = "Selected";

    ScatterSeriesCollection   series;                    // points, name, color, marker size, visible flag.
                                                         // Can carry its own connection lines value.
    std::string               title;
    std::string               x_axis_label;
    std::string               y_axis_label;
    bool                      x_axis_is_datetime = false;
    bool                      y_axis_is_datetime = false;

    // Data type and representation per axis (0 = x, 1 = y). The series hold every value as
    // double, so a tick label can only be formatted correctly if these are kept alongside.
    PropertyDataType          axis_data_type[ 2 ] = { PropertyDataType::DOUBLE,
                                                      PropertyDataType::DOUBLE };
    dbContent::Representation axis_repr[ 2 ]      = { dbContent::Representation::STANDARD,
                                                      dbContent::Representation::STANDARD };

    boost::optional<QRectF>   bounds;                    // of all series, the range of zoom to home

    // Identity of the value domain of the axes. The render widget keeps its axis ranges over a
    // redraw while the id stays the same.
    std::string               domain_id;

    bool                      valid = false;             // false: the render widget shows the empty chart

    /// The visible series in the order they are drawn: the order of the collection, a later series covers
    /// an earlier one. The series of the selected points is the last one, so it is on top.
    std::vector<const ScatterSeriesCollection::DataSeries*> seriesInDrawOrder() const
    {
        std::vector<const ScatterSeriesCollection::DataSeries*> order;
        const ScatterSeriesCollection::DataSeries*              selected = nullptr;

        for (const auto& series_it : series.dataSeries())
        {
            if (!series_it.second.visible)
                continue;

            if (series_it.second.name == SelectedSeriesName)
                selected = &series_it.second;
            else
                order.push_back(&series_it.second);
        }

        if (selected)
            order.push_back(selected);

        return order;
    }
};

enum class ScatterPlotLegendMode
{
    Hidden,   // no legend, the layer panel of the view lists the series (Scatterplot View)
    RightSide // right of the chart as a list, the last entry tells the count of the series that do not fit (report)
};

/**
 * Settings of the scatterplot drawing that come from the owner of the render widget, not from the data.
 */
struct ScatterPlotRenderOptions
{
    bool                  use_connection_lines = false; // used if the display data carries no value
    // true: Qt Charts draws the points with OpenGL, for the millions of points of a view.
    // false: the points are drawn into an image inside the plot area, without an OpenGL surface.
    bool                  use_opengl           = true;
    ScatterPlotLegendMode legend_mode          = ScatterPlotLegendMode::Hidden;
};
