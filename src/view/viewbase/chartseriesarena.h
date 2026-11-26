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

#include <cstddef>

namespace QtCharts
{
    class QLineSeries;
    class QScatterSeries;
}

/**
 * Allocates the accelerated (OpenGL) series of the charts from one fixed memory block.
 *
 * Qt Charts keeps the accelerated series in a map keyed by the series pointer, so they render in
 * ascending pointer order and the series with the highest pointer is on top. The block has a lower
 * region for the data series and an upper region for the selection series, so the selection of a
 * chart is always above its data, whatever addresses the heap of the platform hands out.
 *
 * The series are normal Qt Charts series: the chart owns and deletes them. Their class returns the
 * memory to the arena. When a region is full, a series is allocated on the heap (logged), without
 * the order guarantee.
 */
namespace chart_series_arena
{
    /// scatter series for the data of a chart, in the lower region
    QtCharts::QScatterSeries* createDataScatterSeries();
    /// line series for the data of a chart (e.g. connection lines), in the lower region
    QtCharts::QLineSeries* createDataLineSeries();
    /// line series for the selection of a chart, in the upper region
    QtCharts::QLineSeries* createSelectionLineSeries();

    /// true if the series memory is in the data region
    bool inDataRegion(const void* series);
    /// true if the series memory is in the selection region
    bool inSelectionRegion(const void* series);

    /// number of slots in use, per region
    size_t numDataSeries();
    size_t numSelectionSeries();
}
