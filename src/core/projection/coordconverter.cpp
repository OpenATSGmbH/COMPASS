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

#include "coordconverter.h"

#include <iostream>

#include <ogr_spatialref.h>
#include <gdal_version.h>

namespace projection
{

/**
*/
std::vector<boost::optional<QPointF>> CoordConverter::convert(const std::vector<QPointF>& points,
                                                              const std::string& srs_src,
                                                              const std::string& srs_dst)
{
    std::vector<boost::optional<QPointF>> points_tr;

    if (points.empty())
        return points_tr;

    size_t n = points.size();

    points_tr.resize(n);

    // Create source and target spatial references
    OGRSpatialReference sourceSRS, targetSRS;

#if GDAL_VERSION_MAJOR >= 3
    sourceSRS.importFromWkt(srs_src.c_str());
    targetSRS.importFromWkt(srs_dst.c_str());
#elif GDAL_VERSION_MAJOR <= 2
    char* source_array = new char[srs_src.length()+1];
    strcpy(source_array, srs_src.c_str()); // Copy the contents of the std::string into the char array

    char* target_array = new char[srs_dst.length()+1];
    strcpy(target_array, srs_dst.c_str()); // Copy the contents of the std::string into the char array

    char* source_array_in = source_array;
    char* target_array_in = target_array;

    sourceSRS.importFromWkt(&source_array_in);
    targetSRS.importFromWkt(&target_array_in);

    delete[] source_array;
    delete[] target_array;
#endif

    // Create the coordinate transformation
    OGRCoordinateTransformation* coordTransform = OGRCreateCoordinateTransformation(&sourceSRS, &targetSRS);
    if (!coordTransform)
        return {};

    // Transform coords
    double x, y;
    for (size_t i = 0; i < n; ++i)
    {
        x = points[ i ].x();
        y = points[ i ].y();
        if (!coordTransform->Transform(1, &x, &y))
            continue;

        points_tr[ i ] = QPointF(x, y);
    }

    // Clean up
    OGRCoordinateTransformation::DestroyCT(coordTransform);

    return points_tr;
}

/**
*/
QRectF CoordConverter::convert(const QRectF& roi,
                               const std::string& srs_src,
                               const std::string& srs_dst)
{
    std::vector<QPointF> points;
    points.push_back(roi.topLeft());
    points.push_back(roi.topRight());
    points.push_back(roi.bottomRight());
    points.push_back(roi.bottomLeft());

    auto points_tr = CoordConverter::convert(points, srs_src, srs_dst);

    boost::optional<double> lon_min, lon_max, lat_min, lat_max;
    for (const auto& pos : points_tr)
    {
        if (!pos.has_value())
            return QRectF();
        
        if (!lon_min.has_value() || pos->x() < lon_min.value()) lon_min = pos->x();
        if (!lat_min.has_value() || pos->y() < lat_min.value()) lat_min = pos->y();
        if (!lon_max.has_value() || pos->x() > lon_max.value()) lon_max = pos->x();
        if (!lat_max.has_value() || pos->y() > lat_max.value()) lat_max = pos->y();
    }

    return QRectF(lon_min.value(), 
                  lat_min.value(), 
                  lon_max.value() - lon_min.value(), 
                  lat_max.value() - lat_min.value());
}

}
