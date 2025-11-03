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

#include "rs2gcoordinatesystem.h"

#include <vector>
#include <tuple>
#include <memory>

namespace dbContent
{

class DataSourceCompoundCoverage
{
public:
    DataSourceCompoundCoverage();

    void clear();
    void addRangeCircle (unsigned int ds_id, double center_lat, double center_long, double range_m);

    bool isInside (double pos_lat, double pos_long) const;

    void finalize();
    bool hasCircles() const;

private:
    bool is_finalized_ {false};

    std::vector<std::tuple<unsigned int, double, double, double>> range_circles_;
    std::vector<std::pair<std::unique_ptr<RS2GCoordinateSystem>, double>> range_circles_cs_; // cs -> range
};

}

