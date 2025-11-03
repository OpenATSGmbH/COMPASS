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

#include "datasourcecompoundcoverage.h"
#include "logger.h"
#include "traced_assert.h"

using namespace std;

namespace dbContent
{

DataSourceCompoundCoverage::DataSourceCompoundCoverage()
{
}

void DataSourceCompoundCoverage::clear()
{
    range_circles_.clear();
    range_circles_cs_.clear();

    is_finalized_ = false;
}

void DataSourceCompoundCoverage::addRangeCircle (unsigned int ds_id, double center_lat, double center_long, double range_m)
{
    range_circles_.push_back(std::tuple<unsigned int, double, double, double> {
                                 ds_id, center_lat, center_long, range_m});
}

bool DataSourceCompoundCoverage::isInside (double pos_lat, double pos_long) const
{
    traced_assert(is_finalized_);

    // if no info, true
    if (!range_circles_cs_.size())
    {
        logdbg << "no circ, true";

        return true;
    }

    // check inside any

    double local_x, local_y, local_z;
    double pos_rng_m;

    for (auto& rng_circ : range_circles_cs_)
    {
        rng_circ.first->geodesic2LocalCart(pos_lat, pos_long, 0, local_x, local_y, local_z);

        pos_rng_m = sqrt(pow(local_x, 2) + pow(local_y, 2));

        if (pos_rng_m <= rng_circ.second)
        {
            logdbg << "inside circ, true";
            return true;
        }
        else
            logdbg << "outside circ, range "
                   << pos_rng_m << " max " << rng_circ.second << ", false";
    }

    logdbg << "not inside any circ, false";
    return false;
}

void DataSourceCompoundCoverage::finalize()
{
    traced_assert(!is_finalized_);

    for (auto& rng_circ : range_circles_)
    {
        std::unique_ptr<RS2GCoordinateSystem> ptr;
        ptr.reset(new RS2GCoordinateSystem(get<0>(rng_circ), get<1>(rng_circ), get<2>(rng_circ), 0));

        range_circles_cs_.emplace_back(
                    std::make_pair(std::move(ptr), get<3>(rng_circ)));
    }

    is_finalized_ = true;
}

bool DataSourceCompoundCoverage::hasCircles() const
{
    traced_assert(is_finalized_);

    return range_circles_cs_.size();
}

}
