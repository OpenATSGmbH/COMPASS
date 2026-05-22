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

#include "targetreportdefs.h"
#include "radarbiasinfo.h"

#include "boost/optional.hpp"

class Projection;
namespace dbContent { class TargetReportAccessor; }

namespace radar_bias
{

/**
 * Retrieve the baro altitude for a radar TR.
 * Returns (true, altitude_ft) if available, (false, 0) otherwise.
 * For ground-only sources or on-ground TRs, uses the given ground altitude.
 */
std::pair<bool, float> getBaroAltitude(
    const dbContent::targetReport::ReconstructorInfo& tr,
    bool ground_only,
    double ground_altitude_m);

/**
 * Apply radar bias correction to a target report by re-projecting its polar
 * coordinates (range/azimuth) through the projection with the given bias.
 *
 * Returns the corrected position, or nullopt on projection failure or missing data.
 *
 * @param tr              target report (needs position_, buffer_index_)
 * @param accessor        accessor for radar range/azimuth
 * @param projection      current projection (must have coordinate systems added)
 * @param ds_id           data source id (for projection lookup)
 * @param bias_info       radar bias to apply
 * @param alt_given       whether altitude is available
 * @param alt_ft          altitude in feet (baro or ground)
 * @param ignore_range_azimuth  if true, derive polar from position (CAT062-like sources)
 */
boost::optional<dbContent::targetReport::Position> correctPosition(
    const dbContent::targetReport::ReconstructorInfo& tr,
    const dbContent::TargetReportAccessor& accessor,
    Projection& projection,
    unsigned int ds_id,
    RadarBiasInfo& bias_info,
    bool alt_given,
    float alt_ft,
    bool ignore_range_azimuth = false);

} // namespace radar_bias
