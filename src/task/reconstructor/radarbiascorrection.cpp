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

#include "radarbiascorrection.h"
#include "targetreportaccessor.h"
#include "projection.h"
#include "global.h"
#include "logger.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;

namespace radar_bias
{

std::pair<bool, float> getBaroAltitude(
    const dbContent::targetReport::ReconstructorInfo& tr,
    bool ground_only,
    double ground_altitude_m)
{
    if (ground_only || tr.isOnGround())
        return {true, static_cast<float>(ground_altitude_m * M2FT)};

    if (tr.barometric_altitude_)
        return {true, tr.barometric_altitude_->altitude_};

    return {false, 0.0f};
}

boost::optional<dbContent::targetReport::Position> correctPosition(
    const dbContent::targetReport::ReconstructorInfo& tr,
    const dbContent::TargetReportAccessor& accessor,
    Projection& projection,
    unsigned int ds_id,
    RadarBiasInfo& bias_info,
    bool alt_given,
    float alt_ft,
    bool ignore_range_azimuth)
{
    boost::optional<double> range_nm = accessor.radarRange(tr.buffer_index_);
    boost::optional<double> azimuth_deg = accessor.radarAzimuth(tr.buffer_index_);

    double range_m = range_nm ? *range_nm * NM2M : 0.0;

    if (ignore_range_azimuth || !range_nm || !azimuth_deg)
    {
        if (!tr.position_)
            return {};

        double azimuth_calc_rad, slant_range_calc_m, ground_range_calc_m, alt_wgs_m;
        double altitude_m = alt_given ? alt_ft * FT2M : 0.0;

        projection.wgs842PolarHorizontal(ds_id,
                                         tr.position_->latitude_, tr.position_->longitude_, altitude_m,
                                         azimuth_calc_rad, slant_range_calc_m, ground_range_calc_m, alt_wgs_m);

        azimuth_deg = azimuth_calc_rad * RAD2DEG;
        range_m = slant_range_calc_m;
    }

    if (!azimuth_deg)
        return {};

    double lat_cor_deg, lon_cor_deg, alt_wgs_m;
    bool ok;

    if (bias_info.azimuth_bias_valid_ && bias_info.range_bias_valid_)
    {
        ok = projection.polarToWGS84(ds_id, (*azimuth_deg) * DEG2RAD, range_m,
                                     alt_given, alt_ft,
                                     bias_info,
                                     lat_cor_deg, lon_cor_deg, alt_wgs_m);
    }
    else
    {
        ok = projection.polarToWGS84(ds_id, (*azimuth_deg) * DEG2RAD, range_m,
                                     alt_given, alt_ft,
                                     lat_cor_deg, lon_cor_deg, alt_wgs_m);
    }

    if (!ok)
    {
        logdbg << "radar_bias::correctPosition: projection failed"
               << " azimuth_deg " << *azimuth_deg
               << " range_m " << String::doubleToStringPrecision(range_m, 2)
               << " alt_given " << alt_given
               << " alt_ft " << String::doubleToStringPrecision(alt_ft, 2);
        return {};
    }

    return dbContent::targetReport::Position{lat_cor_deg, lon_cor_deg};
}

} // namespace radar_bias
