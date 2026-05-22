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
#include "accuracyestimatorbase.h"
#include "radarbiasinfo.h"

#include <map>

class SimpleReconstructor;

class SimpleAccuracyEstimator : public AccuracyEstimatorBase
{
public:
    SimpleAccuracyEstimator();

    virtual void init (ReconstructorBase* reconstructor_ptr) override;
    virtual void postProccessNewSlice() override;

    virtual void validate (dbContent::targetReport::ReconstructorInfo& tr) override;

    virtual bool canCorrectPosition(const dbContent::targetReport::ReconstructorInfo& tr) override;
    virtual void correctPosition(dbContent::targetReport::ReconstructorInfo& tr) override;

    virtual dbContent::targetReport::PositionAccuracy positionAccuracy (
        const dbContent::targetReport::ReconstructorInfo& tr) override;
    virtual dbContent::targetReport::VelocityAccuracy velocityAccuracy (
        const dbContent::targetReport::ReconstructorInfo& tr) override;
    virtual dbContent::targetReport::AccelerationAccuracy accelerationAccuracy (
        const dbContent::targetReport::ReconstructorInfo& tr) override;

private:

    /// Per-channel radar accuracy (polar model).
    struct ChannelAccuracy
    {
        bool valid {false};
        double azimuth_stddev_deg {0};
        double range_stddev_m {0};
    };

    struct RadarSourceInfo
    {
        RadarBiasInfo bias_info;

        ChannelAccuracy primary;   // PSR
        ChannelAccuracy secondary; // SSR
        ChannelAccuracy mode_s;    // Mode S

        bool ground_only {false};
        double ground_altitude_m {0};
        bool ignore_range_azimuth {false};
    };

    const ChannelAccuracy& bestChannelForTR(
        const RadarSourceInfo& src,
        const dbContent::targetReport::ReconstructorInfo& tr) const;

    static dbContent::targetReport::PositionAccuracy polarToCartesianAccuracy(
        double azimuth_stddev_deg, double range_stddev_m,
        double range_m, double bearing_rad);

    std::map<unsigned int, RadarSourceInfo> radar_sources_; // ds_id -> info
};

