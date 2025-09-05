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

#include "kalman_defs.h"
#include "measurement.h"

#include <ostream>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace reconstruction
{

enum MapProjectionMode
{
    MapProjectNone = 0, //no projection (cartesian coords pre-stored in the measurements are used,
                        //unprojection to geodetic should happen outside of reconstructor)

    MapProjectStatic,   //a single map projection is used for the whole track (origin of projection is set to
                        //center of data bounds, might result in inaccuracies in locations far away from the projection center)
    
    MapProjectDynamic   //the map projection varies dynamically based on a maximum distance threshold (most accurate,
                        //but results in some computation overhead)
};

enum class MapProjDistanceCheck
{
    Cart,
    WGS84
}; 

enum class StateInterpMode
{
    BlendHalf,      // time-projected kalman states will be blended halfways
    BlendLinear,    // time-projected kalman states will be blended linearly with time
    BlendStdDev,    // states will be weighted by time-projected stddevs
    BlendVar        // states will be weighted by time-projected variances
};

/**
*/
struct Reference : public Measurement
{
    bool reset_pos      = false; // is this a position where kalman filter was resetted (e.g. due to long time offset)
    bool nospeed_pos    = false; // did this position not obtain speed information
    bool noaccel_pos    = false; // did this position not obtain acceleration information
    bool nostddev_pos   = false; // did this position not obtain stddev information
    bool projchange_pos = false; // position where a change of map projection happened

    bool ref_interp     = false; // reference has been interpolated (e.g. from kalman samples)

    Eigen::MatrixXd cov; // covariance matrix
    double          dt;  // last timestep (only set for non-interpolated)
};

/**
*/
struct MeasurementInterp : public Measurement
{
    MeasurementInterp() = default;
    MeasurementInterp(const Measurement& mm, 
                      bool is_interp,
                      bool is_interp_first, 
                      bool is_corrected) : Measurement(mm), corrected(is_corrected) 
    {
        mm_interp       = is_interp;
        mm_interp_first = is_interp_first;
    }

    bool corrected = false; //interpolation failed, so the measurement was interpolated linearly
};

/**
*/
struct Uncertainty
{
    double pos_var   = 100.0;
    double speed_var = 100.0;
    double acc_var   = 100.0;
};

/**
*/
struct InterpOptions
{
    double sample_dt = 1.0;
    double max_dt    = 30.0;
};

/**
*/
struct UpdateStats
{
    bool set = false;

    void resetChainInternals()
    {
        num_fresh           = 0;
        num_updated         = 0;
        num_valid           = 0;
        num_failed          = 0;
        num_failed_numeric  = 0;
        num_failed_badstate = 0;
        num_failed_other    = 0;
        num_skipped         = 0;

        num_proj_changed = 0;
    };

    //preemptive insertion check
    size_t num_checked            = 0;
    size_t num_skipped_preemptive = 0;
    size_t num_replaced           = 0;
    size_t num_inserted           = 0;

    //chain internals
    size_t num_fresh           = 0;
    size_t num_updated         = 0;
    size_t num_valid           = 0;
    size_t num_failed          = 0;
    size_t num_failed_numeric  = 0;
    size_t num_failed_badstate = 0;
    size_t num_failed_other    = 0;
    size_t num_skipped         = 0;

    size_t num_proj_changed = 0;
};

/**
*/
struct PredictionStats
{
    size_t num_predictions     = 0;
    size_t num_failed          = 0;
    size_t num_failed_numeric  = 0;
    size_t num_failed_badstate = 0;
    size_t num_failed_other    = 0;
    size_t num_fixed           = 0;

    size_t num_proj_changed = 0;
};

/**
 */
enum PredictionCompareFlags
{
    PredCompLikelihood    = 1 << 0,  // compute the likelihood of the prediction to the provided reference measurement
    PredCompLogLikelihood = 1 << 1,  // compute the log-likelihood of the prediction to the provided reference measurement
    PredCompMahalanobis   = 1 << 2   // compute the mahalanobis distance of the prediction to the provided reference measurement
};

/**
 * Values comparing a predicted kalman state to a reference measurement.
 */
struct PredictionComparison
{
    bool valid() const
    {
        if (likelihood.has_value() && !std::isfinite(likelihood.value()))
            return false;
        if (log_likelihood.has_value() && !std::isfinite(log_likelihood.value()))
            return false;
        if (mahalanobis.has_value() && !std::isfinite(mahalanobis.value()))
            return false;
        
        return true;
    }

    boost::optional<double> likelihood;
    boost::optional<double> log_likelihood;
    boost::optional<double> mahalanobis;
};

/**
 * Reconstruction usage information for a target report.
 */
struct TRUsage
{
    /**
     */
    enum Flags
    {
        Repeated         = 1 << 0,
        SkippedTimeStep  = 1 << 1,
        SkippedNumeric   = 1 << 2,
        SkippedInvalid   = 1 << 3,
        SkippedUnknown   = 1 << 4,
        SkippedSmoothing = 1 << 5,
    };

    bool isRepeated() const
    {
        return (flags & Repeated) != 0;
    };

    unsigned long            rec_num;
    boost::posix_time::ptime t;
    bool                     used  = false;
    unsigned char            flags = 0;
};


} // namespace reconstruction
