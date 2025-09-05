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

#include "spline_interpolator.h"
#include "kalman_interface.h"

#include "util/number.h"
#include "util/timeconv.h"

#include "spline.h"

#include <Eigen/Eigenvalues>

namespace reconstruction
{

namespace
{
    double interp(double v0, double v1, double f)
    {
        return (1.0 - f) * v0 + f * v1;
    }

    void interpolateVector2D(boost::optional<double>& vx_interp,
                             boost::optional<double>& vy_interp,
                             const boost::optional<double>& vx0,
                             const boost::optional<double>& vy0,
                             const boost::optional<double>& vx1,
                             const boost::optional<double>& vy1,
                             double interp_factor)
    {
        //convert to polar
        auto speed_angle0 = Utils::Number::speedVec2SpeedAngle(vx0.value(), vy0.value());
        auto speed_angle1 = Utils::Number::speedVec2SpeedAngle(vx1.value(), vy1.value());

        if (speed_angle0.first < 1e-07 || speed_angle1.first < 1e-07)
        {
            //speed value very small => interpolate speed vectors linear instead
            vx_interp = interp(vx0.value(), vx1.value(), interp_factor);
            vy_interp = interp(vy0.value(), vy1.value(), interp_factor);
        }
        else
        {
            //interpolate speed and bearing individually
            double speed_interp    = interp(speed_angle0.first, speed_angle1.first, interp_factor);
            double da              = Utils::Number::calculateMinAngleDifference(speed_angle1.second, speed_angle0.second);
            double angle_interp    = speed_angle0.second + interp_factor * da;
            auto   speedvec_interp = Utils::Number::speedAngle2SpeedVec(speed_interp, angle_interp);

            vx_interp = speedvec_interp.first;
            vy_interp = speedvec_interp.second;
        }
    }
}

/**
 * Interpolate state vector (usually containing position and e.g. velocity vector, acceleration vector).
*/
Eigen::VectorXd SplineInterpolator::interpStateVector(const Eigen::VectorXd& x0, 
                                                      const Eigen::VectorXd& x1, 
                                                      double interp_factor)
{
    return (1.0 - interp_factor) * x0 + interp_factor * x1;
}

/**
 * Interpolate covariance matrices.
 */
Eigen::MatrixXd SplineInterpolator::interpCovarianceMat(const Eigen::MatrixXd& C0, 
                                                        const Eigen::MatrixXd& C1, 
                                                        double interp_factor,
                                                        CovMatInterpMode covmat_interp_mode,
                                                        bool* ok)
{
    if (ok)
        *ok = false;

    Eigen::MatrixXd C;
    bool is_ok = false;

    if (covmat_interp_mode == CovMatInterpMode::WassersteinDistance)
    {
        //https://stats.stackexchange.com/questions/351043/interpolate-covariance-matrix
        //https://link.springer.com/article/10.1007/s41884-021-00052-8

        //C(t) = (1-t)^2 * C0 + t^2 * C1 + (1-t) * t * [ (C0*C1)^(1/2) + (C1*C0)^(1/2) ]

        //should
        // - yield SPD matrices
        // - interpolate C0 and C1 smoothly

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver01(C0 * C1);
        auto C0C1_sqrt = solver01.operatorSqrt();

        if (solver01.info() == Eigen::ComputationInfo::Success)
        {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver10(C1 * C0);
            auto C1C0_sqrt = solver10.operatorSqrt();

            if (solver10.info() == Eigen::ComputationInfo::Success)
            {
                const double t0 = (1.0 - interp_factor);
                const double t1 = interp_factor;

                C     = t0*t0 * C0 + t1*t1 * C1 + t0*t1 * (C0C1_sqrt + C1C0_sqrt);
                is_ok = true;
            }
        }
    }
    else if (covmat_interp_mode == CovMatInterpMode::NearestNeighbor)
    {
        //simple nearest neighbor
        C     = interp_factor <= 0.5 ? C0 : C1;
        is_ok = true;
    }
    else if (covmat_interp_mode == CovMatInterpMode::Linear)
    {
        //linear interpolation of cov matrices should lead to symmetrical semi-positive-definite matrices in interval t=[0,1]
        //(see white, padmanabhan - including parameter dependence in the data and covariance for cosmological inference)
        // => not sure if this is true though

        //(1 - t) * COV0 + t * COV1 = (1 - t) * |stddev_x0^2 cov_xy0    | + t * |stddev_x1^2 cov_xy1    |
        //                                      |cov_xy0     stddev_y0^2|       |cov_xy1     stddev_y1^2|

        C     = (1.0 - interp_factor) * C0 + interp_factor * C1;
        is_ok = true;
    }

    if (ok)
        *ok = is_ok;

    return C;
}

/**
 * Interpolate covariance matrix values stored in the given measurements.
 */
void SplineInterpolator::interpCovarianceMat(Measurement& mm_interp,
                                             const Measurement& mm0,
                                             const Measurement& mm1,
                                             double interp_factor,
                                             CovMatInterpMode covmat_interp_mode)
{
    //this mode is handled separately, because it needs constructed covariance matrices for interpolation
    if (covmat_interp_mode == CovMatInterpMode::WassersteinDistance)
    {
        //get covariance mat entries available in both matrices
        unsigned char flags = mm0.covMatFlags() & mm1.covMatFlags();

        //interpolate covmats
        bool cov_mat_ok;
        auto C = interpCovarianceMat(mm0.covMat(flags), mm0.covMat(flags), interp_factor, covmat_interp_mode, &cov_mat_ok);
        traced_assert(cov_mat_ok);

        //set entries which were available in the original matrices
        mm_interp.setFromCovMat(C, flags);

        return;
    }

    //all other modes are handled on a per-accuracy-type basis
    if (mm0.hasStdDevPosition() && mm1.hasStdDevPosition())
    {
        if (covmat_interp_mode == CovMatInterpMode::NearestNeighbor)
        {
            mm_interp.x_stddev = interp_factor <= 0.5 ? mm0.x_stddev.value() : mm1.x_stddev.value();
            mm_interp.y_stddev = interp_factor <= 0.5 ? mm0.y_stddev.value() : mm1.y_stddev.value();
        }
        else if (covmat_interp_mode == CovMatInterpMode::Linear)
        {
            double x_stddev_sqr0 = mm0.x_stddev.value() * mm0.x_stddev.value();
            double y_stddev_sqr0 = mm0.y_stddev.value() * mm0.y_stddev.value();

            double x_stddev_sqr1 = mm1.x_stddev.value() * mm1.x_stddev.value();
            double y_stddev_sqr1 = mm1.y_stddev.value() * mm1.y_stddev.value();

            mm_interp.x_stddev = std::sqrt(interp(x_stddev_sqr0, x_stddev_sqr1, interp_factor));
            mm_interp.y_stddev = std::sqrt(interp(y_stddev_sqr0, y_stddev_sqr1, interp_factor));
        }
    }
    if (mm0.xy_cov.has_value() && mm1.xy_cov.has_value())
    {
        if (covmat_interp_mode == CovMatInterpMode::NearestNeighbor)
        {
            mm_interp.xy_cov = interp_factor <= 0.5 ? mm0.xy_cov.value() : mm1.xy_cov.value();
        }
        else if (covmat_interp_mode == CovMatInterpMode::Linear)
        {
            mm_interp.xy_cov = interp(mm0.xy_cov.value(), mm1.xy_cov.value(), interp_factor);
        }
    }
    if (mm0.hasStdDevVelocity() && mm1.hasStdDevVelocity())
    {
        if (covmat_interp_mode == CovMatInterpMode::NearestNeighbor)
        {
            mm_interp.vx_stddev = interp_factor <= 0.5 ? mm0.vx_stddev.value() : mm1.vx_stddev.value();
            mm_interp.vy_stddev = interp_factor <= 0.5 ? mm0.vy_stddev.value() : mm1.vy_stddev.value();
        }
        else if (covmat_interp_mode == CovMatInterpMode::Linear)
        {
            double vx_stddev_sqr0 = mm0.vx_stddev.value() * mm0.vx_stddev.value();
            double vy_stddev_sqr0 = mm0.vy_stddev.value() * mm0.vy_stddev.value();

            double vx_stddev_sqr1 = mm1.vx_stddev.value() * mm1.vx_stddev.value();
            double vy_stddev_sqr1 = mm1.vy_stddev.value() * mm1.vy_stddev.value();

            mm_interp.vx_stddev = std::sqrt(interp(vx_stddev_sqr0, vx_stddev_sqr1, interp_factor));
            mm_interp.vy_stddev = std::sqrt(interp(vy_stddev_sqr0, vy_stddev_sqr1, interp_factor));
        }
    }
    if (mm0.hasStdDevAccel() && mm1.hasStdDevAccel())
    {
        if (covmat_interp_mode == CovMatInterpMode::NearestNeighbor)
        {
            mm_interp.ax_stddev = interp_factor <= 0.5 ? mm0.ax_stddev.value() : mm1.ax_stddev.value();
            mm_interp.ay_stddev = interp_factor <= 0.5 ? mm0.ay_stddev.value() : mm1.ay_stddev .value();
        }
        else if (covmat_interp_mode == CovMatInterpMode::Linear)
        {
            double ax_stddev_sqr0 = mm0.ax_stddev.value() * mm0.ax_stddev.value();
            double ay_stddev_sqr0 = mm0.ay_stddev.value() * mm0.ay_stddev.value();

            double ax_stddev_sqr1 = mm1.ax_stddev.value() * mm1.ax_stddev.value();
            double ay_stddev_sqr1 = mm1.ay_stddev.value() * mm1.ay_stddev.value();

            mm_interp.ax_stddev = std::sqrt(interp(ax_stddev_sqr0, ax_stddev_sqr1, interp_factor));
            mm_interp.ay_stddev = std::sqrt(interp(ay_stddev_sqr0, ay_stddev_sqr1, interp_factor));
        }
    }
}

/**
*/
boost::optional<float> SplineInterpolator::interpProcessNoise(const boost::optional<float>& Q_var0,
                                                              const boost::optional<float>& Q_var1,
                                                              double interp_factor)
{
    if (!Q_var0.has_value() && !Q_var1.has_value())
        return boost::optional<float>();

    if (Q_var0.has_value() && !Q_var1.has_value())
    {
        return Q_var0;
    }
    else if (!Q_var0.has_value() && Q_var1.has_value())
    {
        return Q_var1;
    }
    else //both available
    {
        if (Q_var0.value() == Q_var1.value())
            return Q_var0;

        //interpolate process noise linear
        double Q_std0 = std::sqrt(Q_var0.value());
        double Q_std1 = std::sqrt(Q_var1.value());
        double Q_std  = interp(Q_std0, Q_std1, interp_factor);

        return Q_std * Q_std;
    }

    return boost::optional<float>();
}

/**
*/
MeasurementInterp SplineInterpolator::interpMeasurement(const Eigen::Vector2d& pos, 
                                                        const boost::posix_time::ptime& t, 
                                                        const Measurement& mm0, 
                                                        const Measurement& mm1, 
                                                        double interp_factor,
                                                        CoordSystem coord_sys,
                                                        CovMatInterpMode covmat_interp_mode)
{
    if (interp_factor == 0.0)
        return MeasurementInterp(mm0, true, true, false);
    if (interp_factor == 1.0)
        return MeasurementInterp(mm1, true, true, false);

    MeasurementInterp mm_interp;
    mm_interp.source_id       = mm0.source_id; //!note: other components (e.g. reference calculation) rely on this assumption!
    mm_interp.t               = t;
    mm_interp.mm_interp       = true;
    mm_interp.mm_interp_first = false;

    traced_assert(interp_factor >= 0.0 && interp_factor <= 1.0);
    
    //set position
    mm_interp.position2D(pos, coord_sys);

    //interpolate speed/acc vectors
    if (mm0.hasVelocity() && mm1.hasVelocity())
    {
        interpolateVector2D(mm_interp.vx, mm_interp.vy, mm0.vx, mm0.vy, mm1.vx, mm1.vy, interp_factor);
    }
    if (mm0.hasAcceleration() && mm1.hasAcceleration())
    {
        interpolateVector2D(mm_interp.ax, mm_interp.ay, mm0.ax, mm0.ay, mm1.ax, mm1.ay, interp_factor);
    }

    //interpolate cov mat values
    interpCovarianceMat(mm_interp, mm0, mm1, interp_factor, covmat_interp_mode);

    //interpolate process noise if set
    if (mm0.Q_var.has_value() || mm1.Q_var.has_value())
        mm_interp.Q_var = interpProcessNoise(mm0.Q_var, mm1.Q_var, interp_factor);
    
    //interpolate process noise used for interpolation if set
    if (mm0.Q_var_interp.has_value() || mm1.Q_var_interp.has_value())
        mm_interp.Q_var_interp = interpProcessNoise(mm0.Q_var_interp, mm1.Q_var_interp, interp_factor);
    
    return mm_interp;
}

/**
*/
CoordSystem SplineInterpolator::coordSystem() const
{
    return (config_.interpolate_cart ? CoordSystem::Cart :
                                       CoordSystem::WGS84);
}

/**
*/
Eigen::Vector2d SplineInterpolator::position2D(const Measurement& mm) const
{
    return mm.position2D(coordSystem());
}

/**
*/
void SplineInterpolator::position2D(Measurement& mm, const Eigen::Vector2d& pos) const
{
    return mm.position2D(pos, coordSystem());
}

/**
*/
MeasurementInterp SplineInterpolator::generateMeasurement(const Eigen::Vector2d& pos, 
                                                          const boost::posix_time::ptime& t,
                                                          const Measurement& mm0,
                                                          const Measurement& mm1,
                                                          double interp_factor,
                                                          bool corrected) const
{
    auto mm_interp = SplineInterpolator::interpMeasurement(pos, t, mm0, mm1, interp_factor, coordSystem(), config_.covmat_interp_mode);
    mm_interp.corrected = corrected;

    return mm_interp;
}

/**
*/
MeasurementInterp SplineInterpolator::generateMeasurement(const Measurement& mm) const
{
    return generateMeasurement(position2D(mm), mm.t, mm, mm, 0.0, false);
}

/**
*/
size_t SplineInterpolator::estimatedSamples(const Measurement& mm0, 
                                            const Measurement& mm1,
                                            double dt)
{
    return (size_t)(std::ceil(KalmanInterface::timestep(mm0, mm1) / dt) + 2);
}

/**
*/
void SplineInterpolator::finalizeInterp(std::vector<MeasurementInterp>& samples, const Measurement& mm_last) const
{
    //remove last sample
    if (samples.size() >= 2)
        samples.pop_back();

    //replace with last input measurement
    samples.push_back(generateMeasurement(mm_last));
}

namespace
{
    void determineFirstPosInIntervalFlag(MeasurementInterp& mm_interp,
                                         const boost::posix_time::ptime& interval_t0,
                                         const std::vector<MeasurementInterp>& last_measurements,
                                         const boost::optional<size_t>& idx)
    {
        assert(!idx.has_value() || idx.value() < last_measurements.size());

        mm_interp.mm_interp_first = true;

        //last measurements empty => must be first in interval
        if (last_measurements.empty())
            return;

        //current mm's index is at 0 => must be first in interval
        if (idx.has_value() && idx.value() == 0)
            return;

        const auto& last_mm = idx.has_value() ? last_measurements[ idx.value() - 1 ] : last_measurements.back();

        //last mm is in interval and is already first => can't be first
        if (last_mm.t >= interval_t0 && last_mm.mm_interp_first)
        {
            mm_interp.mm_interp_first = false;
            return;
        }

        //must be first
    }
}

/**
*/
std::vector<MeasurementInterp> SplineInterpolator::interpolateLinear(const std::vector<Measurement>& measurements) const
{
    if (measurements.empty())
        return {};

    size_t n = measurements.size();

    std::vector<MeasurementInterp> interpolation;
    interpolation.push_back(generateMeasurement(measurements[ 0 ]));

    if (n == 1)
        return interpolation;

    boost::posix_time::milliseconds time_incr((int)(config().sample_dt * 1000.0));

    auto tcur = measurements[ 0 ].t + time_incr;

    interpolation.reserve(estimatedSamples(measurements.front(), measurements.back(), config_.sample_dt));

    for (size_t i = 1; i < n; ++i)
    {
        const auto& mm0 = measurements[i - 1];
        const auto& mm1 = measurements[i    ];

        auto pos0 = position2D(mm0);
        auto pos1 = position2D(mm1);

        auto t0 = mm0.t;
        auto t1 = mm1.t;

        double dt01 = Utils::Time::partialSeconds(t1 - t0);

        bool first = true;
        while (tcur >= t0 && tcur < t1)
        {
            if (dt01 < config().min_dt)
            {
                //very small time interval => store second measurement and leave interval
                interpolation.push_back(generateMeasurement(measurements[ i ]));
                tcur += time_incr;
                break;
            }

            //interpolate between measurements
            double dt = Utils::Time::partialSeconds(tcur - t0);
            double f  = dt / dt01;

            auto pos_interp = pos0 * (1.0 - f) + pos1 * f;

            auto mm_gen = generateMeasurement(pos_interp, tcur, mm0, mm1, f, false);

            //detect if this is the first sample in the interval
            if (first)
            {
                determineFirstPosInIntervalFlag(mm_gen, t0, interpolation, {});
                first = false;
            }
            
            interpolation.push_back(mm_gen);

            tcur += time_incr;
        }
    }

    finalizeInterp(interpolation, measurements.back());

    interpolation.shrink_to_fit();

    return interpolation;
}

/**
*/
std::vector<std::vector<Measurement>> SplineInterpolator::splitMeasurements(const std::vector<Measurement>& measurements,
                                                                            double max_dt)
{
    size_t n = measurements.size();
    if (n == 0)
        return {};

    std::vector<std::vector<Measurement>> split_measurements;
    std::vector<Measurement> current;

    current.push_back(measurements[ 0 ]);

    for (size_t i = 1; i < n; ++i)
    {
        const auto& mm0 = measurements[i - 1];
        const auto& mm1 = measurements[i    ];

        double dt = KalmanInterface::timestep(mm0, mm1);

        if (dt > max_dt)
        {
            if (current.size() > 0)
            {
                split_measurements.push_back(current);
                current.clear();
            }
        }

        current.push_back(mm1);
    }

    if (current.size() > 0)
        split_measurements.push_back(current);

    return split_measurements;
}

/**
*/
std::vector<MeasurementInterp> SplineInterpolator::interpolate(const std::vector<Measurement>& measurements)
{
    size_t n = measurements.size();

    if (n == 0)
        return {};

    //split up measurements time-based
    auto measurements_parts = SplineInterpolator::splitMeasurements(measurements, config().max_dt);

    std::vector<MeasurementInterp> interpolation;

    //interpolate individual parts
    for (const auto& p : measurements_parts)
    {
        auto res = interpolatePart(p);

        if (res.size() > 0)
            interpolation.insert(interpolation.end(), res.begin(), res.end());
    }
    
    return interpolation;
}

/**
*/
bool SplineInterpolator::isFishySegment(const Measurement& mm0, 
                                        const Measurement& mm1, 
                                        const std::vector<MeasurementInterp>& segment,
                                        size_t n) const
{
    auto p0 = position2D(mm0);
    auto p1 = position2D(mm1);

    double d_seg = (p0 - p1).norm();
    double d_max = d_seg * config().max_segment_distance_factor;

    auto p_mid = (p0 + p1) / 2;
    
    for (size_t i = 0; i < n; ++i)
    {
        double d = (p_mid - position2D(segment[ i ])).norm();
        if (d > d_max)
            return true;
    }

    return false;
}

/**
*/
std::vector<MeasurementInterp> SplineInterpolator::interpolatePart(const std::vector<Measurement>& measurements) const
{
    size_t n = measurements.size();

    if (n == 0)
        return {};

    if (n < 4)
        return interpolateLinear(measurements);

    auto offs = position2D(measurements[ 0 ]);

    std::vector<double> x, y, params;
    std::vector<size_t> indices;

    params.reserve(n);
    x.reserve(n);
    y.reserve(n);
    indices.reserve(n);

    auto addMeasurement = [&] (size_t idx, double p)
    {
        const auto& mm = measurements[ idx ];

        Eigen::Vector2d pos = position2D(mm) - offs;

        x.push_back(pos[ 0 ]);
        y.push_back(pos[ 1 ]);
        indices.push_back(idx);
        params.push_back(p);
    };

    addMeasurement(0, 0.0);

    double total_len = 0.0;
    
    for (size_t i = 1; i < n; ++i)
    {
        auto idx_last = indices.back();

        double d  = (position2D(measurements[idx_last])- position2D(measurements[i])).norm();
        double dt = KalmanInterface::timestep(measurements[idx_last], measurements[i]);

        if (d < config().min_len || dt < config().min_dt)
            continue;

        total_len += d;

        addMeasurement(i, total_len);
    }

    size_t np = params.size();

    if (np < 4)
        return interpolateLinear(measurements);

    for (double& p : params)
        p /= total_len;

    tk::spline sx(params, x);
    tk::spline sy(params, y);

    std::vector<MeasurementInterp> result;

    const auto& mm_first = measurements[indices.front()];
    const auto& mm_last  = measurements[indices.back() ];

    result.push_back(generateMeasurement(mm_first));

    boost::posix_time::ptime tcur = mm_first.t;

    boost::posix_time::milliseconds time_incr((int)(config_.sample_dt * 1000.0));

    //prereserve samples array
    size_t max_samples = estimatedSamples(mm_first, mm_last, config_.sample_dt);
    std::vector<MeasurementInterp> tmp_refs(max_samples);

    result.reserve(max_samples);

    for (size_t i = 1; i < np; ++i)
    {
        auto idx0 = indices[i - 1];
        auto idx1 = indices[i    ];

        const auto& mm0 = measurements[idx0];
        const auto& mm1 = measurements[idx1];

        auto t0 = mm0.t;
        auto t1 = mm1.t;

        double dt01 = Utils::Time::partialSeconds(t1 - t0);

        size_t num_refs_segment = 0;

        auto t_cur_seg = tcur;

        bool first = true;

        //interpolate segment with spline
        while (t_cur_seg >= t0 && t_cur_seg < t1)
        {
            double dt = Utils::Time::partialSeconds(t_cur_seg - t0);
            double f  = dt / dt01;
            double p  =(1.0 - f) * params[i - 1] + f * params[ i ];

            Eigen::Vector2d pos_interp(sx(p), sy(p));
            pos_interp += offs;

            traced_assert(num_refs_segment < tmp_refs.size());

            //collect temporary references
            auto mm_gen = generateMeasurement(pos_interp, t_cur_seg, mm0, mm1, f, false);

            //detect if this is the first sample in the interval
            if (first)
            {
                determineFirstPosInIntervalFlag(mm_gen, t0, tmp_refs, num_refs_segment);
                first = false;
            }

            tmp_refs[num_refs_segment++] = mm_gen;

            t_cur_seg += time_incr;
        }

        //check segment references => segment fishy?
        if (config().check_fishy_segments && isFishySegment(mm0, mm1, tmp_refs, num_refs_segment))
        {
            //reset and interpolate segment linear instead
            t_cur_seg        = tcur;
            num_refs_segment = 0;

            auto pos0 = position2D(mm0);
            auto pos1 = position2D(mm1);

            first = false;

            while (t_cur_seg >= t0 && t_cur_seg < t1)
            {
                double dt = Utils::Time::partialSeconds(t_cur_seg - t0);
                double f  = dt / dt01;

                Eigen::Vector2d pos_interp = pos0 * (1.0 - f) + pos1 * f;
            
                traced_assert(num_refs_segment < tmp_refs.size());

                auto mm_gen = generateMeasurement(pos_interp, t_cur_seg, mm0, mm1, f, true);

                //detect if this is the first sample in the interval
                if (first)
                {
                    determineFirstPosInIntervalFlag(mm_gen, t0, tmp_refs, num_refs_segment);
                    first = false;
                }

                tmp_refs[num_refs_segment++] = mm_gen;

                t_cur_seg += time_incr;
            }
        }

        //collect temp references
        tcur = t_cur_seg;

        if (num_refs_segment > 0)
            result.insert(result.end(), tmp_refs.begin(), tmp_refs.begin() + num_refs_segment);
    }

    finalizeInterp(result, measurements.back());

    result.shrink_to_fit();

    return result;
}

} // namespace reconstruction
