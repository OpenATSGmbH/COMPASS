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

#include "kalman_filter_linear.h"

namespace kalman
{

/**
*/
class KalmanFilterUM2D : public KalmanFilterLinear
{
public:
    KalmanFilterUM2D(bool track_velocities);
    virtual ~KalmanFilterUM2D();

    void xPos(double& x, double& y, const kalman::Vector& x_vec) const override final;
    void xPos(kalman::Vector& x_vec, double x, double y) const override final;
    double xVar(const kalman::Matrix& P) const override final;
    double yVar(const kalman::Matrix& P) const override final;
    double xyCov(const kalman::Matrix& P) const override final;
    boost::optional<double> xVel(const kalman::Vector& x_vec) const override final;
    boost::optional<double> yVel(const kalman::Vector& x_vec) const override final;
    boost::optional<double> vxVar(const kalman::Matrix& P) const override final;
    boost::optional<double> vyVar(const kalman::Matrix& P) const override final;

protected:
    void configureFMat(Matrix& F, double dt) const;
    void configureQMat(Matrix& Q, double dt, double var) const;

private:
    void invertState(Vector& x) const;

    using KalmanFilterLinear::setFMatFunc;
    using KalmanFilterLinear::setQMatFunc;

    using KalmanFilterLinear::bMat;
    using KalmanFilterLinear::fMat;
    using KalmanFilterLinear::hMat;
    using KalmanFilterLinear::mMat;
    using KalmanFilterLinear::qMat;

    using KalmanFilterLinear::setB;
    using KalmanFilterLinear::setF;
    using KalmanFilterLinear::setH;
    using KalmanFilterLinear::setM;
    using KalmanFilterLinear::setQ;

    using KalmanFilterLinear::setControlTransitionMat;
    using KalmanFilterLinear::setStateTransitionMat;
    using KalmanFilterLinear::setMeasurementMat;
    using KalmanFilterLinear::setProcessMMCrossCorrMat;
    using KalmanFilterLinear::setProcessUncertMat;

    using KalmanFilter::setInvertStateFunc;
    using KalmanFilter::setZFunc;
    using KalmanFilter::setRFunc;
};

} // namespace kalman
