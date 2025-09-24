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

#include "kalman_filter_linear.h"
#include "logger.h"

#include <Eigen/Dense>

namespace kalman
{

/**
*/
KalmanFilterLinear::KalmanFilterLinear(size_t dim_x, 
                                       size_t dim_z,
                                       size_t dim_u,
                                       const FMatFunc& F_func,
                                       const QMatFunc& Q_func)
:   KalmanFilter(dim_x, dim_z)
,   dim_u_      (dim_u)
,   F_func_     (F_func)
,   Q_func_     (Q_func)
{
    F_.setIdentity(dim_x, dim_x);
    H_.setZero(dim_z, dim_x);
    M_.setZero(dim_x, dim_z);
}

/**
*/
KalmanFilterLinear::~KalmanFilterLinear() = default;

/**
*/
void KalmanFilterLinear::init(const KalmanState& state, bool xP_only)
{
    KalmanFilter::init(state, xP_only);

    if (!xP_only)
    {
        F_ = state.F;
    }
}

/**
*/
void KalmanFilterLinear::state(kalman::KalmanState& s, bool xP_only) const
{
    KalmanFilter::state(s, xP_only);

    if (!xP_only)
    {
        s.F = F_;
    }
}

/**
*/
void KalmanFilterLinear::state(kalman::BasicKalmanState& s, bool xP_only) const
{
    KalmanFilter::state(s, xP_only);

    if (!xP_only)
    {
        s.F = F_;
    }
}

/**
*/
bool KalmanFilterLinear::validateState(const kalman::KalmanState& s, bool xP_only) const
{
    if (!KalmanFilter::validateState(s, xP_only))
        return false;

    if (!xP_only && (static_cast<size_t>(s.F.cols()) != dim_x_ ||
                     static_cast<size_t>(s.F.rows()) != dim_x_))
        return false;

    return true;
}

/**
*/
bool KalmanFilterLinear::validateState(const kalman::BasicKalmanState& s, bool xP_only) const
{
    if (!KalmanFilter::validateState(s, xP_only))
        return false;

    if (!xP_only && (static_cast<size_t>(s.F.cols()) != dim_x_ ||
                     static_cast<size_t>(s.F.rows()) != dim_x_))
        return false;

    return true;
}

/**
*/
void KalmanFilterLinear::printState(std::stringstream& strm, const std::string& prefix) const
{
    KalmanFilter::printState(strm, prefix);

    strm << prefix << "F_:\n" << F_ << "\n";
}

/**
*/
void KalmanFilterLinear::printExtendedState(std::stringstream& strm, const std::string& prefix) const
{
    KalmanFilter::printExtendedState(strm, prefix);

    if (z_.has_value())
        strm << prefix << "z_:\n" << z_.value() << "\n";
}

/**
*/
void KalmanFilterLinear::printIntermSteps(std::stringstream& strm, const std::string& prefix) const
{
    KalmanFilter::printIntermSteps(strm, prefix);
}

/**
*/
void KalmanFilterLinear::updateInternalMatrices_impl(double dt, double Q_var)
{
    traced_assert(F_func_);
    traced_assert(Q_func_);

    F_func_(F_, dt);
    Q_func_(Q_, std::fabs(dt), Q_var);
}

/**
 * General predict from current state (internal).
*/
KalmanFilter::Error KalmanFilterLinear::predict(Vector& x,
                                                Matrix& P,
                                                const Matrix& F,
                                                const Matrix& Q,
                                                const OMatrix& B,
                                                const OVector& u) const
{
    // x = Fx + Bu
    x = F * x_;
    if (B.has_value() && u.has_value())
        x += B.value() * u.value();
    
    // P = FPF' + Q
    P = alpha_sq_ * (F * P_ * F.transpose()) + Q;

    return Error::NoError;
}

/**
 * Standard predict invoked by base.
*/
KalmanFilter::Error KalmanFilterLinear::predict_impl(Vector& x, 
                                                     Matrix& P,
                                                     double dt,
                                                     double Q_var,
                                                     const OVector& u)
{
    return predict(x, P, F_, Q_, B_, u);
}

/**
 * General update current state (internal).
*/
KalmanFilter::Error KalmanFilterLinear::update(const Vector& z,
                                               const Matrix& R,
                                               const Matrix& H)
{
    z_.reset();

    // y = z - Hx
    // error (residual) between measurement and prediction
    y_ = z - H * x_;

    // common subexpression for speed
    Matrix PHT = P_ * H.transpose();

    // S = HPH' + R
    // project system uncertainty into measurement space
    S_  = (H * PHT) + R;

    // check if S is invertible
    if (!KalmanFilter::invertMatrix(SI_, S_))
        return Error::Numeric;

    // K = PH'inv(S)
    // map system uncertainty into kalman gain
    K_ = PHT * SI_;

    // x = x + Ky
    // predict new x with residual scaled by the kalman gain
    x_ += K_ * y_;

    // P = (I-KH)P(I-KH)' + KRK'
    // This is more numerically stable
    // and works for non-optimal K vs the equation
    // P = (I-KH)P usually seen in the literature.

    Matrix I_KH = I_ - K_ * H;
    P_ = I_KH * P_ * I_KH.transpose() + K_ * R * K_.transpose();

    // save measurement
    z_ = z;

    return Error::NoError;
}

/**
 * Standard update invoked by base.
*/
KalmanFilter::Error KalmanFilterLinear::update_impl(const Vector& z, 
                                                    const Matrix& R)
{
    return update(z, R, H_);
}

/**
*/
KalmanFilter::Error KalmanFilterLinear::generateMeasurement_impl(Vector& x_pred_mm,
                                                                 Matrix& P_pred_mm,
                                                                 const Vector& x_pred, 
                                                                 const Matrix& P_pred) const
{
    // x_mm = Hx
    // project state to measurement space
    x_pred_mm = H_ * x_pred;

    // S = HPH' + R
    // project system uncertainty into measurement space
    P_pred_mm  = (H_ * P_pred * H_.transpose());

    return Error::NoError;
}

/**
*/
KalmanFilter::Error KalmanFilterLinear::predictState_impl(Vector& x, 
                                                          Matrix& P,
                                                          double dt,
                                                          double Q_var,
                                                          bool mt_safe,
                                                          const OVector& u,
                                                          KalmanState* state) const
{
    traced_assert(F_func_);
    traced_assert(Q_func_);

    KalmanFilter::Error err;

    if (state)
    {
        //use state internal matrices
        F_func_(state->F, dt);
        Q_func_(state->Q, dt, Q_var);

        err = predict(x, P, state->F, state->Q, B_, u);
    }
    else
    {
        kalman::Matrix F, Q;

        F_func_(F, dt);
        Q_func_(Q, dt, Q_var);

        err = predict(x, P, F, Q, B_, u);
    }

    return err;
}

/**
*/
bool KalmanFilterLinear::smooth_impl(std::vector<kalman::Vector>& x_smooth,
                                     std::vector<kalman::Matrix>& P_smooth,
                                     const std::vector<KalmanState>& states,
                                     const XTransferFunc& x_tr,
                                     double smooth_scale,
                                     bool stop_on_fail,
                                     std::vector<bool>* state_valid,
                                     std::vector<RTSDebugInfo>* debug_infos) const
{
    if (states.empty())
        return true;

    int n     = (int)states.size();
    int dim_x = (int)states[0].x.size();

    x_smooth.resize(n);
    P_smooth.resize(n);

    if (state_valid)
        state_valid->assign(n, true);

    std::vector<kalman::Matrix> Pp(n);
    std::vector<kalman::Matrix> K (n);

    for (int i = 0; i < n; ++i)
    {
        x_smooth   [ i ] = states[ i ].x;
        P_smooth   [ i ] = states[ i ].P;
        Pp         [ i ] = states[ i ].P;
        K          [ i ].setZero(dim_x, dim_x);
    }

    if (states.size() < 2)
        return true;

    kalman::Vector x_smooth_1_tr;

    for (int k = n - 2; k >= 0; --k)
    {
        const auto& F_1  = states[ k+1 ].F;
        const auto& Q_1  = states[ k+1 ].Q;
        auto        F_1t = F_1.transpose();

        Pp[ k ] = F_1 * P_smooth[ k ] * F_1t + Q_1;

        if (!Eigen::FullPivLU<Eigen::MatrixXd>(Pp[ k ]).isInvertible())
            return false;

        K[ k ]  = P_smooth[ k ] * F_1t * Pp[ k ].inverse();
        K[ k ] *= smooth_scale;

        //get last smoothed state vector
        if (x_tr) x_tr(x_smooth_1_tr, x_smooth[ k+1 ], k + 1, k);
        const kalman::Vector& x_smooth_1 = x_tr ? x_smooth_1_tr : x_smooth[ k+1 ];

        x_smooth[ k ] += K[ k ] * (x_smooth_1 - F_1 * x_smooth[ k ]);
        P_smooth[ k ] += K[ k ] * (P_smooth[ k+1 ] - Pp[ k ]) * ((const kalman::Matrix&)K[ k ]).transpose();

        bool state_ok = (stop_on_fail || state_valid) ? checkState(x_smooth[ k ], P_smooth[ k ]) : true;

        if (stop_on_fail && !state_ok)
            return false;

        if (state_valid)
            (*state_valid)[ k ] = state_ok;
    }

    return true;
}

/**
*/
bool KalmanFilterLinear::smoothingStep_impl(Vector& x0_smooth,
                                            Matrix& P0_smooth,
                                            Vector& x1_pred,
                                            Matrix& P1_pred,
                                            const Vector& x1_smooth_tr,
                                            const Matrix& P1_smooth,
                                            const BasicKalmanState& state1,
                                            double dt1,
                                            double smooth_scale,
                                            RTSStepInfo* debug_info) const
{
    const auto& F_1  = state1.F;
    const auto& Q_1  = state1.Q;
    auto        F_1t = F_1.transpose();

    x1_pred = F_1 * x0_smooth;
    P1_pred = F_1 * P0_smooth * F_1t + Q_1;

    if (!Eigen::FullPivLU<Eigen::MatrixXd>(P1_pred).isInvertible())
    {
        loginf << "Could not invert P1_pred:\n" << P1_pred;
        return false;
    }

    Matrix K = P0_smooth * F_1t * P1_pred.inverse();
    K *= smooth_scale;

    x0_smooth += K * (x1_smooth_tr - x1_pred);
    P0_smooth += K * (P1_smooth - P1_pred) * ((const kalman::Matrix&)K).transpose();

    return true;
}

} // namespace kalman
