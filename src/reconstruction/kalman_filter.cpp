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

#include "kalman_filter.h"
#include "gaussian_pdf.h"
#include "logger.h"

#include <Eigen/Dense>

namespace kalman
{

/**
*/
KalmanFilter::KalmanFilter(size_t dim_x, 
                           size_t dim_z)
    :   dim_x_(dim_x)
    ,   dim_z_(dim_z)
{
    x_.setZero(dim_x);
    P_.setIdentity(dim_x, dim_x);
    Q_.setIdentity(dim_x, dim_x);
    R_.setIdentity(dim_z, dim_z);

    K_.setZero(dim_x, dim_z);
    y_.setZero(dim_z);
    S_.setZero(dim_z, dim_z);
    SI_.setZero(dim_z, dim_z);

    I_.setIdentity(dim_x, dim_x);

    x_backup_ = x_;
    P_backup_ = P_;

    x_prior_ = x_;
    P_prior_ = P_;

    x_post_ = x_;
    P_post_ = P_;
}

/**
*/
KalmanFilter::~KalmanFilter() = default;

/**
 */
void KalmanFilter::enableDebugging(bool ok)
{
    debug_ = ok;
}

/**
*/
void KalmanFilter::init(const Vector& x, const Matrix& P)
{
    // reset to force recompute
    resetLikelihoods();

    x_ = x;
    P_ = P;

    x_backup_ = x;
    P_backup_ = P;
}

/**
*/
void KalmanFilter::init(const Vector& x, const Matrix& P, double Q_var)
{
    init(x, P);

    Q_var_ = Q_var;
}

/**
*/
void KalmanFilter::init(const Vector& x, const Matrix& P, double dt, double Q_var)
{
    init(x, P, Q_var);

    updateInternalMatrices(dt, Q_var);
}


/**
*/
void KalmanFilter::init(const BasicKalmanState& state, bool xP_only)
{
    init(state.x, state.P);

    if (!xP_only)
    {
        Q_ = state.Q;
    }
}

/**
*/
void KalmanFilter::init(const KalmanState& state, bool xP_only)
{
    init(state.x, state.P);

    if (!xP_only)
    {
        Q_     = state.Q;
        Q_var_ = state.Q_var;
    }
}

/**
*/
kalman::KalmanState KalmanFilter::state(bool xP_only) const
{
    kalman::KalmanState s;
    state(s, xP_only);

    return s;
}

/**
*/
void KalmanFilter::state(kalman::BasicKalmanState& s, bool xP_only) const
{
    s.x = x_;
    s.P = P_;

    if (!xP_only)
    {
        s.Q = Q_;
    }
}

/**
*/
bool KalmanFilter::validateState(const kalman::KalmanState& s, bool xP_only) const
{
    if (static_cast<size_t>(s.x.size()) != dim_x_ ||
        static_cast<size_t>(s.P.cols()) != dim_x_ ||
        static_cast<size_t>(s.P.rows()) != dim_x_)
        return false;

    if (!xP_only && (static_cast<size_t>(s.Q.cols()) != dim_x_ ||
                     static_cast<size_t>(s.Q.rows()) != dim_x_))
        return false;

    return true;
}

/**
*/
bool KalmanFilter::validateState(const kalman::BasicKalmanState& s, bool xP_only) const
{
    if (static_cast<size_t>(s.x.size()) != dim_x_ ||
        static_cast<size_t>(s.P.cols()) != dim_x_ ||
        static_cast<size_t>(s.P.rows()) != dim_x_)
        return false;

    if (!xP_only && (static_cast<size_t>(s.Q.cols()) != dim_x_ ||
                     static_cast<size_t>(s.Q.rows()) != dim_x_))
        return false;


    return true;
}

/**
*/
void KalmanFilter::state(kalman::KalmanState& s, bool xP_only) const
{
    s.x = x_;
    s.P = P_;

    if (!xP_only)
    {
        s.Q     = Q_;
        s.Q_var = Q_var_;
    }
}

/**
*/
bool KalmanFilter::checkVariances(const Vector& x, const Matrix& P)
{
    size_t n = x.size();
    for (size_t i = 0; i < n; ++i)
        if (P(i, i) < 0)
            return false;

    return true;
}

/**
*/
bool KalmanFilter::checkState(const Vector& x, const Matrix& P)
{
    //check for finite values
    size_t n = x.size();
    for (size_t i = 0; i < n; ++i)
    {
        if (!std::isfinite(x[ i ]) ||
            !std::isfinite(P(i, i)))
            return false;
    };

    //check for positive variances
    if (!checkVariances(x, P))
        return false;

    return true;
}

/**
*/
bool KalmanFilter::checkState() const
{
    return checkState(x_, P_);
}

/**
*/
bool KalmanFilter::checkVariances() const
{
    return checkVariances(x_, P_);
}

/**
*/
void KalmanFilter::backup(bool recursive)
{
    x_backup_     = x_;
    P_backup_     = P_;
    Q_var_backup_ = Q_var_;
}

/**
*/
void KalmanFilter::revert(bool recursive)
{
    x_     = x_backup_;
    P_     = P_backup_;
    Q_var_ = Q_var_backup_;
}

/**
*/
void KalmanFilter::invert(bool recursive)
{
    invertState(x_);
}

/**
*/
void KalmanFilter::invertState(Vector& x_inv, const Vector& x) const
{
    x_inv = x;
    invertState(x_inv);
}

/**
*/
void KalmanFilter::invertState(Vector& x) const
{
    traced_assert(invert_state_func_);
    invert_state_func_(x);
}

/**
*/
void KalmanFilter::postConditionP(Matrix& P) const
{
    for (size_t i = 0; i < dim_x_; ++i)
        if (P(i, i) < 0.1)
            P(i, i) = 0.1;
}

/**
*/
void KalmanFilter::fixEstimate(const Vector& x,
                               Matrix& P,
                               bool* fixed) const
{
    if (fixed)
        *fixed = false;

    if (!checkVariances(x, P))
    {
        postConditionP(P);

        if (fixed)
            *fixed = true;
    }
}

/**
*/
void KalmanFilter::resetLikelihoods()
{
    log_likelihood_.reset();
    likelihood_.reset();
    mahalanobis_.reset();
}

/**
*/
void KalmanFilter::updateInternalMatrices(double dt, double Q_var)
{
    Q_var_ = Q_var;

    updateInternalMatrices_impl(dt, Q_var_);
}

/**
 * Predict using time delta dt.
 * dt must not be negative.
*/
KalmanFilter::Error KalmanFilter::predict(double dt, 
                                          double Q_var,
                                          const OVector& u)
{
    // reset to force recompute
    resetLikelihoods();

    // save backup state
    backup();

    if (dt != 0.0) // only predict if there is a timestep
    {
        //update internal time-dependent matrices
        updateInternalMatrices(dt, Q_var);

        // predict
        auto err = predict_impl(x_, P_, dt, Q_var_, u);
        if (err != Error::NoError)
            return err;
    }

    // save prior state
    x_prior_ = x_;
    P_prior_ = P_;

    if (!checkState())
        return Error::InvalidState;

    return Error::NoError;
}

/**
 * Updates the filter by integrating measurement z.
*/
KalmanFilter::Error KalmanFilter::update(const Vector& z,
                                         const OMatrix& R)
{
    // reset to force recompute
    resetLikelihoods();

    const Matrix& R__ = R.has_value() ? R.value() : R_;
    
    auto err = update_impl(z_func_ ? z_func_(z) : z, 
                           R_func_ ? R_func_(R__) : R__);

    if (err != Error::NoError)
    {
        x_post_ = x_;
        P_post_ = P_;
        y_.setZero(dim_z_);
        return err;
    }

    // save posterior state
    x_post_ = x_;
    P_post_ = P_;

    if (!checkState())
        return Error::InvalidState;

    return Error::NoError;
}

/**
 * Combined predict and update.
 * Will sutomatically revert to the initial state
 * when failing, making it the safe choice when executing a kalman step.
*/
KalmanFilter::Error KalmanFilter::predictAndUpdate(double dt,
                                                   double Q_var,
                                                   const Vector& z,
                                                   const OMatrix& R,
                                                   const OVector& u)
{
    //predict
    auto err = predict(dt, Q_var, u);
    if (err != Error::NoError)
    {
        //revert to initial state
        revert();
        return err;
    }
    
    //update
    err = update(z, R);
    if (err != Error::NoError)
    {
        //revert to initial state
        revert();
        return err;
    }

    return Error::NoError;
}

/**
 * Predicts the state at time delta dt without altering the filter's internal state.
 * dt may be negative in order to predict into the past.
 * notice: NOT mutlithreading safe.
*/
KalmanFilter::Error KalmanFilter::predictState(Vector& x, 
                                               Matrix& P,
                                               double dt,
                                               bool fix_estimate,
                                               bool* fixed,
                                               const OVector& u,
                                               KalmanState* state,
                                               const boost::optional<double>& Q_var) const
{
    if (fixed)
        *fixed = false;

    //determine used process noise (override > passed externally > last value)
    const double Q_var_used = (Q_var.has_value() ? Q_var.value() : Q_var_);
    
    if (dt < 0.0)
    {
        //backup current state
        kalman::Vector x_backup = x_;

        //invert state...
        invertState(x_, x_backup);

        //...and use positive timestep
        dt = std::fabs(dt);
        
        Error err = predictState_impl(x, P, dt, Q_var_used, true, u, state);

        //invert state...
        if (err == Error::NoError)
            invertState(x);
        
        //revert state
        x_ = x_backup;

        if (err != Error::NoError)
            return err;
    }
    else
    {
        Error err = predictState_impl(x, P, dt, Q_var_used, true, u, state);

        if (err != Error::NoError)
            return err;
    }

    if (fix_estimate)
        fixEstimate(x, P, fixed);

    if (!checkState(x, P))
        return Error::InvalidState;

    if (state)
    {
        state->dt = dt;
        state->x  = x;
        state->P  = P;
    }

    return Error::NoError;
}

/**
*/
KalmanFilter::Error KalmanFilter::generateMeasurement(Vector& x_pred_mm,
                                                      Matrix& P_pred_mm,
                                                      const Vector& x_pred, 
                                                      const Matrix& P_pred) const
{
    return generateMeasurement_impl(x_pred_mm, P_pred_mm, x_pred, P_pred);
}

/**
*/
KalmanFilter::Error KalmanFilter::generateMeasurement_impl(Vector& x_pred_mm,
                                                           Matrix& P_pred_mm,
                                                           const Vector& x_pred, 
                                                           const Matrix& P_pred) const
{
    throw std::runtime_error("KalmanFilter: generateMeasurement_impl: not implemented");
    return Error::Unknown;
}

/**
*/
bool KalmanFilter::smooth(std::vector<kalman::Vector>& x_smooth,
                          std::vector<kalman::Matrix>& P_smooth,
                          const std::vector<KalmanState>& states,
                          const XTransferFunc& x_tr,
                          double smooth_scale,
                          bool stop_on_fail,
                          std::vector<bool>* state_valid,
                          std::vector<RTSDebugInfo>* debug_infos) const
{
    return smooth_impl(x_smooth, P_smooth, states, x_tr, smooth_scale, stop_on_fail, state_valid, debug_infos);
}

/**
*/
void KalmanFilter::printState(std::stringstream& strm, const std::string& prefix) const
{
    strm << prefix << "x_:\n" << x_ << "\n";
    strm << prefix << "P_:\n" << P_ << "\n";
    strm << prefix << "Q_:\n" << Q_ << "\n";
    strm << prefix << "Q_var: " << Q_var_ << "\n";
}

/**
*/
void KalmanFilter::printExtendedState(std::stringstream& strm, const std::string& prefix) const
{
    strm << prefix << "R_:\n" << R_ << "\n";
}

/**
*/
void KalmanFilter::printIntermSteps(std::stringstream& strm, const std::string& prefix) const
{
    strm << prefix << "K_:\n" << K_ << "\n";
    strm << prefix << "P_prior:\n" << P_prior_ << "\n";
    strm << prefix << "S_:\n" << S_ << "\n";
    strm << prefix << "PivLU(S_):\n" << Eigen::FullPivLU<Eigen::MatrixXd>(S_).matrixLU() << "\n";
}

/**
*/
std::string KalmanFilter::asString(int info_flags, const std::string& prefix) const
{
    std::stringstream ss;

    if (info_flags & InfoFlags::InfoState)
        printState(ss, prefix);

    if (info_flags & InfoFlags::InfoStateExt)
        printExtendedState(ss, prefix);

    if (info_flags & InfoFlags::InfoIntermSteps)
        printIntermSteps(ss, prefix);

    return ss.str();
}

/**
*/
void KalmanFilter::continuousWhiteNoise(Matrix& Q_noise, 
                                        size_t dim, 
                                        double dt, 
                                        double spectral_density, 
                                        size_t block_size)
{
    if (dim < 2 || dim > 4)
        throw std::runtime_error("KalmanFilter::continuousWhiteNoise(): dim must be between 2 and 4");
    if (block_size < 1)
        throw std::runtime_error("KalmanFilter::continuousWhiteNoise(): block size must be > 0");

    //@TODO CHECK
    //dt = std::max(1.0, dt);

    size_t full_size = dim * block_size;

    Q_noise.setZero(full_size, full_size);

    const double dt2 = dt * dt;

    if (dim == 2)
    {
        Matrix Q;
        Q.setZero(2, 2);
        Q(0, 0) = dt2 * dt / 3.0;  Q(0, 1) = dt2 / 2.0;
        Q(1, 0) = dt2      / 2.0;  Q(1, 1) = dt;

        if (block_size == 1)
        {
            Q_noise = Q;
        }
        else
        {
            for (size_t i = 0; i < block_size; ++i)
                Q_noise.block<2, 2>(i * 2, i * 2) = Q;
        }
    }
    else if (dim == 3)
    {
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;

        Matrix Q;
        Q.setZero(3, 3);
        Q(0, 0) = dt3 * dt2 / 20.0;  Q(0, 1) = dt4 / 8.0; Q(0, 2) = dt3 / 6.0;
        Q(1, 0) = dt4       / 8.0 ;  Q(1, 1) = dt3 / 3.0; Q(1, 2) = dt2 / 2.0;
        Q(2, 0) = dt3       / 6.0 ;  Q(2, 1) = dt2 / 2.0; Q(2, 2) = dt;

        if (block_size == 1)
        {
            Q_noise = Q;
        }
        else
        {
            for (size_t i = 0; i < block_size; ++i)
                Q_noise.block<3, 3>(i * 3, i * 3) = Q;
        }
    }
    else // dim == 4
    {
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;
        const double dt5 = dt3 * dt2;
        const double dt6 = dt2 * dt2 * dt2;

        Matrix Q;
        Q.setZero(4, 4);
        Q(0, 0) = dt6 * dt / 252.0;  Q(0, 1) = dt6 / 72.0; Q(0, 2) = dt5 / 30.0; Q(0, 3) = dt4 / 24.0;
        Q(1, 0) = dt6      / 72.0;   Q(1, 1) = dt5 / 20.0; Q(1, 2) = dt4 / 8.0 ; Q(1, 3) = dt3 / 6.0;
        Q(2, 0) = dt5      / 30.0;   Q(2, 1) = dt4 / 8.0 ; Q(2, 2) = dt3 / 3.0 ; Q(2, 3) = dt2 / 2.0;
        Q(3, 0) = dt4      / 24.0;   Q(3, 1) = dt3 / 6.0 ; Q(3, 2) = dt2 / 2.0 ; Q(3, 3) = dt;

        if (block_size == 1)
        {
            Q_noise = Q;
        }
        else
        {
            for (size_t i = 0; i < block_size; ++i)
                Q_noise.block<4, 4>(i * 4, i * 4) = Q;
        }
    }

    Q_noise *= spectral_density;
}

/**
*/
Matrix KalmanFilter::continuousWhiteNoise(size_t dim,
                                          double dt,
                                          double spectral_density,
                                          size_t block_size)
{
    Matrix Q_noise;
    continuousWhiteNoise(Q_noise, dim, dt, spectral_density, block_size);
    return Q_noise;
}

/**
*/
bool KalmanFilter::invertMatrix(Matrix& Pinv, const Matrix& P)
{
    if (!Eigen::FullPivLU<Eigen::MatrixXd>(P).isInvertible())
        return false;

    Pinv = P.inverse();

    return true;
}

/**
*/
boost::optional<double> KalmanFilter::likelihood(const Vector& x, const Matrix& P, bool check_eps)
{
    GaussianPDF pdf(P);
    return pdf.likelihood(x, check_eps);
}

/**
*/
boost::optional<double> KalmanFilter::logLikelihood(const Vector& x, const Matrix& P, bool check_eps)
{
    GaussianPDF pdf(P);
    return pdf.logLikelihood(x, check_eps);
}

namespace helpers
{
    double mahalanobisSqrPInv(const Vector& dx, const Matrix& P_inv)
    {
        return double(dx.transpose() * P_inv * dx);
    }
}

/**
*/
boost::optional<double> KalmanFilter::mahalanobis(const Vector& dx, const Matrix& P, bool P_is_inv)
{
    auto d_sqr = KalmanFilter::mahalanobisSqr(dx, P, P_is_inv);
    if (!d_sqr.has_value())
        return {};

    return std::sqrt(d_sqr.value());
}

/**
*/
boost::optional<double> KalmanFilter::mahalanobisSqr(const Vector& dx, const Matrix& P, bool P_is_inv)
{
    if (P_is_inv)
        return helpers::mahalanobisSqrPInv(dx, P);
    
    Matrix P_inv;
    if (!KalmanFilter::invertMatrix(P_inv, P))
        return {};
    
    return helpers::mahalanobisSqrPInv(dx, P_inv);
}

/**
*/
boost::optional<double> KalmanFilter::logLikelihood(bool check_eps) const
{
    if (log_likelihood_.has_value())
        return log_likelihood_;

    log_likelihood_ = KalmanFilter::logLikelihood(y_, S_, check_eps);

    return log_likelihood_;
}

/**
*/
boost::optional<double> KalmanFilter::likelihood(bool check_eps) const
{
    if (likelihood_.has_value())
        return likelihood_;

    likelihood_ = KalmanFilter::likelihood(y_, S_, check_eps);

    return likelihood_;
}

/**
*/
boost::optional<double> KalmanFilter::mahalanobis() const
{
    if (mahalanobis_.has_value())
        return mahalanobis_;

    mahalanobis_ = KalmanFilter::mahalanobis(y_, SI_, true);

    return mahalanobis_;
}

/**
*/
void KalmanFilter::xPos(double& x, double& y) const
{
    xPos(x, y, getX());
}

/**
*/
void KalmanFilter::xPos(double& x, double& y, const kalman::Vector& x_vec) const
{
    bool implemented = false;
    traced_assert(implemented);
}

/**
*/
void KalmanFilter::xPos(kalman::Vector& x_vec, double x, double y) const
{
    bool implemented = false;
    traced_assert(implemented);
}

/**
*/
double KalmanFilter::xVar(const kalman::Matrix& P) const
{
    bool implemented = false;
    traced_assert(implemented);

    return 0.0;
}

/**
*/
double KalmanFilter::yVar(const kalman::Matrix& P) const
{
    bool implemented = false;
    traced_assert(implemented);

    return 0.0;
}

/**
*/
double KalmanFilter::xyCov(const kalman::Matrix& P) const
{
    bool implemented = false;
    traced_assert(implemented);

    return 0.0;
}

} // namespace kalman
