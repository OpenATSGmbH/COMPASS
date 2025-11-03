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

#include "kalman_interface.h"
#include "reconstruction_defs.h"
#include "kalman_projection.h"
#include "kalman_filter.h"

#include "timeconv.h"
#include "logger.h"

namespace reconstruction
{

/**
*/
KalmanInterface::KalmanInterface() = default;

/**
*/
KalmanInterface::KalmanInterface(kalman::KalmanFilter* kalman_filter)
:   kalman_filter_(kalman_filter)
{
    traced_assert(kalman_filter_);
}

/**
*/
KalmanInterface::~KalmanInterface() = default;

/**
*/
void KalmanInterface::setKalmanFilter(kalman::KalmanFilter* kalman_filter)
{
    traced_assert(kalman_filter);
    kalman_filter_.reset(kalman_filter);
}

/**
*/
double KalmanInterface::timestep(const boost::posix_time::ptime& ts0, const boost::posix_time::ptime& ts1)
{
    return (ts1 >= ts0 ? Utils::Time::partialSeconds(ts1 - ts0) : -Utils::Time::partialSeconds(ts0 - ts1));
}

/**
*/
double KalmanInterface::timestep(const Measurement& mm) const
{
    return KalmanInterface::timestep(ts_, mm.t);
}

/**
*/
double KalmanInterface::timestep(const Measurement& mm0, const Measurement& mm1)
{
    return KalmanInterface::timestep(mm0.t, mm1.t);
}

/**
*/
double KalmanInterface::distanceSqr(const Measurement& mm) const
{
    //last cartesian position of kalman
    double x0, y0;
    xPos(x0, y0);

    return (Eigen::Vector2d(x0, y0) - Eigen::Vector2d(mm.x, mm.y)).squaredNorm();
}

/**
*/
void KalmanInterface::kalmanInit(kalman::KalmanState& init_state,
                                 const Measurement& mm,
                                 const reconstruction::Uncertainty& default_uncert,
                                 double Q_var)
{
    traced_assert(kalman_filter_);

    ts_ = mm.t;
    
    const double dt_start = 1.0;
    const double Q_var_in = mm.Q_var.has_value() ? mm.Q_var.value() : Q_var;

    //init kalman
    //@TODO: cache x and P somewhere
    kalman::Vector x;
    kalman::Matrix P;
    stateVecXFromMM(x, mm);
    covarianceMatP(P, mm, default_uncert);
    //measurementUncertMatR(kalman_filter_->rMat(), mm, default_uncert);

    kalman_filter_->init(x, P, dt_start, Q_var_in);

    if (verbosity() > 1)
    {
        loginf << "[Reinit Kalman] \n"
               << kalman_filter_->asString(kalman::InfoState);
    }

    kalman_filter_->state(init_state);
    init_state.dt = dt_start;
}

/**
*/
void KalmanInterface::kalmanInit(const kalman::KalmanState& init_state,
                                 const boost::posix_time::ptime& ts)
{
    traced_assert(kalman_filter_);

    ts_ = ts;
    
    kalman_filter_->init(init_state);
}

/**
*/
void KalmanInterface::kalmanInit(const kalman::Vector& x,
                                 const kalman::Matrix& P,
                                 const boost::posix_time::ptime& ts,
                                 double Q_var)
{
    traced_assert(kalman_filter_);

    ts_ = ts;
    
    kalman_filter_->init(x, P, Q_var);
}

/**
*/
kalman::KalmanError KalmanInterface::kalmanStep(kalman::KalmanState& new_state,
                                                const Measurement& mm, 
                                                const reconstruction::Uncertainty& default_uncert, 
                                                double Q_var)
{
    traced_assert(kalman_filter_);

    const double dt = timestep(mm);

    //set kalman internal matrices
    measurementUncertMatR(kalman_filter_->rMat(), mm, default_uncert);

    //get measurement vector
    kalman::Vector z;
    measurementVecZ(z, mm);

    if (verbosity() > 1)
    {
        loginf << "[Kalman step] dt = " << dt << "\n" 
               << kalman_filter_->asString(kalman::InfoState)
               << "z: " << z << "\n";
    }

    double Q_var_in = mm.Q_var.has_value() ? mm.Q_var.value() : Q_var;

    auto err = kalman_filter_->predictAndUpdate(dt, Q_var_in, z, {}, {});

    if (err != kalman::KalmanError::NoError)
        return err;

    new_state    = kalman_filter_->state();
    new_state.dt = dt;

    ts_ = mm.t;

    return kalman::KalmanError::NoError;
}

/**
*/
kalman::KalmanError KalmanInterface::kalmanPrediction(kalman::Vector& x,
                                                      kalman::Matrix& P,
                                                      double dt,
                                                      bool fix_estimate,
                                                      bool* fixed,
                                                      const boost::optional<double>& Q_var) const
{
    traced_assert(kalman_filter_);

    auto err = kalman_filter_->predictState(x, P, dt, fix_estimate, fixed, {}, nullptr, Q_var);
    if (err != kalman::KalmanError::NoError)
            return err;

    return kalman::KalmanError::NoError;
}

/**
*/
kalman::KalmanError KalmanInterface::kalmanPrediction(kalman::Vector& x,
                                                      kalman::Matrix& P,
                                                      const boost::posix_time::ptime& ts,
                                                      bool fix_estimate,
                                                      bool* fixed,
                                                      const boost::optional<double>& Q_var) const
{
    double dt = KalmanInterface::timestep(ts_, ts);
    return kalmanPrediction(x, P, dt, fix_estimate, fixed, Q_var);
}

/**
*/
kalman::KalmanError KalmanInterface::kalmanPredictionMM(kalman::ProbState& pred_mm,
                                                        const kalman::Vector& x_pred,
                                                        const kalman::Matrix& P_pred) const
{
    traced_assert(kalman_filter_);

    auto err = kalman_filter_->generateMeasurement(pred_mm.x, pred_mm.P, x_pred, P_pred);
    if (err != kalman::KalmanError::NoError)
        return err;

    return kalman::KalmanError::NoError;
}

/**
*/
kalman::KalmanError KalmanInterface::comparePrediction(PredictionComparison& comparison,
                                                       const kalman::ProbState& pred_mm,
                                                       const Measurement& mm,
                                                       const reconstruction::Uncertainty& default_uncert,
                                                       int comparison_flags) const
{
    //get measurement vector from mm
    kalman::Vector z;
    measurementVecZ(z, mm);

    //get measurement uncert mat
    kalman::Matrix R;
    measurementUncertMatR(R, mm, default_uncert);

    //residual
    const auto y = z - pred_mm.x;

    //add measurement uncertainty
    const kalman::Matrix S = pred_mm.P + R;

    const bool CheckEps = true;

    bool has_num_error = false;

    if (comparison_flags & PredictionCompareFlags::PredCompLikelihood)
    {
        comparison.likelihood = kalman::KalmanFilter::likelihood(y, S, CheckEps);
        if (!comparison.likelihood.has_value())
            has_num_error = true;
    }
    if (comparison_flags & PredictionCompareFlags::PredCompLogLikelihood)
    {
        comparison.log_likelihood = kalman::KalmanFilter::logLikelihood(y, S, CheckEps);
        if (!comparison.log_likelihood.has_value())
            has_num_error = true;
    }
    if (comparison_flags & PredictionCompareFlags::PredCompMahalanobis)
    {
        comparison.mahalanobis = kalman::KalmanFilter::mahalanobis(y, S, false);
        if (!comparison.mahalanobis.has_value())
            has_num_error = true;
    }

    if (has_num_error || !comparison.valid())
        return kalman::KalmanError::Numeric;
    
    return kalman::KalmanError::NoError;
}

/**
*/
bool KalmanInterface::smoothUpdates(std::vector<kalman::KalmanUpdate>& updates,
                                    size_t idx0,
                                    size_t idx1,
                                    KalmanProjectionHandler& proj_handler,
                                    double smooth_scale,
                                    kalman::SmoothFailStrategy fail_strategy,
                                    std::vector<kalman::RTSDebugInfo>* debug_infos) const
{
    traced_assert(kalman_filter_);
    traced_assert(idx1 >= idx0);

    size_t n = idx1 - idx0 + 1;

    if (n <= 1)
        return true;

    std::vector<kalman::Vector> x_smooth;
    std::vector<kalman::Matrix> P_smooth;
    std::vector<bool>           state_valid_tmp;

    //if (debug_infos)
    //    loginf << "#updates: " << updates.size() << ", idx0: " << idx0 << ", idx1: " << idx1 << ", n: " << n;

    //@TODO: duplicate states really needed!?
    std::vector<kalman::KalmanState> states(n);
    for (size_t i = idx0; i <= idx1; ++i)
        states[ i - idx0 ] = updates[ i ].state;

    //get reprojection transfer func
    auto x_tr = proj_handler.reprojectionTransform(&updates, this, idx0);

    //smooth states
    size_t n_debug_infos_before = debug_infos ? debug_infos->size() : 0;

    bool ok = kalman_filter_->smooth(x_smooth, 
                                     P_smooth, 
                                     states, 
                                     x_tr, 
                                     smooth_scale, 
                                     fail_strategy == kalman::SmoothFailStrategy::Stop, 
                                     fail_strategy == kalman::SmoothFailStrategy::SetInvalid ? &state_valid_tmp : nullptr, 
                                     debug_infos);
    if (debug_infos)
    {
        for (size_t i = n_debug_infos_before; i < debug_infos->size(); ++i)
        {
            auto& di = debug_infos->at(i);

            di.update_idx       += idx0;
            di.projection_center = updates.at(di.update_idx).projection_center;
        }
    }

    if (!ok)
        return false;
    
    //write smoothed states back to updates
    for (size_t i = 0; i < n; ++i)
    {
        updates[ idx0 + i ].state.x = x_smooth[ i ];
        updates[ idx0 + i ].state.P = P_smooth[ i ];

        if (fail_strategy == kalman::SmoothFailStrategy::SetInvalid && !state_valid_tmp[ i ])
            updates[ idx0 + i ].valid = false;
    }
    
    return true;
}

/**
 * Note: !Assumes that it is ok to modify the filter's internal state!
*/
kalman::KalmanError KalmanInterface::interpStep(kalman::KalmanState& state_interp,
                                                const kalman::KalmanState& state0,
                                                const kalman::KalmanState& state1,
                                                double dt,
                                                bool fix_estimate,
                                                bool* fixed,
                                                const boost::optional<double>& Q_var) const
{
    traced_assert(kalman_filter_);

    //backup current state
    auto state_backup = kalman_filter_->state();

    //init to interp state
    kalman_filter_->init(state0);

    //predict
    auto err = kalman_filter_->predictState(state_interp.x,
                                            state_interp.P,
                                            dt,
                                            fix_estimate,
                                            fixed,
                                            {},
                                            &state_interp,
                                            Q_var);
    //revert
    kalman_filter_->init(state_backup);

    return err;
}

/**
*/
bool KalmanInterface::checkKalmanStateNumerical(const kalman::KalmanState& state) const
{
    traced_assert(kalman_filter_);
    return kalman_filter_->checkState(state.x, state.P);
}

/**
*/
bool KalmanInterface::validateState(const kalman::KalmanState& state) const
{
    traced_assert(kalman_filter_);
    return kalman_filter_->validateState(state);
}

/**
*/
void KalmanInterface::storeState(Measurement& mm, 
                                 const kalman::KalmanState& state, 
                                 int submodel_idx) const
{
    if (submodel_idx >= 0)
    {
        traced_assert(state.imm_state);
        const auto& state_model = state.imm_state->filter_states.at(submodel_idx);
        storeState(mm, state_model.x, state_model.P, submodel_idx);
    }
    else
    {
        storeState(mm, state.x, state.P, submodel_idx);
    }
}

/**
*/
void KalmanInterface::storeState(Measurement& mm, 
                                 const kalman::Vector& x, 
                                 const kalman::Matrix& P,
                                 int submodel_idx) const
{
    traced_assert(kalman_filter_);
    kalman::KalmanFilter* filter = submodel_idx >= 0 ? kalman_filter_->subModel(submodel_idx) : kalman_filter_.get();
    traced_assert(filter);

    filter->xPos(mm.x, mm.y, x);

    mm.x_stddev = std::sqrt(filter->xVar(P));
    mm.y_stddev = std::sqrt(filter->yVar(P));
    mm.xy_cov   = filter->xyCov(P);

    mm.vx = filter->xVel(x);
    mm.vy = filter->yVel(x);

    mm.ax = filter->xAcc(x);
    mm.ay = filter->yAcc(x);
}

/**
*/
Measurement KalmanInterface::currentStateAsMeasurement() const
{
    Measurement mm;
    storeState(mm, currentState());

    return mm;
}

/**
*/
kalman::KalmanState KalmanInterface::currentState() const
{
    traced_assert(kalman_filter_);
    return kalman_filter_->state();
}

/**
*/
void KalmanInterface::stateVecX(const kalman::Vector& x)
{
    traced_assert(kalman_filter_);
    kalman_filter_->setX(x);
}

/**
*/
void KalmanInterface::stateVecX(const kalman::KalmanState& state)
{
    traced_assert(kalman_filter_);
    kalman_filter_->setX(state);
}

/**
*/
void KalmanInterface::xPos(double& x, double& y, int submodel_idx) const
{
    traced_assert(kalman_filter_);
    kalman::KalmanFilter* filter = submodel_idx >= 0 ? kalman_filter_->subModel(submodel_idx) : kalman_filter_.get();
    traced_assert(filter);

    filter->xPos(x, y);
}

/**
*/
void KalmanInterface::xPos(double& x, double& y, const kalman::Vector& x_vec, int submodel_idx) const
{
    traced_assert(kalman_filter_);
    kalman::KalmanFilter* filter = submodel_idx >= 0 ? kalman_filter_->subModel(submodel_idx) : kalman_filter_.get();
    traced_assert(filter);

    filter->xPos(x, y, x_vec);
}

/**
*/
void KalmanInterface::xPos(kalman::Vector& x_vec, double x, double y, int submodel_idx) const
{
    traced_assert(kalman_filter_);
    kalman::KalmanFilter* filter = submodel_idx >= 0 ? kalman_filter_->subModel(submodel_idx) : kalman_filter_.get();
    traced_assert(filter);

    filter->xPos(x_vec, x, y);
}

/**
*/
double KalmanInterface::xVar(const kalman::Matrix& P, int submodel_idx) const
{
    traced_assert(kalman_filter_);
    kalman::KalmanFilter* filter = submodel_idx >= 0 ? kalman_filter_->subModel(submodel_idx) : kalman_filter_.get();
    traced_assert(filter);

    return filter->xVar(P);
}

/**
*/
double KalmanInterface::yVar(const kalman::Matrix& P, int submodel_idx) const
{
    traced_assert(kalman_filter_);
    kalman::KalmanFilter* filter = submodel_idx >= 0 ? kalman_filter_->subModel(submodel_idx) : kalman_filter_.get();
    traced_assert(filter);

    return filter->yVar(P);
}

/**
*/
double KalmanInterface::xyCov(const kalman::Matrix& P, int submodel_idx) const
{
    traced_assert(kalman_filter_);
    kalman::KalmanFilter* filter = submodel_idx >= 0 ? kalman_filter_->subModel(submodel_idx) : kalman_filter_.get();
    traced_assert(filter);

    return filter->xyCov(P);
}

/**
*/
std::string KalmanInterface::asString(int flags, 
                                      const std::string& prefix) const
{
    traced_assert(kalman_filter_);
    return kalman_filter_->asString(flags, prefix);
}

/**
*/
void KalmanInterface::enableDebugging(bool ok) 
{ 
    traced_assert(kalman_filter_);
    kalman_filter_->enableDebugging(ok);
}

} // reconstruction
