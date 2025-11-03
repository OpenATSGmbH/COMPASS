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

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace kalman
{
    class KalmanFilter;
}

namespace reconstruction
{

struct Measurement;
struct Uncertainty;
struct KalmanUpdate;
struct PredictionComparison;

class KalmanProjectionHandler;

/**
 * Provides needed data structures for kalman and means to retrieve data from kalman state. 
 * Derive for a certain kalman variant.
 */
class KalmanInterface
{
public:
    KalmanInterface();
    KalmanInterface(kalman::KalmanFilter* kalman_filter);
    virtual ~KalmanInterface();

    virtual KalmanInterface* clone() const = 0;

    std::string asString(int flags, 
                         const std::string& prefix = "") const;

    const boost::posix_time::ptime& currentTime() const { return ts_; }

    Measurement currentStateAsMeasurement() const;
    kalman::KalmanState currentState() const;

    //initialize kalman state
    void kalmanInit(kalman::KalmanState& init_state,
                    const Measurement& mm,
                    const reconstruction::Uncertainty& default_uncert,
                    double Q_var);
    void kalmanInit(const kalman::KalmanState& init_state,
                    const boost::posix_time::ptime& ts);
    void kalmanInit(const kalman::Vector& x,
                    const kalman::Matrix& P,
                    const boost::posix_time::ptime& ts,
                    double Q_var);
    
    //integrate measurement and obtain next kalman state
    kalman::KalmanError kalmanStep(kalman::KalmanState& new_state,
                                   const Measurement& mm, 
                                   const reconstruction::Uncertainty& default_uncert, 
                                   double Q_var);
    
    //predict kalman state
    kalman::KalmanError kalmanPrediction(kalman::Vector& x,
                                         kalman::Matrix& P,
                                         double dt,
                                         bool fix_estimate,
                                         bool* fixed = nullptr,
                                         const boost::optional<double>& Q_var = boost::optional<double>()) const;
    kalman::KalmanError kalmanPrediction(kalman::Vector& x,
                                         kalman::Matrix& P,
                                         const boost::posix_time::ptime& ts,
                                         bool fix_estimate,
                                         bool* fixed = nullptr,
                                         const boost::optional<double>& Q_var = boost::optional<double>()) const;
    kalman::KalmanError kalmanPredictionMM(kalman::ProbState& pred_mm,
                                           const kalman::Vector& x_pred,
                                           const kalman::Matrix& P_pred) const;
    kalman::KalmanError comparePrediction(PredictionComparison& comparison,
                                          const kalman::ProbState& pred_mm,
                                          const Measurement& mm,
                                          const reconstruction::Uncertainty& default_uncert,
                                          int comparison_flags) const;

    //smooth kalman updates  
    bool smoothUpdates(std::vector<kalman::KalmanUpdate>& updates,
                       size_t idx0,
                       size_t idx1,
                       KalmanProjectionHandler& proj_handler,
                       double smooth_scale = 1.0,
                       kalman::SmoothFailStrategy fail_strategy = kalman::SmoothFailStrategy::Stop,
                       std::vector<kalman::RTSDebugInfo>* debug_infos = nullptr) const; 

    //interpolation of kalman states
    kalman::KalmanError interpStep(kalman::KalmanState& state_interp,
                                   const kalman::KalmanState& state0,
                                   const kalman::KalmanState& state1,
                                   double dt,
                                   bool fix_estimate,
                                   bool* fixed = nullptr,
                                   const boost::optional<double>& Q_var = boost::optional<double>()) const;
    //kalman state integrity
    bool checkKalmanStateNumerical(const kalman::KalmanState& state) const;
    bool validateState(const kalman::KalmanState& state) const;
    
    //needed for feeding kalman
    void stateVecX(const kalman::Vector& x);
    void stateVecX(const kalman::KalmanState& state);

    virtual void stateVecXFromMM(kalman::Vector& x, const Measurement& mm) const = 0;
    virtual void measurementVecZ(kalman::Vector& z, const Measurement& mm) const = 0;
    virtual void covarianceMatP(kalman::Matrix& P,
                                const Measurement& mm, 
                                const reconstruction::Uncertainty& default_uncert) const = 0;
    virtual void measurementUncertMatR(kalman::Matrix& R, 
                                       const Measurement& mm, 
                                       const reconstruction::Uncertainty& default_uncert) const = 0;
    
    //needed for retrieval from kalman
    void storeState(Measurement& mm, 
                    const kalman::KalmanState& state,
                    int submodel_idx = -1) const;
    void storeState(Measurement& mm, 
                    const kalman::Vector& x, 
                    const kalman::Matrix& P,
                    int submodel_idx = -1) const;
    //helpers
    double timestep(const Measurement& mm) const;
    static double timestep(const Measurement& mm0, const Measurement& mm1);
    static double timestep(const boost::posix_time::ptime& ts0, const boost::posix_time::ptime& ts1);
    double distanceSqr(const Measurement& mm) const;

    void xPos(double& x, double& y, int submodel_idx = -1) const;
    void xPos(double& x, double& y, const kalman::Vector& x_vec, int submodel_idx = -1) const;
    void xPos(kalman::Vector& x_vec, double x, double y, int submodel_idx = -1) const;
    double xVar(const kalman::Matrix& P, int submodel_idx = -1) const;
    double yVar(const kalman::Matrix& P, int submodel_idx = -1) const;
    double xyCov(const kalman::Matrix& P, int submodel_idx = -1) const;

    void setVerbosity(int v) { verbosity_ = v; }
    void enableDebugging(bool ok);

protected:
    int verbosity() const { return verbosity_; }

    void setKalmanFilter(kalman::KalmanFilter* kalman_filter);

private:
    boost::posix_time::ptime ts_;

    int verbosity_ = 0;

    std::unique_ptr<kalman::KalmanFilter> kalman_filter_;
};

} // reconstruction
