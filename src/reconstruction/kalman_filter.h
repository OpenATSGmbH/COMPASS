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
#include "global.h"

#include <vector>

#include <Eigen/Core>

#include <boost/optional.hpp>

namespace kalman
{

/**
*/
class KalmanFilter
{
public:
    typedef KalmanError     Error;
    typedef KalmanInfoFlags InfoFlags;

    typedef std::function<void(Vector&)>         InvertStateFunc;
    typedef std::function<Vector(const Vector&)> ZFunc;
    typedef std::function<Matrix(const Matrix&)> RFunc;

    KalmanFilter(size_t dim_x, 
                 size_t dim_z);
    virtual ~KalmanFilter();

    size_t dimX() const { return dim_x_; }
    size_t dimZ() const { return dim_z_; }

    virtual void enableDebugging(bool ok);

    //modifies internal state
    Error predict(double dt,
                  double Q_var,
                  const OVector& u = OVector());
    Error update(const Vector& z,
                 const OMatrix& R = OMatrix());
    Error predictAndUpdate(double dt,
                           double Q_var,
                           const Vector& z,
                           const OMatrix& R = OMatrix(),
                           const OVector& u = OVector());

    //keeps internal state as is
    Error predictState(Vector& x, 
                       Matrix& P,
                       double dt,
                       bool fix_estimate = false,
                       bool* fixed = nullptr,
                       const OVector& u = OVector(),
                       KalmanState* state = nullptr,
                       const boost::optional<double>& Q_var = boost::optional<double>()) const;
    Error generateMeasurement(Vector& x_pred_mm,
                              Matrix& P_pred_mm,
                              const Vector& x_pred, 
                              const Matrix& P_pred) const;

    bool smooth(std::vector<kalman::Vector>& x_smooth,
                std::vector<kalman::Matrix>& P_smooth,
                const std::vector<KalmanState>& states,
                const XTransferFunc& x_tr = XTransferFunc(),
                double smooth_scale = 1.0,
                bool stop_on_fail = false,
                std::vector<bool>* state_valid = nullptr,
                std::vector<RTSDebugInfo>* debug_infos = nullptr) const;

    bool checkState() const;
    bool checkVariances() const;

    virtual void backup(bool recursive = false);
    virtual void revert(bool recursive = false);
    virtual void invert(bool recursive = false);

    virtual void init(const Vector& x, const Matrix& P, double dt, double Q_var);
    virtual void init(const Vector& x, const Matrix& P, double Q_var);
    virtual void init(const BasicKalmanState& state, bool xP_only = false);
    virtual void init(const KalmanState& state, bool xP_only = false);

    const Vector& getX() const { return x_; }
    const Matrix& getP() const { return P_; }
    const Matrix& getQ() const { return Q_; }
    const Matrix& getR() const { return R_; }

    double getProcessNoiseVariance() const { return Q_var_; }

    virtual void setX(const KalmanState& state) { setX(state.x); }
    void setX(const Vector& x) { x_ = x; }
    void setP(const Matrix& P) { P_ = P; }
    void setQ(const Matrix& Q) { Q_ = Q; }
    void setR(const Matrix& R) { R_ = R; }

    void setStateVec(const Vector& x) { setX(x); }
    void setCovarianceMat(const Matrix& P) { setP(P); }
    void setProcessUncertMat(const Matrix& Q) { setQ(Q); }
    void setMeasurementUncertMat(const Matrix& R) { setR(R); }

    Vector& xVec() { return x_; }
    Matrix& pMat() { return P_; }
    Matrix& qMat() { return Q_; }
    Matrix& rMat() { return R_; }

    void setInvertStateFunc(const InvertStateFunc& func) { invert_state_func_ = func; }
    void setZFunc(const ZFunc& func) { z_func_ = func; }
    void setRFunc(const RFunc& func) { R_func_ = func; }

    boost::optional<double> logLikelihood(bool check_eps) const;
    boost::optional<double> likelihood(bool check_eps) const;
    boost::optional<double> mahalanobis() const;

    kalman::KalmanState state(bool xP_only = false) const;
    virtual void state(kalman::KalmanState& s, bool xP_only = false) const;
    virtual void state(kalman::BasicKalmanState& s, bool xP_only = false) const;
    virtual bool validateState(const kalman::KalmanState& s, bool xP_only = false) const;
    virtual bool validateState(const kalman::BasicKalmanState& s, bool xP_only = false) const;

    virtual std::string asString(int info_flags = InfoFlags::InfoAll, const std::string& prefix = "") const;

    void invertState(Vector& x_inv, const Vector& x) const;
    void invertState(Vector& x) const;

    void xPos(double& x, double& y) const;
    virtual void xPos(double& x, double& y, const kalman::Vector& x_vec) const;
    virtual void xPos(kalman::Vector& x_vec, double x, double y) const;
    virtual double xVar(const kalman::Matrix& P) const;
    virtual double yVar(const kalman::Matrix& P) const;
    virtual double xyCov(const kalman::Matrix& P) const;
    virtual boost::optional<double> xVel(const kalman::Vector& x_vec) const { return boost::optional<double>(); }
    virtual boost::optional<double> yVel(const kalman::Vector& x_vec) const { return boost::optional<double>(); }
    virtual boost::optional<double> xAcc(const kalman::Vector& x_vec) const { return boost::optional<double>(); }
    virtual boost::optional<double> yAcc(const kalman::Vector& x_vec) const { return boost::optional<double>(); }

    virtual bool supportsSubmodels() const { return false; }
    virtual size_t numSubmodels() const { return 0; }
    virtual KalmanFilter* subModel(size_t idx) { return nullptr; }
    virtual const KalmanFilter* subModel(size_t idx) const { return nullptr; }

    bool isDebug() const { return debug_; }
    
    static bool checkState(const Vector& x, const Matrix& P);
    static bool checkVariances(const Vector& x, const Matrix& P);

    static Matrix continuousWhiteNoise(size_t dim, double dt = 1.0, double spectral_density = 1.0, size_t block_size = 1);
    static void continuousWhiteNoise(Matrix& Q_noise, size_t dim, double dt = 1.0, double spectral_density = 1.0, size_t block_size = 1);

    static bool invertMatrix(Matrix& Pinv, const Matrix& P);
    static boost::optional<double> likelihood(const Vector& x, const Matrix& P, bool check_eps);
    static boost::optional<double> logLikelihood(const Vector& x, const Matrix& P, bool check_eps);
    static boost::optional<double> mahalanobis(const Vector& dx, const Matrix& P, bool P_is_inv);
    static boost::optional<double> mahalanobisSqr(const Vector& dx, const Matrix& P, bool P_is_inv);
    
protected:
#if USE_EXPERIMENTAL_SOURCE
    friend class KalmanFilterIMM;
#endif

    void updateInternalMatrices(double dt, double Q_var);
    void fixEstimate(const Vector& x,
                     Matrix& P,
                     bool* fixed = nullptr) const;

    virtual void init(const Vector& x, const Matrix& P);

    virtual void updateInternalMatrices_impl(double dt, double Q_var) = 0;

    virtual Error predict_impl(Vector& x, 
                               Matrix& P,
                               double dt, 
                               double Q_var,
                               const OVector& u) = 0;
    virtual Error predictState_impl(Vector& x, 
                                    Matrix& P,
                                    double dt,
                                    double Q_var,
                                    bool mt_safe,
                                    const OVector& u,
                                    KalmanState* state) const = 0;
    virtual Error generateMeasurement_impl(Vector& x_pred_mm,
                                           Matrix& P_pred_mm,
                                           const Vector& x_pred, 
                                           const Matrix& P_pred) const;
    virtual Error update_impl(const Vector& z,
                              const Matrix& R) = 0;
    virtual bool smooth_impl(std::vector<Vector>& x_smooth,
                             std::vector<Matrix>& P_smooth,
                             const std::vector<KalmanState>& states,
                             const XTransferFunc& x_tr,
                             double smooth_scale,
                             bool stop_on_fail,
                             std::vector<bool>* state_valid,
                             std::vector<RTSDebugInfo>* debug_infos) const = 0;
    virtual bool smoothingStep_impl(Vector& x0_smooth,
                                    Matrix& P0_smooth,
                                    Vector& x1_pred,
                                    Matrix& P1_pred,
                                    const Vector& x1_smooth_tr,
                                    const Matrix& P1_smooth,
                                    const BasicKalmanState& state1,
                                    double dt1,
                                    double smooth_scale,
                                    RTSStepInfo* debug_info) const { return false; }

    virtual void printState(std::stringstream& strm, const std::string& prefix) const;
    virtual void printExtendedState(std::stringstream& strm, const std::string& prefix) const;
    virtual void printIntermSteps(std::stringstream& strm, const std::string& prefix) const;

    size_t dim_x_;
    size_t dim_z_;

    mutable Vector x_; // state
    mutable Matrix P_; // uncertainty covariance
    Matrix         Q_; // process uncertainty
    Matrix         R_; // measurement uncertainty

    Matrix K_;  // kalman gain
    Vector y_;
    Matrix S_;  // system uncertainty
    Matrix SI_; // inverse system uncertainty

    Matrix I_;  // identity matrix

private:
    void postConditionP(Matrix& P) const;
    void resetLikelihoods();

    mutable boost::optional<double> likelihood_;
    mutable boost::optional<double> log_likelihood_;
    mutable boost::optional<double> mahalanobis_;

    //before predict()
    mutable Vector x_backup_;
    mutable Matrix P_backup_;
    mutable double Q_var_backup_;

    //after predict()
    Vector x_prior_;
    Matrix P_prior_;

    //after update()
    Vector x_post_;
    Matrix P_post_;

    InvertStateFunc invert_state_func_;
    ZFunc           z_func_;
    RFunc           R_func_;

    double                  Q_var_;
    bool                    debug_ = false;
};

} // namespace kalman
