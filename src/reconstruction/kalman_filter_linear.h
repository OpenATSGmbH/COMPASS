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

#include "kalman_filter.h"
#include "kalman_defs.h"

#include <vector>

#include <Eigen/Core>

#include <boost/optional.hpp>

namespace kalman
{

/**
*/
class KalmanFilterLinear : public KalmanFilter
{
public:
    typedef std::function<void(Matrix&, double)>         FMatFunc;
    typedef std::function<void(Matrix&, double, double)> QMatFunc;

    KalmanFilterLinear(size_t dim_x, 
                       size_t dim_z, 
                       size_t dim_u,
                       const FMatFunc& F_func = FMatFunc(),
                       const QMatFunc& Q_func = QMatFunc());
    virtual ~KalmanFilterLinear();

    size_t dimU() const { return dim_u_; }

    void init(const KalmanState& state, bool xP_only = false) override final;
    void state(kalman::KalmanState& s, bool xP_only = false) const override final;
    void state(kalman::BasicKalmanState& s, bool xP_only = false) const override final;
    bool validateState(const kalman::KalmanState& s, bool xP_only = false) const override final;
    bool validateState(const kalman::BasicKalmanState& s, bool xP_only = false) const override final;

    void setB(const Matrix& B) { B_ = B; }
    void setF(const Matrix& F) { F_ = F; }
    void setH(const Matrix& H) { H_ = H; }
    void setM(const Matrix& M) { M_ = M; }

    void setControlTransitionMat(const Matrix& B) { setB(B); }
    void setStateTransitionMat(const Matrix& F) { setF(F); }
    void setMeasurementMat(const Matrix& H) { setH(H); }
    void setProcessMMCrossCorrMat(const Matrix& M) { setM(M); }

    const boost::optional<Matrix>& getB() const { return B_; }
    Matrix& bMat() { traced_assert(B_.has_value()); return B_.value(); }
    const Matrix& getF() const { return F_; }
    Matrix& fMat() { return F_; }
    const Matrix& getH() const { return H_; }
    Matrix& hMat() { return H_; }
    const Matrix& getM() const { return M_; }
    Matrix& mMat() { return M_; }

    void setFMatFunc(const FMatFunc& func) { F_func_ = func; }
    void setQMatFunc(const QMatFunc& func) { Q_func_ = func; }

protected:
    Error predict(Vector& x,
                  Matrix& P,
                  const Matrix& F,
                  const Matrix& Q,
                  const OMatrix& B,
                  const OVector& u = OVector()) const;
    Error update(const Vector& z,
                 const Matrix& R,
                 const Matrix& H);
    
    Error predict_impl(Vector& x, 
                       Matrix& P,
                       double dt,
                       double Q_var,
                       const OVector& u) override final;
    Error predictState_impl(Vector& x, 
                            Matrix& P,
                            double dt,
                            double Q_var,
                            bool mt_safe,
                            const OVector& u,
                            KalmanState* state) const override final;
    Error generateMeasurement_impl(Vector& x_pred_mm,
                                   Matrix& P_pred_mm,
                                   const Vector& x_pred, 
                                   const Matrix& P_pred) const override final;
    Error update_impl(const Vector& z,
                      const Matrix& R) override final;
    bool smooth_impl(std::vector<kalman::Vector>& x_smooth,
                     std::vector<kalman::Matrix>& P_smooth,
                     const std::vector<KalmanState>& states,
                     const XTransferFunc& x_tr,
                     double smooth_scale,
                     bool stop_on_fail,
                     std::vector<bool>* state_valid,
                     std::vector<RTSDebugInfo>* debug_infos) const override final;
    bool smoothingStep_impl(Vector& x0_smooth,
                            Matrix& P0_smooth,
                            Vector& x1_pred,
                            Matrix& P1_pred,
                            const Vector& x1_smooth_tr,
                            const Matrix& P1_smooth,
                            const BasicKalmanState& state1,
                            double dt1,
                            double smooth_scale,
                            RTSStepInfo* debug_info) const override final;

    void updateInternalMatrices_impl(double dt, double Q_var) override final;

    void printState(std::stringstream& strm, const std::string& prefix) const override final;
    void printExtendedState(std::stringstream& strm, const std::string& prefix) const override final;
    void printIntermSteps(std::stringstream& strm, const std::string& prefix) const override final;

private:
    size_t dim_u_;

    Matrix  F_; // state transition matrix
    OMatrix B_; // control transition matrix
    Matrix  M_; // process-measurement cross correlation
    Matrix  H_; // measurement function
    
    OVector z_;

    FMatFunc F_func_;
    QMatFunc Q_func_;

    double alpha_sq_ = 1.0; // fading memory control

    using KalmanFilter::dim_x_;
    using KalmanFilter::dim_z_;

    using KalmanFilter::x_;
    using KalmanFilter::P_;
    using KalmanFilter::Q_;
    using KalmanFilter::R_;

    using KalmanFilter::K_;
    using KalmanFilter::y_;
    using KalmanFilter::S_;
    using KalmanFilter::SI_;

    using KalmanFilter::I_;
};

} // namespace kalman
