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

#include "reconstruction_defs.h"

#include <memory>

#include <QPointF>

namespace reconstruction
{

class KalmanInterface;
class KalmanProjectionHandler;

/**
 * Kalman-filter-based state estimator.
 * - operates on a KalmanInterface to interact with a certain kalman variant
 * - provides kalman features such as
 *     - initialization of kalman filter
 *     - integration of measurements into kalman filter
 *     - smoothing of kalman states
 *     - interpolation of kalman states
 *     - predictions based on a certain kalman state
 *     - handling projection into local map crs
 *     - etc.
 * - holds a current state - generally not multithreading safe
 */
class KalmanEstimator
{
public:
    struct Settings
    {
        enum ReinitFlags
        {
            ReinitCheckTime     = 1 << 0, //checks for reinit using a timestep threshold
            ReinitCheckDistance = 1 << 1  //checks for reinit using a metric distance-based threshold
        };

        enum class StepFailStrategy
        {
            Assert = 0,      //assert if step fails
            Reinit,          //reinit kalman if step fails       
            ReturnInvalid    //return error and set update to invalid
        };

        Settings();

        kalman::Vector immMuInit() const;
        kalman::Matrix immMInit() const;

        double Q_var       = 900.0;   // default variance of kalman process (30*30)
        double R_var_undef = HighVar; // high variance for undefined values   (1000*1000)

        //size_t min_chain_size     = 2;       // minimum number of consecutive kalman updates without reinit            (0 = do not check)
        double max_distance_cart  = 50000.0; // maximum allowed distance of consecutive measurements in meters         (0 = do not check)
        double min_dt             = 0.001;   // minimum allowed time difference of consecutive measurements in seconds (0 = do not check)
        double max_dt             = 11.0;    // maximum allowed time difference of consecutive measurements in seconds (0 = do not check)

        double min_pred_dt = 0.001;

        int reinit_check_flags = ReinitCheckTime; // checks used for reinitialization of kalman

        MapProjDistanceCheck proj_distance_check     = MapProjDistanceCheck::Cart; // projection center distance check type
        double               max_proj_distance_cart  = 20000.0;                    // maximum distance from the current map projection origin in meters 
                                                                                   // before changing the projection center
        double               max_proj_distance_wgs84 = 0.2;                        // maximum distance from the current map projection origin in degrees 
                                                                                   // before changing the projection center

        double          resample_dt          = 2.0;                       // resampling step size in seconds
        double          resample_Q_var       = 100.0;                     // resampling process noise (10*10)
        StateInterpMode resample_interp_mode = StateInterpMode::BlendVar; // kalman state interpolation mode used during resampling

        StepFailStrategy step_fail_strategy = StepFailStrategy::ReturnInvalid;

        double smoothing_scale = 0.1;

        bool track_velocities      = true;
        bool track_accelerations   = true;

        bool fix_predictions        = true;
        bool fix_predictions_interp = false;

        bool extract_wgs84_pos = false;

        bool allow_backwards_step = false;
        
        int  verbosity   = 0;
        bool debug       = false;

        reconstruction::Uncertainty default_uncert; //default uncertainties used if none are provided in the measurement

        double imm_prob_transition_likely   = 5e-13; // probability to change to a likely state (e.g. from zero to accelerated)
        double imm_prob_transition_unlikely = 5e-13; // probability to change to an unlikely state (e.g. from zero to uniform)
    };

    enum class ReinitState
    {
        ReinitNotNeeded = 0,
        ReinitDistance,
        ReinitTime
    };

    enum class StepResult
    {
        Success = 0,
        FailStepTooSmall,
        FailKalmanError,
        FailResultInvalid
    };

    struct StepInfo
    {
        void reset();
        bool isOk() const;

        StepResult          result            = StepResult::Success; 
        kalman::KalmanError kalman_error      = kalman::KalmanError::NoError;
        bool                reinit_after_fail = false;
        bool                proj_changed      = false;
    };

    typedef std::vector<kalman::KalmanUpdate> Updates;
    typedef std::function<void(Updates&, size_t, size_t)> ChainFunc;

    KalmanEstimator();
    virtual ~KalmanEstimator();

    bool isInit() const;

    void init(std::unique_ptr<KalmanInterface>&& interface);
    void init(kalman::KalmanType ktype);
    
    void kalmanInit(kalman::KalmanUpdate& update,
                    const Measurement& mm);
    void kalmanInit(const kalman::KalmanUpdate& update);
    void kalmanInit(const kalman::KalmanUpdateMinimal& update);

    StepResult kalmanStep(kalman::KalmanUpdate& update,
                          const Measurement& mm,
                          const double* max_dt_override = nullptr);
    
    kalman::KalmanError kalmanPrediction(Measurement* mm,
                                         kalman::GeoProbState* gp_state,
                                         kalman::GeoProbState* gp_state_mm,
                                         double dt,
                                         bool* fixed = nullptr) const;
    kalman::KalmanError kalmanPrediction(Measurement* mm,
                                         kalman::GeoProbState* gp_state,
                                         kalman::GeoProbState* gp_state_mm,
                                         const boost::posix_time::ptime& ts,
                                         bool* fixed = nullptr) const;
    kalman::KalmanError kalmanPrediction(Measurement* mm,
                                         kalman::GeoProbState* gp_state,
                                         kalman::GeoProbState* gp_state_mm,
                                         const kalman::KalmanUpdate& ref_update,
                                         const boost::posix_time::ptime& ts,
                                         bool* fixed = nullptr,
                                         bool* proj_changed = nullptr);
    kalman::KalmanError kalmanPrediction(Measurement* mm,
                                         kalman::GeoProbState* gp_state,
                                         kalman::GeoProbState* gp_state_mm,
                                         const kalman::KalmanUpdateMinimal& ref_update,
                                         const boost::posix_time::ptime& ts,
                                         bool* fixed = nullptr,
                                         bool* proj_changed = nullptr);
    kalman::KalmanError kalmanPrediction(Measurement* mm,
                                         kalman::GeoProbState* gp_state,
                                         kalman::GeoProbState* gp_state_mm,
                                         const kalman::KalmanUpdate& ref_update0,
                                         const kalman::KalmanUpdate& ref_update1,
                                         const boost::posix_time::ptime& ts,
                                         size_t* num_fixed = nullptr,
                                         size_t* num_proj_changed = nullptr);
    kalman::KalmanError kalmanPrediction(Measurement* mm,
                                         kalman::GeoProbState* gp_state,
                                         kalman::GeoProbState* gp_state_mm,
                                         const kalman::KalmanUpdateMinimal& ref_update0,
                                         const kalman::KalmanUpdateMinimal& ref_update1,
                                         const boost::posix_time::ptime& ts,
                                         size_t* num_fixed = nullptr,
                                         size_t* num_proj_changed = nullptr);

    kalman::KalmanError comparePrediction(PredictionComparison& comparison,
                                          const kalman::GeoProbState& pred_state_mm,
                                          const Measurement& mm,
                                          int comparison_flags,
                                          KalmanProjectionHandler* phandler = nullptr) const;

    void storeUpdate(Measurement& mm, 
                     const kalman::KalmanUpdate& update,
                     bool force_wgs84_computation = false) const;
    void storeUpdate(Measurement& mm, 
                     const kalman::KalmanUpdateMinimal& update,
                     bool force_wgs84_computation = false) const;
    void storeUpdate(Reference& ref, 
                     const kalman::KalmanUpdate& update,
                     bool force_wgs84_computation = false) const;
    void storeUpdate(Reference& ref, 
                     const kalman::KalmanUpdateMinimal& update,
                     bool force_wgs84_computation = false) const;
    void storeUpdates(std::vector<Reference>& refs,
                      const std::vector<kalman::KalmanUpdate>& updates,
                      std::vector<boost::optional<Eigen::Vector2d>>* speedvec_tippos_wgs84 = nullptr,
                      std::vector<boost::optional<Eigen::Vector2d>>* accelvec_tippos_wgs84 = nullptr) const;
    void storeUpdateAndUnproject(Measurement& mm, 
                                 const kalman::KalmanUpdate& update,
                                 KalmanProjectionHandler& phandler,
                                 boost::optional<Eigen::Vector2d>* speedvec_tippos_wgs84 = nullptr,
                                 boost::optional<Eigen::Vector2d>* accelvec_tippos_wgs84 = nullptr,
                                 int submodel_idx = -1) const;
    void storeUpdateAndUnproject(Reference& ref, 
                                 const kalman::KalmanUpdate& update,
                                 KalmanProjectionHandler& phandler,
                                 boost::optional<Eigen::Vector2d>* speedvec_tippos_wgs84 = nullptr,
                                 boost::optional<Eigen::Vector2d>* accelvec_tippos_wgs84 = nullptr) const;
    
    bool smoothUpdates(std::vector<kalman::KalmanUpdate>& updates,
                       kalman::SmoothFailStrategy fail_strategy,
                       std::vector<kalman::RTSDebugInfo>* debug_infos = nullptr) const;
    bool interpUpdates(std::vector<kalman::KalmanUpdate>& interp_updates,
                       std::vector<kalman::KalmanUpdate>& updates,
                       size_t* num_steps_failed = nullptr) const;

    kalman::KalmanState currentState() const;
    Measurement currentStateAsMeasurement() const;
    QPointF currentPositionCart() const;
    QPointF currentPositionWGS84() const;
    const boost::posix_time::ptime& currentTime() const;
    const StepInfo& stepInfo() const;

    bool checkKalmanStateNumerical(kalman::KalmanState& state) const;
    bool checkPrediction(const Measurement& mm) const;
    bool validateState(const kalman::KalmanState& state) const;

    void changeProjection(kalman::Vector& state_vec, 
                          const Eigen::Vector2d& proj_center_old, 
                          const Eigen::Vector2d& proj_center_new,
                          KalmanProjectionHandler* proj_handler_ext = nullptr) const;
    Eigen::Vector2d state2LatLon(const kalman::Vector& state_vec, 
                                 const Eigen::Vector2d& proj_center,
                                 KalmanProjectionHandler* proj_handler_ext = nullptr) const;
    Eigen::Vector2d mmPos2Cart(const Measurement& mm,
                               const Eigen::Vector2d& proj_center,
                               KalmanProjectionHandler* proj_handler_ext = nullptr) const;

    std::string asString(int flags = kalman::KalmanInfoFlags::InfoAll, const std::string& prefix = "") const;

    void enableDebugging(bool ok);

    const KalmanInterface* interface() const { return kalman_interface_.get(); }

    static std::unique_ptr<KalmanInterface> createInterface(kalman::KalmanType ktype, 
                                                            const Settings& settings);

    static void extractVelAccPositionsWGS84(std::vector<boost::optional<Eigen::Vector2d>>& speedvec_tippos_wgs84,
                                            std::vector<boost::optional<Eigen::Vector2d>>& accelvec_tippos_wgs84,
                                            const std::vector<Measurement>& measurements);
    static void extractVelAccPositionsWGS84(std::vector<boost::optional<Eigen::Vector2d>>& speedvec_tippos_wgs84,
                                            std::vector<boost::optional<Eigen::Vector2d>>& accelvec_tippos_wgs84,
                                            const std::vector<Reference>& references);

    Settings& settings() { return settings_; }

    static const double HighStdDev;
    static const double HighVar;

private:
    void initDataStructs(kalman::KalmanUpdate& update,
                         const Measurement& mm);
    ReinitState needsReinit(const Measurement& mm,
                            const double* max_dt_override) const;
    void kalmanInterfaceReinit(kalman::KalmanUpdate& update,
                               const Measurement& mm);
    kalman::KalmanError kalmanInterfaceStep(kalman::KalmanUpdate& update,
                                            const Measurement& mm);
    void checkProjection(kalman::KalmanUpdate& update);
    void initProjection(double lat_proj_center,
                        double lon_proj_center,
                        const boost::optional<double>& lat_mm, 
                        const boost::optional<double>& lon_mm);

    void storePositionWGS84(kalman::KalmanUpdate& update);

    StepResult kalmanStepInternal(kalman::KalmanUpdate& update,
                                  const Measurement& mm,
                                  const double* max_dt_override);

    void executeChainFunc(Updates& updates, const ChainFunc& func) const;

    bool interpUpdates(std::vector<kalman::KalmanUpdate>& interp_updates,
                       const std::vector<kalman::KalmanUpdate>& updates,
                       size_t idx0,
                       size_t idx1,
                       double dt_sec,
                       double min_dt_sec,
                       double Q_var,
                       StateInterpMode interp_mode,
                       KalmanProjectionHandler& proj_handler,
                       size_t* num_steps_failed = nullptr) const;
    kalman::KalmanError predictBetween(kalman::KalmanUpdateMinimal& update_interp,
                                       const kalman::KalmanUpdateMinimal& update0,
                                       const kalman::KalmanUpdateMinimal& update1,
                                       const boost::posix_time::ptime& ts,
                                       double min_dt_sec,
                                       StateInterpMode interp_mode,
                                       KalmanProjectionHandler& proj_handler,
                                       size_t* num_fixed,
                                       size_t* num_proj_changed) const;
    
    void storeUpdate(Reference& ref, 
                     const kalman::KalmanUpdate& update,
                     KalmanProjectionHandler& phandler,
                     boost::optional<Eigen::Vector2d>* speedvec_tippos_wgs84 = nullptr,
                     boost::optional<Eigen::Vector2d>* accelvec_tippos_wgs84 = nullptr) const;
    void storeUpdate(Measurement& mm, 
                     const kalman::KalmanUpdate& update,
                     KalmanProjectionHandler& phandler,
                     boost::optional<Eigen::Vector2d>* speedvec_tippos_wgs84 = nullptr,
                     boost::optional<Eigen::Vector2d>* accelvec_tippos_wgs84 = nullptr,
                     int submodel_idx = -1) const;

    reconstruction::Uncertainty defaultUncert(const Measurement& mm) const;

    bool checkState(const kalman::KalmanUpdate& update) const;

    static void extractVelAccPosWGS84(boost::optional<Eigen::Vector2d>& speedvec_tippos_wgs84,
                                      boost::optional<Eigen::Vector2d>& accelvec_tippos_wgs84,
                                      KalmanProjectionHandler& phandler,
                                      const Measurement& mm);

    std::unique_ptr<KalmanInterface>         kalman_interface_;
    std::unique_ptr<KalmanProjectionHandler> proj_handler_;

    Settings settings_;

    double max_distance_sqr_;

    StepInfo step_info_;
};

} // reconstruction
