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

#include "kalman_estimator.h"
#include "kalman_projection.h"
#include "kalman_interface.h"

#include "targetreportdefs.h"

#include "timeconv.h"
#include "spline_interpolator.h"

#include "logger.h"
#include "global.h"

#include "kalman_interface_umkalman2d.h"
#if USE_EXPERIMENTAL_SOURCE
#include "kalman_interface_amkalman2d.h"
#include "kalman_interface_imm2d.h"
#include "kalman_interface_umkalman2dfull.h"
#endif

namespace reconstruction
{

const double KalmanEstimator::HighStdDev = 1000.0;
const double KalmanEstimator::HighVar    = KalmanEstimator::HighStdDev * KalmanEstimator::HighStdDev;

/**
*/
KalmanEstimator::Settings::Settings()
{
}

/**
*/
kalman::Vector KalmanEstimator::Settings::immMuInit() const
{
    kalman::Vector imm_mu_init;

    imm_mu_init.resize(3);
    imm_mu_init << 0.33, 0.34, 0.33;

    return imm_mu_init;
}

/**
*/
kalman::Matrix KalmanEstimator::Settings::immMInit() const
{
    kalman::Matrix imm_M_init;

    imm_M_init.resize(3, 3);

#if 0
    const double M_change        = imm_prob_transition_likely + imm_prob_transition_unlikely;
    const double M_remain        = std::fma(-M_change, 1.0, 1.0); // 1.0 - M_change;
    const double M_change_mid    = std::fma(-M_remain, 1.0, 1.0) * 0.5;
    const double M_change_big    = imm_prob_transition_likely;
    const double M_change_small  = imm_prob_transition_unlikely;
#else
    const double factor   = 0.5;
    const double M_remain = 0.999999999999999999;
    const double M_change = 0.000000000000000001;

    const double M_change_mid    = M_change * 0.5;
    const double M_change_big    = M_change * factor;
    const double M_change_small  = M_change * (1.0 - factor);
#endif

                   //   zero,             uniform,              accelerated
    imm_M_init <<       M_remain,         M_change_small,       M_change_big,     // zero
                        M_change_small,   M_remain,             M_change_big,     // uniform
                        M_change_mid,     M_change_mid,         M_remain;         // accelerated

    // std::cout << std::setprecision(24) << "remain: " << M_remain << "\n"
    //                                    << "mid:    " << M_change_mid << "\n"
    //                                    << "big:    " << M_change_big << "\n"
    //                                    << "small:  " << M_change_small << "\n";
    // traced_assert(false);

    return imm_M_init;
}

/**
*/
void KalmanEstimator::StepInfo::reset()
{
    result            = StepResult::Success;
    kalman_error      = kalman::KalmanError::NoError;
    reinit_after_fail = false;
    proj_changed      = false;
}

/**
*/
bool KalmanEstimator::StepInfo::isOk() const
{
    return (result == StepResult::Success);
}

/**
*/
KalmanEstimator::KalmanEstimator()
:   proj_handler_(new KalmanProjectionHandler)
{
}

/**
*/
KalmanEstimator::~KalmanEstimator() = default;

/**
*/
bool KalmanEstimator::isInit() const
{
    return (kalman_interface_ != nullptr);
}

/**
 * Initializes the estimator based on its current settings and the passed interface.
*/
void KalmanEstimator::init(std::unique_ptr<KalmanInterface>&& interface)
{
    traced_assert(interface);
    
    kalman_interface_ = std::move(interface);
    traced_assert(kalman_interface_);

    //configure & init interface
    kalman_interface_->setVerbosity(settings_.verbosity);
    
    //configure projection handler
    proj_handler_->settings().proj_dist_check        = settings_.proj_distance_check;
    proj_handler_->settings().proj_max_dist_cart_sqr = settings_.max_proj_distance_cart * settings_.max_proj_distance_cart;
    proj_handler_->settings().proj_max_dist_wgs84    = settings_.max_proj_distance_wgs84;

    //cache some values
    max_distance_sqr_ = settings_.max_distance_cart * settings_.max_distance_cart;
}

/**
*/
std::unique_ptr<KalmanInterface> KalmanEstimator::createInterface(kalman::KalmanType ktype,
                                                                  const Settings& settings)
{
    if (ktype == kalman::KalmanType::UMKalman2D)
    {
        return std::unique_ptr<KalmanInterface>(new reconstruction::KalmanInterfaceUMKalman2D(settings.track_velocities));
    }
    else if (ktype == kalman::KalmanType::AMKalman2D)
    {
#if USE_EXPERIMENTAL_SOURCE
        return std::unique_ptr<KalmanInterface>(new reconstruction::KalmanInterfaceAMKalman2D());
#else
        throw std::runtime_error("KalmanEstimator: createInterface: reconstructor type not supported by build");
#endif
    }
    else if (ktype == kalman::KalmanType::IMMKalman2D)
    {
#if USE_EXPERIMENTAL_SOURCE
        return std::unique_ptr<KalmanInterface>(new reconstruction::KalmanInterfaceIMM2D(settings.immMuInit(), settings.immMInit()));
#else
        throw std::runtime_error("KalmanEstimator: createInterface: reconstructor type not supported by build");
#endif
    }
    else if (ktype == kalman::KalmanType::UMKalman2DFull)
    {
#if USE_EXPERIMENTAL_SOURCE
        return std::unique_ptr<KalmanInterface>(new reconstruction::KalmanInterfaceUMKalman2DFull(settings.track_velocities));
#else
        throw std::runtime_error("KalmanEstimator: createInterface: reconstructor type not supported by build");
#endif
    }
    else
    {
        throw std::runtime_error("KalmanEstimator: createInterface: unknown kalman type");
    }

    return {};
}

/**
 * Initializes the estimator based on its current settings and the passed kalman variant.
 */
void KalmanEstimator::init(kalman::KalmanType ktype)
{
    auto interface = KalmanEstimator::createInterface(ktype, settings_);
    init(std::move(interface));
}

/**
*/
kalman::KalmanState KalmanEstimator::currentState() const
{
    traced_assert(isInit());
    return kalman_interface_->currentState();
}

/**
*/
Measurement KalmanEstimator::currentStateAsMeasurement() const
{
    traced_assert(isInit());
    return kalman_interface_->currentStateAsMeasurement();
}

/**
*/
QPointF KalmanEstimator::currentPositionCart() const
{
    double x_cart, y_cart;
    kalman_interface_->xPos(x_cart, y_cart);

    return QPointF(x_cart, y_cart);
}

/**
*/
QPointF KalmanEstimator::currentPositionWGS84() const
{
    auto pos_cart = currentPositionCart();

    //unproject to wgs84
    double lat, lon;
    proj_handler_->unproject(lat, lon, pos_cart.x(), pos_cart.y());

    return QPointF(lat, lon);
}

/**
*/
const boost::posix_time::ptime& KalmanEstimator::currentTime() const
{
    traced_assert(isInit());
    return kalman_interface_->currentTime();
}

/**
*/
const KalmanEstimator::StepInfo& KalmanEstimator::stepInfo() const
{
    return step_info_;
}

/**
 * Notice: no reprojection to wgs84 will be executed, the wgs84 pos of the update will be stored to the mm.
*/
void KalmanEstimator::storeUpdate(Measurement& mm, 
                                  const kalman::KalmanUpdate& update,
                                  bool force_wgs84_computation) const
{
    traced_assert(isInit());

    kalman_interface_->storeState(mm, update.state);

    mm.t         = update.t;
    mm.source_id = update.source_id;

    //determine wgs84 pos
    bool wgs84_pos_handled = true;
    if (update.has_wgs84_pos)
    {
        //update has its own wgs84 pos => use it
        mm.lat = update.lat;
        mm.lon = update.lon;
    }
    else if (update.projection_center == proj_handler_->projectionCenter())
    {
        //update at least has the current projection center => unproject to lat/lon
        proj_handler_->unproject(mm.lat, mm.lon, mm.x, mm.y);
    }
    else if (force_wgs84_computation)
    {
        //force reprojection to wgs84 (overhead due to center change)
        proj_handler_->unproject(mm.lat, mm.lon, mm.x, mm.y, &update.projection_center);
    }
    else
    {
        //no way to determine wgs84 pos (at least not in a cheap way due to needed projection center change)
        wgs84_pos_handled = false;
    }
    traced_assert(wgs84_pos_handled);
}

/**
 * Notice: no reprojection to wgs84 will be executed, the wgs84 pos of the update will be stored to the mm.
*/
void KalmanEstimator::storeUpdate(Measurement& mm, 
                                  const kalman::KalmanUpdateMinimal& update,
                                  bool force_wgs84_computation) const
{
    traced_assert(isInit());

    kalman_interface_->storeState(mm, update.x, update.P);

    mm.t = update.t;

    //determine wgs84 pos
    bool wgs84_pos_handled = true;
    if (update.has_wgs84_pos)
    {
        //update has its own wgs84 pos => use it
        mm.lat = update.lat;
        mm.lon = update.lon;
    }
    else if (update.projection_center == proj_handler_->projectionCenter())
    {
        //update at least has the current projection center => unproject to lat/lon
        proj_handler_->unproject(mm.lat, mm.lon, mm.x, mm.y);
    }
    else if (force_wgs84_computation)
    {
        //force reprojection to wgs84 (overhead due to center change)
        proj_handler_->unproject(mm.lat, mm.lon, mm.x, mm.y, &update.projection_center);
    }
    else
    {
        //no way to determine wgs84 pos (at least not in a cheap way due to needed projection center change)
        wgs84_pos_handled = false;
    }
    traced_assert(wgs84_pos_handled);
}

/**
 * Notice: no reprojection to wgs84 will be executed, the wgs84 pos of the update will be stored to the mm.
*/
void KalmanEstimator::storeUpdate(Reference& ref, 
                                  const kalman::KalmanUpdate& update,
                                  bool force_wgs84_computation) const
{
    traced_assert(isInit());

    kalman_interface_->storeState(ref, update.state);

    ref.t         = update.t;
    ref.cov       = update.state.P;
    ref.source_id = update.source_id;

    //determine wgs84 pos
    bool wgs84_pos_handled = true;
    if (update.has_wgs84_pos)
    {
        //update has its own wgs84 pos => use it
        ref.lat = update.lat;
        ref.lon = update.lon;
    }
    else if (update.projection_center == proj_handler_->projectionCenter())
    {
        //update at least has the current projection center => unproject to lat/lon
        proj_handler_->unproject(ref.lat, ref.lon, ref.x, ref.y);
    }
    else if (force_wgs84_computation)
    {
        //force reprojection to wgs84 (overhead due to center change)
        proj_handler_->unproject(ref.lat, ref.lon, ref.x, ref.y, &update.projection_center);
    }
    else
    {
        //no way to determine wgs84 pos (at least not in a cheap way due to needed projection center change)
        wgs84_pos_handled = false;
    }
    traced_assert(wgs84_pos_handled);
}

/**
 * Notice: no reprojection to wgs84 will be executed, the wgs84 pos of the update will be stored to the mm.
*/
void KalmanEstimator::storeUpdate(Reference& ref, 
                                  const kalman::KalmanUpdateMinimal& update,
                                  bool force_wgs84_computation) const
{
    traced_assert(isInit());

    kalman_interface_->storeState(ref, update.x, update.P);

    ref.t   = update.t;
    ref.cov = update.P;

    //determine wgs84 pos
    bool wgs84_pos_handled = true;
    if (update.has_wgs84_pos)
    {
        //update has its own wgs84 pos => use it
        ref.lat = update.lat;
        ref.lon = update.lon;
    }
    else if (update.projection_center == proj_handler_->projectionCenter())
    {
        //update at least has the current projection center => unproject to lat/lon
        proj_handler_->unproject(ref.lat, ref.lon, ref.x, ref.y);
    }
    else if (force_wgs84_computation)
    {
        //force reprojection to wgs84 (overhead due to center change)
        proj_handler_->unproject(ref.lat, ref.lon, ref.x, ref.y, &update.projection_center);
    }
    else
    {
        //no way to determine wgs84 pos (at least not in a cheap way due to needed projection center change)
        wgs84_pos_handled = false;
    }
    traced_assert(wgs84_pos_handled);
}

/**
 * Notice: will reproject from the update's local system to wgs84 using the passed projection handler.
*/
void KalmanEstimator::storeUpdateAndUnproject(Measurement& mm, 
                                              const kalman::KalmanUpdate& update,
                                              KalmanProjectionHandler& phandler,
                                              boost::optional<Eigen::Vector2d>* speedvec_tippos_wgs84,
                                              boost::optional<Eigen::Vector2d>* accelvec_tippos_wgs84,
                                              int submodel_idx) const
{
    storeUpdate(mm, update, phandler, speedvec_tippos_wgs84, accelvec_tippos_wgs84, submodel_idx);
}

/**
 * Notice: will reproject from the update's local system to wgs84 using the passed projection handler.
*/
void KalmanEstimator::storeUpdateAndUnproject(Reference& ref, 
                                              const kalman::KalmanUpdate& update,
                                              KalmanProjectionHandler& phandler,
                                              boost::optional<Eigen::Vector2d>* speedvec_tippos_wgs84,
                                              boost::optional<Eigen::Vector2d>* accelvec_tippos_wgs84) const
{
    storeUpdate(ref, update, phandler, speedvec_tippos_wgs84, accelvec_tippos_wgs84, -1);
}

/**
 * Notice: will reproject from the update's local system to wgs84.
 * Internal version.
*/
void KalmanEstimator::storeUpdate(Measurement& mm, 
                                  const kalman::KalmanUpdate& update,
                                  KalmanProjectionHandler& phandler,
                                  boost::optional<Eigen::Vector2d>* speedvec_tippos_wgs84,
                                  boost::optional<Eigen::Vector2d>* accelvec_tippos_wgs84,
                                  int submodel_idx) const
{
    traced_assert(isInit());

    if (speedvec_tippos_wgs84)
        speedvec_tippos_wgs84->reset();
    if (accelvec_tippos_wgs84)
        accelvec_tippos_wgs84->reset();

    kalman_interface_->storeState(mm, update.state, submodel_idx);

    mm.t         = update.t;
    mm.Q_var     = update.state.Q_var;
    mm.source_id = update.source_id;

    //unproject to lat/lon
    phandler.unproject(mm.lat, mm.lon, mm.x, mm.y, &update.projection_center);

    //obtain tip position of speed vector if desired
    if (speedvec_tippos_wgs84 && mm.hasVelocity())
    {
        *speedvec_tippos_wgs84 = Eigen::Vector2d();
        phandler.unproject(speedvec_tippos_wgs84->value()[ 0 ], 
                           speedvec_tippos_wgs84->value()[ 1 ], 
                           mm.x + mm.vx.value(), 
                           mm.y + mm.vy.value(), 
                           &update.projection_center);
    }

    //obtain tip position of acceleration vector if desired
    if (accelvec_tippos_wgs84 && mm.hasAcceleration())
    {
        *accelvec_tippos_wgs84 = Eigen::Vector2d();
        phandler.unproject(accelvec_tippos_wgs84->value()[ 0 ], 
                           accelvec_tippos_wgs84->value()[ 1 ], 
                           mm.x + mm.ax.value(), 
                           mm.y + mm.ay.value(), 
                           &update.projection_center);
    }

    //loginf << "(" << update.projection_center.x() << "," << update.projection_center.y() << ") "
    //                                            << mm.x << "," << mm.y << " => " << mm.lat << "," << mm.lon;
}

/**
 * Notice: will reproject from the update's local system to wgs84.
 * Internal version.
*/
void KalmanEstimator::storeUpdate(Reference& ref, 
                                  const kalman::KalmanUpdate& update,
                                  KalmanProjectionHandler& phandler,
                                  boost::optional<Eigen::Vector2d>* speedvec_tippos_wgs84,
                                  boost::optional<Eigen::Vector2d>* accelvec_tippos_wgs84) const
{
    Measurement* mm = &ref;
    storeUpdate(*mm, update, phandler, speedvec_tippos_wgs84, accelvec_tippos_wgs84);

    ref.cov            = update.state.P;
    ref.projchange_pos = update.proj_changed;
    ref.reset_pos      = update.reinit;
}

/**
 * Notice: will reproject from the updates local system to wgs84.
 */
void KalmanEstimator::storeUpdates(std::vector<Reference>& refs,
                                   const std::vector<kalman::KalmanUpdate>& updates,
                                   std::vector<boost::optional<Eigen::Vector2d>>* speedvec_tippos_wgs84,
                                   std::vector<boost::optional<Eigen::Vector2d>>* accelvec_tippos_wgs84) const
{
    KalmanProjectionHandler phandler;

    size_t n = updates.size();

    refs.resize(n);

    if (speedvec_tippos_wgs84)
        speedvec_tippos_wgs84->assign(n, boost::optional<Eigen::Vector2d>());
    if (accelvec_tippos_wgs84)
        accelvec_tippos_wgs84->assign(n, boost::optional<Eigen::Vector2d>());

    for (size_t i = 0; i < n; ++i)
    {
        storeUpdate(refs[ i ], 
                    updates[ i ], 
                    phandler, 
                    speedvec_tippos_wgs84 ? &(*speedvec_tippos_wgs84)[ i ] : nullptr, 
                    accelvec_tippos_wgs84  ? &(*accelvec_tippos_wgs84 )[ i ] : nullptr);
    }
}

/**
*/
void KalmanEstimator::extractVelAccPosWGS84(boost::optional<Eigen::Vector2d>& speedvec_tippos_wgs84,
                                            boost::optional<Eigen::Vector2d>& accelvec_tippos_wgs84,
                                            KalmanProjectionHandler& phandler,
                                            const Measurement& mm)
{
    speedvec_tippos_wgs84.reset();
    accelvec_tippos_wgs84.reset();

    if (mm.hasVelocity())
    {
        Eigen::Vector2d proj_center(mm.lat, mm.lon);

        speedvec_tippos_wgs84 = Eigen::Vector2d();
        phandler.unproject(speedvec_tippos_wgs84.value()[ 0 ], 
                           speedvec_tippos_wgs84.value()[ 1 ], 
                           mm.vx.value(), 
                           mm.vy.value(),
                           &proj_center);
    }

    if (mm.hasAcceleration())
    {
        Eigen::Vector2d proj_center(mm.lat, mm.lon);

        accelvec_tippos_wgs84 = Eigen::Vector2d();
        phandler.unproject(accelvec_tippos_wgs84.value()[ 0 ], 
                           accelvec_tippos_wgs84.value()[ 1 ], 
                           mm.ax.value(), 
                           mm.ay.value(),
                           &proj_center);
    }
}

/**
*/
void KalmanEstimator::extractVelAccPositionsWGS84(std::vector<boost::optional<Eigen::Vector2d>>& speedvec_tippos_wgs84,
                                                  std::vector<boost::optional<Eigen::Vector2d>>& accelvec_tippos_wgs84,
                                                  const std::vector<Measurement>& measurements)
{
    KalmanProjectionHandler phandler;

    size_t n = measurements.size();

    speedvec_tippos_wgs84.assign(n, boost::optional<Eigen::Vector2d>());
    accelvec_tippos_wgs84.assign(n, boost::optional<Eigen::Vector2d>());

    for (size_t i = 0; i < n; ++i)
        extractVelAccPosWGS84(speedvec_tippos_wgs84[ i ], accelvec_tippos_wgs84[ i ], phandler, measurements[ i ]);
}

/**
*/
void KalmanEstimator::extractVelAccPositionsWGS84(std::vector<boost::optional<Eigen::Vector2d>>& speedvec_tippos_wgs84,
                                                  std::vector<boost::optional<Eigen::Vector2d>>& accelvec_tippos_wgs84,
                                                  const std::vector<Reference>& references)
{
    KalmanProjectionHandler phandler;

    size_t n = references.size();

    speedvec_tippos_wgs84.assign(n, boost::optional<Eigen::Vector2d>());
    accelvec_tippos_wgs84.assign(n, boost::optional<Eigen::Vector2d>());

    for (size_t i = 0; i < n; ++i)
        extractVelAccPosWGS84(speedvec_tippos_wgs84[ i ], accelvec_tippos_wgs84[ i ], phandler, references[ i ]);
}

/**
 * Inits the measurement and its respective update.
*/
void KalmanEstimator::initDataStructs(kalman::KalmanUpdate& update,
                                      const Measurement& mm)
{
    //reset update flags
    update.resetFlags();

    //store info
    update.t         = mm.t;
    update.source_id = mm.source_id;

    //project measurement to cartesian
    //@TODO: slightly hacky: x and y are mutable to do this local projection on-the-fly
    proj_handler_->project(mm.x, mm.y, mm.lat, mm.lon);
}

/**
 * Init kalman from measurement.
 */
void KalmanEstimator::kalmanInit(kalman::KalmanUpdate& update,
                                 const Measurement& mm)
{
    traced_assert(isInit());

    step_info_.reset();

    //init projection
    initProjection(mm.lat, mm.lon, mm.lat, mm.lon);
    
    //init measurement + update
    initDataStructs(update, mm);

    //reinit kalman
    kalmanInterfaceReinit(update, mm);

    update.lat               = mm.lat;
    update.lon               = mm.lon;
    update.projection_center = proj_handler_->projectionCenter();
    update.has_wgs84_pos     = true;
    update.valid             = true;

    //check state => should be ok at this point, otherwise assert
    bool kalman_update_check = checkState(update);
    if (!kalman_update_check)
    {
        logerr << "init from mm yielded nan\n\n"
               << update.state.print() << "\n"
               << mm.asString() << "\n";
        traced_assert(kalman_update_check);
    }
}

/**
 * Init kalman from update.
 */
void KalmanEstimator::kalmanInit(const kalman::KalmanUpdate& update)
{
    traced_assert(isInit());
    traced_assert(update.valid);

    step_info_.reset();

    //init projection
    initProjection(update.projection_center.x(), update.projection_center.y(), {}, {});

    //reinit kalman from update
    kalman_interface_->kalmanInit(update.state, update.t);

    //check state => should be ok at this point, otherwise assert
    bool kalman_update_check = checkState(update);
    if (!kalman_update_check)
    {
        logerr << "init from update yielded nan\n\n"
               << update.state.print() << "\n";
        traced_assert(kalman_update_check);
    }
}

/**
 * Init kalman from minimal update.
 */
void KalmanEstimator::kalmanInit(const kalman::KalmanUpdateMinimal& update)
{
    traced_assert(isInit());
    traced_assert(update.valid);

    step_info_.reset();

    //init projection
    initProjection(update.projection_center.x(), update.projection_center.y(), {}, {}); 

    //reinit kalman from update
    kalman_interface_->kalmanInit(update.x, update.P, update.t, update.Q_var);

    //check state => should be ok at this point, otherwise assert
    bool kalman_update_check = checkState(update);
    if (!kalman_update_check)
    {
        logerr << "init from minimal update yielded nan\n\n"
               << update.x << "\n"
               << update.P << "\n"
               << update.t << "\n"
               << update.Q_var << "\n";
        traced_assert(kalman_update_check);
    }
}

/**
 * Checks if the measurement triggers a reinitialization of the kalman filter based on different criteria.
*/
KalmanEstimator::ReinitState KalmanEstimator::needsReinit(const Measurement& mm) const
{
    if (settings_.reinit_check_flags & Settings::ReinitFlags::ReinitCheckDistance)
    {
        if (settings_.max_distance_cart > 0)
        {
            const double d_sqr = kalman_interface_->distanceSqr(mm);
            if (d_sqr > max_distance_sqr_)
                return KalmanEstimator::ReinitState::ReinitDistance;
        }
    }

    if ((settings_.reinit_check_flags & Settings::ReinitFlags::ReinitCheckTime) && settings_.max_dt > 0)
    {
        const double dt = kalman_interface_->timestep(mm);
        if (std::fabs(dt) > settings_.max_dt)
            return KalmanEstimator::ReinitState::ReinitTime;
    }

    return KalmanEstimator::ReinitState::ReinitNotNeeded;
}

/**
*/
reconstruction::Uncertainty KalmanEstimator::defaultUncert(const Measurement& mm) const
{
    reconstruction::Uncertainty uncert = settings_.default_uncert;

    //set to high uncertainty if value is missing (pos is always available)
    if (!mm.hasVelocity())
        uncert.speed_var = settings_.R_var_undef;
    if (!mm.hasAcceleration())
        uncert.acc_var = settings_.R_var_undef;

    return uncert;
}

/**
 * Reinitializes the kalman filter and marks the update.
*/
void KalmanEstimator::kalmanInterfaceReinit(kalman::KalmanUpdate& update,
                                            const Measurement& mm)
{
    if (settings_.verbosity > 0)
        loginf << "reinitializing kalman filter at t = " << mm.t;

    //reinit kalman state
    kalman_interface_->kalmanInit(update.state, mm, defaultUncert(mm), settings_.Q_var);

    update.reinit = true;
}

/**
 * Executes a kalman step.
*/
kalman::KalmanError KalmanEstimator::kalmanInterfaceStep(kalman::KalmanUpdate& update,
                                                         const Measurement& mm)
{
    auto err = kalman_interface_->kalmanStep(update.state, mm, defaultUncert(mm), settings_.Q_var);

    if (err != kalman::KalmanError::NoError)
    {
        if (settings_.verbosity > 0)
            logwrn << "kalman step failed @ t=" << mm.t << " (ErrCode" << (int)err << ")";
        return err;
    }

    return kalman::KalmanError::NoError;
}

/**
*/
void KalmanEstimator::storePositionWGS84(kalman::KalmanUpdate& update)
{
    //!projection center must match!
    traced_assert(proj_handler_->projectionCenter() == update.projection_center);

    //get local cartesian coordinates
    double x, y;
    kalman_interface_->xPos(x, y, update.state.x);

    //unproject to wgs84
    proj_handler_->unproject(update.lat, update.lon, x, y);

    update.has_wgs84_pos = true;
}

/**
 * Change projection center if needed and keep kalman state coordinates up-to-date with projection.
*/
void KalmanEstimator::checkProjection(kalman::KalmanUpdate& update)
{
    //change projection?
    if (proj_handler_->changeProjectionIfNeeded(update, *kalman_interface_))
    {
        //projection changed => update kalman state to reprojected position
        kalman_interface_->stateVecX(update.state);

        if (settings_.verbosity > 1)
            loginf << "rhanged map projection @t=" << update.t;

        step_info_.proj_changed = true;
        update.proj_changed = true;
    }

    //store current projection to update
    update.projection_center = proj_handler_->projectionCenter();
}

/**
*/
void KalmanEstimator::initProjection(double lat_proj_center,
                                     double lon_proj_center,
                                     const boost::optional<double>& lat_mm, 
                                     const boost::optional<double>& lon_mm)
{
    //skip reinit if in wgs84 range of current projection
    if (lat_mm.has_value() &&
        lon_mm.has_value() &&
        proj_handler_->valid() && 
        settings_.proj_distance_check == MapProjDistanceCheck::Cart &&
        proj_handler_->inRangeWGS84(lat_mm.value(), lon_mm.value()))
        return;

    proj_handler_->initProjection(lat_proj_center, lon_proj_center);

    step_info_.proj_changed = true;
}

/**
*/
void KalmanEstimator::changeProjection(kalman::Vector& state_vec, 
                                       const Eigen::Vector2d& proj_center_old, 
                                       const Eigen::Vector2d& proj_center_new,
                                       KalmanProjectionHandler* proj_handler_ext) const
{
    if (proj_center_old == proj_center_new)
        return;

    auto proj_handler = proj_handler_ext ? proj_handler_ext : proj_handler_.get();

    double x, y;
    kalman_interface_->xPos(x, y, state_vec);

    double lat, lon;
    proj_handler->unproject(lat, lon, x, y, &proj_center_old);

    proj_handler->initProjection(proj_center_new.x(), proj_center_new.y());
    proj_handler->project(x, y, lat, lon);

    kalman_interface_->xPos(state_vec, x, y);
}

/**
*/
KalmanEstimator::StepResult KalmanEstimator::kalmanStep(kalman::KalmanUpdate& update,
                                                        const Measurement& mm)
{
    step_info_.reset();

    auto result = kalmanStepInternal(update, mm);

    step_info_.result = result;

    if (result == KalmanEstimator::StepResult::Success)
    {
        if (step_info_.reinit_after_fail)
            traced_assert(step_info_.kalman_error != kalman::KalmanError::NoError);
        else
            traced_assert(step_info_.kalman_error == kalman::KalmanError::NoError);
    }

    return result;
}

/**
*/
KalmanEstimator::StepResult KalmanEstimator::kalmanStepInternal(kalman::KalmanUpdate& update,
                                                                const Measurement& mm)
{
    traced_assert(isInit());

    //init measurement (will also project the mm wgs84 pos to the current local system)
    initDataStructs(update, mm);

    //check if timestep is too small
    auto tstep = kalman_interface_->timestep(mm);
    traced_assert(settings_.allow_backwards_step || tstep >= 0);

    if (std::fabs(tstep) < settings_.min_dt)
    {
        if (settings_.verbosity > 0)
            logwrn << "step " << kalman_interface_->timestep(mm) << " too small (<" << settings_.min_dt << "), skipping...";
        return KalmanEstimator::StepResult::FailStepTooSmall;
    }

    //check if reinit is needed
    if (needsReinit(mm) != ReinitState::ReinitNotNeeded)
    {
        //reinit
        kalmanInterfaceReinit(update, mm);
    }
    else
    {
        //normal step
        auto kalman_error   = kalmanInterfaceStep(update, mm);
        auto kalman_step_ok = kalman_error == kalman::KalmanError::NoError;
        bool update_ok      = kalman_step_ok && checkState(update);

        step_info_.kalman_error = kalman_error;

        //handle failed step?
        if (!update_ok)
        {
            if (settings_.verbosity > 0)
                logwrn << "step failed";

            KalmanEstimator::StepResult result = !kalman_step_ok ? KalmanEstimator::StepResult::FailKalmanError :
                                                                   KalmanEstimator::StepResult::FailResultInvalid;
            //print kalman state
            //kalman_interface_->printState();

            //print mm
            //loginf << "could not integrate measurement:\n" << mm.toString();

            //!note: the interface should always revert to the old state if a step fails!
            //!we thus assume that the last state is fully intact!
            if (settings_.step_fail_strategy == Settings::StepFailStrategy::Reinit)
            {
                //reinit to new measurement
                kalmanInterfaceReinit(update, mm);

                step_info_.reinit_after_fail = true;
            }
            else if (settings_.step_fail_strategy == Settings::StepFailStrategy::ReturnInvalid)
            {
                //keep old state and return error
                return result;
            }
            else
            {
                //assert on error
                traced_assert(update_ok);
                return result;
            }
        }
    }

    //check state => should be ok at this point, otherwise assert
    bool kalman_update_check = checkState(update);
    if (!kalman_update_check)
    {
        logerr << "step yielded nan for dt = " << tstep << "\n\n"
               << kalman_interface_->asString(kalman::KalmanInfoFlags::InfoAll) << "\n"
               << update.state.print() << "\n";
        traced_assert(kalman_update_check);
    }

    //update projection if needed
    checkProjection(update);

    //store wgs84 pos?
    if (settings_.extract_wgs84_pos)
        storePositionWGS84(update);

    update.valid = true;

    return KalmanEstimator::StepResult::Success;
}

/**
*/
bool KalmanEstimator::checkPrediction(const Measurement& mm) const
{
    if (mm.x_stddev.has_value() && !std::isfinite(mm.x_stddev.value()))
        return false;
    if (mm.y_stddev.has_value() && !std::isfinite(mm.y_stddev.value()))
        return false;
    if (mm.xy_cov.has_value() && !std::isfinite(mm.xy_cov.value()))
        return false;
    if (mm.vx_stddev.has_value() && !std::isfinite(mm.vx_stddev.value()))
        return false;
    if (mm.vy_stddev.has_value() && !std::isfinite(mm.vy_stddev.value()))
        return false;
    if (mm.ax_stddev.has_value() && !std::isfinite(mm.ax_stddev.value()))
        return false;
    if (mm.ay_stddev.has_value() && !std::isfinite(mm.ay_stddev.value()))
        return false;

    return true;
}

/**
*/
bool KalmanEstimator::checkState(const kalman::KalmanUpdate& update) const
{
    if (!kalman_interface_->checkKalmanStateNumerical(update.state))
        return false;

    double x, y;
    kalman_interface_->xPos(x, y, update.state.x);

    if (!std::isfinite(x) || !std::isfinite(y))
        return false;

    double xvar  = kalman_interface_->xVar(update.state.P);
    double yvar  = kalman_interface_->yVar(update.state.P);
    double xycov = kalman_interface_->xyCov(update.state.P);

    if (!std::isfinite(xvar) || !std::isfinite(yvar) || !std::isfinite(xycov))
        return false;

    return true;
}

/**
*/
kalman::KalmanError KalmanEstimator::kalmanPrediction(Measurement* mm,
                                                      kalman::GeoProbState* gp_state,
                                                      kalman::GeoProbState* gp_state_mm,
                                                      double dt,
                                                      bool* fixed) const
{
    traced_assert(isInit());

    kalman::KalmanState state;
    auto kalman_err = kalman_interface_->kalmanPrediction(state.x, 
                                                          state.P, 
                                                          dt,
                                                          settings_.fix_predictions,
                                                          fixed);
    if (kalman_err != kalman::KalmanError::NoError)
    {
        //debatable: should the step fail strategy apply here?
        if (settings_.step_fail_strategy == Settings::StepFailStrategy::Assert)
            traced_assert(kalman_err == kalman::KalmanError::NoError);
        return kalman_err;
    }

    if (mm)
    {
        kalman_interface_->storeState(*mm, state);
        proj_handler_->unproject(mm->lat, mm->lon, mm->x, mm->y);

        bool kalman_prediction_check = checkPrediction(*mm);
        if (!kalman_prediction_check)
        {
            logerr << "prediction yielded nan for dt = " << dt << "\n\n"
                << kalman_interface_->asString(kalman::KalmanInfoFlags::InfoAll) << "\n";
            traced_assert(kalman_prediction_check);
        }
    }

    if (gp_state)
    {
        gp_state->x           = state.x;
        gp_state->P           = state.P;
        gp_state->proj_center = proj_handler_->projectionCenter();
    }

    if (gp_state_mm)
    {
        auto kalman_err = kalman_interface_->kalmanPredictionMM(*gp_state_mm, state.x, state.P);
        traced_assert(kalman_err == kalman::KalmanError::NoError);

        gp_state->proj_center = proj_handler_->projectionCenter();
    }

    return kalman::KalmanError::NoError;
}

/**
*/
kalman::KalmanError KalmanEstimator::kalmanPrediction(Measurement* mm,
                                                      kalman::GeoProbState* gp_state,
                                                      kalman::GeoProbState* gp_state_mm,
                                                      const boost::posix_time::ptime& ts,
                                                      bool* fixed) const
{
    traced_assert(isInit());

    kalman::KalmanState state;

    const auto& curr_ts = kalman_interface_->currentTime();
    double dt = Utils::Time::partialSeconds(ts >= curr_ts ? ts - curr_ts : curr_ts - ts);
    if (dt <= settings_.min_pred_dt)
    {
        //close enough to current state => reuse
        state = kalman_interface_->currentState();

        if (fixed)
            *fixed = false;
    }
    else
    {
        //predict from current state
        auto kalman_err = kalman_interface_->kalmanPrediction(state.x, 
                                                              state.P, 
                                                              ts, 
                                                              settings_.fix_predictions, 
                                                              fixed);
        if (kalman_err != kalman::KalmanError::NoError)
        {
            //debatable: should the step fail strategy apply here?
            if (settings_.step_fail_strategy == Settings::StepFailStrategy::Assert)
                traced_assert(kalman_err == kalman::KalmanError::NoError);
            return kalman_err;
        }
    }

    if (mm)
    {
        kalman_interface_->storeState(*mm, state);
        proj_handler_->unproject(mm->lat, mm->lon, mm->x, mm->y);

        bool kalman_prediction_check = checkPrediction(*mm);
        if (!kalman_prediction_check)
        {
            logerr << "STATE x = \n" << state.x << "\n"
                << "STATE P = \n" << state.P;

            logerr << "prediction yielded nan\n\n"
                << "ts_cur: " << Utils::Time::toString(kalman_interface_->currentTime()) << "\n"
                << "ts:     " << Utils::Time::toString(ts) << "\n"
                << "dt:     " << KalmanInterface::timestep(kalman_interface_->currentTime(), ts) << "\n"
                << kalman_interface_->asString(kalman::KalmanInfoFlags::InfoAll) << "\n"
                << mm->asString() << "\n";
            traced_assert(kalman_prediction_check);
        }
    }

    if (gp_state)
    {
        gp_state->x           = state.x;
        gp_state->P           = state.P;
        gp_state->proj_center = proj_handler_->projectionCenter();
    }

    if (gp_state_mm)
    {
        auto kalman_err = kalman_interface_->kalmanPredictionMM(*gp_state_mm, state.x, state.P);
        traced_assert(kalman_err == kalman::KalmanError::NoError);

        gp_state->proj_center = proj_handler_->projectionCenter();
    }

    return kalman::KalmanError::NoError;
}

/**
 * Note: Changes the estimator's current state to the passed one.
*/
kalman::KalmanError KalmanEstimator::kalmanPrediction(Measurement* mm,
                                                      kalman::GeoProbState* gp_state,
                                                      kalman::GeoProbState* gp_state_mm,
                                                      const kalman::KalmanUpdate& ref_update,
                                                      const boost::posix_time::ptime& ts,
                                                      bool* fixed,
                                                      bool* proj_changed)
{
    kalmanInit(ref_update);

    if (proj_changed)
        *proj_changed = stepInfo().proj_changed;

    return kalmanPrediction(mm, gp_state, gp_state_mm, ts, fixed);
}

/**
 * Note: Changes the estimator's current state to the passed one.
*/
kalman::KalmanError KalmanEstimator::kalmanPrediction(Measurement* mm,
                                                      kalman::GeoProbState* gp_state,
                                                      kalman::GeoProbState* gp_state_mm,
                                                      const kalman::KalmanUpdateMinimal& ref_update,
                                                      const boost::posix_time::ptime& ts,
                                                      bool* fixed,
                                                      bool* proj_changed)
{
    kalmanInit(ref_update);

    if (proj_changed)
        *proj_changed = stepInfo().proj_changed;

    return kalmanPrediction(mm, gp_state, gp_state_mm, ts, fixed);
}

/**
 * Prediction by interpolation of two reference update predictions.
 * Note: Changes the estimator's current state to the passed one.
 */
kalman::KalmanError KalmanEstimator::kalmanPrediction(Measurement* mm,
                                                      kalman::GeoProbState* gp_state,
                                                      kalman::GeoProbState* gp_state_mm,
                                                      const kalman::KalmanUpdate& ref_update0,
                                                      const kalman::KalmanUpdate& ref_update1,
                                                      const boost::posix_time::ptime& ts,
                                                      size_t* num_fixed,
                                                      size_t* num_proj_changed)
{
    return kalmanPrediction(mm, gp_state, gp_state_mm, ref_update0.minimalInfo(), ref_update1.minimalInfo(), ts, num_fixed, num_proj_changed);
}

/**
 * Prediction by interpolation of two reference update predictions.
 * Note: Changes the estimator's current state to the passed one.
 */
kalman::KalmanError KalmanEstimator::kalmanPrediction(Measurement* mm,
                                                      kalman::GeoProbState* gp_state,
                                                      kalman::GeoProbState* gp_state_mm,
                                                      const kalman::KalmanUpdateMinimal& ref_update0,
                                                      const kalman::KalmanUpdateMinimal& ref_update1,
                                                      const boost::posix_time::ptime& ts,
                                                      size_t* num_fixed,
                                                      size_t* num_proj_changed)
{
    traced_assert(isInit());

    kalman::KalmanUpdateMinimal update_interp;
    auto kalman_error = predictBetween(update_interp,
                                       ref_update0,
                                       ref_update1,
                                       ts,
                                       settings_.min_pred_dt,
                                       settings_.resample_interp_mode,
                                       *proj_handler_,
                                       num_fixed,
                                       num_proj_changed);

    if (kalman_error != kalman::KalmanError::NoError)
    {
        //debatable: should the step fail strategy apply here?
        if (settings_.step_fail_strategy == Settings::StepFailStrategy::Assert)
            traced_assert(kalman_error == kalman::KalmanError::NoError);
        return kalman_error;
    }

    if (mm)
    {
        kalman_interface_->storeState(*mm, update_interp.x, update_interp.P);
        proj_handler_->unproject(mm->lat, mm->lon, mm->x, mm->y, &update_interp.projection_center);

        bool kalman_prediction_check = checkPrediction(*mm);
        if (!kalman_prediction_check)
        {
            logerr << "prediction yielded nan in interval\n\n"
                << "t0: " << Utils::Time::toString(ref_update0.t) << "\n"
                << "t1: " << Utils::Time::toString(ref_update1.t) << "\n"
                << "ts: " << Utils::Time::toString(ts)            << "\n"
                << "x0:\n" << ref_update0.x << "\n"
                << "P0:\n" << ref_update0.P << "\n"
                << "x1:\n" << ref_update0.x << "\n"
                << "P1:\n" << ref_update0.P << "\n";
            traced_assert(kalman_prediction_check);
        }
    }

    if (gp_state)
    {
        gp_state->x           = update_interp.x;
        gp_state->P           = update_interp.P;
        gp_state->proj_center = update_interp.projection_center;
    }

    if (gp_state_mm)
    {
        auto kalman_err = kalman_interface_->kalmanPredictionMM(*gp_state_mm, update_interp.x, update_interp.P);
        traced_assert(kalman_err == kalman::KalmanError::NoError);

        gp_state->proj_center = update_interp.projection_center;
    }

    return kalman::KalmanError::NoError;
}

/**
*/
kalman::KalmanError KalmanEstimator::comparePrediction(PredictionComparison& comparison,
                                                       const kalman::GeoProbState& pred_state_mm,
                                                       const Measurement& mm,
                                                       int comparison_flags,
                                                       KalmanProjectionHandler* phandler) const
{
    traced_assert(isInit());

    KalmanProjectionHandler* phandler_used = phandler ? phandler : proj_handler_.get();
    traced_assert(phandler_used);

    //backup cart coords
    double x_backup = mm.x;
    double y_backup = mm.y;

    //project measurement to prediction coord system
    phandler_used->initProjection(pred_state_mm.proj_center.x(), pred_state_mm.proj_center.y());
    phandler_used->project(mm.x, mm.y, mm.lat, mm.lon);

    //compare measurement to prediction
    auto err = kalman_interface_->comparePrediction(comparison, pred_state_mm, mm, defaultUncert(mm), comparison_flags);

    //restore cart coords
    mm.x = x_backup;
    mm.y = y_backup;

    return err;
}

/**
*/
void KalmanEstimator::executeChainFunc(Updates& updates, const ChainFunc& func) const
{
    if (!func || updates.empty())
        return;

    if (updates.size() < 2)
    {
        func(updates, 0, 0);
        return;
    }

    size_t idx0 = 0;
    size_t n    = updates.size();
    for (size_t i = 1; i < n; ++i)
    {
        if (updates[ i ].reinit)
        {
            //chain ended => execute func on chain and start new one
            func(updates, idx0, i - 1);
            idx0 = i;
        }
    }

    //last chain
    func(updates, idx0, n - 1);
}

/**
*/
bool KalmanEstimator::smoothUpdates(std::vector<kalman::KalmanUpdate>& updates,
                                    kalman::SmoothFailStrategy fail_strategy,
                                    std::vector<kalman::RTSDebugInfo>* debug_infos) const
{
    traced_assert(isInit());

    KalmanProjectionHandler phandler;

    bool ok = true;
    
    auto func = [ & ] (std::vector<kalman::KalmanUpdate>& updates, size_t idx0, size_t idx1)
    {
        //@TODO: what happens if smoothing fails? => assert for the moment
        ok = ok && kalman_interface_->smoothUpdates(updates, 
                                                    idx0, 
                                                    idx1, 
                                                    phandler, 
                                                    settings_.smoothing_scale,
                                                    fail_strategy,
                                                    debug_infos);
        //assert(ok);
    };

    executeChainFunc(updates, func);

    return ok;
}

/**
*/
bool KalmanEstimator::interpUpdates(std::vector<kalman::KalmanUpdate>& interp_updates,
                                    std::vector<kalman::KalmanUpdate>& updates,
                                    size_t* num_steps_failed) const
{
    if (num_steps_failed)
        *num_steps_failed = 0;

    KalmanProjectionHandler phandler;

    interp_updates.clear();

    bool ok = true;

    auto func = [ & ] (std::vector<kalman::KalmanUpdate>& updates, size_t idx0, size_t idx1)
    {
        std::vector<kalman::KalmanUpdate> interp_updates_chain;
        ok = ok && this->interpUpdates(interp_updates_chain,
                                       updates, 
                                       idx0, 
                                       idx1, 
                                       settings_.resample_dt,
                                       settings_.min_pred_dt,
                                       settings_.resample_Q_var,
                                       settings_.resample_interp_mode,
                                       phandler,
                                       num_steps_failed);
        
        interp_updates.insert(interp_updates.end(), interp_updates_chain.begin(), interp_updates_chain.end());
    };

    executeChainFunc(updates, func);

    return ok;
}

namespace
{
    /**
     * Generates factors for belnding states.
    */
    double blendFunc(double dt0, 
                     double dt1, 
                     double dt,
                     const kalman::KalmanState& state0,
                     const kalman::KalmanState& state1,
                     StateInterpMode mode,
                     const KalmanInterface& kalman_interface)
    {
        if (mode == StateInterpMode::BlendLinear)
        {
            return dt0 / dt;
        }
        else if (mode == StateInterpMode::BlendStdDev)
        {
            double stddev_x0 = std::sqrt(kalman_interface.xVar(state0.P));
            double stddev_y0 = std::sqrt(kalman_interface.yVar(state0.P));

            double stddev_x1 = std::sqrt(kalman_interface.xVar(state1.P));
            double stddev_y1 = std::sqrt(kalman_interface.yVar(state1.P));

            double w0 = std::max(stddev_x0, stddev_y0);
            double w1 = std::max(stddev_x1, stddev_y1);

            return w0 / (w0 + w1);
        }
        else if (mode == StateInterpMode::BlendVar)
        {
            double var_x0 = kalman_interface.xVar(state0.P);
            double var_y0 = kalman_interface.yVar(state0.P);

            double var_x1 = kalman_interface.xVar(state1.P);
            double var_y1 = kalman_interface.yVar(state1.P);

            double w0 = std::max(var_x0, var_y0);
            double w1 = std::max(var_x1, var_y1);

            return w0 / (w0 + w1);
        }

        //StateInterpMode::BlendHalf
        return 0.5;
    };
}

/**
*/
bool KalmanEstimator::interpUpdates(std::vector<kalman::KalmanUpdate>& interp_updates,
                                    const std::vector<kalman::KalmanUpdate>& updates,
                                    size_t idx0,
                                    size_t idx1,
                                    double dt_sec,
                                    double min_dt_sec,
                                    double Q_var,
                                    StateInterpMode interp_mode,
                                    KalmanProjectionHandler& proj_handler,
                                    size_t* num_steps_failed) const
{
    traced_assert(isInit());

    interp_updates.clear();

    traced_assert(idx1 >= idx0);

    size_t n = idx1 - idx0 + 1;

    if (n <= 1)
        return true;

    const auto& first_update = updates[ idx0 ];
    const auto& last_update  = updates[ idx1 ];

    double dt_total = Utils::Time::partialSeconds(last_update.t - first_update.t);
    size_t n_estim  = std::ceil(dt_total / dt_sec) + 2;

    interp_updates.reserve(n_estim);

    auto addUpdate = [ & ] (const kalman::KalmanState& state, 
                            const Eigen::Vector2d& proj_center, 
                            const boost::posix_time::ptime& t)
    {
        interp_updates.push_back(kalman::KalmanUpdate());
        interp_updates.back().state             = state;
        interp_updates.back().projection_center = proj_center;
        interp_updates.back().t                 = t;
        interp_updates.back().valid             = true;
        interp_updates.back().reinit            = false;

        //@TODO: maybe set source id of first interval update? 
    };

    addUpdate(first_update.state, first_update.projection_center, first_update.t);

    boost::posix_time::milliseconds time_incr((int)(dt_sec * 1000.0));

    auto tcur = first_update.t + time_incr;

    size_t small_intervals = 0;

    auto x_tr_func = proj_handler.reprojectionTransform(&updates, kalman_interface_.get(), 0);

    kalman::KalmanState state1_tr;

    size_t failed_steps = 0;

    for (size_t i = idx0 + 1; i <= idx1; ++i)
    {
        const auto& update0       = updates[ i - 1];
        const auto& update1       = updates[ i    ];
        const auto& state0        = update0.state;
        const auto& state1        = update1.state;
        const auto& proj_center0  = update0.projection_center;
        const auto& proj_center1  = update1.projection_center;

        //use either process noise stored in update or default process noise
        double Q_var0 = update0.Q_var_interp.has_value() ? update0.Q_var_interp.value() : Q_var;
        double Q_var1 = update1.Q_var_interp.has_value() ? update1.Q_var_interp.value() : Q_var;

        //if (settings_.debug)
        //    loginf << "interpolating using Q_var " << Q_var0 << " / " << Q_var1; 

        auto t0 = update0.t;
        auto t1 = update1.t;

        if (tcur < t0 || tcur >= t1)
            continue;

        //transfer state1 to map projection of state0?
        if (x_tr_func)
        { 
            state1_tr = state1;
            x_tr_func(state1_tr.x, state1.x, i, i - 1);
        }
        const auto& state1_ref = x_tr_func ? state1_tr : state1;

        const double dt = Utils::Time::partialSeconds(t1 - t0);

        while (tcur >= t0 && tcur < t1)
        {
            double dt0 = Utils::Time::partialSeconds(tcur - t0);
            double dt1 = Utils::Time::partialSeconds(t1 - tcur);
            if (dt0 <= min_dt_sec)
            {
                //use first ref if timestep from first ref is very small
                addUpdate(state0, proj_center0, t0);

                ++small_intervals;
                tcur = t0 + time_incr;
                continue;
            }
            if (dt1 <= min_dt_sec)
            {
                //use second ref if timestep from second ref is very small
                addUpdate(state1, proj_center1, t1);

                ++small_intervals;
                tcur = t1 + time_incr;
                continue;
            }

            kalman::KalmanState new_state0, new_state1;
            kalman::KalmanError error0, error1;

            error0 = kalman_interface_->interpStep(new_state0, state0, state1_ref,  dt0, settings_.fix_predictions_interp, nullptr, Q_var0);
            error1 = kalman_interface_->interpStep(new_state1, state1_ref, state0, -dt1, settings_.fix_predictions_interp, nullptr, Q_var1);

            if (error0 == kalman::KalmanError::NoError &&
                error1 == kalman::KalmanError::NoError)
            {
                const double interp_factor = blendFunc(dt0, dt1, dt, new_state0, new_state1, interp_mode, *kalman_interface_);

                //std::cout << interp_factor << std::endl;

                kalman::KalmanState new_state;
                new_state.x = SplineInterpolator::interpStateVector(new_state0.x, new_state1.x, interp_factor);
                new_state.P = SplineInterpolator::interpCovarianceMat(new_state0.P, new_state1.P, interp_factor, SplineInterpolator::CovMatInterpMode::Linear);

                // Reference ref;
                // ref.t              = tcur;
                // ref.source_id      = ref0.source_id;
                // ref.noaccel_pos    = ref0.noaccel_pos || ref1.noaccel_pos;
                // ref.nospeed_pos    = ref0.nospeed_pos || ref1.nospeed_pos;
                // ref.nostddev_pos   = ref0.nostddev_pos || ref1.nostddev_pos;
                // ref.mm_interp      = ref0.mm_interp || ref1.mm_interp;
                // ref.projchange_pos = ref1.projchange_pos; //if a change in map projection happened between the two reference states, 
                //                                           //this should be logged in the second reference
                // ref.ref_interp     = true;

                addUpdate(new_state, proj_center0, tcur);
            }
            else
            {
                ++failed_steps;
            }

            tcur += time_incr;
        }
    }

    if (num_steps_failed)
        *num_steps_failed += failed_steps;

    if (settings_.verbosity >= 1 && small_intervals > 0)
        logdbg << "encountered " << small_intervals << " small interval(s) during resampling";

    if (interp_updates.size() >= 2)
    {
        interp_updates.pop_back();
        addUpdate(last_update.state, last_update.projection_center, last_update.t);
    }

    return (failed_steps == 0);
}

/**
*/
kalman::KalmanError KalmanEstimator::predictBetween(kalman::KalmanUpdateMinimal& update_interp,
                                                    const kalman::KalmanUpdateMinimal& update0,
                                                    const kalman::KalmanUpdateMinimal& update1,
                                                    const boost::posix_time::ptime& ts,
                                                    double min_dt_sec,
                                                    StateInterpMode interp_mode,
                                                    KalmanProjectionHandler& proj_handler,
                                                    size_t* num_fixed,
                                                    size_t* num_proj_changed) const
{
    traced_assert(isInit());
    traced_assert(update0.valid);
    traced_assert(update1.valid);
    traced_assert(ts >= update0.t && ts <= update1.t);

    if (num_fixed)
        *num_fixed = 0;
    if (num_proj_changed)
        *num_proj_changed = 0;
    
    double dt  = Utils::Time::partialSeconds(update1.t - update0.t);
    double dt0 = Utils::Time::partialSeconds(ts - update0.t);
    double dt1 = Utils::Time::partialSeconds(update1.t - ts);

    if (dt0 <= min_dt_sec)
    {
        update_interp = update0;
        return kalman::KalmanError::NoError;
    }
    if (dt1 <= min_dt_sec)
    {
        update_interp = update1;
        return kalman::KalmanError::NoError;
    }

    kalman::KalmanState state0;
    state0.x = update0.x;
    state0.P = update0.P;

    kalman::KalmanState state1_tr;
    state1_tr.P = update1.P;

    //reproject update1 into update0's coord system
    proj_handler.xReprojected(state1_tr.x, *kalman_interface_, update1.x, update1.projection_center, update0.projection_center, num_proj_changed);

    bool fixed0, fixed1;

    //predict forth and back to the specified time
    kalman::KalmanState new_state0, new_state1;

    auto error0 = kalman_interface_->interpStep(new_state0, state0, state1_tr,  dt0, settings_.fix_predictions, &fixed0);
    auto error1 = kalman_interface_->interpStep(new_state1, state1_tr, state0, -dt1, settings_.fix_predictions, &fixed1);

    if (error0 != kalman::KalmanError::NoError)
        return error0;

    if (error1 != kalman::KalmanError::NoError)
        return error1;

    if (num_fixed)
    {
        *num_fixed = 0;
        if (fixed0) *num_fixed += 1;
        if (fixed1) *num_fixed += 1;
    }

    //get interpolation factor
    double interp_factor = blendFunc(dt0, dt1, dt, new_state0, new_state1, interp_mode, *kalman_interface_);

    //interpolate predicted states
    update_interp.x                 = SplineInterpolator::interpStateVector(new_state0.x, new_state1.x, interp_factor);
    update_interp.P                 = SplineInterpolator::interpCovarianceMat(new_state0.P, new_state1.P, interp_factor, SplineInterpolator::CovMatInterpMode::Linear);
    update_interp.projection_center = update0.projection_center;
    update_interp.t                 = ts;
    update_interp.valid             = true;

    return kalman::KalmanError::NoError;
}

/**
*/
std::string KalmanEstimator::asString(int flags, const std::string& prefix) const
{
    if (kalman_interface_)
        return kalman_interface_->asString(flags, prefix);

    return "";
}

/**
*/
void KalmanEstimator::enableDebugging(bool ok)
{
    settings_.debug = ok;

    if (kalman_interface_)
        return kalman_interface_->enableDebugging(ok);
}

/**
*/
bool KalmanEstimator::checkKalmanStateNumerical(kalman::KalmanState& state) const
{
    traced_assert(kalman_interface_);
    return kalman_interface_->checkKalmanStateNumerical(state);
}

/**
*/
bool KalmanEstimator::validateState(const kalman::KalmanState& state) const
{
    traced_assert(kalman_interface_);
    return kalman_interface_->validateState(state);
}

} // reconstruction
