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

#include "kalman_online_tracker.h"

#include "kalman_interface.h"

namespace reconstruction
{

/**
*/
KalmanOnlineTracker::KalmanOnlineTracker()
:   estimator_(new KalmanEstimator)
{
}

/**
*/
KalmanOnlineTracker::~KalmanOnlineTracker() = default;

/**
*/
void KalmanOnlineTracker::reset()
{
    current_update_.reset();
}

/**
*/
bool KalmanOnlineTracker::isInit() const
{
    return estimator_->isInit();
}

/**
*/
void KalmanOnlineTracker::init(std::unique_ptr<KalmanInterface>&& interface)
{
    traced_assert(interface);
    estimator_->init(std::move(interface));
}

/**
*/
void KalmanOnlineTracker::init(kalman::KalmanType ktype)
{
    estimator_->init(ktype);
}

/**
*/
bool KalmanOnlineTracker::track(const Measurement& mm)
{
    traced_assert(isInit());

    if (!isTracking())
    {
        kalmanInit(mm);
    }
    else
    {
        if (estimator_->kalmanStep(tmp_update_, mm) == KalmanEstimator::StepResult::Success)
            current_update_ = tmp_update_;
    }

    traced_assert(current_update_.has_value() && current_update_->t == estimator_->currentTime());

    return (estimator_->stepInfo().result == KalmanEstimator::StepResult::Success);
}

/**
*/
bool KalmanOnlineTracker::track(const kalman::KalmanUpdate& update)
{
    traced_assert(isInit());

    kalmanInit(update);

    traced_assert(current_update_.has_value() && current_update_->t == estimator_->currentTime());

    return (estimator_->stepInfo().result == KalmanEstimator::StepResult::Success);
}

/**
*/
bool KalmanOnlineTracker::track(const kalman::KalmanUpdateMinimal& update)
{
    traced_assert(isInit());

    kalmanInit(update);

    traced_assert(current_update_.has_value() && current_update_->t == estimator_->currentTime());

    return (estimator_->stepInfo().result == KalmanEstimator::StepResult::Success);
}

/**
 */
double KalmanOnlineTracker::timeDiffSec(const boost::posix_time::ptime& ts) const
{
    if (!isTracking())
        return std::numeric_limits<double>::max();

    const auto& ts_cur = estimator_->currentTime();

    return KalmanInterface::timestep(ts_cur, ts);
}

/**
*/
bool KalmanOnlineTracker::canPredict(const boost::posix_time::ptime& ts) const
{
    return canPredict(ts, settings_.prediction_max_tdiff_sec);
}

/**
*/
bool KalmanOnlineTracker::canPredict(const boost::posix_time::ptime& ts,
                                     double max_tdiff_sec) const
{
    if (!isTracking())
        return false;

    bool allow_backwards = estimator_->settings().allow_backwards_step;
    
    double dt = timeDiffSec(ts);
    if (!allow_backwards && dt < 0)
        return false;

    if (std::fabs(dt) > max_tdiff_sec)
        return false;

    return true;
}

/**
*/
kalman::KalmanError KalmanOnlineTracker::predict(Measurement* mm,
                                                 kalman::GeoProbState* gp_state,
                                                 kalman::GeoProbState* gp_state_mm,
                                                 const boost::posix_time::ptime& ts,
                                                 bool* fixed) const
{
    traced_assert(isInit());
    traced_assert(isTracking());

    return estimator_->kalmanPrediction(mm, gp_state, gp_state_mm, ts, fixed);
}

/**
*/
void KalmanOnlineTracker::kalmanInit(const Measurement& mm)
{
    traced_assert(isInit());

    if (!current_update_.has_value())
        current_update_ = kalman::KalmanUpdate();

    estimator_->kalmanInit(current_update_.value(), mm);
}

/**
*/
void KalmanOnlineTracker::kalmanInit(const kalman::KalmanUpdate& update)
{
    traced_assert(isInit());

    current_update_ = update;

    estimator_->kalmanInit(current_update_.value());
}

/**
*/
void KalmanOnlineTracker::kalmanInit(const kalman::KalmanUpdateMinimal& update)
{
    traced_assert(isInit());

    current_update_ = kalman::KalmanUpdate(update);

    estimator_->kalmanInit(current_update_.value());
}

/**
*/
bool KalmanOnlineTracker::isTracking() const
{
    return current_update_.has_value();
}

/**
*/
KalmanEstimator::Settings& KalmanOnlineTracker::estimatorSettings()
{
    return estimator_->settings();
}

/**
*/
const boost::optional<kalman::KalmanUpdate>& KalmanOnlineTracker::currentState() const
{
    return current_update_;
}

/**
*/
boost::optional<reconstruction::Measurement> KalmanOnlineTracker::currentMeasurement() const
{
    if (!current_update_.has_value())
        return {};

    traced_assert(isInit());

    reconstruction::Measurement mm;
    estimator_->storeUpdate(mm, current_update_.value());

    return mm;
}

/**
*/
const boost::posix_time::ptime& KalmanOnlineTracker::currentTime() const
{
    return estimator_->currentTime();
}

/**
*/
const KalmanEstimator::StepInfo& KalmanOnlineTracker::stepInfo() const
{
    return estimator_->stepInfo();
}

/**
*/
const KalmanEstimator& KalmanOnlineTracker::estimator() const
{
    return *estimator_;
}

} // reconstruction
