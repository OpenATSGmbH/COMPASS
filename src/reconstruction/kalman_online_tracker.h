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
#include "kalman_estimator.h"

#include <memory>

namespace reconstruction
{

class KalmanInterface;
class KalmanProjectionHandler;

/**
 * Online kalman tracker 
 * - consumes time-ordered measurements
 * - re-estimates the state on adding a new measurement
 * - able to make predictions based on the current (last) state
 * - can be reset to start a new track
*/
class KalmanOnlineTracker
{
public:
    struct Settings
    {
        double prediction_max_tdiff_sec = 10.0;
    };

    KalmanOnlineTracker();
    virtual ~KalmanOnlineTracker();

    void reset();

    bool isInit() const;
    void init(std::unique_ptr<KalmanInterface>&& interface);
    void init(kalman::KalmanType ktype);

    bool track(const Measurement& mm); 
    bool track(const kalman::KalmanUpdate& update);
    bool track(const kalman::KalmanUpdateMinimal& update);

    double timeDiffSec(const boost::posix_time::ptime& ts) const;

    bool canPredict(const boost::posix_time::ptime& ts) const;
    bool canPredict(const boost::posix_time::ptime& ts,
                    double max_tdiff_sec) const;
    kalman::KalmanError predict(Measurement* mm,
                                kalman::GeoProbState* gp_state,
                                kalman::GeoProbState* gp_state_mm,
                                const boost::posix_time::ptime& ts,
                                bool* fixed = nullptr) const;

    bool isTracking() const;

    Settings& settings() { return settings_; }
    KalmanEstimator::Settings& estimatorSettings();

    const boost::optional<kalman::KalmanUpdate>& currentState() const;
    boost::optional<reconstruction::Measurement> currentMeasurement() const;
    const boost::posix_time::ptime& currentTime() const;
    const KalmanEstimator::StepInfo& stepInfo() const;
    const KalmanEstimator& estimator() const;

private:
    void kalmanInit(const Measurement& mm);
    void kalmanInit(const kalman::KalmanUpdate& update);
    void kalmanInit(const kalman::KalmanUpdateMinimal& update);

    std::unique_ptr<KalmanEstimator>      estimator_;
    boost::optional<kalman::KalmanUpdate> current_update_;
    kalman::KalmanUpdate                  tmp_update_;

    Settings settings_;
};

} // reconstruction
