#include "catch.hpp"

#include "kalman_estimator.h"
#include "test_reconstruction_common.h"

#include <cmath>

using namespace reconstruction;
using namespace test_reconstruction;

namespace
{

double rmsError2D(const std::vector<Reference>& refs, const SyntheticTrack& track)
{
    double sum = 0.0;
    size_t n   = 0;

    for (const auto& ref : refs)
    {
        double t_s = (ref.t - track.t0).total_milliseconds() / 1000.0;

        double lat_true = track.base_lat + (track.vy_ms * t_s) / MetersPerDegLat;
        double lon_true = track.base_lon + (track.vx_ms * t_s) / metersPerDegLon(track.base_lat);

        double d = distanceM(lat_true, lon_true, ref.lat, ref.lon);
        sum += d * d;
        ++n;
    }

    REQUIRE(n > 0);
    return std::sqrt(sum / n);
}

double rmsErrorMeasurements(const std::vector<Measurement>& mms, const SyntheticTrack& track)
{
    double sum = 0.0;

    for (size_t i = 0; i < mms.size(); ++i)
    {
        double lat_true, lon_true;
        track.truePosition(i, lat_true, lon_true);

        double d = distanceM(lat_true, lon_true, mms[i].lat, mms[i].lon);
        sum += d * d;
    }

    return std::sqrt(sum / mms.size());
}

} // namespace

TEST_CASE("kalman estimator filters and smooths a noisy track", "[reconstruction][kalman_estimator]")
{
    SyntheticTrack track;
    track.sigma_m = 50.0;

    const size_t n = 90;
    auto mms = track.generate(n);

    KalmanEstimator estimator;
    estimator.init(kalman::KalmanType::UMKalman2D);

    kalman::KalmanUpdate update;
    estimator.kalmanInit(update, mms[0]);
    REQUIRE(update.valid);

    std::vector<kalman::KalmanUpdate> updates;
    updates.push_back(update);

    for (size_t i = 1; i < n; ++i)
    {
        auto result = estimator.kalmanStep(update, mms[i]);
        REQUIRE(result == KalmanEstimator::StepResult::Success);
        REQUIRE(update.valid);
        CHECK_FALSE(update.reinit);

        updates.push_back(update);
    }

    double rms_measurements = rmsErrorMeasurements(mms, track);

    // forward filtering reduces error vs raw measurements
    std::vector<Reference> refs_filtered;
    estimator.storeUpdates(refs_filtered, updates);
    REQUIRE(refs_filtered.size() == n);

    double rms_filtered = rmsError2D(refs_filtered, track);
    CHECK(rms_filtered < rms_measurements);

    // RTS smoothing further improves (or at least does not degrade)
    auto updates_smoothed = updates;
    REQUIRE(estimator.smoothUpdates(updates_smoothed, kalman::SmoothFailStrategy::SetInvalid));

    std::vector<Reference> refs_smoothed;
    estimator.storeUpdates(refs_smoothed, updates_smoothed);
    REQUIRE(refs_smoothed.size() == n);

    double rms_smoothed = rmsError2D(refs_smoothed, track);
    CHECK(rms_smoothed < rms_filtered * 1.05);
    CHECK(rms_smoothed < rms_measurements);
}

TEST_CASE("kalman estimator resamples at fixed intervals", "[reconstruction][kalman_estimator]")
{
    SyntheticTrack track;
    track.dt_s = 4.0; // measurements every 4 s

    const size_t n = 20;
    auto mms = track.generate(n);

    KalmanEstimator estimator;
    estimator.settings().resample_dt = 2.0;
    estimator.init(kalman::KalmanType::UMKalman2D);

    kalman::KalmanUpdate update;
    estimator.kalmanInit(update, mms[0]);

    std::vector<kalman::KalmanUpdate> updates;
    updates.push_back(update);

    for (size_t i = 1; i < n; ++i)
    {
        REQUIRE(estimator.kalmanStep(update, mms[i]) == KalmanEstimator::StepResult::Success);
        updates.push_back(update);
    }

    std::vector<kalman::KalmanUpdate> interp_updates;
    REQUIRE(estimator.interpUpdates(interp_updates, updates));
    REQUIRE(!interp_updates.empty());

    // resampled to 2 s: 76 s span -> ~39 updates
    CHECK(interp_updates.size() >= updates.size());

    // most intervals at the resample step size; segment boundaries may
    // fall back to the original 4 s measurement spacing
    size_t num_at_resample_dt = 0;

    for (size_t i = 1; i < interp_updates.size(); ++i)
    {
        double dt = (interp_updates[i].t - interp_updates[i - 1].t).total_milliseconds() / 1000.0;
        CHECK(dt > 0.0);
        CHECK(dt <= 4.1);

        if (dt <= 2.1)
            ++num_at_resample_dt;
    }

    CHECK(num_at_resample_dt >= (interp_updates.size() - 1) * 6 / 10);
}

TEST_CASE("kalman estimator reinitializes on large time gaps", "[reconstruction][kalman_estimator]")
{
    SyntheticTrack track;
    auto mms = track.generate(10);

    KalmanEstimator estimator;
    estimator.init(kalman::KalmanType::UMKalman2D); // default max_dt 11 s

    kalman::KalmanUpdate update;
    estimator.kalmanInit(update, mms[0]);

    for (size_t i = 1; i < mms.size(); ++i)
        REQUIRE(estimator.kalmanStep(update, mms[i]) == KalmanEstimator::StepResult::Success);

    // measurement far in the future -> reinit instead of step
    Measurement mm_late = mms.back();
    mm_late.t += boost::posix_time::seconds(60);

    auto result = estimator.kalmanStep(update, mm_late);
    REQUIRE(result == KalmanEstimator::StepResult::Success);
    CHECK(update.reinit);
}
