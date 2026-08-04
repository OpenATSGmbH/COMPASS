#include "catch.hpp"

#include "spline_interpolator.h"
#include "test_reconstruction_common.h"

#include <cmath>

using namespace reconstruction;
using namespace test_reconstruction;

namespace
{

Measurement makeMeasurement(double lat, double lon,
                            const boost::posix_time::ptime& t)
{
    Measurement mm;
    mm.t   = t;
    mm.lat = lat;
    mm.lon = lon;
    return mm;
}

} // namespace

TEST_CASE("spline interpolator resamples a straight line", "[reconstruction][spline]")
{
    // measurements 4 s apart along a straight east-bound line, resampled to 1 s
    const double base_lat = 47.0;
    const double base_lon = 16.0;
    const double v_ms     = 50.0;
    const double dt_in    = 4.0;
    const size_t n_in     = 11;

    boost::posix_time::ptime t0(boost::gregorian::date(2026, 6, 9),
                                boost::posix_time::time_duration(12, 0, 0));

    std::vector<Measurement> mms;
    for (size_t i = 0; i < n_in; ++i)
        mms.push_back(makeMeasurement(
            base_lat,
            base_lon + (v_ms * i * dt_in) / metersPerDegLon(base_lat),
            t0 + boost::posix_time::seconds((long)(i * dt_in))));

    SplineInterpolator interp;
    interp.config().sample_dt = 1.0;

    auto samples = interp.interpolate(mms);

    // 40 s span at 1 s sampling
    REQUIRE(samples.size() >= 40);
    REQUIRE(samples.size() <= 42);

    // endpoints preserved
    CHECK(distanceM(samples.front().lat, samples.front().lon,
                    mms.front().lat, mms.front().lon) < 0.01);
    CHECK(distanceM(samples.back().lat, samples.back().lon,
                    mms.back().lat, mms.back().lon) < 0.5);

    // samples stay on the line (constant latitude), longitudes and
    // timestamps monotonic (the first input sample may appear twice)
    for (size_t i = 0; i < samples.size(); ++i)
    {
        CHECK(std::fabs(samples[i].lat - base_lat) * MetersPerDegLat < 0.5);

        if (i > 0)
        {
            CHECK(samples[i].lon >= samples[i - 1].lon);
            CHECK(samples[i].t >= samples[i - 1].t);
        }
    }

    // full time span covered
    CHECK((samples.back().t - samples.front().t).total_seconds() == 40);
}

TEST_CASE("spline interpolator handles a curved trajectory", "[reconstruction][spline]")
{
    // quarter circle of radius 5 km flown in 90 s, input every 5 s
    const double base_lat = 47.0;
    const double base_lon = 16.0;
    const double radius_m = 5000.0;
    const double dt_in    = 5.0;
    const size_t n_in     = 19;

    boost::posix_time::ptime t0(boost::gregorian::date(2026, 6, 9),
                                boost::posix_time::time_duration(12, 0, 0));

    std::vector<Measurement> mms;
    for (size_t i = 0; i < n_in; ++i)
    {
        double angle = (M_PI / 2.0) * (double)i / (double)(n_in - 1);
        double x = radius_m * std::cos(angle);
        double y = radius_m * std::sin(angle);

        mms.push_back(makeMeasurement(base_lat + y / MetersPerDegLat,
                                      base_lon + x / metersPerDegLon(base_lat),
                                      t0 + boost::posix_time::seconds((long)(i * dt_in))));
    }

    SplineInterpolator interp;
    interp.config().sample_dt = 1.0;

    auto samples = interp.interpolate(mms);

    REQUIRE(samples.size() >= 90);

    // interpolated points stay close to the circle
    for (const auto& s : samples)
    {
        double r = distanceM(base_lat, base_lon, base_lat, s.lon);
        double y = (s.lat - base_lat) * MetersPerDegLat;
        double x = (s.lon - base_lon) * metersPerDegLon(base_lat);
        r = std::sqrt(x * x + y * y);

        CHECK(std::fabs(r - radius_m) < 25.0);
    }
}

TEST_CASE("spline interpolator splits on time gaps", "[reconstruction][spline]")
{
    boost::posix_time::ptime t0(boost::gregorian::date(2026, 6, 9),
                                boost::posix_time::time_duration(12, 0, 0));

    std::vector<Measurement> mms;

    // two groups of measurements separated by a 60 s gap
    for (size_t i = 0; i < 5; ++i)
        mms.push_back(makeMeasurement(47.0, 16.0 + i * 0.001,
                                      t0 + boost::posix_time::seconds((long)(i * 4))));
    for (size_t i = 0; i < 5; ++i)
        mms.push_back(makeMeasurement(47.0, 16.1 + i * 0.001,
                                      t0 + boost::posix_time::seconds((long)(80 + i * 4))));

    auto parts = SplineInterpolator::splitMeasurements(mms, 30.0);

    REQUIRE(parts.size() == 2);
    CHECK(parts[0].size() == 5);
    CHECK(parts[1].size() == 5);
}

TEST_CASE("spline state vector and covariance blending", "[reconstruction][spline]")
{
    Eigen::VectorXd x0(2), x1(2);
    x0 << 0.0, 10.0;
    x1 << 100.0, 20.0;

    auto x_mid = SplineInterpolator::interpStateVector(x0, x1, 0.5);
    CHECK(x_mid(0) == Approx(50.0));
    CHECK(x_mid(1) == Approx(15.0));

    CHECK(SplineInterpolator::interpStateVector(x0, x1, 0.0)(0) == Approx(0.0));
    CHECK(SplineInterpolator::interpStateVector(x0, x1, 1.0)(0) == Approx(100.0));

    Eigen::MatrixXd C0(2, 2), C1(2, 2);
    C0 << 4.0, 0.0, 0.0, 4.0;
    C1 << 16.0, 0.0, 0.0, 16.0;

    bool ok = false;
    auto C_mid = SplineInterpolator::interpCovarianceMat(
        C0, C1, 0.5, SplineInterpolator::CovMatInterpMode::Linear, &ok);

    CHECK(ok);
    CHECK(C_mid(0, 0) == Approx(10.0));
    CHECK(C_mid(1, 1) == Approx(10.0));
}
