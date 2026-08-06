#pragma once

#include "measurement.h"

#include <cmath>
#include <random>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace test_reconstruction
{

// flat-earth conversion helpers, accurate enough for the small synthetic
// test areas used here (< 30 km around the base position)

constexpr double MetersPerDegLat = 111320.0;

inline double metersPerDegLon(double lat_deg)
{
    return MetersPerDegLat * std::cos(lat_deg * M_PI / 180.0);
}

inline double distanceM(double lat0, double lon0, double lat1, double lon1)
{
    double dy = (lat1 - lat0) * MetersPerDegLat;
    double dx = (lon1 - lon0) * metersPerDegLon(lat0);
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * Generates a constant-velocity WGS-84 track with gaussian position noise.
 * Deterministic for a given seed.
 */
struct SyntheticTrack
{
    double base_lat  = 47.0;
    double base_lon  = 16.0;
    double vx_ms     = 150.0; // east
    double vy_ms     = 0.0;   // north
    double dt_s      = 1.0;
    double sigma_m   = 50.0;
    unsigned seed    = 4711;

    boost::posix_time::ptime t0 { boost::gregorian::date(2026, 6, 9),
                                  boost::posix_time::time_duration(12, 0, 0) };

    void truePosition(size_t idx, double& lat, double& lon) const
    {
        double t = idx * dt_s;
        lat = base_lat + (vy_ms * t) / MetersPerDegLat;
        lon = base_lon + (vx_ms * t) / metersPerDegLon(base_lat);
    }

    std::vector<reconstruction::Measurement> generate(size_t count) const
    {
        std::mt19937 rng(seed);
        std::normal_distribution<double> noise(0.0, sigma_m);

        std::vector<reconstruction::Measurement> mms;
        mms.reserve(count);

        for (size_t i = 0; i < count; ++i)
        {
            double lat, lon;
            truePosition(i, lat, lon);

            reconstruction::Measurement mm;
            mm.source_id = i;
            mm.t         = t0 + boost::posix_time::milliseconds((long)(i * dt_s * 1000.0));
            mm.lat       = lat + noise(rng) / MetersPerDegLat;
            mm.lon       = lon + noise(rng) / metersPerDegLon(base_lat);
            mm.x_stddev  = sigma_m;
            mm.y_stddev  = sigma_m;

            mms.push_back(mm);
        }

        return mms;
    }
};

} // namespace test_reconstruction
