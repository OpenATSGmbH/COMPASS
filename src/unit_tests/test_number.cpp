#include "catch.hpp"
#include "number.h"
#include "rs2gcoordinatesystem.h"
#include "radarbiasinfo.h"

#include <cmath>
#include <random>

using namespace Utils::Number;

TEST_CASE("deg2rad and rad2deg", "[number][conversion]")
{
    REQUIRE(deg2rad(0.0) == Approx(0.0));
    REQUIRE(deg2rad(180.0) == Approx(M_PI));
    REQUIRE(deg2rad(90.0) == Approx(M_PI / 2.0));
    REQUIRE(deg2rad(360.0) == Approx(2.0 * M_PI));
    REQUIRE(deg2rad(-90.0) == Approx(-M_PI / 2.0));

    REQUIRE(rad2deg(0.0) == Approx(0.0));
    REQUIRE(rad2deg(M_PI) == Approx(180.0));
    REQUIRE(rad2deg(M_PI / 2.0) == Approx(90.0));

    // roundtrip
    REQUIRE(rad2deg(deg2rad(47.5)) == Approx(47.5));
    REQUIRE(deg2rad(rad2deg(1.23)) == Approx(1.23));
}

TEST_CASE("knots2MPS and mps2Knots", "[number][conversion]")
{
    REQUIRE(knots2MPS(0.0) == Approx(0.0));
    REQUIRE(knots2MPS(1.0) == Approx(0.514444));
    REQUIRE(mps2Knots(0.0) == Approx(0.0));
    REQUIRE(mps2Knots(1.0) == Approx(1.94384));

    // roundtrip (within floating point tolerance)
    REQUIRE(mps2Knots(knots2MPS(100.0)) == Approx(100.0).epsilon(1e-4));
}

TEST_CASE("round", "[number]")
{
    REQUIRE(round(3.14159, 2) == Approx(3.14));
    REQUIRE(round(3.14159, 4) == Approx(3.1416));
    REQUIRE(round(3.14159, 0) == Approx(3.0));
    REQUIRE(round(-2.555, 2) == Approx(-2.56));
    REQUIRE(round(0.0, 5) == Approx(0.0));
}

TEST_CASE("calculateAngle", "[number]")
{
    // 47 degrees, 26 minutes, 58.9526 seconds
    REQUIRE(calculateAngle(47.0, 26.0, 58.9526) == Approx(47.44971).epsilon(1e-4));
    REQUIRE(calculateAngle(0.0, 0.0, 0.0) == Approx(0.0));
    REQUIRE(calculateAngle(90.0, 0.0, 0.0) == Approx(90.0));
    REQUIRE(calculateAngle(0.0, 30.0, 0.0) == Approx(0.5));
    REQUIRE(calculateAngle(0.0, 0.0, 3600.0) == Approx(1.0));
}

TEST_CASE("dsId encoding/decoding", "[number]")
{
    unsigned int sac = 12;
    unsigned int sic = 34;
    unsigned int ds_id = dsIdFrom(sac, sic);

    REQUIRE(sacFromDsId(ds_id) == sac);
    REQUIRE(sicFromDsId(ds_id) == sic);

    // boundary: sac=0, sic=0
    REQUIRE(dsIdFrom(0, 0) == 0);
    REQUIRE(sacFromDsId(0) == 0);
    REQUIRE(sicFromDsId(0) == 0);

    // boundary: sac=1, sic=254
    ds_id = dsIdFrom(1, 254);
    REQUIRE(sacFromDsId(ds_id) == 1);
    REQUIRE(sicFromDsId(ds_id) == 254);
}

TEST_CASE("bearing2Vec and vec2Bearing", "[number]")
{
    // North (0 deg) -> dx=0, dy=1
    auto [dx, dy] = bearing2Vec(0.0);
    REQUIRE(dx == Approx(0.0).margin(1e-10));
    REQUIRE(dy == Approx(1.0));

    // East (90 deg) -> dx=1, dy=0
    std::tie(dx, dy) = bearing2Vec(90.0);
    REQUIRE(dx == Approx(1.0));
    REQUIRE(dy == Approx(0.0).margin(1e-10));

    // roundtrip
    REQUIRE(vec2Bearing(0.0, 1.0) == Approx(0.0));
    REQUIRE(vec2Bearing(1.0, 0.0) == Approx(90.0));
}

TEST_CASE("calculateMinAngleDifference", "[number]")
{
    REQUIRE(calculateMinAngleDifference(0.0, 0.0) == Approx(0.0));
    REQUIRE(calculateMinAngleDifference(10.0, 20.0) == Approx(10.0));
    REQUIRE(calculateMinAngleDifference(350.0, 10.0) == Approx(20.0));
    REQUIRE(calculateMinAngleDifference(10.0, 350.0) == Approx(-20.0));
    REQUIRE(calculateMinAngleDifference(0.0, 180.0) == Approx(-180.0));
}

TEST_CASE("recNum encoding/decoding", "[number]")
{
    unsigned long rec_num = 12345;
    unsigned int dbcont_id = 42;

    unsigned long encoded = recNumAddDBContId(rec_num, dbcont_id);
    REQUIRE(recNumGetWithoutDBContId(encoded) == rec_num);
    REQUIRE(recNumGetDBContId(encoded) == dbcont_id);
}

TEST_CASE("convertLatitude", "[number][conversion]")
{
    bool ok = false;

    double lat = convertLatitude("47:26:58.9526N", ok);
    REQUIRE(ok);
    REQUIRE(lat == Approx(47.44971).epsilon(1e-4));

    lat = convertLatitude("47:26:58.9526S", ok);
    REQUIRE(ok);
    REQUIRE(lat == Approx(-47.44971).epsilon(1e-4));

    // no hemisphere suffix defaults to N
    lat = convertLatitude("47:26:58.9526", ok);
    REQUIRE(ok);
    REQUIRE(lat == Approx(47.44971).epsilon(1e-4));

    // empty string
    convertLatitude("", ok);
    REQUIRE_FALSE(ok);
}

TEST_CASE("convertLongitude", "[number][conversion]")
{
    bool ok = false;

    double lon = convertLongitude("008:34:25.5397E", ok);
    REQUIRE(ok);
    REQUIRE(lon == Approx(8.57376).epsilon(1e-4));

    lon = convertLongitude("008:34:25.5397W", ok);
    REQUIRE(ok);
    REQUIRE(lon == Approx(-8.57376).epsilon(1e-4));
}

// --- Azimuth Bias Estimation ---

TEST_CASE("estimateAzimuthBias zero bias", "[number][bias]")
{
    std::vector<double> ref = {10.0, 50.0, 120.0, 200.0, 300.0};
    std::vector<double> tst = {10.0, 50.0, 120.0, 200.0, 300.0};

    auto result = estimateAzimuthBias(ref, tst);
    REQUIRE(result.valid);
    REQUIRE(result.azimuth_bias_deg == Approx(0.0).margin(1e-10));
}

TEST_CASE("estimateAzimuthBias known bias", "[number][bias]")
{
    // bias = tst - ref = +2.0 deg
    std::vector<double> ref = {10.0, 50.0, 120.0, 200.0, 300.0};
    std::vector<double> tst = {12.0, 52.0, 122.0, 202.0, 302.0};

    auto result = estimateAzimuthBias(ref, tst);
    REQUIRE(result.valid);
    REQUIRE(result.azimuth_bias_deg == Approx(2.0).epsilon(1e-6));
}

TEST_CASE("estimateAzimuthBias wraparound", "[number][bias]")
{
    // bias of +1.5 deg near 0/360 boundary
    std::vector<double> ref = {359.0, 0.5, 1.0, 358.0};
    std::vector<double> tst = {0.5,   2.0, 2.5, 359.5};

    auto result = estimateAzimuthBias(ref, tst);
    REQUIRE(result.valid);
    REQUIRE(result.azimuth_bias_deg == Approx(1.5).epsilon(1e-6));
}

TEST_CASE("estimateAzimuthBias outlier filtering", "[number][bias]")
{
    // 4 good pairs with +1.0 bias, 1 outlier with 50 deg diff
    std::vector<double> ref = {10.0, 50.0, 120.0, 200.0, 300.0};
    std::vector<double> tst = {11.0, 51.0, 121.0, 201.0, 350.0}; // last is outlier

    auto result = estimateAzimuthBias(ref, tst, 30.0);
    REQUIRE(result.valid);
    REQUIRE(result.azimuth_bias_deg == Approx(1.0).epsilon(1e-6));
}

TEST_CASE("estimateAzimuthBias too high", "[number][bias]")
{
    // all diffs are ~35 deg, exceeding max_bias_deg=30
    std::vector<double> ref = {10.0, 50.0, 120.0};
    std::vector<double> tst = {45.0, 85.0, 155.0};

    auto result = estimateAzimuthBias(ref, tst, 40.0, 30.0);
    REQUIRE_FALSE(result.valid);
}

TEST_CASE("estimateAzimuthBias empty input", "[number][bias]")
{
    auto result = estimateAzimuthBias({}, {});
    REQUIRE_FALSE(result.valid);
}

// --- Range Bias/Gain Estimation ---

TEST_CASE("estimateRangeBiasGain zero bias", "[number][bias]")
{
    std::vector<double> ranges = {10000.0, 20000.0, 30000.0, 50000.0, 80000.0};

    auto result = estimateRangeBiasGain(ranges, ranges);
    REQUIRE(result.valid);
    REQUIRE(result.range_bias_m == Approx(0.0).margin(1.0));
    REQUIRE(result.range_gain == Approx(0.0).margin(1e-6));
}

TEST_CASE("estimateRangeBiasGain known bias and gain", "[number][bias]")
{
    // tst = ref * (1 + gain) + bias
    // with gain = 0.01, bias = 50.0
    double gain = 0.01;
    double bias = 50.0;

    std::vector<double> ref_ranges = {10000.0, 20000.0, 30000.0, 50000.0, 80000.0};
    std::vector<double> tst_ranges;

    for (double r : ref_ranges)
        tst_ranges.push_back(r * (1.0 + gain) + bias);

    auto result = estimateRangeBiasGain(tst_ranges, ref_ranges);
    REQUIRE(result.valid);
    REQUIRE(result.range_bias_m == Approx(bias).epsilon(1e-4));
    REQUIRE(result.range_gain == Approx(gain).epsilon(1e-6));
}

TEST_CASE("estimateRangeBiasGain outlier filtering", "[number][bias]")
{
    // 4 good pairs (no bias), 1 outlier with 2x range ratio
    std::vector<double> ref = {10000.0, 20000.0, 30000.0, 50000.0, 80000.0};
    std::vector<double> tst = {10000.0, 20000.0, 30000.0, 50000.0, 160000.0}; // last is 2x

    auto result = estimateRangeBiasGain(tst, ref, 0.1);
    REQUIRE(result.valid);
    REQUIRE(result.range_bias_m == Approx(0.0).margin(1.0));
    REQUIRE(result.range_gain == Approx(0.0).margin(1e-6));
}

TEST_CASE("estimateRangeBiasGain empty input", "[number][bias]")
{
    auto result = estimateRangeBiasGain({}, {});
    REQUIRE_FALSE(result.valid);
}

TEST_CASE("estimateRangeBiasGain single point", "[number][bias]")
{
    // need at least 2 points for linear regression
    auto result = estimateRangeBiasGain({10000.0}, {10000.0});
    REQUIRE_FALSE(result.valid);
}

// --- End-to-end: bias estimation + position correction ---

// Haversine great-circle distance in metres
static double haversineDistance(double lat1_deg, double lon1_deg,
                                double lat2_deg, double lon2_deg)
{
    const double R = 6371000.0;
    double lat1 = lat1_deg * M_PI / 180.0;
    double lat2 = lat2_deg * M_PI / 180.0;
    double dlat = (lat2_deg - lat1_deg) * M_PI / 180.0;
    double dlon = (lon2_deg - lon1_deg) * M_PI / 180.0;

    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0)
             + std::cos(lat1) * std::cos(lat2)
             * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return R * c;
}

TEST_CASE("radar bias estimation and correction end-to-end", "[number][bias][e2e]")
{
    const unsigned int num_iterations = 20;
    std::mt19937 rng(42); // fixed seed for reproducibility

    // sensible parameter distributions
    std::uniform_real_distribution<double> lat_dist(-60.0, 70.0);
    std::uniform_real_distribution<double> lon_dist(-170.0, 170.0);
    std::uniform_real_distribution<double> alt_dist(0.0, 800.0);
    std::uniform_real_distribution<double> az_bias_dist(-2.0, 2.0);
    std::uniform_real_distribution<double> rng_bias_dist(-400.0, 400.0);
    std::uniform_real_distribution<double> rng_gain_dist(-0.02, 0.02);

    for (unsigned int iter = 0; iter < num_iterations; ++iter)
    {
        double radar_lat = lat_dist(rng);
        double radar_lon = lon_dist(rng);
        double radar_alt = alt_dist(rng);

        double true_az_bias_deg  = az_bias_dist(rng);
        double true_range_bias_m = rng_bias_dist(rng);
        double true_range_gain   = rng_gain_dist(rng);

        INFO("iteration " << iter
             << " radar=(" << radar_lat << ", " << radar_lon << ", " << radar_alt << ")"
             << " az_bias=" << true_az_bias_deg
             << " rng_bias=" << true_range_bias_m
             << " rng_gain=" << true_range_gain);

        RS2GCoordinateSystem cs(iter + 1, radar_lat, radar_lon, radar_alt);

        // ground-only targets: altitude = radar altitude, so slant_range ≈ ground_range
        const double target_alt = radar_alt;

        struct Position { double lat_deg; double lon_deg; };

        std::vector<double> ref_azms_deg, tst_azms_deg;
        std::vector<double> ref_ground_ranges, tst_ground_ranges;
        std::vector<Position> ref_positions, tst_positions;

        // generate targets at 24 azimuths x 8 ranges = 192 points
        for (double az_deg = 5.0; az_deg < 360.0; az_deg += 15.0)
        {
            for (double slant_range = 10000.0; slant_range <= 80000.0; slant_range += 10000.0)
            {
                double az_rad = az_deg * M_PI / 180.0;

                // forward project true polar -> reference WGS-84 position
                double gr_ref, ecef_x, ecef_y, ecef_z;
                bool ok = cs.calculateRadSlt2Geocentric(
                    az_rad, slant_range, true, target_alt,
                    gr_ref, ecef_x, ecef_y, ecef_z);
                REQUIRE(ok);

                double ref_lat, ref_lon, ref_h;
                ok = cs.geocentric2Geodesic(ecef_x, ecef_y, ecef_z, ref_lat, ref_lon, ref_h);
                REQUIRE(ok);
                ref_positions.push_back({ref_lat, ref_lon});

                // convert ref position back to polar to get ref azimuth and ground range
                double lx, ly, lz;
                cs.geodesic2LocalCart(ref_lat * M_PI / 180.0, ref_lon * M_PI / 180.0,
                                      ref_h, lx, ly, lz);

                double ref_az_rad, ref_sr, ref_gr, ref_alt_out;
                cs.localCart2RadarSlant(lx, ly, lz, ref_az_rad, ref_sr, ref_gr, ref_alt_out);

                double ref_az_d = ref_az_rad * 180.0 / M_PI;
                if (ref_az_d < 0) ref_az_d += 360.0;

                ref_azms_deg.push_back(ref_az_d);
                ref_ground_ranges.push_back(ref_gr);

                // apply bias to create simulated "measured" radar data
                double tst_az_d = ref_az_d + true_az_bias_deg;
                double tst_gr   = ref_gr * (1.0 + true_range_gain) + true_range_bias_m;

                tst_azms_deg.push_back(tst_az_d);
                tst_ground_ranges.push_back(tst_gr);

                // convert biased measurement to WGS-84 "test" position
                double tst_az_rad = tst_az_d * M_PI / 180.0;
                double tst_lx = tst_gr * std::sin(tst_az_rad);
                double tst_ly = tst_gr * std::cos(tst_az_rad);
                double tst_lz = 0.0;

                double tst_ex, tst_ey, tst_ez;
                cs.localCart2Geocentric(tst_lx, tst_ly, tst_lz, tst_ex, tst_ey, tst_ez);

                double tst_lat, tst_lon, tst_h;
                ok = cs.geocentric2Geodesic(tst_ex, tst_ey, tst_ez, tst_lat, tst_lon, tst_h);
                REQUIRE(ok);
                tst_positions.push_back({tst_lat, tst_lon});
            }
        }

        size_t n = ref_positions.size();
        REQUIRE(n == 192);

        // --- Step 1: Estimate biases from ref/tst polar data ---

        auto az_result = estimateAzimuthBias(ref_azms_deg, tst_azms_deg);
        REQUIRE(az_result.valid);
        REQUIRE(az_result.azimuth_bias_deg == Approx(true_az_bias_deg).epsilon(0.01));

        auto rg_result = estimateRangeBiasGain(tst_ground_ranges, ref_ground_ranges);
        REQUIRE(rg_result.valid);
        REQUIRE(rg_result.range_bias_m == Approx(true_range_bias_m).epsilon(0.01));
        REQUIRE(rg_result.range_gain   == Approx(true_range_gain).epsilon(0.01));

        // --- Step 2: Apply bias correction to test measurements ---

        RadarBiasInfo estimated_bias;
        estimated_bias.azimuth_bias_valid_ = true;
        estimated_bias.azimuth_bias_deg_   = az_result.azimuth_bias_deg;
        estimated_bias.range_bias_valid_   = true;
        estimated_bias.range_bias_m_       = rg_result.range_bias_m;
        estimated_bias.range_gain_         = rg_result.range_gain;

        std::vector<Position> cor_positions;

        for (size_t i = 0; i < n; ++i)
        {
            double tst_az_rad = tst_azms_deg[i] * M_PI / 180.0;

            double ecef_x, ecef_y, ecef_z;
            cs.calculateRadSlt2Geocentric(
                tst_az_rad, tst_ground_ranges[i],
                true, target_alt, estimated_bias,
                ecef_x, ecef_y, ecef_z);

            double cor_lat, cor_lon, cor_h;
            cs.geocentric2Geodesic(ecef_x, ecef_y, ecef_z, cor_lat, cor_lon, cor_h);
            cor_positions.push_back({cor_lat, cor_lon});
        }

        // --- Step 3: Compute errors ---

        std::vector<double> org_errors, cor_errors;

        for (size_t i = 0; i < n; ++i)
        {
            org_errors.push_back(haversineDistance(
                tst_positions[i].lat_deg, tst_positions[i].lon_deg,
                ref_positions[i].lat_deg, ref_positions[i].lon_deg));

            cor_errors.push_back(haversineDistance(
                cor_positions[i].lat_deg, cor_positions[i].lon_deg,
                ref_positions[i].lat_deg, ref_positions[i].lon_deg));
        }

        auto org_stats = getMedianStatistics(org_errors);
        auto cor_stats = getMedianStatistics(cor_errors);

        double org_median = std::get<0>(org_stats);
        double cor_median = std::get<0>(cor_stats);

        INFO("org median=" << org_median << " cor median=" << cor_median);

        // original error must be significant (bias creates real displacement)
        REQUIRE(org_median > 10.0);

        // corrected error must be much smaller than original
        REQUIRE(cor_median < org_median);
        REQUIRE(cor_median < 5.0);
    }
}
