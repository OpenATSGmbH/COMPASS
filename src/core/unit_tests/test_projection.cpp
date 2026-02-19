#include "catch.hpp"
#include "rs2gcoordinatesystem.h"
#include "radarbiasinfo.h"

#include <cmath>

// WGS-84 constants (duplicated here for reference-value calculations)
static const double WGS84_A  = 6378137.0;
static const double WGS84_F  = 1.0 / 298.257223563;
static const double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

static const double DEG2RAD_T = M_PI / 180.0;
static const double RAD2DEG_T = 180.0 / M_PI;

// Helper: compute ECEF from geodetic (for independent verification)
static void referenceGeodetic2ECEF(double lat_rad, double lon_rad, double h,
                                   double& x, double& y, double& z)
{
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    x = (N + h) * cos_lat * cos(lon_rad);
    y = (N + h) * cos_lat * sin(lon_rad);
    z = (N * (1.0 - WGS84_E2) + h) * sin_lat;
}

// ---------------------------------------------------------------------------
// geodesic2Geocentric: WGS-84 geodetic -> ECEF
// ---------------------------------------------------------------------------
TEST_CASE("geodesic2Geocentric known points", "[projection][rs2g]")
{
    // Radar at equator/prime meridian at 0 m altitude
    RS2GCoordinateSystem cs(1, 0.0, 0.0, 0.0);

    SECTION("equator / prime meridian")
    {
        double x, y, z;
        cs.geodesic2Geocentric(0.0, 0.0, 0.0, x, y, z);

        REQUIRE(x == Approx(WGS84_A).epsilon(1e-6));
        REQUIRE(y == Approx(0.0).margin(1e-3));
        REQUIRE(z == Approx(0.0).margin(1e-3));
    }

    SECTION("equator / 90 deg east")
    {
        double x, y, z;
        cs.geodesic2Geocentric(0.0, M_PI / 2.0, 0.0, x, y, z);

        REQUIRE(x == Approx(0.0).margin(1e-3));
        REQUIRE(y == Approx(WGS84_A).epsilon(1e-6));
        REQUIRE(z == Approx(0.0).margin(1e-3));
    }

    SECTION("north pole")
    {
        double x, y, z;
        cs.geodesic2Geocentric(M_PI / 2.0, 0.0, 0.0, x, y, z);

        // polar radius = a * sqrt(1-e2)
        double polar_r = WGS84_A * sqrt(1.0 - WGS84_E2);
        REQUIRE(x == Approx(0.0).margin(1e-3));
        REQUIRE(y == Approx(0.0).margin(1e-3));
        REQUIRE(z == Approx(polar_r).epsilon(1e-6));
    }

    SECTION("with altitude")
    {
        double h = 10000.0; // 10 km
        double x, y, z;
        cs.geodesic2Geocentric(0.0, 0.0, h, x, y, z);

        REQUIRE(x == Approx(WGS84_A + h).epsilon(1e-6));
        REQUIRE(y == Approx(0.0).margin(1e-3));
        REQUIRE(z == Approx(0.0).margin(1e-3));
    }

    SECTION("arbitrary point matches reference implementation")
    {
        // 48.1°N, 16.5°E, 200 m altitude
        double lat_rad = 48.1 * DEG2RAD_T;
        double lon_rad = 16.5 * DEG2RAD_T;
        double h = 200.0;

        double x, y, z;
        cs.geodesic2Geocentric(lat_rad, lon_rad, h, x, y, z);

        double rx, ry, rz;
        referenceGeodetic2ECEF(lat_rad, lon_rad, h, rx, ry, rz);

        REQUIRE(x == Approx(rx).epsilon(1e-9));
        REQUIRE(y == Approx(ry).epsilon(1e-9));
        REQUIRE(z == Approx(rz).epsilon(1e-9));
    }
}

// ---------------------------------------------------------------------------
// geocentric2Geodesic: ECEF -> WGS-84 geodetic
// ---------------------------------------------------------------------------
TEST_CASE("geocentric2Geodesic known points", "[projection][rs2g]")
{
    RS2GCoordinateSystem cs(1, 0.0, 0.0, 0.0);

    SECTION("equator / prime meridian")
    {
        double lat, lon, h;
        bool ok = cs.geocentric2Geodesic(WGS84_A, 0.0, 0.0, lat, lon, h);

        REQUIRE(ok);
        REQUIRE(lat == Approx(0.0).margin(1e-6));
        REQUIRE(lon == Approx(0.0).margin(1e-6));
        REQUIRE(h == Approx(0.0).margin(0.01)); // sub-centimetre
    }

    SECTION("equator / 90 deg east")
    {
        double lat, lon, h;
        bool ok = cs.geocentric2Geodesic(0.0, WGS84_A, 0.0, lat, lon, h);

        REQUIRE(ok);
        REQUIRE(lat == Approx(0.0).margin(1e-6));
        REQUIRE(lon == Approx(90.0).epsilon(1e-6));
        REQUIRE(h == Approx(0.0).margin(0.01));
    }

    SECTION("near north pole")
    {
        // Exact pole (d_xy=0) is a numerical singularity for the iterative
        // ECEF->geodetic algorithm; test a point very close to the pole instead.
        double near_pole_lat = 89.999 * DEG2RAD_T;
        double ref_x, ref_y, ref_z;
        referenceGeodetic2ECEF(near_pole_lat, 0.0, 0.0, ref_x, ref_y, ref_z);

        double lat, lon, h;
        bool ok = cs.geocentric2Geodesic(ref_x, ref_y, ref_z, lat, lon, h);

        REQUIRE(ok);
        REQUIRE(lat == Approx(89.999).margin(0.01));
        REQUIRE(h == Approx(0.0).margin(1.0));
    }
}

// ---------------------------------------------------------------------------
// geodesic <-> geocentric roundtrip
// ---------------------------------------------------------------------------
TEST_CASE("geodesic2Geocentric and geocentric2Geodesic roundtrip", "[projection][rs2g]")
{
    RS2GCoordinateSystem cs(1, 0.0, 0.0, 0.0);

    struct TestPoint { double lat_deg; double lon_deg; double h_m; };

    TestPoint points[] = {
        {  0.0,    0.0,     0.0 },
        { 48.1,   16.5,   200.0 },    // Vienna area
        {-33.86, 151.21,   50.0 },    // Sydney
        { 51.47,  -0.46,   80.0 },    // London Heathrow
        { 71.0,   25.0,  1000.0 },    // high latitude
        {  0.0,  180.0,     0.0 },    // date line
    };

    for (auto& pt : points)
    {
        INFO("lat=" << pt.lat_deg << " lon=" << pt.lon_deg << " h=" << pt.h_m);

        double lat_rad = pt.lat_deg * DEG2RAD_T;
        double lon_rad = pt.lon_deg * DEG2RAD_T;

        double x, y, z;
        cs.geodesic2Geocentric(lat_rad, lon_rad, pt.h_m, x, y, z);

        double lat_out, lon_out, h_out;
        bool ok = cs.geocentric2Geodesic(x, y, z, lat_out, lon_out, h_out);

        REQUIRE(ok);
        REQUIRE(lat_out == Approx(pt.lat_deg).margin(1e-6));
        REQUIRE(lon_out == Approx(pt.lon_deg).margin(1e-6));
        REQUIRE(h_out   == Approx(pt.h_m).margin(0.01));
    }
}

// ---------------------------------------------------------------------------
// localCart <-> geocentric roundtrip
// ---------------------------------------------------------------------------
TEST_CASE("localCart2Geocentric and geocentric2LocalCart roundtrip", "[projection][rs2g]")
{
    // Radar near Vienna: 48.1°N, 16.5°E, 200 m
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 200.0);

    struct TestVec { double x; double y; double z; };

    TestVec vecs[] = {
        {     0.0,      0.0,     0.0 },   // origin = radar position
        {  1000.0,      0.0,     0.0 },   // 1 km east
        {     0.0,   1000.0,     0.0 },   // 1 km north
        {     0.0,      0.0,  1000.0 },   // 1 km up
        { -5000.0,   3000.0,   500.0 },   // arbitrary
        { 50000.0, -30000.0,  8000.0 },   // large offsets
    };

    for (auto& v : vecs)
    {
        INFO("local x=" << v.x << " y=" << v.y << " z=" << v.z);

        double ex, ey, ez;
        cs.localCart2Geocentric(v.x, v.y, v.z, ex, ey, ez);

        double lx, ly, lz;
        cs.geocentric2LocalCart(ex, ey, ez, lx, ly, lz);

        REQUIRE(lx == Approx(v.x).margin(1e-6));
        REQUIRE(ly == Approx(v.y).margin(1e-6));
        REQUIRE(lz == Approx(v.z).margin(1e-6));
    }
}

// ---------------------------------------------------------------------------
// local cartesian origin maps to radar's ECEF position
// ---------------------------------------------------------------------------
TEST_CASE("local origin equals radar ECEF position", "[projection][rs2g]")
{
    double lat_deg = 48.1, lon_deg = 16.5, alt_m = 200.0;
    RS2GCoordinateSystem cs(1, lat_deg, lon_deg, alt_m);

    // local (0,0,0) should map to the radar's ECEF
    double ex, ey, ez;
    cs.localCart2Geocentric(0.0, 0.0, 0.0, ex, ey, ez);

    // independently compute radar ECEF
    double ref_x, ref_y, ref_z;
    referenceGeodetic2ECEF(lat_deg * DEG2RAD_T, lon_deg * DEG2RAD_T, alt_m,
                           ref_x, ref_y, ref_z);

    REQUIRE(ex == Approx(ref_x).margin(0.01));
    REQUIRE(ey == Approx(ref_y).margin(0.01));
    REQUIRE(ez == Approx(ref_z).margin(0.01));
}

// ---------------------------------------------------------------------------
// geodesic2LocalCart: radar's own position should be local (0,0,0)
// ---------------------------------------------------------------------------
TEST_CASE("geodesic2LocalCart radar position is origin", "[projection][rs2g]")
{
    double lat_deg = 48.1, lon_deg = 16.5, alt_m = 200.0;
    RS2GCoordinateSystem cs(1, lat_deg, lon_deg, alt_m);

    double lx, ly, lz;
    cs.geodesic2LocalCart(lat_deg * DEG2RAD_T, lon_deg * DEG2RAD_T, alt_m, lx, ly, lz);

    REQUIRE(lx == Approx(0.0).margin(1e-3));
    REQUIRE(ly == Approx(0.0).margin(1e-3));
    REQUIRE(lz == Approx(0.0).margin(1e-3));
}

// ---------------------------------------------------------------------------
// radarSlant2LocalCart <-> localCart2RadarSlant roundtrip
// ---------------------------------------------------------------------------
TEST_CASE("radarSlant2LocalCart and localCart2RadarSlant roundtrip", "[projection][rs2g]")
{
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 200.0);

    struct SlantInput {
        double azimuth_deg;
        double slant_range_m;
        double altitude_m;
    };

    SlantInput inputs[] = {
        {   0.0,  50000.0,  5000.0 },  // north, 50 km, 5 km alt
        {  90.0,  80000.0, 10000.0 },  // east, 80 km, 10 km alt
        { 180.0,  30000.0,  3000.0 },  // south
        { 270.0, 100000.0, 12000.0 },  // west
        {  45.0,  60000.0,  7000.0 },  // NE
        { 225.0,  20000.0,  1000.0 },  // SW
    };

    for (auto& inp : inputs)
    {
        INFO("az=" << inp.azimuth_deg << " range=" << inp.slant_range_m
             << " alt=" << inp.altitude_m);

        double az_rad = inp.azimuth_deg * DEG2RAD_T;

        double ground_range, lx, ly, lz;
        cs.radarSlant2LocalCart(az_rad, inp.slant_range_m,
                                true, inp.altitude_m, ground_range,
                                lx, ly, lz);

        // reverse
        double az_out, sr_out, gr_out, alt_out;
        cs.localCart2RadarSlant(lx, ly, lz, az_out, sr_out, gr_out, alt_out);

        // azimuth roundtrip
        double az_out_deg = az_out * RAD2DEG_T;
        if (az_out_deg < 0) az_out_deg += 360.0;
        double expected_az = inp.azimuth_deg;
        if (expected_az < 0) expected_az += 360.0;
        REQUIRE(az_out_deg == Approx(expected_az).margin(0.01));

        // ground range should match
        REQUIRE(gr_out == Approx(ground_range).margin(0.1));
    }
}

// ---------------------------------------------------------------------------
// Full pipeline: radar slant -> ECEF -> geodetic, then reverse back
// ---------------------------------------------------------------------------
TEST_CASE("full pipeline radarSlant to geodesic roundtrip", "[projection][rs2g]")
{
    // Radar at 48.1°N, 16.5°E, 200 m altitude
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 200.0);

    double azimuth_rad = 45.0 * DEG2RAD_T;
    double slant_range = 60000.0; // 60 km
    double target_alt  = 8000.0;  // 8 km WGS-84 altitude

    // Forward: radar slant -> ECEF
    double ground_range, ecef_x, ecef_y, ecef_z;
    bool ok = cs.calculateRadSlt2Geocentric(azimuth_rad, slant_range,
                                             true, target_alt, ground_range,
                                             ecef_x, ecef_y, ecef_z);
    REQUIRE(ok);
    REQUIRE(ground_range > 0.0);
    REQUIRE(ground_range < slant_range); // ground range < slant range when target above radar

    // ECEF -> geodetic
    double lat_deg, lon_deg, h_m;
    ok = cs.geocentric2Geodesic(ecef_x, ecef_y, ecef_z, lat_deg, lon_deg, h_m);
    REQUIRE(ok);

    // Sanity: target should be northeast of radar (az=45°)
    REQUIRE(lat_deg > 48.1);   // further north
    REQUIRE(lon_deg > 16.5);   // further east

    // Reverse: geodetic -> ECEF -> local cart -> radar slant
    double ecef_x2, ecef_y2, ecef_z2;
    cs.geodesic2Geocentric(lat_deg * DEG2RAD_T, lon_deg * DEG2RAD_T, h_m,
                            ecef_x2, ecef_y2, ecef_z2);

    REQUIRE(ecef_x2 == Approx(ecef_x).margin(0.01));
    REQUIRE(ecef_y2 == Approx(ecef_y).margin(0.01));
    REQUIRE(ecef_z2 == Approx(ecef_z).margin(0.01));

    double lx, ly, lz;
    cs.geocentric2LocalCart(ecef_x2, ecef_y2, ecef_z2, lx, ly, lz);

    double az_out, sr_out, gr_out, alt_out;
    cs.localCart2RadarSlant(lx, ly, lz, az_out, sr_out, gr_out, alt_out);

    double az_out_deg = az_out * RAD2DEG_T;
    REQUIRE(az_out_deg == Approx(45.0).margin(0.1));
    REQUIRE(gr_out == Approx(ground_range).margin(1.0));
}

// ---------------------------------------------------------------------------
// radarSlant2LocalCart with bias correction
// ---------------------------------------------------------------------------
TEST_CASE("radarSlant2LocalCart with bias correction", "[projection][rs2g]")
{
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 200.0);

    double az_rad = 90.0 * DEG2RAD_T;
    double slant_range = 50000.0;
    double altitude = 5000.0;

    // without bias
    double gr_no_bias, lx_nb, ly_nb, lz_nb;
    cs.radarSlant2LocalCart(az_rad, slant_range, true, altitude, gr_no_bias,
                            lx_nb, ly_nb, lz_nb);

    // with zero bias (should give same result)
    RadarBiasInfo zero_bias;
    zero_bias.azimuth_bias_valid_ = true;
    zero_bias.azimuth_bias_deg_   = 0.0;
    zero_bias.range_bias_valid_   = true;
    zero_bias.range_bias_m_       = 0.0;
    zero_bias.range_gain_         = 0.0;

    double lx_zb, ly_zb, lz_zb;
    cs.radarSlant2LocalCart(az_rad, slant_range, true, altitude, zero_bias,
                            lx_zb, ly_zb, lz_zb);

    REQUIRE(lx_zb == Approx(lx_nb).margin(1e-6));
    REQUIRE(ly_zb == Approx(ly_nb).margin(1e-6));
    REQUIRE(lz_zb == Approx(lz_nb).margin(1e-6));

    // with non-zero azimuth bias: shifts result in azimuth direction
    RadarBiasInfo az_bias;
    az_bias.azimuth_bias_valid_ = true;
    az_bias.azimuth_bias_deg_   = 1.0; // 1 degree bias
    az_bias.range_bias_valid_   = true;
    az_bias.range_bias_m_       = 0.0;
    az_bias.range_gain_         = 0.0;

    double lx_ab, ly_ab, lz_ab;
    cs.radarSlant2LocalCart(az_rad, slant_range, true, altitude, az_bias,
                            lx_ab, ly_ab, lz_ab);

    // with positive az bias, the corrected azimuth is smaller, so x should differ
    REQUIRE(lx_ab != Approx(lx_nb).margin(1.0));
    // z remains the same (elevation unchanged)
    REQUIRE(lz_ab == Approx(lz_nb).margin(1e-6));

    // with range bias: shifts ground range
    RadarBiasInfo rng_bias;
    rng_bias.azimuth_bias_valid_ = true;
    rng_bias.azimuth_bias_deg_   = 0.0;
    rng_bias.range_bias_valid_   = true;
    rng_bias.range_bias_m_       = 500.0; // 500 m range bias
    rng_bias.range_gain_         = 0.0;

    double lx_rb, ly_rb, lz_rb;
    cs.radarSlant2LocalCart(az_rad, slant_range, true, altitude, rng_bias,
                            lx_rb, ly_rb, lz_rb);

    // corrected range is smaller, so the position should be closer to radar
    double dist_no_bias = sqrt(lx_nb * lx_nb + ly_nb * ly_nb);
    double dist_rng_bias = sqrt(lx_rb * lx_rb + ly_rb * ly_rb);
    REQUIRE(dist_rng_bias < dist_no_bias);
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------
TEST_CASE("zero slant range gives radar position", "[projection][rs2g]")
{
    double lat_deg = 48.1, lon_deg = 16.5, alt_m = 200.0;
    RS2GCoordinateSystem cs(1, lat_deg, lon_deg, alt_m);

    double ground_range, ecef_x, ecef_y, ecef_z;
    bool ok = cs.calculateRadSlt2Geocentric(0.0, 0.0, false, 0.0,
                                             ground_range, ecef_x, ecef_y, ecef_z);
    REQUIRE(ok);
    REQUIRE(ground_range == Approx(0.0).margin(1e-6));

    double lat_out, lon_out, h_out;
    ok = cs.geocentric2Geodesic(ecef_x, ecef_y, ecef_z, lat_out, lon_out, h_out);
    REQUIRE(ok);
    REQUIRE(lat_out == Approx(lat_deg).margin(0.01));
    REQUIRE(lon_out == Approx(lon_deg).margin(0.01));
    REQUIRE(h_out   == Approx(alt_m).margin(1.0));
}

TEST_CASE("all four cardinal azimuths produce correct quadrants", "[projection][rs2g]")
{
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 200.0);
    double range = 50000.0;
    double alt   = 5000.0;

    // North (az=0): y positive, x ~0
    {
        double gr, lx, ly, lz;
        cs.radarSlant2LocalCart(0.0, range, true, alt, gr, lx, ly, lz);
        REQUIRE(ly > 0.0);
        REQUIRE(fabs(lx) < 1.0);
    }
    // East (az=pi/2): x positive, y ~0
    {
        double gr, lx, ly, lz;
        cs.radarSlant2LocalCart(M_PI / 2.0, range, true, alt, gr, lx, ly, lz);
        REQUIRE(lx > 0.0);
        REQUIRE(fabs(ly) < 1.0);
    }
    // South (az=pi): y negative, x ~0
    {
        double gr, lx, ly, lz;
        cs.radarSlant2LocalCart(M_PI, range, true, alt, gr, lx, ly, lz);
        REQUIRE(ly < 0.0);
        REQUIRE(fabs(lx) < 1.0);
    }
    // West (az=3pi/2): x negative, y ~0
    {
        double gr, lx, ly, lz;
        cs.radarSlant2LocalCart(3.0 * M_PI / 2.0, range, true, alt, gr, lx, ly, lz);
        REQUIRE(lx < 0.0);
        REQUIRE(fabs(ly) < 1.0);
    }
}

// ---------------------------------------------------------------------------
// getGroundRange: base class method
// ---------------------------------------------------------------------------
TEST_CASE("getGroundRange basic properties", "[projection][rs2g]")
{
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 200.0);

    SECTION("ground range with no altitude defaults to radar height")
    {
        double gr, adj_alt;
        cs.getGroundRange(50000.0, false, 0.0, gr, adj_alt);

        // When has_altitude=false, elevation_m defaults to h_r_ (same as radar).
        // On a curved earth the elevation angle is slightly negative (looking
        // down along the curvature), so ground range is very close to slant
        // range and adjusted altitude is less than radar height.
        REQUIRE(gr == Approx(50000.0).epsilon(0.01));
        REQUIRE(adj_alt < 200.0); // below radar due to earth curvature
        REQUIRE(adj_alt > -200.0);
    }

    SECTION("ground range with high altitude")
    {
        double gr, adj_alt;
        cs.getGroundRange(50000.0, true, 10000.0, gr, adj_alt);

        // with target at 10 km and radar at 200 m, ground range < slant range
        REQUIRE(gr < 50000.0);
        REQUIRE(gr > 0.0);
    }

    SECTION("zero slant range gives zero ground range")
    {
        double gr, adj_alt;
        cs.getGroundRange(0.0, true, 10000.0, gr, adj_alt);
        REQUIRE(gr == Approx(0.0).margin(1e-6));
    }
}

// ---------------------------------------------------------------------------
// rs2gElevation: elevation angle calculation
// ---------------------------------------------------------------------------
TEST_CASE("rs2gElevation basic properties", "[projection][rs2g]")
{
    // Radar at 200 m altitude
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 200.0);

    SECTION("target at same height as radar gives near-zero elevation")
    {
        double elev = cs.rs2gElevation(200.0, 50000.0);
        REQUIRE(fabs(elev) < 0.01); // near zero radians
    }

    SECTION("target well above radar gives positive elevation")
    {
        double elev = cs.rs2gElevation(10000.0, 50000.0);
        REQUIRE(elev > 0.0);
    }

    SECTION("target below radar gives negative elevation")
    {
        double elev = cs.rs2gElevation(0.0, 50000.0);
        REQUIRE(elev < 0.0);
    }

    SECTION("zero slant range gives zero elevation")
    {
        double elev = cs.rs2gElevation(10000.0, 0.0);
        REQUIRE(elev == Approx(0.0));
    }
}

// ---------------------------------------------------------------------------
// Different radar locations produce consistent results
// ---------------------------------------------------------------------------
TEST_CASE("coordinate systems at different positions", "[projection][rs2g]")
{
    // Two radars at different locations looking at the same geographic point
    RS2GCoordinateSystem cs1(1, 48.0, 16.0, 100.0);
    RS2GCoordinateSystem cs2(2, 49.0, 17.0, 300.0);

    // A point somewhere in between: 48.5°N, 16.5°E, 5000 m
    double lat_rad = 48.5 * DEG2RAD_T;
    double lon_rad = 16.5 * DEG2RAD_T;
    double h = 5000.0;

    // Both should produce the same ECEF coordinates
    double lx1, ly1, lz1;
    cs1.geodesic2LocalCart(lat_rad, lon_rad, h, lx1, ly1, lz1);

    double ex1, ey1, ez1;
    cs1.localCart2Geocentric(lx1, ly1, lz1, ex1, ey1, ez1);

    double lx2, ly2, lz2;
    cs2.geodesic2LocalCart(lat_rad, lon_rad, h, lx2, ly2, lz2);

    double ex2, ey2, ez2;
    cs2.localCart2Geocentric(lx2, ly2, lz2, ex2, ey2, ez2);

    REQUIRE(ex1 == Approx(ex2).margin(0.01));
    REQUIRE(ey1 == Approx(ey2).margin(0.01));
    REQUIRE(ez1 == Approx(ez2).margin(0.01));

    // Local coordinates should differ because the radar positions differ
    REQUIRE_FALSE(lx1 == Approx(lx2).margin(1.0));
}

// ---------------------------------------------------------------------------
// Altitude difference: radar elevated, target at WGS84 height 0
// ---------------------------------------------------------------------------
TEST_CASE("rs2gElevation with target below radar", "[projection][rs2g][altitude]")
{
    // Radar at 500 m altitude
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 500.0);

    // Target at height 0 (on ellipsoid surface), various slant ranges
    double ranges[] = {5000.0, 10000.0, 30000.0, 50000.0, 100000.0};

    for (double rho : ranges)
    {
        INFO("slant_range=" << rho);

        // target below radar → negative elevation
        double elev = cs.rs2gElevation(0.0, rho);
        REQUIRE(elev < 0.0);

        // target at radar height → near-zero elevation (only earth curvature effect)
        double elev_same = cs.rs2gElevation(500.0, rho);
        REQUIRE(fabs(elev_same) < fabs(elev));

        // target above radar → positive elevation
        double elev_above = cs.rs2gElevation(5000.0, rho);
        REQUIRE(elev_above > elev);
    }
}

TEST_CASE("getGroundRange with target at WGS84 height 0", "[projection][rs2g][altitude]")
{
    // Radar at 500 m
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 500.0);

    double slant_range = 50000.0;

    // Target at height 0 (below radar)
    double gr_below, adj_alt_below;
    cs.getGroundRange(slant_range, true, 0.0, gr_below, adj_alt_below);

    // Target at radar height (same altitude)
    double gr_same, adj_alt_same;
    cs.getGroundRange(slant_range, true, 500.0, gr_same, adj_alt_same);

    // Target at 10km (above radar)
    double gr_above, adj_alt_above;
    cs.getGroundRange(slant_range, true, 10000.0, gr_above, adj_alt_above);

    // All ground ranges should be less than slant range
    REQUIRE(gr_below < slant_range);
    REQUIRE(gr_same <= slant_range); // approximately equal for same height
    REQUIRE(gr_above < slant_range);

    // Higher targets have larger elevation angles → shorter ground range
    REQUIRE(gr_above < gr_below);

    // Same-height should be closest to slant range
    REQUIRE(gr_same > gr_below);
    REQUIRE(gr_same > gr_above);

    // Adjusted altitude for target below radar: h_r + slant*sin(elev).
    // Includes earth curvature effect so can be well below 0 at long ranges.
    REQUIRE(adj_alt_below < 500.0);
    REQUIRE(adj_alt_below > -500.0);
}

TEST_CASE("full pipeline roundtrip with target at WGS84 height 0", "[projection][rs2g][altitude]")
{
    // Radar elevated at 500m, targets on the ellipsoid surface (height 0)
    double radar_lat = 48.1, radar_lon = 16.5, radar_alt = 500.0;
    RS2GCoordinateSystem cs(1, radar_lat, radar_lon, radar_alt);

    // A reference point at WGS84 height 0, roughly 40 km NE of radar
    double ref_lat_deg = 48.35;
    double ref_lon_deg = 16.85;
    double ref_height = 0.0;

    // Convert reference to local cartesian
    double lx, ly, lz;
    cs.geodesic2LocalCart(ref_lat_deg * DEG2RAD_T, ref_lon_deg * DEG2RAD_T, ref_height,
                          lx, ly, lz);

    // local z should be negative (target is below the radar).
    // It is more negative than -radar_alt due to earth curvature at ~40 km distance:
    // surface drops by ~d²/(2*R) ≈ 40000²/(2*6370000) ≈ 126 m
    REQUIRE(lz < 0.0);
    REQUIRE(lz < -radar_alt); // more negative than just the height difference

    // Get radar slant parameters
    double az_rad, slant_range, ground_range, alt_out;
    cs.localCart2RadarSlant(lx, ly, lz, az_rad, slant_range, ground_range, alt_out);

    // alt_out = h_r_ + lz, which is an approximation of the target height.
    // At ~40 km distance, earth curvature causes ~126 m difference between the
    // local tangent plane and the ellipsoid surface, so alt_out is not exactly 0.
    REQUIRE(alt_out < radar_alt);  // target is below radar

    // slant range should be greater than ground range (height difference)
    REQUIRE(slant_range > ground_range);

    // Now do the forward path: polar → ECEF → geodetic
    double gr_fwd, ecef_x, ecef_y, ecef_z;
    bool ok = cs.calculateRadSlt2Geocentric(
        az_rad, slant_range, true, ref_height,
        gr_fwd, ecef_x, ecef_y, ecef_z);
    REQUIRE(ok);

    double lat_out, lon_out, h_out;
    ok = cs.geocentric2Geodesic(ecef_x, ecef_y, ecef_z, lat_out, lon_out, h_out);
    REQUIRE(ok);

    // Should recover the original reference position
    REQUIRE(lat_out == Approx(ref_lat_deg).margin(0.001)); // ~100m
    REQUIRE(lon_out == Approx(ref_lon_deg).margin(0.001));
}

TEST_CASE("slant-to-ground range correction consistency with altitude", "[projection][rs2g][altitude]")
{
    // Verify that radarSlant2LocalCart and getGroundRange give consistent results
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 500.0);

    double azimuths_deg[] = {0.0, 45.0, 90.0, 180.0, 270.0};
    double target_heights[] = {0.0, 100.0, 500.0, 2000.0, 10000.0};
    double slant_range = 50000.0;

    for (double az_deg : azimuths_deg)
    {
        for (double target_h : target_heights)
        {
            INFO("az=" << az_deg << " target_h=" << target_h);

            double az_rad = az_deg * DEG2RAD_T;

            // Method 1: radarSlant2LocalCart gives ground_range directly
            double gr1, lx, ly, lz;
            cs.radarSlant2LocalCart(az_rad, slant_range, true, target_h,
                                    gr1, lx, ly, lz);

            // Method 2: getGroundRange
            double gr2, adj_alt;
            cs.getGroundRange(slant_range, true, target_h, gr2, adj_alt);

            // Both should give the same ground range
            REQUIRE(gr1 == Approx(gr2).epsilon(1e-10));

            // Ground range should be consistent with local xy
            double gr_from_xy = sqrt(lx * lx + ly * ly);
            REQUIRE(gr_from_xy == Approx(gr1).epsilon(1e-10));
        }
    }
}

// ---------------------------------------------------------------------------
// calculateRadSlt2Geocentric with bias
// ---------------------------------------------------------------------------
TEST_CASE("calculateRadSlt2Geocentric with bias", "[projection][rs2g]")
{
    RS2GCoordinateSystem cs(1, 48.1, 16.5, 200.0);

    double az_rad = 45.0 * DEG2RAD_T;
    double slant_range = 60000.0;
    double altitude = 8000.0;

    // without bias
    double gr, ex1, ey1, ez1;
    cs.calculateRadSlt2Geocentric(az_rad, slant_range, true, altitude,
                                   gr, ex1, ey1, ez1);

    // with zero bias
    RadarBiasInfo zero_bias;
    zero_bias.azimuth_bias_valid_ = true;
    zero_bias.azimuth_bias_deg_   = 0.0;
    zero_bias.range_bias_valid_   = true;
    zero_bias.range_bias_m_       = 0.0;
    zero_bias.range_gain_         = 0.0;

    double ex2, ey2, ez2;
    cs.calculateRadSlt2Geocentric(az_rad, slant_range, true, altitude,
                                   zero_bias, ex2, ey2, ez2);

    REQUIRE(ex2 == Approx(ex1).margin(0.01));
    REQUIRE(ey2 == Approx(ey1).margin(0.01));
    REQUIRE(ez2 == Approx(ez1).margin(0.01));
}
