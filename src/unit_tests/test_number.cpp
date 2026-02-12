#include "catch.hpp"
#include "number.h"

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
