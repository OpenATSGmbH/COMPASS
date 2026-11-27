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

#include "catch.hpp"

#include "fft.h"

#include <boost/optional.hpp>

#include <GeographicLib/Geodesic.hpp>

#include <cmath>

using namespace context;

namespace
{
    const double fft_lat = 48.1;
    const double fft_lon = 16.6;

    // roughly 1 km north of the FFT, well inside the match radius
    const double near_lat = 48.109;
    const double near_lon = 16.6;

    // roughly 22 km north of the FFT, well outside the match radius
    const double far_lat = 48.3;
    const double far_lon = 16.6;

    const unsigned int fft_acad     = 0x00ABCD;
    const unsigned int fft_mode_3a  = 03456; // octal, stored as the 12 bit value
    const float        fft_mode_c   = 500.0f;

    const boost::optional<unsigned int> no_acad;
    const boost::optional<unsigned int> no_mode_3a;
    const boost::optional<float>        no_mode_c;

    /// FFT with a position and both secondary identification values set.
    FFT fullFFT()
    {
        FFT fft;
        fft.name("FULL");
        fft.latitude(fft_lat);
        fft.longitude(fft_lon);
        fft.altitude(1000.0);
        fft.modeSAddress(fft_acad);
        fft.mode3ACode(fft_mode_3a);

        return fft;
    }

    /// FFT identified by its Mode S address only, as used for Mode S capable sensors.
    FFT modeSFFT()
    {
        FFT fft;
        fft.name("MODE_S");
        fft.latitude(fft_lat);
        fft.longitude(fft_lon);
        fft.altitude(1000.0);
        fft.modeSAddress(fft_acad);

        return fft;
    }

    /// FFT identified by its Mode 3/A code only, as used for Mode A/C sensors.
    FFT modeACFFT()
    {
        FFT fft;
        fft.name("MODE_AC");
        fft.latitude(fft_lat);
        fft.longitude(fft_lon);
        fft.altitude(1000.0);
        fft.mode3ACode(fft_mode_3a);

        return fft;
    }

    /// FFT with a position only, which can not be identified in the data.
    FFT positionOnlyFFT()
    {
        FFT fft;
        fft.name("POSITION_ONLY");
        fft.latitude(fft_lat);
        fft.longitude(fft_lon);
        fft.altitude(1000.0);

        return fft;
    }
}

// ============================================================
// FFT value accessors
// ============================================================

TEST_CASE("FFT secondary identification accessors", "[context][fft]")
{
    FFT fft;

    SECTION("empty FFT has no values")
    {
        REQUIRE(!fft.hasModeSAddress());
        REQUIRE(!fft.hasMode3ACode());
        REQUIRE(!fft.hasModeCCode());
        REQUIRE(!fft.hasSecondaryIdentification());
    }

    SECTION("Mode S address round trip")
    {
        fft.modeSAddress(fft_acad);

        REQUIRE(fft.hasModeSAddress());
        REQUIRE(fft.modeSAddress() == fft_acad);
        REQUIRE(fft.hasSecondaryIdentification());
    }

    SECTION("Mode 3/A code round trip")
    {
        fft.mode3ACode(fft_mode_3a);

        REQUIRE(fft.hasMode3ACode());
        REQUIRE(fft.mode3ACode() == fft_mode_3a);
        REQUIRE(fft.hasSecondaryIdentification());
    }

    SECTION("Mode C code round trip")
    {
        fft.modeCCode(fft_mode_c);

        REQUIRE(fft.hasModeCCode());
        REQUIRE(fft.modeCCode() == Approx(fft_mode_c));
    }

    SECTION("Mode C code alone is no secondary identification")
    {
        fft.modeCCode(fft_mode_c);

        REQUIRE(!fft.hasSecondaryIdentification());
    }

    SECTION("values survive a JSON round trip")
    {
        fft.name("RT");
        fft.modeSAddress(fft_acad);
        fft.mode3ACode(fft_mode_3a);
        fft.modeCCode(fft_mode_c);

        FFT restored = FFT::fromJSON(fft.toJSON());

        REQUIRE(restored.modeSAddress() == fft_acad);
        REQUIRE(restored.mode3ACode() == fft_mode_3a);
        REQUIRE(restored.modeCCode() == Approx(fft_mode_c));
    }
}

// ============================================================
// Always FFT code 7777
// ============================================================

TEST_CASE("FFT always FFT code 7777", "[context][fft]")
{
    SECTION("7777 octal is the always FFT code")
    {
        REQUIRE(FFT::always_fft_mode_3a_code_ == 07777);
        REQUIRE(FFT::isAlwaysFFTCode(boost::optional<unsigned int>(07777)));
    }

    SECTION("other codes are not")
    {
        REQUIRE(!FFT::isAlwaysFFTCode(boost::optional<unsigned int>(07000)));
        REQUIRE(!FFT::isAlwaysFFTCode(boost::optional<unsigned int>(fft_mode_3a)));
        REQUIRE(!FFT::isAlwaysFFTCode(boost::optional<unsigned int>(0)));
    }

    SECTION("a missing code is not")
    {
        REQUIRE(!FFT::isAlwaysFFTCode(no_mode_3a));
    }
}

// ============================================================
// Mode S data
// ============================================================

TEST_CASE("FFT matching with Mode S data", "[context][fft]")
{
    FFT fft = modeSFFT();

    SECTION("matching address inside the radius matches")
    {
        auto result = fft.matches(near_lat, near_lon, fft_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(result.first);
        REQUIRE(result.second == Approx(1000.0f));
    }

    SECTION("matching address outside the radius does not match")
    {
        auto result = fft.matches(far_lat, far_lon, fft_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("different address vetoes the match")
    {
        auto result = fft.matches(near_lat, near_lon, boost::optional<unsigned int>(0x00BEEF),
                                  false, no_mode_3a, no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("ignore_mode_s discards the only identification, so no match")
    {
        auto result = fft.matches(near_lat, near_lon, fft_acad, true, no_mode_3a, no_mode_c);

        REQUIRE(!result.first);
    }
}

// ============================================================
// Mode A/C only data, no Mode S address
// ============================================================

TEST_CASE("FFT matching with Mode A/C only data", "[context][fft]")
{
    FFT fft = modeACFFT();

    SECTION("matching Mode 3/A code inside the radius matches")
    {
        auto result = fft.matches(near_lat, near_lon, no_acad, false,
                                  boost::optional<unsigned int>(fft_mode_3a), no_mode_c);

        REQUIRE(result.first);
        REQUIRE(result.second == Approx(1000.0f));
    }

    SECTION("matching Mode 3/A code outside the radius does not match")
    {
        auto result = fft.matches(far_lat, far_lon, no_acad, false,
                                  boost::optional<unsigned int>(fft_mode_3a), no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("different Mode 3/A code vetoes the match")
    {
        auto result = fft.matches(near_lat, near_lon, no_acad, false,
                                  boost::optional<unsigned int>(07000), no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("CAT001 data matches, since Mode S is not involved")
    {
        auto result = fft.matches(near_lat, near_lon, no_acad, true,
                                  boost::optional<unsigned int>(fft_mode_3a), no_mode_c);

        REQUIRE(result.first);
    }

    SECTION("a Mode S only FFT is never matched by Mode A/C data")
    {
        FFT mode_s_fft = modeSFFT();

        auto result = mode_s_fft.matches(near_lat, near_lon, no_acad, false,
                                         boost::optional<unsigned int>(fft_mode_3a), no_mode_c);

        REQUIRE(!result.first);
    }
}

// ============================================================
// Primary only data
// ============================================================

TEST_CASE("FFT matching with primary only data", "[context][fft]")
{
    SECTION("a primary plot at the FFT position does not match")
    {
        FFT fft = fullFFT();

        auto result = fft.matches(near_lat, near_lon, no_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("a primary plot exactly on the FFT position does not match")
    {
        FFT fft = fullFFT();

        auto result = fft.matches(fft_lat, fft_lon, no_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("a primary plot does not match a Mode A/C FFT either")
    {
        FFT fft = modeACFFT();

        auto result = fft.matches(near_lat, near_lon, no_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(!result.first);
    }
}

// ============================================================
// FFT without secondary identification
// ============================================================

TEST_CASE("FFT without secondary identification never matches", "[context][fft]")
{
    FFT fft = positionOnlyFFT();

    REQUIRE(!fft.hasSecondaryIdentification());

    SECTION("Mode S data does not match")
    {
        auto result = fft.matches(near_lat, near_lon, fft_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("Mode A/C data does not match")
    {
        auto result = fft.matches(near_lat, near_lon, no_acad, false,
                                  boost::optional<unsigned int>(fft_mode_3a), no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("primary only data does not match")
    {
        auto result = fft.matches(near_lat, near_lon, no_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(!result.first);
    }
}

// ============================================================
// Mode C code, confirmation only
// ============================================================

TEST_CASE("FFT Mode C code confirms but never identifies", "[context][fft]")
{
    SECTION("matching Mode C code keeps a Mode S match")
    {
        FFT fft = modeSFFT();
        fft.modeCCode(fft_mode_c);

        auto result = fft.matches(near_lat, near_lon, fft_acad, false, no_mode_3a,
                                  boost::optional<float>(fft_mode_c));

        REQUIRE(result.first);
    }

    SECTION("different Mode C code vetoes a Mode S match")
    {
        FFT fft = modeSFFT();
        fft.modeCCode(fft_mode_c);

        auto result = fft.matches(near_lat, near_lon, fft_acad, false, no_mode_3a,
                                  boost::optional<float>(3000.0f));

        REQUIRE(!result.first);
    }

    SECTION("a Mode C only FFT is never matched")
    {
        FFT fft = positionOnlyFFT();
        fft.modeCCode(fft_mode_c);

        auto result = fft.matches(near_lat, near_lon, no_acad, false, no_mode_3a,
                                  boost::optional<float>(fft_mode_c));

        REQUIRE(!result.first);
    }

    SECTION("a missing Mode C code in the data is no veto")
    {
        FFT fft = modeSFFT();
        fft.modeCCode(fft_mode_c);

        auto result = fft.matches(near_lat, near_lon, fft_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(result.first);
    }
}

// ============================================================
// Position handling
// ============================================================

TEST_CASE("FFT position handling", "[context][fft]")
{
    SECTION("an FFT without a position matches on the secondary value alone")
    {
        FFT fft;
        fft.name("NO_POSITION");
        fft.modeSAddress(fft_acad);

        auto result = fft.matches(far_lat, far_lon, fft_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(result.first);
    }

    SECTION("an FFT without an altitude reports altitude zero")
    {
        FFT fft;
        fft.name("NO_ALTITUDE");
        fft.latitude(fft_lat);
        fft.longitude(fft_lon);
        fft.modeSAddress(fft_acad);

        auto result = fft.matches(near_lat, near_lon, fft_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(result.first);
        REQUIRE(result.second == Approx(0.0f));
    }

    SECTION("the default match radius is 5000 m")
    {
        REQUIRE(FFT::DefaultMaxPlotDistanceM == Approx(5000.0));

        FFT fft;
        REQUIRE(fft.maxPlotDistanceM() == Approx(5000.0));
    }

    SECTION("an FFT can carry its own match radius")
    {
        FFT fft;
        fft.info()["max_plot_distance_m"] = 50.0;

        REQUIRE(fft.maxPlotDistanceM() == Approx(50.0));
    }
}

// ============================================================
// Both secondary values set
// ============================================================

TEST_CASE("FFT matching with both secondary values set", "[context][fft]")
{
    FFT fft = fullFFT();

    SECTION("both values matching matches")
    {
        auto result = fft.matches(near_lat, near_lon, fft_acad, false,
                                  boost::optional<unsigned int>(fft_mode_3a), no_mode_c);

        REQUIRE(result.first);
    }

    SECTION("one value matching and the other differing vetoes the match")
    {
        auto result = fft.matches(near_lat, near_lon, fft_acad, false,
                                  boost::optional<unsigned int>(07000), no_mode_c);

        REQUIRE(!result.first);
    }

    SECTION("Mode A/C data matches on the Mode 3/A code alone")
    {
        auto result = fft.matches(near_lat, near_lon, no_acad, false,
                                  boost::optional<unsigned int>(fft_mode_3a), no_mode_c);

        REQUIRE(result.first);
    }

    SECTION("Mode S data matches on the address alone")
    {
        auto result = fft.matches(near_lat, near_lon, fft_acad, false, no_mode_3a, no_mode_c);

        REQUIRE(result.first);
    }
}

TEST_CASE("FFT runtime id", "[context][fft]")
{
    FFT fft;
    fft.name("FFT1");

    FFT other;
    other.name("FFT1");

    SECTION("every FFT gets its own id")
    {
        REQUIRE_FALSE(fft.runtimeId().empty());
        REQUIRE(fft.runtimeId() != other.runtimeId());
    }

    SECTION("copies keep the id, e.g. on a vector reallocation")
    {
        FFT copy = fft;
        REQUIRE(copy.runtimeId() == fft.runtimeId());

        std::vector<FFT> ffts;
        ffts.push_back(fft);
        const std::string id = ffts.front().runtimeId();

        for (int i = 0; i < 100; ++i)
            ffts.push_back(other);

        REQUIRE(ffts.front().runtimeId() == id);
    }

    SECTION("the id is not compared and not serialized")
    {
        REQUIRE(fft == other);

        FFT restored = FFT::fromJSON(fft.toJSON());
        REQUIRE(restored == fft);
        REQUIRE(restored.runtimeId() != fft.runtimeId());
        REQUIRE(fft.toJSON().dump().find(fft.runtimeId()) == std::string::npos);
    }
}

// ============================================================
// Range delay (delayed-reply FFT)
// ============================================================

TEST_CASE("FFT range delay", "[context][fft]")
{
    FFT fft = modeSFFT();

    // the position at which the measuring radar plots the delayed FFT, here "far"
    const std::pair<double, double> expected {far_lat, far_lon};
    unsigned int expected_calls = 0;

    FFT::ExpectedPositionFunc expected_func = [ & ] ()
    {
        ++expected_calls;
        return boost::optional<std::pair<double, double>>(expected);
    };

    SECTION("not set or 0 means no delay")
    {
        REQUIRE(!fft.hasRangeDelay());

        fft.rangeDelayNM(0.0);
        REQUIRE(!fft.hasRangeDelay());

        fft.rangeDelayNM(65.0);
        REQUIRE(fft.hasRangeDelay());
        REQUIRE(fft.rangeDelayNM() == Approx(65.0));
        REQUIRE(FFT::fromJSON(fft.toJSON()).rangeDelayNM() == Approx(65.0));

        fft.removeRangeDelay();
        REQUIRE(!fft.hasRangeDelay());
    }

    SECTION("a target report near the expected position matches")
    {
        auto result = fft.matches(far_lat, far_lon, fft_acad, false, no_mode_3a, no_mode_c, expected_func);

        REQUIRE(result.first);
        REQUIRE(result.second == Approx(1000.0f));
        REQUIRE(expected_calls == 1);
    }

    SECTION("a target report near the site still matches, without the expected position")
    {
        auto result = fft.matches(near_lat, near_lon, fft_acad, false, no_mode_3a, no_mode_c, expected_func);

        REQUIRE(result.first);
        REQUIRE(expected_calls == 0);
    }

    SECTION("a target report near neither position does not match")
    {
        auto result = fft.matches(48.6, 16.6, fft_acad, false, no_mode_3a, no_mode_c, expected_func);

        REQUIRE(!result.first);
    }

    SECTION("the expected position needs the secondary identification")
    {
        auto result = fft.matches(far_lat, far_lon, boost::optional<unsigned int>(0x00BEEF),
                                  false, no_mode_3a, no_mode_c, expected_func);

        REQUIRE(!result.first);
        REQUIRE(expected_calls == 0); // not needed after the identification failed
    }

    SECTION("an unknown expected position does not match away from the site")
    {
        FFT::ExpectedPositionFunc unknown = [] () { return boost::optional<std::pair<double, double>>(); };

        auto result = fft.matches(far_lat, far_lon, fft_acad, false, no_mode_3a, no_mode_c, unknown);

        REQUIRE(!result.first);
    }
}

// ============================================================
// Position check math
// ============================================================

TEST_CASE("FFT flat earth distance", "[context][fft]")
{
    SECTION("same position is 0")
    {
        REQUIRE(FFT::flatEarthDistanceM(45.0, 20.0, 45.0, 20.0) == Approx(0.0).margin(1e-9));
    }

    SECTION("latitude: 111 km per degree")
    {
        REQUIRE(FFT::flatEarthDistanceM(45.001, 20.0, 45.0, 20.0) == Approx(111.0).epsilon(1e-6));
        REQUIRE(FFT::flatEarthDistanceM(44.999, 20.0, 45.0, 20.0) == Approx(111.0).epsilon(1e-6));
    }

    SECTION("longitude: scaled with the cosine of the latitude")
    {
        REQUIRE(FFT::flatEarthDistanceM(60.0, 20.001, 60.0, 20.0) == Approx(55.5).epsilon(1e-6));
        REQUIRE(FFT::flatEarthDistanceM(0.0, 20.001, 0.0, 20.0) == Approx(111.0).epsilon(1e-6));
    }

    SECTION("close to the geodesic distance within the maximum plot distance")
    {
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

        for (double azimuth_deg : {0.0, 45.0, 90.0, 135.0, 180.0, 270.0})
        {
            for (double dist_m : {50.0, 1000.0, 5000.0})
            {
                double lat2, lon2;
                geod.Direct(45.0, 20.0, azimuth_deg, dist_m, lat2, lon2);

                INFO("azimuth " << azimuth_deg << " distance " << dist_m);
                REQUIRE(FFT::flatEarthDistanceM(lat2, lon2, 45.0, 20.0) == Approx(dist_m).epsilon(0.005));
            }
        }
    }

    SECTION("the match distance limit uses this distance")
    {
        FFT fft = modeSFFT();

        // north of the FFT, just inside and just outside of the default 5000 m
        const double inside_lat  = fft_lat + 4990.0 / 111000.0;
        const double outside_lat = fft_lat + 5010.0 / 111000.0;

        REQUIRE(fft.matches(inside_lat, fft_lon, fft_acad, false, no_mode_3a, no_mode_c).first);
        REQUIRE(!fft.matches(outside_lat, fft_lon, fft_acad, false, no_mode_3a, no_mode_c).first);

        // the same limit around the expected position of a delayed-reply FFT
        const double exp_lat = 49.0, exp_lon = 17.0;
        FFT::ExpectedPositionFunc expected = [ & ] ()
        {
            return boost::optional<std::pair<double, double>>(std::make_pair(exp_lat, exp_lon));
        };

        REQUIRE(fft.matches(exp_lat + 4990.0 / 111000.0, exp_lon, fft_acad, false,
                            no_mode_3a, no_mode_c, expected).first);
        REQUIRE(!fft.matches(exp_lat + 5010.0 / 111000.0, exp_lon, fft_acad, false,
                             no_mode_3a, no_mode_c, expected).first);
    }

    SECTION("the per-FFT maximum plot distance")
    {
        FFT fft = modeSFFT();
        fft.info()["max_plot_distance_m"] = 50.0;

        REQUIRE(fft.matches(fft_lat + 40.0 / 111000.0, fft_lon, fft_acad, false, no_mode_3a, no_mode_c).first);
        REQUIRE(!fft.matches(fft_lat + 60.0 / 111000.0, fft_lon, fft_acad, false, no_mode_3a, no_mode_c).first);
    }
}

// ============================================================
// Matching against all FFTs, Mode 3/A 7777
// ============================================================

TEST_CASE("FFT matchesAny", "[context][fft]")
{
    const boost::optional<unsigned int> code_7777(FFT::always_fft_mode_3a_code_);

    // the 7777 entry of a transponder, as defined in a Data Context
    FFT fft_7777;
    fft_7777.name("FFT_7777");
    fft_7777.latitude(fft_lat);
    fft_7777.longitude(fft_lon);
    fft_7777.altitude(2000.0);
    fft_7777.modeSAddress(fft_acad);
    fft_7777.mode3ACode(FFT::always_fft_mode_3a_code_);

    FFT other = fullFFT(); // other transponder, altitude 1000 ft
    other.modeSAddress(0x00BEEF);

    const std::vector<FFT> ffts {other, fft_7777};

    SECTION("a 7777 plot of a defined FFT gets the FFT altitude")
    {
        auto result = FFT::matchesAny(ffts, near_lat, near_lon, fft_acad, false, code_7777, no_mode_c);

        REQUIRE(result.first);
        REQUIRE(result.second == Approx(2000.0f));
    }

    SECTION("a 7777 plot without a matching FFT is an FFT with altitude 0")
    {
        // unknown address
        auto result = FFT::matchesAny(ffts, near_lat, near_lon, boost::optional<unsigned int>(0x00AAAA),
                                      false, code_7777, no_mode_c);
        REQUIRE(result.first);
        REQUIRE(result.second == Approx(0.0f));

        // far from the FFT
        result = FFT::matchesAny(ffts, far_lat, far_lon, fft_acad, false, code_7777, no_mode_c);
        REQUIRE(result.first);
        REQUIRE(result.second == Approx(0.0f));

        // no FFTs defined
        result = FFT::matchesAny({}, near_lat, near_lon, fft_acad, false, code_7777, no_mode_c);
        REQUIRE(result.first);
        REQUIRE(result.second == Approx(0.0f));
    }

    SECTION("a 7777 plot of a delayed-reply FFT at the expected position gets the FFT altitude")
    {
        std::vector<FFT> delayed_ffts = ffts;
        delayed_ffts[ 1 ].rangeDelayNM(30.0);

        unsigned int calls = 0;
        FFT::ExpectedPositionOfFunc expected = [ & ] (const FFT& fft)
        {
            ++calls;
            REQUIRE(fft.name() == "FFT_7777"); // only for the FFT with a delay
            return boost::optional<std::pair<double, double>>(std::make_pair(far_lat, far_lon));
        };

        auto result = FFT::matchesAny(delayed_ffts, far_lat, far_lon, fft_acad, false, code_7777, no_mode_c,
                                      expected);

        REQUIRE(result.first);
        REQUIRE(result.second == Approx(2000.0f));
        REQUIRE(calls == 1);
    }

    SECTION("other plots are matched against the FFTs only")
    {
        auto result = FFT::matchesAny(ffts, near_lat, near_lon, boost::optional<unsigned int>(0x00BEEF),
                                      false, boost::optional<unsigned int>(fft_mode_3a), no_mode_c);
        REQUIRE(result.first);
        REQUIRE(result.second == Approx(1000.0f));

        result = FFT::matchesAny(ffts, near_lat, near_lon, boost::optional<unsigned int>(0x00AAAA),
                                 false, boost::optional<unsigned int>(01234), no_mode_c);
        REQUIRE(!result.first);
    }
}
