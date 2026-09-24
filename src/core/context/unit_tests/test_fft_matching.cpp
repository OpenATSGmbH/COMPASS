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
