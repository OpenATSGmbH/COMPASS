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
#include "rangeedit.h"

using namespace rangeedit;

// ─── Range<double> ──────────────────────────────────────────────────────────

TEST_CASE("Range setLimits resets range to limits", "[gui][range]")
{
    Range<double> r;
    r.setLimits(10.0, 90.0);

    CHECK(r.getLimitMin() == 10.0);
    CHECK(r.getLimitMax() == 90.0);
    CHECK(r.getRangeMin() == 10.0);
    CHECK(r.getRangeMax() == 90.0);
}

TEST_CASE("Range isValid for proper limits and range", "[gui][range]")
{
    Range<double> r;
    r.setLimits(0.0, 100.0);
    CHECK(r.isValid());

    r.setRange(RangeType::Min, 25.0);
    r.setRange(RangeType::Max, 75.0);
    CHECK(r.isValid());
}

TEST_CASE("Range isValid false for equal limits", "[gui][range]")
{
    Range<double> r;
    r.setLimits(50.0, 50.0);
    CHECK_FALSE(r.isValid());
}

TEST_CASE("Range getLimit and getRange by RangeType", "[gui][range]")
{
    Range<double> r;
    r.setLimits(0.0, 100.0);
    r.setRange(RangeType::Min, 20.0);
    r.setRange(RangeType::Max, 80.0);

    CHECK(r.getLimit(RangeType::Min) == 0.0);
    CHECK(r.getLimit(RangeType::Max) == 100.0);
    CHECK(r.getRange(RangeType::Min) == 20.0);
    CHECK(r.getRange(RangeType::Max) == 80.0);
}

TEST_CASE("Range resetRange restores to limits", "[gui][range]")
{
    Range<double> r;
    r.setLimits(0.0, 100.0);
    r.setRange(RangeType::Min, 30.0);
    r.setRange(RangeType::Max, 70.0);

    r.resetRange(RangeType::Min);
    CHECK(r.getRangeMin() == 0.0);
    CHECK(r.getRangeMax() == 70.0);

    r.resetRanges();
    CHECK(r.getRangeMin() == 0.0);
    CHECK(r.getRangeMax() == 100.0);
}

TEST_CASE("Range interpFactor at endpoints and midpoint", "[gui][range]")
{
    Range<double> r;
    r.setLimits(0.0, 100.0);

    // At limits (range == limits after setLimits)
    CHECK(r.interpFactor(RangeType::Min) == Approx(0.0));
    CHECK(r.interpFactor(RangeType::Max) == Approx(1.0));

    // Move min to midpoint
    r.setRange(RangeType::Min, 50.0);
    CHECK(r.interpFactor(RangeType::Min) == Approx(0.5));

    // Move max to quarter point (keep min <= max)
    r.setRange(RangeType::Min, 0.0);  // reset min first
    r.setRange(RangeType::Max, 25.0);
    CHECK(r.interpFactor(RangeType::Max) == Approx(0.25));
}

TEST_CASE("Range interpFactor returns 0 for invalid range", "[gui][range]")
{
    Range<double> r;  // default: all zeros, limits equal → invalid
    CHECK(r.interpFactor(RangeType::Min) == Approx(0.0));
}

TEST_CASE("Range interpolate sets range from factor", "[gui][range]")
{
    Range<double> r;
    r.setLimits(0.0, 200.0);

    r.interpolate(RangeType::Min, 0.25);
    CHECK(r.getRangeMin() == Approx(50.0));

    r.interpolate(RangeType::Max, 0.75);
    CHECK(r.getRangeMax() == Approx(150.0));
}

TEST_CASE("Range interpolate + interpFactor round-trip", "[gui][range]")
{
    Range<double> r;
    r.setLimits(-100.0, 100.0);

    r.interpolate(RangeType::Min, 0.3);
    CHECK(r.interpFactor(RangeType::Min) == Approx(0.3).margin(1e-9));

    r.interpolate(RangeType::Max, 0.8);
    CHECK(r.interpFactor(RangeType::Max) == Approx(0.8).margin(1e-9));
}

TEST_CASE("Range interpolate clamps factor to [0,1]", "[gui][range]")
{
    Range<double> r;
    r.setLimits(0.0, 100.0);

    r.interpolate(RangeType::Min, -0.5);
    CHECK(r.getRangeMin() == Approx(0.0));

    r.interpolate(RangeType::Max, 1.5);
    CHECK(r.getRangeMax() == Approx(100.0));
}

// ─── SerializableRange<double> ──────────────────────────────────────────────

TEST_CASE("SerializableRange setLimits from strings", "[gui][range][serializable]")
{
    SerializableRange<double> r;
    CHECK(r.setLimits("0.0", "100.0"));
    CHECK(r.isValid());
    CHECK(r.getLimitMin() == Approx(0.0));
    CHECK(r.getLimitMax() == Approx(100.0));
}

TEST_CASE("SerializableRange setLimits with invalid string returns false", "[gui][range][serializable]")
{
    SerializableRange<double> r;
    CHECK_FALSE(r.setLimits("abc", "100.0"));
    CHECK_FALSE(r.setLimits("0.0", "xyz"));
}

TEST_CASE("SerializableRange getRangeAsString", "[gui][range][serializable]")
{
    SerializableRange<double> r;
    r.setPrecision(2);
    r.setLimits("0.0", "100.0");

    CHECK(r.getRangeMinAsString() == "0.00");
    CHECK(r.getRangeMaxAsString() == "100.00");
}

TEST_CASE("SerializableRange setRange from strings", "[gui][range][serializable]")
{
    SerializableRange<double> r;
    r.setLimits("0.0", "100.0");

    CHECK(r.setRange(RangeType::Min, "25.0"));
    CHECK(r.getRangeMin() == Approx(25.0));

    CHECK(r.setRange("10.0", "90.0"));
    CHECK(r.getRangeMin() == Approx(10.0));
    CHECK(r.getRangeMax() == Approx(90.0));
}

TEST_CASE("SerializableRange setRange with invalid string returns false", "[gui][range][serializable]")
{
    SerializableRange<double> r;
    r.setLimits("0.0", "100.0");

    CHECK_FALSE(r.setRange(RangeType::Min, "not_a_number"));
}

// ─── SerializableRange<float> ───────────────────────────────────────────────

TEST_CASE("SerializableRange<float> string round-trip", "[gui][range][serializable]")
{
    SerializableRange<float> r;
    r.setPrecision(2);
    CHECK(r.setLimits("0.0", "50.0"));

    CHECK(r.getRangeMinAsString() == "0.00");
    CHECK(r.getRangeMaxAsString() == "50.00");
}
