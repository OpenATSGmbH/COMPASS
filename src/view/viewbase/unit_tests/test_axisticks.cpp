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
#include "axisticks.h"
#include "timeconv.h"

#include <cmath>
#include <set>

using namespace axis_ticks;
using Repr = dbContent::Representation;
using axis_ticks::TimeFields;

namespace
{
    bool isAscending(const std::vector<double>& v)
    {
        for (size_t i = 1; i < v.size(); ++i)
            if (v[ i ] <= v[ i - 1 ])
                return false;

        return true;
    }

    bool allInside(const std::vector<double>& v, double vmin, double vmax)
    {
        const double eps = 1e-9;

        for (double x : v)
            if (x < vmin - eps || x > vmax + eps)
                return false;

        return true;
    }
}

TEST_CASE("isIntegralDataType", "[view][axisticks]")
{
    CHECK(isIntegralDataType(PropertyDataType::BOOL));
    CHECK(isIntegralDataType(PropertyDataType::UCHAR));
    CHECK(isIntegralDataType(PropertyDataType::UINT));
    CHECK(isIntegralDataType(PropertyDataType::ULONGINT));

    CHECK(!isIntegralDataType(PropertyDataType::FLOAT));
    CHECK(!isIntegralDataType(PropertyDataType::DOUBLE));
    CHECK(!isIntegralDataType(PropertyDataType::TIMESTAMP));
    CHECK(!isIntegralDataType(PropertyDataType::STRING));
}

TEST_CASE("needsCustomLabels", "[view][axisticks]")
{
    //a plain double is already presented correctly by Qt
    CHECK(!needsCustomLabels(PropertyDataType::DOUBLE, Repr::STANDARD));

    //on a plain value axis a timestamp reads as milliseconds since epoch. The
    //scatter plot decides on its date time axis before ever asking here.
    CHECK(needsCustomLabels(PropertyDataType::TIMESTAMP, Repr::STANDARD));

    //whole numbers must not be broken into fractional ticks
    CHECK(needsCustomLabels(PropertyDataType::UINT, Repr::STANDARD));
    CHECK(needsCustomLabels(PropertyDataType::BOOL, Repr::STANDARD));

    //a representation always changes how the value reads
    CHECK(needsCustomLabels(PropertyDataType::FLOAT, Repr::SECONDS_TO_TIME));
    CHECK(needsCustomLabels(PropertyDataType::UINT, Repr::DEC_TO_OCTAL));
    CHECK(needsCustomLabels(PropertyDataType::DOUBLE, Repr::FLOAT_PREC2));
}

TEST_CASE("niceStep picks 1, 2 or 5 times a power of ten", "[view][axisticks]")
{
    CHECK(niceStep(0.9, false) == Approx(1.0));
    CHECK(niceStep(1.5, false) == Approx(2.0));
    CHECK(niceStep(3.0, false) == Approx(5.0));
    CHECK(niceStep(7.0, false) == Approx(10.0));
    CHECK(niceStep(120.0, false) == Approx(200.0));
    CHECK(niceStep(0.03, false) == Approx(0.05));

    //an integral step never drops below one
    CHECK(niceStep(0.03, true) == Approx(1.0));
    CHECK(niceStep(0.9, true) == Approx(1.0));
    CHECK(niceStep(3.0, true) == Approx(5.0));
}

TEST_CASE("niceTimeStep snaps to whole minutes and hours", "[view][axisticks]")
{
    CHECK(niceTimeStep(0.003) == Approx(0.005));
    CHECK(niceTimeStep(0.06) == Approx(0.1));
    CHECK(niceTimeStep(0.5) == Approx(0.5));
    CHECK(niceTimeStep(0.7) == Approx(1.0));
    CHECK(niceTimeStep(12.0) == Approx(15.0));
    CHECK(niceTimeStep(40.0) == Approx(60.0));
    CHECK(niceTimeStep(700.0) == Approx(900.0));     // 15 min
    CHECK(niceTimeStep(2000.0) == Approx(3600.0));   // 1 h
    CHECK(niceTimeStep(50000.0) == Approx(86400.0)); // 1 day
}

TEST_CASE("generate returns ascending ticks inside the range", "[view][axisticks]")
{
    auto ticks = generate(0.0, 1000.0, PropertyDataType::DOUBLE, Repr::STANDARD, 8);

    CHECK(!ticks.values.empty());
    CHECK(isAscending(ticks.values));
    CHECK(allInside(ticks.values, 0.0, 1000.0));
    CHECK(ticks.step == Approx(200.0));
    CHECK(ticks.style.decimals == 0);
}

TEST_CASE("generate never produces a fractional step for integral types", "[view][axisticks]")
{
    //five distinct values, e.g. a small enum
    auto ticks = generate(0.0, 4.0, PropertyDataType::UCHAR, Repr::STANDARD, 8);

    CHECK(ticks.step >= 1.0);
    CHECK(ticks.style.decimals == 0);

    for (double v : ticks.values)
        CHECK(v == Approx(std::round(v)));
}

TEST_CASE("generate handles a range narrower than one whole number", "[view][axisticks]")
{
    //a bool column holding only false
    auto ticks = generate(0.0, 0.0, PropertyDataType::BOOL, Repr::STANDARD, 8);

    REQUIRE(ticks.values.size() == 1);
    CHECK(ticks.values.front() == Approx(0.0));

    //a zoom deep into an integral axis
    auto ticks2 = generate(7.2, 7.4, PropertyDataType::UINT, Repr::STANDARD, 8);

    REQUIRE(ticks2.values.size() == 1);
    CHECK(ticks2.values.front() == Approx(7.0));
}

TEST_CASE("generate uses time steps for a time of day axis", "[view][axisticks]")
{
    //one hour of data
    auto ticks = generate(43200.0, 46800.0, PropertyDataType::FLOAT, Repr::SECONDS_TO_TIME, 8);

    CHECK(ticks.step == Approx(600.0)); // 10 min
    CHECK(isAscending(ticks.values));
    CHECK(allInside(ticks.values, 43200.0, 46800.0));

    auto l = labels(ticks, PropertyDataType::FLOAT, Repr::SECONDS_TO_TIME);

    REQUIRE(l.size() == ticks.values.size());

    //at a step of a second or more the milliseconds are noise
    CHECK(l.front() == "12:00:00");
    CHECK(l.back() == "13:00:00");
}

TEST_CASE("generate keeps the milliseconds on a sub second time axis", "[view][axisticks]")
{
    //a zoom down to half a second
    auto ticks = generate(43200.0, 43200.5, PropertyDataType::FLOAT, Repr::SECONDS_TO_TIME, 8);

    CHECK(ticks.step < 1.0);

    auto l = labels(ticks, PropertyDataType::FLOAT, Repr::SECONDS_TO_TIME);

    REQUIRE(!l.empty());

    for (const auto& s : l)
        CHECK(s.size() == 12); // hh:mm:ss.zzz
}

TEST_CASE("generate keeps the tick count near the target", "[view][axisticks]")
{
    const int target = 8;

    const std::vector<std::pair<double, double>> ranges = {
        {    0.0,       1.0 },
        {    0.0,    1000.0 },
        { -500.0,     500.0 },
        {    0.0, 1234567.0 },
        {  1.234,     1.239 }
    };

    for (const auto& r : ranges)
    {
        auto ticks = generate(r.first, r.second, PropertyDataType::DOUBLE, Repr::STANDARD, target);

        INFO("range " << r.first << " to " << r.second);
        CHECK(!ticks.values.empty());
        CHECK(ticks.values.size() <= (size_t)(target + 2));
        CHECK(isAscending(ticks.values));
        CHECK(allInside(ticks.values, r.first, r.second));
    }
}

TEST_CASE("generate produces distinct labels", "[view][axisticks]")
{
    //Qt keys categories by label, so two ticks must never read the same
    const std::vector<std::pair<double, double>> ranges = {
        {   1.234,    1.239 },
        {   0.0,      0.001 },
        {   0.0,   1000.0   },
        { -12.5,     12.5   }
    };

    for (const auto& r : ranges)
    {
        auto ticks = generate(r.first, r.second, PropertyDataType::DOUBLE, Repr::STANDARD, 8);
        auto l     = labels(ticks, PropertyDataType::DOUBLE, Repr::STANDARD);

        std::set<std::string> distinct(l.begin(), l.end());

        INFO("range " << r.first << " to " << r.second);
        CHECK(distinct.size() == l.size());
    }
}

TEST_CASE("generate does not produce a negative zero tick", "[view][axisticks]")
{
    //a range starting just below zero snaps the first tick to a negative zero,
    //which would read as "-0.00"
    auto ticks = generate(-1000.0, 120000.0, PropertyDataType::FLOAT, Repr::FLOAT_PREC2, 8);
    auto l     = labels(ticks, PropertyDataType::FLOAT, Repr::FLOAT_PREC2);

    REQUIRE(!l.empty());

    for (const auto& s : l)
        CHECK(s != "-0.00");

    CHECK(l.front() == "0.00");
}

TEST_CASE("timeFieldsForSpan picks the smallest unambiguous form", "[view][axisticks]")
{
    auto t = [ ] (const std::string& str) { return boost::posix_time::time_from_string(str); };

    CHECK(timeFieldsForSpan(t("2025-12-31 23:00:00"), t("2026-01-01 01:00:00")) == TimeFields::Full);
    CHECK(timeFieldsForSpan(t("2026-09-22 23:00:00"), t("2026-09-23 01:00:00")) == TimeFields::DateTime);
    CHECK(timeFieldsForSpan(t("2026-09-23 10:00:00"), t("2026-09-23 11:00:00")) == TimeFields::Time);
    CHECK(timeFieldsForSpan(t("2026-09-23 10:00:00.000"), t("2026-09-23 10:00:01.500")) == TimeFields::TimeMs);
}

TEST_CASE("timeLabel drops what the span does not need", "[view][axisticks]")
{
    const auto t = boost::posix_time::time_from_string("2026-09-23 10:03:27.500");

    CHECK(timeLabel(t, TimeFields::Full) == "2026-09-23 10:03:27");
    CHECK(timeLabel(t, TimeFields::DateTime) == "09-23 10:03:27");
    CHECK(timeLabel(t, TimeFields::Time) == "10:03:27");
    CHECK(timeLabel(t, TimeFields::TimeMs) == "10:03:27.500");
}

TEST_CASE("a timestamp axis breaks at round times", "[view][axisticks]")
{
    //one hour of data, as milliseconds since epoch
    const double t0 = (double) Utils::Time::toLong(boost::posix_time::time_from_string("2026-09-23 10:00:00"));
    const double t1 = (double) Utils::Time::toLong(boost::posix_time::time_from_string("2026-09-23 11:00:00"));

    auto ticks = generate(t0, t1, PropertyDataType::TIMESTAMP, Repr::STANDARD, 8);

    //ten minutes, not a ragged millisecond step
    CHECK(ticks.step == Approx(600000.0));
    CHECK(ticks.style.time_fields == TimeFields::Time);
    CHECK(isAscending(ticks.values));
    CHECK(allInside(ticks.values, t0, t1));

    auto l = labels(ticks, PropertyDataType::TIMESTAMP, Repr::STANDARD);

    REQUIRE(l.size() == ticks.values.size());
    CHECK(l.front() == "10:00:00");
    CHECK(l.back() == "11:00:00");
}

TEST_CASE("a timestamp axis across a day keeps the date", "[view][axisticks]")
{
    const double t0 = (double) Utils::Time::toLong(boost::posix_time::time_from_string("2026-09-22 22:00:00"));
    const double t1 = (double) Utils::Time::toLong(boost::posix_time::time_from_string("2026-09-23 02:00:00"));

    auto ticks = generate(t0, t1, PropertyDataType::TIMESTAMP, Repr::STANDARD, 8);
    auto l     = labels(ticks, PropertyDataType::TIMESTAMP, Repr::STANDARD);

    CHECK(ticks.style.time_fields == TimeFields::DateTime);

    REQUIRE(!l.empty());
    CHECK(l.front() == "09-22 22:00:00");
    CHECK(l.back() == "09-23 02:00:00");
}

TEST_CASE("generate rejects a broken range", "[view][axisticks]")
{
    CHECK(generate(10.0, 0.0, PropertyDataType::DOUBLE, Repr::STANDARD, 8).values.empty());
    CHECK(generate(std::nan(""), 1.0, PropertyDataType::DOUBLE, Repr::STANDARD, 8).values.empty());
}

TEST_CASE("niceOctalStep and niceHexStep keep the labels round", "[view][axisticks]")
{
    CHECK(niceOctalStep(3.0) == Approx(4.0));
    CHECK(niceOctalStep(5.0) == Approx(8.0));
    CHECK(niceOctalStep(511.875) == Approx(512.0));
    CHECK(niceOctalStep(600.0) == Approx(1024.0));

    CHECK(niceHexStep(9.0) == Approx(16.0));
    CHECK(niceHexStep(5000.0) == Approx(8192.0));
}

TEST_CASE("generated labels of an octal axis are round octal codes", "[view][axisticks]")
{
    //a Mode 3/A axis over the full code range
    auto ticks = generate(0.0, 4095.0, PropertyDataType::UINT, Repr::DEC_TO_OCTAL, 8);
    auto l     = labels(ticks, PropertyDataType::UINT, Repr::DEC_TO_OCTAL);

    REQUIRE(l.size() == ticks.values.size());
    REQUIRE(!l.empty());

    //a decimal step would read as 1750, 3720, 5670
    CHECK(ticks.step == Approx(512.0));
    CHECK(l.front() == "0000");
    CHECK(l.back() == "7000");

    for (const auto& s : l)
    {
        CHECK(s.size() == 4);
        CHECK(s.find_first_not_of("01234567") == std::string::npos);
        CHECK(s.substr(1) == "000");
    }
}

TEST_CASE("generated labels of a hex axis are round hex codes", "[view][axisticks]")
{
    //an Aircraft Address axis over a part of the 24 bit range
    auto ticks = generate(0x400000, 0x44FFFF, PropertyDataType::UINT, Repr::DEC_TO_HEX, 8);
    auto l     = labels(ticks, PropertyDataType::UINT, Repr::DEC_TO_HEX);

    REQUIRE(l.size() == ticks.values.size());
    REQUIRE(!l.empty());

    for (const auto& s : l)
    {
        CHECK(s.size() == 6);
        CHECK(s.find_first_not_of("0123456789ABCDEF") == std::string::npos);
    }
}
