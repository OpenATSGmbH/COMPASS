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
#include "histogram.h"

#include <set>

using namespace histogram_helpers;
using TimeFields = BinLabelStyle::TimeFields;

namespace
{
    boost::posix_time::ptime timeAt(const std::string& str)
    {
        return boost::posix_time::time_from_string(str);
    }

    std::vector<std::string> binLabels(const RawHistogram& h)
    {
        std::vector<std::string> l;
        for (const auto& b : h.getBins())
            l.push_back(b.label);

        return l;
    }
}

TEST_CASE("decimalsForStep follows the bin width", "[histogram][labels]")
{
    CHECK(decimalsForStep(200.0) == 0);
    CHECK(decimalsForStep(1.0) == 0);
    CHECK(decimalsForStep(0.2) == 1);
    CHECK(decimalsForStep(0.05) == 2);
    CHECK(decimalsForStep(0.001) == 3);

    //a width beyond the cap does not grow the label further
    CHECK(decimalsForStep(1e-12) == 6);

    //no usable width, e.g. a category histogram
    CHECK(decimalsForStep(0.0) == -1);
    CHECK(decimalsForStep(-1.0) == -1);
}

TEST_CASE("decimalsForStep is stable at a decade boundary", "[histogram][labels]")
{
    //a width of 0.001 can come out of a subtraction slightly below itself,
    //which must not ask for one decimal more
    CHECK(decimalsForStep(0.0009999999999) == 3);
    CHECK(decimalsForStep((46.004 - 46.0) / 4.0) == 3);
    CHECK(decimalsForStep(0.09999999999) == 1);
}

TEST_CASE("numberLabel formats to the given decimals", "[histogram][labels]")
{
    CHECK(numberLabel(1.2345, 0) == "1");
    CHECK(numberLabel(1.2345, 2) == "1.23");
    CHECK(numberLabel(46.075, 1) == "46.1");

    //default precision keeps the previous behavior
    CHECK(numberLabel(1.5, -1) == std::to_string(1.5));
}

TEST_CASE("numberLabel never prints a negative zero", "[histogram][labels]")
{
    CHECK(numberLabel(-0.0, 2) == "0.00");
    CHECK(numberLabel(-0.001, 2) == "0.00");
    CHECK(numberLabel(-0.006, 2) == "-0.01");
}

TEST_CASE("timeFieldsForSpan picks the smallest unambiguous form", "[histogram][labels]")
{
    CHECK(timeFieldsForSpan(timeAt("2025-12-31 23:00:00"),
                            timeAt("2026-01-01 01:00:00")) == TimeFields::Full);

    CHECK(timeFieldsForSpan(timeAt("2026-09-22 23:00:00"),
                            timeAt("2026-09-23 01:00:00")) == TimeFields::DateTime);

    CHECK(timeFieldsForSpan(timeAt("2026-09-23 10:00:00"),
                            timeAt("2026-09-23 11:00:00")) == TimeFields::Time);

    //a range of seconds needs the milliseconds to stay distinct
    CHECK(timeFieldsForSpan(timeAt("2026-09-23 10:00:00.000"),
                            timeAt("2026-09-23 10:00:01.500")) == TimeFields::TimeMs);
}

TEST_CASE("timeLabel drops what the span does not need", "[histogram][labels]")
{
    const auto t = timeAt("2026-09-23 10:03:27.500");

    CHECK(timeLabel(t, TimeFields::Full) == "2026-09-23 10:03:27");
    CHECK(timeLabel(t, TimeFields::DateTime) == "09-23 10:03:27");
    CHECK(timeLabel(t, TimeFields::Time) == "10:03:27");
    CHECK(timeLabel(t, TimeFields::TimeMs) == "10:03:27.500");

    CHECK(timeLabel(boost::posix_time::ptime(), TimeFields::Time) == "");
}

TEST_CASE("double histogram labels use the bin width", "[histogram][labels]")
{
    HistogramT<double> h;
    REQUIRE(h.createFromRange(5, 0.0, 1.0));

    CHECK(h.labelStyle().decimals == 1);

    //mid values 0.1, 0.3, 0.5, 0.7, 0.9 - not six decimals of noise
    auto labels = binLabels(h.toRaw(false));

    REQUIRE(labels.size() == 5);
    CHECK(labels[ 0 ] == "0.1");
    CHECK(labels[ 2 ] == "0.5");
    CHECK(labels[ 4 ] == "0.9");
}

TEST_CASE("a narrow double histogram keeps enough decimals", "[histogram][labels]")
{
    HistogramT<double> h;
    REQUIRE(h.createFromRange(4, 46.0, 46.004));

    //a bin width of 0.001 puts the centres on 46.0005, 46.0015 and so on, so
    //the label resolves half a width
    CHECK(h.labelStyle().decimals == 4);

    auto labels = binLabels(h.toRaw(false));
    std::set<std::string> distinct(labels.begin(), labels.end());

    //every bin must still read differently
    CHECK(distinct.size() == labels.size());
    CHECK(labels.front() == "46.0005");
}

TEST_CASE("bin labels stay distinct across many ranges", "[histogram][labels]")
{
    const std::vector<std::pair<double, double>> ranges = {
        {      0.0,       1.0 },
        {      0.0,    1000.0 },
        {    -12.5,      12.5 },
        {     46.0,      49.0 },
        {      0.0,     0.001 },
        { 100000.0, 100000.05 }
    };

    for (const auto& r : ranges)
    {
        for (size_t n : { (size_t)5, (size_t)20 })
        {
            HistogramT<double> h;
            REQUIRE(h.createFromRange(n, r.first, r.second));

            auto labels = binLabels(h.toRaw(false));
            std::set<std::string> distinct(labels.begin(), labels.end());

            INFO("range " << r.first << " to " << r.second << " bins " << n);
            CHECK(labels.size() == n);
            CHECK(distinct.size() == labels.size());
        }
    }
}

TEST_CASE("integer histogram labels are whole numbers", "[histogram][labels]")
{
    HistogramT<unsigned int> h;
    REQUIRE(h.createFromRange(4, 0u, 400u));

    auto labels = binLabels(h.toRaw(false));

    REQUIRE(labels.size() == 4);
    CHECK(labels[ 0 ] == "50");
    CHECK(labels[ 3 ] == "350");
}

TEST_CASE("timestamp histogram labels are shortened to the span", "[histogram][labels]")
{
    SECTION("inside one day")
    {
        HistogramT<boost::posix_time::ptime> h;
        REQUIRE(h.createFromRange(2, timeAt("2026-09-23 10:00:00"),
                                     timeAt("2026-09-23 12:00:00")));

        CHECK(h.labelStyle().time_fields == TimeFields::Time);

        auto labels = binLabels(h.toRaw(false));

        REQUIRE(labels.size() == 2);
        CHECK(labels[ 0 ] == "10:30:00");
        CHECK(labels[ 1 ] == "11:30:00");
    }

    SECTION("across a day")
    {
        HistogramT<boost::posix_time::ptime> h;
        REQUIRE(h.createFromRange(2, timeAt("2026-09-22 22:00:00"),
                                     timeAt("2026-09-23 02:00:00")));

        CHECK(h.labelStyle().time_fields == TimeFields::DateTime);

        auto labels = binLabels(h.toRaw(false));

        REQUIRE(labels.size() == 2);
        CHECK(labels[ 0 ] == "09-22 23:00:00");
        CHECK(labels[ 1 ] == "09-23 01:00:00");
    }
}

TEST_CASE("bool histogram labels stay readable", "[histogram][labels]")
{
    HistogramT<bool> h;
    REQUIRE(h.createFromCategories({ false, true }, true));

    auto labels = binLabels(h.toRaw(false));

    REQUIRE(labels.size() == 2);
    CHECK(labels[ 0 ] == "false");
    CHECK(labels[ 1 ] == "true");
}

TEST_CASE("category histogram labels are the values themselves", "[histogram][labels]")
{
    HistogramT<unsigned int> h;
    REQUIRE(h.createFromCategories({ 1u, 2u, 7u }, true));

    auto labels = binLabels(h.toRaw(false));

    REQUIRE(labels.size() == 3);
    CHECK(labels[ 0 ] == "1");
    CHECK(labels[ 1 ] == "2");
    CHECK(labels[ 2 ] == "7");
}
