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
#include "representation.h"
#include "axisticks.h"

using namespace dbContent;

TEST_CASE("representationString context free flags", "[view][representation]")
{
    CHECK(!representationIsContextFree(Representation::STANDARD));
    CHECK(!representationIsContextFree(Representation::DATA_SRC_NAME));
    CHECK(!representationIsContextFree(Representation::MLAT_RUS));

    CHECK(representationIsContextFree(Representation::SECONDS_TO_TIME));
    CHECK(representationIsContextFree(Representation::DEC_TO_OCTAL));
    CHECK(representationIsContextFree(Representation::DEC_TO_HEX));
    CHECK(representationIsContextFree(Representation::FEET_TO_FLIGHTLEVEL));
    CHECK(representationIsContextFree(Representation::CLIMB_DESCENT));
    CHECK(representationIsContextFree(Representation::LINE_NAME));
    CHECK(representationIsContextFree(Representation::FLOAT_PREC0));
    CHECK(representationIsContextFree(Representation::FLOAT_PREC4));
}

TEST_CASE("representationString seconds to time", "[view][representation]")
{
    std::string s;

    REQUIRE(representationString(s, Representation::SECONDS_TO_TIME, 43200.0f));
    CHECK(s == "12:00:00.000");

    REQUIRE(representationString(s, Representation::SECONDS_TO_TIME, 0.0f));
    CHECK(s == "00:00:00.000");

    REQUIRE(representationString(s, Representation::SECONDS_TO_TIME, 86399.5f));
    CHECK(s == "23:59:59.500");
}

TEST_CASE("representationString octal and hex", "[view][representation]")
{
    std::string s;

    REQUIRE(representationString(s, Representation::DEC_TO_OCTAL, (unsigned int) 4095));
    CHECK(s == "7777");

    REQUIRE(representationString(s, Representation::DEC_TO_OCTAL, (unsigned int) 7));
    CHECK(s == "0007");

    REQUIRE(representationString(s, Representation::DEC_TO_HEX, (unsigned int) 3958699));
    CHECK(s == "3C67AB");
}

TEST_CASE("representationString flight level and precision", "[view][representation]")
{
    std::string s;

    REQUIRE(representationString(s, Representation::FEET_TO_FLIGHTLEVEL, 35000.0f));
    CHECK(s == "350");

    REQUIRE(representationString(s, Representation::FLOAT_PREC0, 1.2345));
    CHECK(s == "1");

    REQUIRE(representationString(s, Representation::FLOAT_PREC1, 1.2345));
    CHECK(s == "1.2");

    REQUIRE(representationString(s, Representation::FLOAT_PREC2, 1.2345));
    CHECK(s == "1.23");

    REQUIRE(representationString(s, Representation::FLOAT_PREC4, 1.2345));
    CHECK(s == "1.2345");
}

TEST_CASE("representationString climb descent", "[view][representation]")
{
    std::string s;

    REQUIRE(representationString(s, Representation::CLIMB_DESCENT, (unsigned char) 0));
    CHECK(s == "LVL");

    REQUIRE(representationString(s, Representation::CLIMB_DESCENT, (unsigned char) 1));
    CHECK(s == "CLB");

    REQUIRE(representationString(s, Representation::CLIMB_DESCENT, (unsigned char) 2));
    CHECK(s == "DSC");

    REQUIRE(representationString(s, Representation::CLIMB_DESCENT, (unsigned char) 9));
    CHECK(s == "UDF");
}

TEST_CASE("representationString line name stays inside the four lines", "[view][representation]")
{
    std::string s;

    REQUIRE(representationString(s, Representation::LINE_NAME, (unsigned int) 0));
    CHECK(s == "L1");

    REQUIRE(representationString(s, Representation::LINE_NAME, (unsigned int) 3));
    CHECK(s == "L4");

    //an axis tick can land outside the data range, that must not assert
    REQUIRE(representationString(s, Representation::LINE_NAME, 4.0));
    CHECK(s == "4");
}

TEST_CASE("representationString rejects what it cannot resolve", "[view][representation]")
{
    std::string s = "untouched";

    CHECK(!representationString(s, Representation::STANDARD, 1.0));
    CHECK(s == "untouched");

    CHECK(!representationString(s, Representation::DATA_SRC_NAME, (unsigned int) 12));
    CHECK(s == "untouched");

    CHECK(!representationString(s, Representation::DEC_TO_OCTAL, std::string("1234")));
    CHECK(s == "untouched");
}

TEST_CASE("representationString promotes char sized values", "[view][representation]")
{
    std::string s;

    //without the promotion a char sized value is written as a character
    REQUIRE(representationString(s, Representation::FLOAT_PREC0, (unsigned char) 65));
    CHECK(s == "65");

    REQUIRE(representationString(s, Representation::DEC_TO_OCTAL, (unsigned char) 65));
    CHECK(s == "0101");
}

TEST_CASE("axis tick label casts the double back to the data type", "[view][representation]")
{
    //the whole point of the exercise, an octal stream base does nothing to a double
    CHECK(axis_ticks::label(4095.0, PropertyDataType::UINT, Representation::DEC_TO_OCTAL, 0) == "7777");
    CHECK(axis_ticks::label(43200.0, PropertyDataType::FLOAT, Representation::SECONDS_TO_TIME, 0) == "12:00:00.000");
    CHECK(axis_ticks::label(1.0, PropertyDataType::UCHAR, Representation::CLIMB_DESCENT, 0) == "CLB");
    CHECK(axis_ticks::label(3958699.0, PropertyDataType::UINT, Representation::DEC_TO_HEX, 0) == "3C67AB");
}

TEST_CASE("axis tick label without representation uses the data type", "[view][representation]")
{
    CHECK(axis_ticks::label(42.0, PropertyDataType::UINT, Representation::STANDARD, 0) == "42");
    CHECK(axis_ticks::label(42.4, PropertyDataType::UINT, Representation::STANDARD, 0) == "42");
    CHECK(axis_ticks::label(1.5, PropertyDataType::DOUBLE, Representation::STANDARD, 2) == "1.50");
    CHECK(axis_ticks::label(1.5, PropertyDataType::DOUBLE, Representation::STANDARD, 0) == "2");
    CHECK(axis_ticks::label(1.0, PropertyDataType::BOOL, Representation::STANDARD, 0) == "1");
}
