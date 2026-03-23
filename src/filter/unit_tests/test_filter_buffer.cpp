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
#include "mock_variable_resolver.h"
#include "mode3afilter.h"
#include "modecfilter.h"
#include "acadfilter.h"
#include "acidfilter.h"
#include "primaryonlyfilter.h"
#include "buffer/buffer.h"
#include "dbcontent/dbcontent.h"

TEST_CASE("Mode3AFilter filterBuffer", "[filter][buffer][mode3a]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("Mode3AFilter", "Mode 3/A Codes", {
        {"active", true},
        {"values_str", "3771"}
    });

    Mode3AFilter filter(cfg, nullptr, mock);

    // Create buffer with Mode 3/A Code column (name returned by metaGetVariableName for CAT048)
    PropertyList pl;
    pl.addProperty("Mode 3/A Code", PropertyDataType::UINT);
    auto buffer = std::make_shared<Buffer>(pl, "CAT048");

    auto& vec = buffer->get<unsigned int>("Mode 3/A Code");
    vec.set(0, 2041u);  // matches octal 3771
    vec.set(1, 100u);   // does not match
    vec.setNull(2);      // null

    auto removed = filter.filterBuffer("CAT048", buffer);

    // Index 1 (non-matching) and 2 (null, null_wanted_=false) should be removed
    CHECK(removed.size() == 2);
    CHECK(std::find(removed.begin(), removed.end(), 1) != removed.end());
    CHECK(std::find(removed.begin(), removed.end(), 2) != removed.end());
}

TEST_CASE("ModeCFilter filterBuffer", "[filter][buffer][modec]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ModeCFilter", "Mode C Codes", {
        {"active", true},
        {"min_value", 0.0},
        {"max_value", 5000.0},
        {"null_wanted", false}
    });

    ModeCFilter filter(cfg, nullptr, mock);

    PropertyList pl;
    pl.addProperty("Mode C Code", PropertyDataType::FLOAT);
    auto buffer = std::make_shared<Buffer>(pl, "CAT048");

    auto& vec = buffer->get<float>("Mode C Code");
    vec.set(0, 2500.0f);   // in range
    vec.set(1, 10000.0f);  // out of range
    vec.set(2, -500.0f);   // out of range (below min)
    vec.setNull(3);         // null

    auto removed = filter.filterBuffer("CAT048", buffer);

    CHECK(removed.size() == 3);
    CHECK(std::find(removed.begin(), removed.end(), 1) != removed.end());
    CHECK(std::find(removed.begin(), removed.end(), 2) != removed.end());
    CHECK(std::find(removed.begin(), removed.end(), 3) != removed.end());
}

TEST_CASE("ModeCFilter filterBuffer with null_wanted", "[filter][buffer][modec]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ModeCFilter", "Mode C Codes", {
        {"active", true},
        {"min_value", 0.0},
        {"max_value", 5000.0},
        {"null_wanted", true}
    });

    ModeCFilter filter(cfg, nullptr, mock);

    PropertyList pl;
    pl.addProperty("Mode C Code", PropertyDataType::FLOAT);
    auto buffer = std::make_shared<Buffer>(pl, "CAT048");

    auto& vec = buffer->get<float>("Mode C Code");
    vec.set(0, 2500.0f);   // in range
    vec.setNull(1);         // null — wanted

    auto removed = filter.filterBuffer("CAT048", buffer);

    CHECK(removed.empty());
}

TEST_CASE("ACADFilter filterBuffer", "[filter][buffer][acad]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACADFilter", "Aircraft Address", {
        {"active", true},
        {"values_str", "4C8070"}
    });

    ACADFilter filter(cfg, nullptr, mock);

    PropertyList pl;
    pl.addProperty("Aircraft Address", PropertyDataType::UINT);
    auto buffer = std::make_shared<Buffer>(pl, "CAT048");

    auto& vec = buffer->get<unsigned int>("Aircraft Address");
    vec.set(0, 5013616u);  // matches hex 4C8070
    vec.set(1, 12345u);    // does not match
    vec.setNull(2);         // null

    auto removed = filter.filterBuffer("CAT048", buffer);

    CHECK(removed.size() == 2);
    CHECK(std::find(removed.begin(), removed.end(), 1) != removed.end());
    CHECK(std::find(removed.begin(), removed.end(), 2) != removed.end());
}

TEST_CASE("ACIDFilter filterBuffer", "[filter][buffer][acid]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACIDFilter", "Aircraft Identification", {
        {"active", true},
        {"values_str", "AEE"}
    });

    ACIDFilter filter(cfg, nullptr, mock);

    PropertyList pl;
    pl.addProperty("Aircraft Identification", PropertyDataType::STRING);
    auto buffer = std::make_shared<Buffer>(pl, "CAT048");

    auto& vec = buffer->get<std::string>("Aircraft Identification");
    vec.set(0, std::string("AEE123"));    // matches (contains AEE)
    vec.set(1, std::string("DLH456"));    // does not match
    vec.setNull(2);                        // null

    auto removed = filter.filterBuffer("CAT048", buffer);

    CHECK(removed.size() == 2);
    CHECK(std::find(removed.begin(), removed.end(), 1) != removed.end());
    CHECK(std::find(removed.begin(), removed.end(), 2) != removed.end());
}

TEST_CASE("PrimaryOnlyFilter filterBuffer", "[filter][buffer][primaryonly]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("PrimaryOnlyFilter", "Primary Only", {
        {"active", true}
    });

    PrimaryOnlyFilter filter(cfg, nullptr, mock);

    // CAT048 has m3a, mc, acad, acid, detection_type
    PropertyList pl;
    pl.addProperty("Mode 3/A Code", PropertyDataType::UINT);
    pl.addProperty("Mode C Code", PropertyDataType::FLOAT);
    pl.addProperty("Aircraft Address", PropertyDataType::UINT);
    pl.addProperty("Aircraft Identification", PropertyDataType::STRING);
    pl.addProperty("Type", PropertyDataType::UCHAR);
    auto buffer = std::make_shared<Buffer>(pl, "CAT048");

    auto& m3a = buffer->get<unsigned int>("Mode 3/A Code");
    auto& mc = buffer->get<float>("Mode C Code");
    auto& acad = buffer->get<unsigned int>("Aircraft Address");
    auto& acid = buffer->get<std::string>("Aircraft Identification");
    auto& dtype = buffer->get<unsigned char>("Type");

    // Row 0: all null, PSR detection type — should be kept (primary only)
    m3a.setNull(0);
    mc.setNull(0);
    acad.setNull(0);
    acid.setNull(0);
    dtype.set(0, 1);  // PSR detection

    // Row 1: has m3a — should be removed
    m3a.set(1, 2041u);
    mc.setNull(1);
    acad.setNull(1);
    acid.setNull(1);
    dtype.setNull(1);

    // Row 2: all null, non-PSR detection — should be removed
    m3a.setNull(2);
    mc.setNull(2);
    acad.setNull(2);
    acid.setNull(2);
    dtype.set(2, 2);  // not a PSR type (2 is not in {1,3,6,7})

    auto removed = filter.filterBuffer("CAT048", buffer);

    CHECK(removed.size() == 2);
    CHECK(std::find(removed.begin(), removed.end(), 1) != removed.end());
    CHECK(std::find(removed.begin(), removed.end(), 2) != removed.end());
}
