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
#include "sector.h"

#include <QColor>

#include <vector>

namespace
{
    const std::vector<std::pair<double,double>> square_47_48 { {47.0, 15.0}, {47.0, 16.0}, {48.0, 16.0}, {48.0, 15.0} };
    const std::vector<std::pair<double,double>> square_49_50 { {49.0, 15.0}, {49.0, 16.0}, {50.0, 16.0}, {50.0, 15.0} };
}

TEST_CASE("Sector points setter rebuilds polygon and caches", "[sector]")
{
    Sector sector(1u, "S1", "L1", false, false, QColor("#ff0000"), square_47_48);

    sector.createFastInsideTest(0.0);

    CHECK(sector.isInside(47.5, 15.5, 0.0));
    CHECK_FALSE(sector.isInside(49.5, 15.5, 0.0));

    auto lat = sector.getMinMaxLatitude();
    CHECK(lat.first  == Approx(47.0));
    CHECK(lat.second == Approx(48.0));

    SECTION("new polygon moves the sector")
    {
        sector.points(square_49_50, false);

        REQUIRE(sector.points().size() == 4);

        sector.createFastInsideTest(0.0);

        CHECK_FALSE(sector.isInside(47.5, 15.5, 0.0));
        CHECK(sector.isInside(49.5, 15.5, 0.0));

        lat = sector.getMinMaxLatitude();
        CHECK(lat.first  == Approx(49.0));
        CHECK(lat.second == Approx(50.0));

        auto lon = sector.getMinMaxLongitude();
        CHECK(lon.first  == Approx(15.0));
        CHECK(lon.second == Approx(16.0));
    }

    SECTION("inflated inside test is rebuilt as well")
    {
        sector.createFastInsideTest(0.1);
        CHECK(sector.isInside(48.05, 15.5, 0.1)); // 0.05 deg outside the old polygon

        sector.points(square_49_50, false);
        sector.createFastInsideTest(0.1);

        CHECK_FALSE(sector.isInside(48.05, 15.5, 0.1));
        CHECK(sector.isInside(50.05, 15.5, 0.1));
    }

    SECTION("notify flag controls the save callback")
    {
        unsigned int saved = 0;
        sector.serializeSector(true);
        sector.setSaveCallback([&saved](unsigned int) { ++saved; });

        sector.points(square_49_50, false);
        CHECK(saved == 0);

        sector.points(square_47_48);
        CHECK(saved == 1);

        sector.colorStr("#00ff00", false);
        CHECK(saved == 1);

        sector.name("S2", false);
        CHECK(saved == 1);

        sector.colorStr("#0000ff");
        sector.name("S3");
        CHECK(saved == 3);
    }
}
