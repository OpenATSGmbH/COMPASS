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

#include "task/result/reporttable.h"

TEST_CASE("ReportTableColumn::viewRepresentation - two decimals for floating point columns",
          "[report_table]")
{
    SECTION("length, speed, time and ratio columns get two decimals")
    {
        REQUIRE(ReportTableColumn("distance_m", PropertyDataType::DOUBLE, "Distance", "", "Length", "Meter").viewRepresentation() == "FLOAT_PREC2");
        REQUIRE(ReportTableColumn("speed_mps", PropertyDataType::FLOAT, "Speed", "", "Speed", "Meter/Second").viewRepresentation() == "FLOAT_PREC2");
        REQUIRE(ReportTableColumn("dt_s", PropertyDataType::DOUBLE, "Time", "", "Time", "Second").viewRepresentation() == "FLOAT_PREC2");
        REQUIRE(ReportTableColumn("ratio", PropertyDataType::DOUBLE, "Ratio").viewRepresentation() == "FLOAT_PREC2");
    }

    SECTION("angle columns keep the full value")
    {
        REQUIRE(ReportTableColumn("tst_lat", PropertyDataType::DOUBLE, "Latitude", "", "Angle", "Degree").viewRepresentation().empty());
    }

    SECTION("non floating point columns get no representation")
    {
        REQUIRE(ReportTableColumn("count", PropertyDataType::UINT, "Count").viewRepresentation().empty());
        REQUIRE(ReportTableColumn("gated", PropertyDataType::BOOL, "Gated").viewRepresentation().empty());
        REQUIRE(ReportTableColumn("rec_num", PropertyDataType::ULONGINT, "Record Number").viewRepresentation().empty());
    }

    SECTION("an explicit representation wins")
    {
        REQUIRE(ReportTableColumn("azimuth_deg", PropertyDataType::DOUBLE, "Azimuth", "", "Angle", "Degree", "FLOAT_PREC4").viewRepresentation() == "FLOAT_PREC4");
        REQUIRE(ReportTableColumn("distance_m", PropertyDataType::DOUBLE, "Distance", "", "Length", "Meter", "FLOAT_PREC0").viewRepresentation() == "FLOAT_PREC0");
    }
}
