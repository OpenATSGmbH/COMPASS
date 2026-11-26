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

#include "reporttable.h"

#include "json.hpp"

namespace
{

ReportTableDefinition definitionWithTexts()
{
    auto def = ReportTableDefinition::record("test_table", "Test", { "CAT062" });
    def.addColumn("value", PropertyDataType::DOUBLE, "Value");

    def.detail_code_texts[ 0 ] = "No reference data";
    def.detail_code_texts[ 3 ] = "Outside sector";
    def.detail_code_texts[ 9 ] = "Type reason";

    return def;
}

} // namespace

TEST_CASE("ReportTableDefinition detail code texts", "[report_table]")
{
    const auto def = definitionWithTexts();

    SECTION("texts of the set bits in bit order")
    {
        CHECK(def.detailCodeText(0) == "");
        CHECK(def.detailCodeText(1u << 0) == "No reference data");
        CHECK(def.detailCodeText((1u << 9) | (1u << 0)) == "No reference data, Type reason");
        CHECK(def.detailCodeText((1u << 3) | (1u << 9)) == "Outside sector, Type reason");
    }

    SECTION("a set bit without a text gives no text")
    {
        CHECK(def.detailCodeText(1u << 5) == "");
        CHECK(def.detailCodeText((1u << 5) | (1u << 3)) == "Outside sector");
    }

    SECTION("the texts are stored with the definition")
    {
        ReportTableDefinition read;
        REQUIRE(read.fromJSON(def.toJSON()));

        CHECK(read.detail_code_texts == def.detail_code_texts);
        CHECK(read.detailCodeText((1u << 9) | (1u << 3)) == "Outside sector, Type reason");
    }

    SECTION("a definition stored before the detail codes reads without texts")
    {
        auto j = def.toJSON();
        j.erase("detail_code_texts");

        ReportTableDefinition read;
        REQUIRE(read.fromJSON(j));

        CHECK(read.detail_code_texts.empty());
        CHECK(read.detailCodeText(1u << 0) == "");
    }
}
