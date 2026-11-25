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

#include "results/reporttablecontent.h"
#include "results/base/detailcode.h"

#include "buffer.h"
#include "reporttable.h"

#include <algorithm>
#include <vector>

namespace DetailCode = EvaluationRequirementResult::DetailCode;

namespace
{

ReportTableDefinition definition()
{
    auto def = ReportTableDefinition::record("test_table", "Test", { "CAT062" });
    def.addColumn("check_passed", PropertyDataType::BOOL, "Check Passed");
    def.addColumn("pos_inside", PropertyDataType::BOOL, "Position Inside");
    def.addColumn("ref_exists", PropertyDataType::BOOL, "Reference Exists");
    def.addColumn(ReportTableDefinition::DetailCodeColumnName, PropertyDataType::UINT, "Detail Code");

    DetailCode::addCommonTexts(def);
    DetailCode::addTypeText(def, DetailCode::typeBit(1), "Type reason");

    return def;
}

/// one row with the given flags, none = null
struct Row
{
    boost::optional<bool>         check_passed;
    boost::optional<bool>         pos_inside;
    boost::optional<bool>         ref_exists;
    boost::optional<unsigned int> code;
};

std::string comment(const Row& r)
{
    const auto def = definition();
    Buffer buffer(def.propertyList());

    // a set value creates the row, so the buffer has one row in any case
    buffer.get<unsigned int>(ReportTableDefinition::UTNColumnName).set(0, 1);

    if (r.check_passed) buffer.get<bool>("check_passed").set(0, *r.check_passed);
    if (r.pos_inside)   buffer.get<bool>("pos_inside").set(0, *r.pos_inside);
    if (r.ref_exists)   buffer.get<bool>("ref_exists").set(0, *r.ref_exists);
    if (r.code)         buffer.get<unsigned int>(ReportTableDefinition::DetailCodeColumnName).set(0, *r.code);

    return ReportTableContent::rowComment(def, buffer, 0);
}

} // namespace

TEST_CASE("ReportTableContent rowComment with detail codes", "[report_table_content]")
{
    SECTION("no code: the flag texts as before")
    {
        CHECK(comment({ false, {}, {}, {} }) == "Check failed");
        CHECK(comment({ {}, false, {}, {} }) == "Outside sector");
        CHECK(comment({ {}, {}, false, {} }) == "No reference");
        CHECK(comment({ true, true, true, {} }) == "");
    }

    SECTION("the code texts come first")
    {
        CHECK(comment({ {}, true, true, DetailCode::RefInaccurate }) == "Inaccurate reference position");
        CHECK(comment({ false, true, true, DetailCode::typeBit(1) }) == "Type reason, Check failed");
    }

    SECTION("a flag text the code already says is left out")
    {
        CHECK(comment({ {}, false, true, DetailCode::OutsideSector }) == "Outside sector");
        CHECK(comment({ {}, {}, false, DetailCode::NoRefData }) == "No reference data");
        CHECK(comment({ {}, {}, false, DetailCode::NoRefSpeed }) == "No reference speed");
    }

    SECTION("a code of 0 gives no text")
    {
        CHECK(comment({ true, true, true, 0u }) == "");
    }

    SECTION("the code column is not shown as a column of the details table")
    {
        const auto headers = ReportTableContent::detailsTableHeaders(definition());

        CHECK(std::find(headers.begin(), headers.end(), "Detail Code") == headers.end());
        CHECK(headers.back() == ReportTableContent::ColumnCommentHeader);
    }
}
