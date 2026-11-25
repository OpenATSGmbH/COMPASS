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

#include "dbcontent/target/targetreportmapping.h"

#include <cmath>
#include <vector>

using namespace dbContent;
using namespace dbContent::TargetReport;
using namespace boost::posix_time;

namespace
{

const ptime T0(boost::gregorian::date(2026, 1, 1), hours(12));

ptime at(double seconds)
{
    return T0 + milliseconds(std::llround(seconds * 1000.0));
}

// internal index = position in the list, external index = 100 + internal index
IndexMap makeIndexes(const std::vector<double>& seconds)
{
    IndexMap indexes;

    for (unsigned int i = 0; i < seconds.size(); ++i)
        indexes.insert({at(seconds[i]), Index(100 + i, i)});

    return indexes;
}

void requireEntry(const boost::optional<DataID>& id, double seconds, unsigned int idx_internal)
{
    REQUIRE(id.has_value());
    REQUIRE(id->timestamp() == at(seconds));
    REQUIRE(id->index().idx_internal == idx_internal);
    REQUIRE(id->index().idx_external == 100 + idx_internal);
}

} // namespace

TEST_CASE("findBracket", "[target_report_mapping]")
{
    SECTION("empty collection")
    {
        Bracket b = findBracket(IndexMap(), at(10));
        REQUIRE(!b.before);
        REQUIRE(!b.exact);
        REQUIRE(!b.after);
    }

    IndexMap indexes = makeIndexes({10, 20, 30});

    SECTION("before the first entry")
    {
        Bracket b = findBracket(indexes, at(5));
        REQUIRE(!b.before);
        REQUIRE(!b.exact);
        requireEntry(b.after, 10, 0);
    }

    SECTION("after the last entry")
    {
        Bracket b = findBracket(indexes, at(35));
        requireEntry(b.before, 30, 2);
        REQUIRE(!b.exact);
        REQUIRE(!b.after);
    }

    SECTION("between two entries")
    {
        Bracket b = findBracket(indexes, at(15));
        requireEntry(b.before, 10, 0);
        REQUIRE(!b.exact);
        requireEntry(b.after, 20, 1);
    }

    SECTION("exact first entry")
    {
        Bracket b = findBracket(indexes, at(10));
        REQUIRE(!b.before);
        requireEntry(b.exact, 10, 0);
        requireEntry(b.after, 20, 1);
    }

    SECTION("exact middle entry")
    {
        Bracket b = findBracket(indexes, at(20));
        requireEntry(b.before, 10, 0);
        requireEntry(b.exact, 20, 1);
        requireEntry(b.after, 30, 2);
    }

    SECTION("exact last entry")
    {
        Bracket b = findBracket(indexes, at(30));
        requireEntry(b.before, 20, 1);
        requireEntry(b.exact, 30, 2);
        REQUIRE(!b.after);
    }

    SECTION("several entries at one timestamp: last before, first exact and after")
    {
        IndexMap dup = makeIndexes({10, 10, 20, 20, 30, 30});

        Bracket between = findBracket(dup, at(15));
        requireEntry(between.before, 10, 1);
        requireEntry(between.after, 20, 2);

        Bracket exact = findBracket(dup, at(20));
        requireEntry(exact.before, 10, 1);
        requireEntry(exact.exact, 20, 2);
        requireEntry(exact.after, 30, 4);
    }
}

TEST_CASE("findDataMapping", "[target_report_mapping]")
{
    IndexMap indexes = makeIndexes({10, 20, 30});

    SECTION("exact entry is ref1 only")
    {
        DataMapping m = findDataMapping(indexes, at(20));
        REQUIRE(m.timestamp_ == at(20));
        REQUIRE(m.has_ref1_);
        REQUIRE(m.timestamp_ref1_ == at(20));
        requireEntry(m.dataid_ref1_, 20, 1);
        REQUIRE(!m.has_ref2_);
    }

    SECTION("between two entries: ref1 before, ref2 after")
    {
        DataMapping m = findDataMapping(indexes, at(25));
        REQUIRE(m.has_ref1_);
        REQUIRE(m.timestamp_ref1_ == at(20));
        requireEntry(m.dataid_ref1_, 20, 1);
        REQUIRE(m.has_ref2_);
        REQUIRE(m.timestamp_ref2_ == at(30));
        requireEntry(m.dataid_ref2_, 30, 2);
    }

    SECTION("outside the collection or empty: no reference")
    {
        for (const auto& m : {findDataMapping(indexes, at(5)),
                              findDataMapping(indexes, at(35)),
                              findDataMapping(IndexMap(), at(20))})
        {
            REQUIRE(!m.has_ref1_);
            REQUIRE(!m.has_ref2_);
        }
    }

    SECTION("no position or speed is set")
    {
        DataMapping m = findDataMapping(indexes, at(25));
        REQUIRE(!m.has_ref_pos_);
        REQUIRE(!m.has_ref_spd_);
        REQUIRE(!m.has_ref_acc_);
        REQUIRE(!m.has_ref_rocd_);
    }
}

TEST_CASE("findDataMappingTimes", "[target_report_mapping]")
{
    IndexMap indexes = makeIndexes({10, 20, 30});

    SECTION("between two entries")
    {
        DataMappingTimes m = findDataMappingTimes(indexes, at(15));
        REQUIRE(m.timestamp_ == at(15));
        REQUIRE(m.has_other1_);
        REQUIRE(m.timestamp_other1_ == at(10));
        requireEntry(m.dataid_other1_, 10, 0);
        REQUIRE(m.has_other2_);
        REQUIRE(m.timestamp_other2_ == at(20));
        requireEntry(m.dataid_other2_, 20, 1);
    }

    SECTION("exact middle entry: other1 before, other2 exact")
    {
        DataMappingTimes m = findDataMappingTimes(indexes, at(20));
        REQUIRE(m.has_other1_);
        requireEntry(m.dataid_other1_, 10, 0);
        REQUIRE(m.has_other2_);
        requireEntry(m.dataid_other2_, 20, 1);
    }

    SECTION("exact first entry: other2 exact, no other1")
    {
        DataMappingTimes m = findDataMappingTimes(indexes, at(10));
        REQUIRE(!m.has_other1_);
        REQUIRE(m.has_other2_);
        REQUIRE(m.timestamp_other2_ == at(10));
        requireEntry(m.dataid_other2_, 10, 0);
    }

    SECTION("exact last entry")
    {
        DataMappingTimes m = findDataMappingTimes(indexes, at(30));
        REQUIRE(m.has_other1_);
        requireEntry(m.dataid_other1_, 20, 1);
        REQUIRE(m.has_other2_);
        requireEntry(m.dataid_other2_, 30, 2);
    }

    SECTION("only one neighbor and no exact entry: none")
    {
        for (const auto& m : {findDataMappingTimes(indexes, at(5)),
                              findDataMappingTimes(indexes, at(35)),
                              findDataMappingTimes(IndexMap(), at(20))})
        {
            REQUIRE(!m.has_other1_);
            REQUIRE(!m.has_other2_);
        }
    }
}
