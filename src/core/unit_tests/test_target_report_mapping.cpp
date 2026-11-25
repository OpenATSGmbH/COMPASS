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
#include "projection/transformation.h"

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

Sample sample(double seconds, double latitude, double longitude)
{
    Sample s;
    s.timestamp = at(seconds);
    s.pos       = TargetPosition(latitude, longitude, false, false, 0.0f);
    return s;
}

TargetVelocity velocity(double speed_ms, double track_angle_deg)
{
    TargetVelocity v;
    v.speed_       = speed_ms;
    v.track_angle_ = track_angle_deg;
    return v;
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

TEST_CASE("interpolate", "[target_report_mapping]")
{
    Transformation trafo;

    // 0.01 deg is about 1.1 km north and 0.74 km east at 48 deg latitude, the straight
    // line in the local plane deviates from the geodesic by much less than 1e-6 deg
    const double deg_margin = 1e-6;

    SECTION("position at the midpoint of a north-south line")
    {
        auto s = interpolate(sample(0, 48.0, 16.0), sample(10, 48.01, 16.0), at(5), trafo);
        REQUIRE(s.has_value());
        REQUIRE(s->timestamp == at(5));
        REQUIRE(s->pos.latitude_ == Approx(48.005).margin(deg_margin));
        REQUIRE(s->pos.longitude_ == Approx(16.0).margin(deg_margin));
    }

    SECTION("position at a quarter of an east-west line")
    {
        auto s = interpolate(sample(0, 48.0, 16.0), sample(10, 48.0, 16.02), at(2.5), trafo);
        REQUIRE(s.has_value());
        REQUIRE(s->pos.latitude_ == Approx(48.0).margin(deg_margin));
        REQUIRE(s->pos.longitude_ == Approx(16.005).margin(deg_margin));
    }

    SECTION("at the times of the samples")
    {
        auto s1 = interpolate(sample(0, 48.0, 16.0), sample(10, 48.01, 16.01), at(0), trafo);
        REQUIRE(s1.has_value());
        REQUIRE(s1->pos.latitude_ == Approx(48.0).margin(deg_margin));
        REQUIRE(s1->pos.longitude_ == Approx(16.0).margin(deg_margin));

        auto s2 = interpolate(sample(0, 48.0, 16.0), sample(10, 48.01, 16.01), at(10), trafo);
        REQUIRE(s2.has_value());
        REQUIRE(s2->pos.latitude_ == Approx(48.01).margin(deg_margin));
        REQUIRE(s2->pos.longitude_ == Approx(16.01).margin(deg_margin));
    }

    SECTION("altitude")
    {
        Sample s1 = sample(0, 48.0, 16.0);
        Sample s2 = sample(10, 48.01, 16.0);

        SECTION("both: linear, marked as calculated")
        {
            s1.pos.has_altitude_ = true;
            s1.pos.altitude_     = 1000;
            s2.pos.has_altitude_ = true;
            s2.pos.altitude_     = 2000;

            auto s = interpolate(s1, s2, at(5), trafo);
            REQUIRE(s.has_value());
            REQUIRE(s->pos.has_altitude_);
            REQUIRE(s->pos.altitude_calculated_);
            REQUIRE(s->pos.altitude_ == Approx(1500));
        }

        SECTION("only the first: its altitude")
        {
            s1.pos.has_altitude_ = true;
            s1.pos.altitude_     = 1000;

            auto s = interpolate(s1, s2, at(5), trafo);
            REQUIRE(s->pos.has_altitude_);
            REQUIRE(s->pos.altitude_ == Approx(1000));
        }

        SECTION("only the second: its altitude")
        {
            s2.pos.has_altitude_ = true;
            s2.pos.altitude_     = 2000;

            auto s = interpolate(s1, s2, at(5), trafo);
            REQUIRE(s->pos.has_altitude_);
            REQUIRE(s->pos.altitude_ == Approx(2000));
        }

        SECTION("none")
        {
            auto s = interpolate(s1, s2, at(5), trafo);
            REQUIRE(!s->pos.has_altitude_);
        }
    }

    SECTION("speed and track angle")
    {
        Sample s1 = sample(0, 48.0, 16.0);
        Sample s2 = sample(10, 48.01, 16.0);

        SECTION("both: linear speed, track angle across north")
        {
            s1.speed = velocity(100, 350);
            s2.speed = velocity(120, 10);

            auto mid = interpolate(s1, s2, at(5), trafo);
            REQUIRE(mid->speed.has_value());
            REQUIRE(mid->speed->speed_ == Approx(110));
            REQUIRE(mid->speed->track_angle_ == Approx(0.0).margin(1e-9));

            auto quarter = interpolate(s1, s2, at(2.5), trafo);
            REQUIRE(quarter->speed->speed_ == Approx(105));
            REQUIRE(quarter->speed->track_angle_ == Approx(355));
        }

        SECTION("only one: no speed")
        {
            s1.speed = velocity(100, 350);

            auto s = interpolate(s1, s2, at(5), trafo);
            REQUIRE(s.has_value());
            REQUIRE(!s->speed);
        }
    }

    SECTION("acceleration and rate of climb/descent")
    {
        Sample s1 = sample(0, 48.0, 16.0);
        Sample s2 = sample(10, 48.01, 16.0);

        s1.acceleration = 1.0;
        s2.acceleration = 3.0;
        s1.rocd         = 500.0;
        s2.rocd         = 1500.0;

        auto s = interpolate(s1, s2, at(5), trafo);
        REQUIRE(s->acceleration.has_value());
        REQUIRE(*s->acceleration == Approx(2.0));
        REQUIRE(s->rocd.has_value());
        REQUIRE(*s->rocd == Approx(1000.0));

        s2.rocd.reset();
        REQUIRE(!interpolate(s1, s2, at(5), trafo)->rocd);
    }

    SECTION("same position: first sample's position, no speed, acceleration or rocd")
    {
        Sample s1 = sample(0, 48.0, 16.0);
        Sample s2 = sample(10, 48.0, 16.0);

        s1.pos.has_altitude_ = true;
        s1.pos.altitude_     = 1000;
        s2.pos.has_altitude_ = true;
        s2.pos.altitude_     = 2000;
        s1.speed = velocity(0, 90);
        s2.speed = velocity(0, 90);
        s1.acceleration = s2.acceleration = 0.0;
        s1.rocd = s2.rocd = 0.0;

        auto s = interpolate(s1, s2, at(5), trafo);
        REQUIRE(s.has_value());
        REQUIRE(s->pos.latitude_ == 48.0);
        REQUIRE(s->pos.longitude_ == 16.0);
        REQUIRE(s->pos.altitude_ == Approx(1000));
        REQUIRE(!s->pos.altitude_calculated_);
        REQUIRE(!s->speed);
        REQUIRE(!s->acceleration);
        REQUIRE(!s->rocd);
    }
}
