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

#include "requirement/position/positionhelpers.h"
#include "results/base/detailcode.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional/optional_io.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::PositionHelpers;

using boost::posix_time::ptime;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;
using dbContent::TargetPosition;
using dbContent::TargetVelocity;
using dbContent::TargetReport::DataID;
using dbContent::TargetReport::Index;
using dbContent::TargetReport::IndexMap;

namespace DetailCode = EvaluationRequirementResult::DetailCode;

namespace
{

ptime at(double t_s)
{
    return time_from_string("2026-01-01 12:00:00.000") + milliseconds((long)std::llround(t_s * 1000.0));
}

const double RefLat = 47.0;
const double RefLon = 16.0;

// length of one degree of latitude at RefLat [m]
const double MetersPerDegLat = 111170.0;

/// a test report and the reference data the evaluation maps to it
struct Report
{
    double                  t_s           = 0.0;
    double                  error_m       = 0.0;  // test position north of the reference position
    bool                    has_ref_data  = true;
    bool                    has_ref_pos   = true;
    bool                    has_ref_speed = true;
    double                  ref_track_deg = 0.0;
    bool                    inside        = true;
    boost::optional<double> ref_acc;
};

TargetPosition refPos()
{
    return TargetPosition(RefLat, RefLon, false, false, 0.0f);
}

TargetPosition tstPos(const Report& report)
{
    return TargetPosition(RefLat + report.error_m / MetersPerDegLat, RefLon, false, false, 0.0f);
}

ReportSettings reportSettings(bool skip_no_data_details = false)
{
    ReportSettings s;
    s.ref_min_accuracy     = 10.0;
    s.skip_no_data_details = skip_no_data_details;
    return s;
}

DistanceSettings distanceSettings(double threshold, COMPARISON_TYPE check_type = COMPARISON_TYPE::LESS_THAN_OR_EQUAL)
{
    DistanceSettings s;
    s.threshold  = threshold;
    s.check_type = check_type;
    return s;
}

using Evaluate = std::function<PositionEvaluation(const IndexMap&, const TstPosFunc&, const ReferenceLookup&)>;

Evaluate distances(const DistanceSettings& settings, const ReportSettings& report_settings = reportSettings())
{
    return [ = ] (const IndexMap& tst, const TstPosFunc& tst_pos, const ReferenceLookup& ref)
    {
        return evaluateDistances(tst, tst_pos, ref, report_settings, settings);
    };
}

Evaluate distancesRMS(double threshold)
{
    return [ = ] (const IndexMap& tst, const TstPosFunc& tst_pos, const ReferenceLookup& ref)
    {
        return evaluateDistancesRMS(tst, tst_pos, ref, reportSettings(), threshold);
    };
}

Evaluate trackOffsets(TrackOffsetComponent component, double max_abs_value)
{
    return [ = ] (const IndexMap& tst, const TstPosFunc& tst_pos, const ReferenceLookup& ref)
    {
        return evaluateTrackOffsets(tst, tst_pos, ref, reportSettings(), component, max_abs_value);
    };
}

/// the reference lookups of each report, in call order, as "<report> <lookup>"
using Calls = std::vector<std::string>;

PositionEvaluation run(const std::vector<Report>& reports, const Evaluate& evaluate,
                       bool with_speed = false, Calls* calls = nullptr)
{
    IndexMap tst_indexes;

    for (unsigned int i = 0; i < reports.size(); ++i)
        tst_indexes.insert({ at(reports[ i ].t_s), Index(i, i) });

    auto report = [ & ] (const DataID& id, const std::string& lookup) -> const Report&
    {
        if (calls)
            calls->push_back(std::to_string(id.index().idx_internal) + " " + lookup);

        return reports.at(id.index().idx_internal);
    };

    ReferenceLookup ref;
    ref.has_ref_data   = [ & ] (const DataID& id) { return report(id, "data").has_ref_data; };
    ref.ref_pos        = [ & ] (const DataID& id)
    {
        return report(id, "pos").has_ref_pos ? boost::optional<TargetPosition>(refPos()) : boost::none;
    };
    ref.ref_pos_inside = [ & ] (const DataID& id) { return report(id, "inside").inside; };
    ref.ref_min_acc    = [ & ] (const DataID& id) { return report(id, "acc").ref_acc; };

    if (with_speed)
        ref.ref_speed  = [ & ] (const DataID& id) -> boost::optional<TargetVelocity>
        {
            const Report& r = report(id, "speed");

            if (!r.has_ref_speed)
                return boost::none;

            TargetVelocity spd;
            spd.track_angle_ = r.ref_track_deg;
            spd.speed_       = 100.0;
            return spd;
        };

    return evaluate(tst_indexes,
                    [ & ] (const DataID& id) { return tstPos(reports.at(id.index().idx_internal)); },
                    ref);
}

/// what the report table shows of a detail
std::string kindOf(const PositionDetail& detail)
{
    if (detail.code & DetailCode::NoRef)
        return "no ref";
    if (detail.pos_inside == false)
        return "outside";
    if (!detail.check_passed.has_value())
        return "no comparison";

    return *detail.check_passed ? "passed" : "failed";
}

std::vector<std::string> kindsOf(const PositionEvaluation& evaluation)
{
    std::vector<std::string> kinds;

    for (const auto& detail : evaluation.details)
        kinds.push_back(kindOf(detail));

    return kinds;
}

} // namespace

TEST_CASE("Position Distance classifies each test report", "[eval][position_reports]")
{
    Report no_ref_data;    no_ref_data.t_s = 0;  no_ref_data.has_ref_data = false;
    Report no_ref_pos;     no_ref_pos.t_s = 1;   no_ref_pos.has_ref_pos = false;
    Report outside;        outside.t_s = 2;      outside.inside = false;
    Report inaccurate;     inaccurate.t_s = 3;   inaccurate.error_m = 5;  inaccurate.ref_acc = 10.5;
    Report acc_at_limit;   acc_at_limit.t_s = 4; acc_at_limit.error_m = 5;  acc_at_limit.ref_acc = 10.0;
    Report no_acc_failed;  no_acc_failed.t_s = 5; no_acc_failed.error_m = 50;
    Report accurate;       accurate.t_s = 6;     accurate.error_m = 25;  accurate.ref_acc = 2.0;

    const std::vector<Report> reports {no_ref_data, no_ref_pos, outside, inaccurate, acc_at_limit, no_acc_failed, accurate};

    SECTION("counts")
    {
        const auto counts = run(reports, distances(distanceSettings(30.0))).counts;

        CHECK(counts.num_pos == 7);
        CHECK(counts.num_no_ref == 2);
        CHECK(counts.num_pos_outside == 1);
        CHECK(counts.num_pos_inside == 4); // the inaccurate reference report counts as inside
        CHECK(counts.num_ref_inaccurate == 1);
        CHECK(counts.num_pos_calc_errors == 0);
        CHECK(counts.num_comp_passed == 2);
        CHECK(counts.num_comp_failed == 1);
    }

    SECTION("one detail per report, in time order")
    {
        const auto evaluation = run(reports, distances(distanceSettings(30.0)));

        REQUIRE(kindsOf(evaluation) == std::vector<std::string>{
                    "no ref", "no ref", "outside", "no comparison", "passed", "failed", "passed"});

        for (unsigned int i = 0; i < reports.size(); ++i)
        {
            CHECK(evaluation.details[ i ].timestamp == at(reports[ i ].t_s));
            CHECK(evaluation.details[ i ].tst_pos.latitude_ == tstPos(reports[ i ]).latitude_);
        }
    }

    SECTION("detail values")
    {
        const auto& d = run(reports, distances(distanceSettings(30.0))).details;

        // the reason of each report as detail code bit, none for the compared ones
        CHECK(d[ 0 ].code == DetailCode::NoRefData);
        CHECK(d[ 1 ].code == DetailCode::NoRefPosition);
        CHECK(d[ 2 ].code == 0); // pos_inside false says it
        CHECK(d[ 3 ].code == DetailCode::RefInaccurate);
        CHECK(d[ 4 ].code == 0);
        CHECK(d[ 5 ].code == 0);
        CHECK(d[ 6 ].code == 0);

        // no reference: no reference position, no inside (comment "No reference data", not "Outside sector")
        for (unsigned int i : {0, 1})
        {
            CHECK(!d[ i ].ref_pos.has_value());
            CHECK(!d[ i ].pos_inside);
        }

        // outside the sector, inaccurate reference: reference position and inside
        CHECK(d[ 2 ].ref_pos.has_value());
        CHECK(d[ 2 ].pos_inside == false);
        CHECK(d[ 3 ].ref_pos.has_value());
        CHECK(d[ 3 ].pos_inside == true);

        // no comparison: no value, not passed
        for (unsigned int i : {0, 1, 2, 3})
        {
            CHECK(!d[ i ].value);
            CHECK(!d[ i ].check_passed.has_value());
        }

        for (unsigned int i : {4, 5, 6})
        {
            REQUIRE(d[ i ].ref_pos.has_value());
            CHECK(d[ i ].ref_pos->latitude_ == RefLat);
            CHECK(d[ i ].pos_inside == true);
            REQUIRE(d[ i ].value);
            CHECK(*d[ i ].value == Approx(reports[ i ].error_m).epsilon(0.005));
        }

        CHECK(d[ 4 ].check_passed == true);
        CHECK(d[ 5 ].check_passed == false);
        CHECK(d[ 6 ].check_passed == true);
    }

    SECTION("skipped no-data details: only the details of the comparisons, same counts")
    {
        const auto evaluation = run(reports, distances(distanceSettings(30.0), reportSettings(true)));
        const auto all = run(reports, distances(distanceSettings(30.0)));

        CHECK(kindsOf(evaluation) == std::vector<std::string>{"passed", "failed", "passed"});
        CHECK(evaluation.counts.num_no_ref == all.counts.num_no_ref);
        CHECK(evaluation.counts.num_pos_outside == all.counts.num_pos_outside);
        CHECK(evaluation.counts.num_ref_inaccurate == all.counts.num_ref_inaccurate);
        CHECK(evaluation.counts.num_comp_passed == all.counts.num_comp_passed);
    }

    SECTION("no test reports: nothing counted")
    {
        const auto evaluation = run({}, distances(distanceSettings(30.0)));

        CHECK(evaluation.counts.num_pos == 0);
        CHECK(evaluation.details.empty());
    }
}

TEST_CASE("Reference data is read only as far as needed", "[eval][position_reports]")
{
    Report no_ref_data;   no_ref_data.has_ref_data = false;
    Report no_ref_pos;    no_ref_pos.t_s = 1;    no_ref_pos.has_ref_pos = false;
    Report no_ref_speed;  no_ref_speed.t_s = 2;  no_ref_speed.has_ref_speed = false;
    Report outside;       outside.t_s = 3;       outside.inside = false;
    Report compared;      compared.t_s = 4;

    const std::vector<Report> reports {no_ref_data, no_ref_pos, no_ref_speed, outside, compared};

    SECTION("without the reference speed (Position Distance)")
    {
        Calls calls;
        run(reports, distances(distanceSettings(30.0)), false, &calls);

        CHECK(calls == Calls{"0 data",
                             "1 data", "1 pos",
                             "2 data", "2 pos", "2 inside", "2 acc",
                             "3 data", "3 pos", "3 inside",
                             "4 data", "4 pos", "4 inside", "4 acc"});
    }

    SECTION("with the reference speed (Position Along): a missing speed counts as no reference")
    {
        Calls calls;
        const auto evaluation = run(reports, trackOffsets(TrackOffsetComponent::Along, 30.0), true, &calls);

        CHECK(calls == Calls{"0 data",
                             "1 data", "1 pos",
                             "2 data", "2 pos", "2 speed",
                             "3 data", "3 pos", "3 speed", "3 inside",
                             "4 data", "4 pos", "4 speed", "4 inside", "4 acc"});

        CHECK(evaluation.counts.num_no_ref == 3);
        CHECK(evaluation.counts.num_pos_outside == 1);
        CHECK(evaluation.counts.num_comp_passed == 1);
        CHECK(kindsOf(evaluation)[ 2 ] == "no ref");
    }
}

TEST_CASE("Details of the test reports without a comparison are the same for all requirements", "[eval][position_reports]")
{
    Report no_ref_data;   no_ref_data.has_ref_data = false;
    Report no_ref_speed;  no_ref_speed.t_s = 1;  no_ref_speed.has_ref_speed = false;
    Report outside;       outside.t_s = 2;       outside.inside = false;
    Report inaccurate;    inaccurate.t_s = 3;    inaccurate.ref_acc = 20.0;

    const std::vector<Report> reports {no_ref_data, no_ref_speed, outside, inaccurate};

    const auto distance = run(reports, distances(distanceSettings(30.0)));
    const auto rms      = run(reports, distancesRMS(30.0));
    const auto along    = run(reports, trackOffsets(TrackOffsetComponent::Along, 30.0), true);
    const auto across   = run(reports, trackOffsets(TrackOffsetComponent::Across, 30.0), true);

    SECTION("no value, no check_passed (not an issue), reference position if known")
    {
        for (const auto* evaluation : {&distance, &rms, &along, &across})
        {
            for (unsigned int i : {0, 2, 3})
            {
                CHECK(!evaluation->details[ i ].value);
                CHECK(!evaluation->details[ i ].check_passed.has_value());
            }

            CHECK(!evaluation->details[ 0 ].ref_pos.has_value());
            CHECK(evaluation->details.back().ref_pos.has_value());
            CHECK(evaluation->details.back().pos_inside == true);
        }
    }

    SECTION("a missing reference speed counts only where the speed is needed")
    {
        CHECK(kindsOf(distance) == std::vector<std::string>{"no ref", "passed", "outside", "no comparison"});
        CHECK(kindsOf(along) == std::vector<std::string>{"no ref", "no ref", "outside", "no comparison"});

        CHECK(along.details[ 1 ].code == DetailCode::NoRefSpeed);
        REQUIRE(along.details[ 1 ].ref_pos.has_value());
        CHECK(!along.details[ 1 ].pos_inside);
    }

    SECTION("same counts")
    {
        for (const auto* evaluation : {&distance, &rms, &along, &across})
        {
            CHECK(evaluation->counts.num_pos_outside == 1);
            CHECK(evaluation->counts.num_ref_inaccurate == 1);
        }

        CHECK(distance.counts.num_no_ref == 1);
        CHECK(along.counts.num_no_ref == 2);
    }
}

TEST_CASE("Position Distance compares the distance with the threshold", "[eval][position_reports]")
{
    Report report;
    report.error_m = 25;

    CHECK(run({report}, distances(distanceSettings(30.0, COMPARISON_TYPE::LESS_THAN_OR_EQUAL))).counts.num_comp_passed == 1);
    CHECK(run({report}, distances(distanceSettings(20.0, COMPARISON_TYPE::LESS_THAN_OR_EQUAL))).counts.num_comp_failed == 1);
    CHECK(run({report}, distances(distanceSettings(20.0, COMPARISON_TYPE::GREATER_THAN))).counts.num_comp_passed == 1);

    // a test position south of the reference: the distance is positive
    report.error_m = -25;
    const auto evaluation = run({report}, distances(distanceSettings(30.0)));

    REQUIRE(evaluation.details.at(0).value);
    CHECK(*evaluation.details.at(0).value == Approx(25.0).epsilon(0.005));
    CHECK(evaluation.counts.num_comp_passed == 1);
}

TEST_CASE("Position Distance with averaging: one comparison per window", "[eval][position_reports]")
{
    auto s = distanceSettings(30.0);
    s.use_averaging      = true;
    s.averaging_window_s = 5.0;

    std::vector<Report> reports(5);
    reports[ 0 ].t_s = 0;  reports[ 0 ].error_m = 10;
    reports[ 1 ].t_s = 1;  reports[ 1 ].error_m = -10;
    reports[ 2 ].t_s = 2;  reports[ 2 ].error_m = 10;
    reports[ 3 ].t_s = 3;  reports[ 3 ].has_ref_data = false;
    reports[ 4 ].t_s = 20; reports[ 4 ].error_m = 40;

    const auto evaluation = run(reports, distances(s));

    SECTION("counts")
    {
        CHECK(evaluation.counts.num_pos == 5);
        CHECK(evaluation.counts.num_no_ref == 1);
        CHECK(evaluation.counts.num_pos_inside == 4);
        CHECK(evaluation.counts.num_comp_passed == 1);
        CHECK(evaluation.counts.num_comp_failed == 1);
    }

    SECTION("details: the reports without a comparison, then one per window")
    {
        REQUIRE(kindsOf(evaluation) == std::vector<std::string>{"no ref", "passed", "failed"});

        const auto& first_window = evaluation.details[ 1 ];

        CHECK(first_window.timestamp == at(0));
        CHECK(first_window.tst_pos.latitude_ == tstPos(reports[ 0 ]).latitude_);
        REQUIRE(first_window.ref_pos.has_value());
        CHECK(first_window.pos_inside == true);
        REQUIRE(first_window.value);
        // the errors north and south cancel out: mean error 10/3 m
        CHECK(*first_window.value == Approx(10.0 / 3.0).epsilon(0.005));

        const auto& second_window = evaluation.details[ 2 ];

        CHECK(second_window.timestamp == at(20));
        REQUIRE(second_window.value);
        CHECK(*second_window.value == Approx(40.0).epsilon(0.005));
    }
}

TEST_CASE("Position Distance RMS: passed if the distance is <= threshold", "[eval][position_reports]")
{
    Report near;  near.error_m = 20;
    Report far;   far.t_s = 1;  far.error_m = -40;

    const auto evaluation = run({near, far}, distancesRMS(30.0));

    REQUIRE(kindsOf(evaluation) == std::vector<std::string>{"passed", "failed"});
    CHECK(evaluation.counts.num_comp_passed == 1);
    CHECK(evaluation.counts.num_comp_failed == 1);

    REQUIRE(evaluation.details[ 1 ].value);
    CHECK(*evaluation.details[ 1 ].value == Approx(40.0).epsilon(0.005));
}

TEST_CASE("Position Along and Across: offset on the reference track", "[eval][position_reports]")
{
    // reference track north: a test position north of the reference is ahead
    Report ahead;    ahead.error_m = 10;
    Report behind;   behind.t_s = 1;  behind.error_m = -3;
    // reference track east: a test position north of the reference is left of the track
    Report left;     left.t_s = 2;    left.error_m = 10;  left.ref_track_deg = 90.0;

    const std::vector<Report> reports {ahead, behind, left};

    SECTION("along-track: signed value, passed if its absolute value is <= max")
    {
        const auto evaluation = run(reports, trackOffsets(TrackOffsetComponent::Along, 5.0), true);

        REQUIRE(kindsOf(evaluation) == std::vector<std::string>{"failed", "passed", "passed"});
        CHECK(*evaluation.details[ 0 ].value == Approx(10.0).epsilon(0.005));
        CHECK(*evaluation.details[ 1 ].value == Approx(-3.0).epsilon(0.005));
        CHECK(*evaluation.details[ 2 ].value == Approx(0.0).margin(0.01));
        CHECK(evaluation.counts.num_comp_failed == 1);
        CHECK(evaluation.counts.num_comp_passed == 2);
    }

    SECTION("across-track: positive right of the track")
    {
        const auto evaluation = run(reports, trackOffsets(TrackOffsetComponent::Across, 5.0), true);

        REQUIRE(kindsOf(evaluation) == std::vector<std::string>{"passed", "passed", "failed"});
        CHECK(*evaluation.details[ 0 ].value == Approx(0.0).margin(0.01));
        CHECK(*evaluation.details[ 2 ].value == Approx(-10.0).epsilon(0.005));
        CHECK(evaluation.details[ 2 ].check_passed == false);
    }

    SECTION("an invalid reference track angle: calculation error, no value")
    {
        Report no_track;
        no_track.ref_track_deg = std::numeric_limits<double>::quiet_NaN();

        const auto evaluation = run({no_track}, trackOffsets(TrackOffsetComponent::Along, 5.0), true);

        REQUIRE(kindsOf(evaluation) == std::vector<std::string>{"no comparison"});
        CHECK(evaluation.counts.num_pos_calc_errors == 1);
        CHECK(evaluation.details[ 0 ].code == DetailCode::ValueInvalid);
        CHECK(evaluation.counts.num_comp_passed + evaluation.counts.num_comp_failed == 0);
        CHECK(!evaluation.details[ 0 ].value);
        CHECK(!evaluation.details[ 0 ].check_passed.has_value());
        CHECK(evaluation.details[ 0 ].ref_pos.has_value());
        CHECK(evaluation.details[ 0 ].pos_inside == true);
    }
}
