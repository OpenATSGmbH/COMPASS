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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional/optional_io.hpp>

#include <cmath>
#include <string>
#include <vector>

using namespace EvaluationRequirement;
using namespace EvaluationRequirement::PositionHelpers;

using boost::posix_time::ptime;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;
using dbContent::TargetPosition;
using dbContent::TargetReport::DataID;
using dbContent::TargetReport::Index;
using dbContent::TargetReport::IndexMap;

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
    double                  t_s          = 0.0;
    double                  error_m      = 0.0;  // test position north of the reference position
    bool                    has_ref_data = true;
    bool                    has_ref_pos  = true;
    bool                    inside       = true;
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

DistanceSettings settings(double threshold, COMPARISON_TYPE check_type = COMPARISON_TYPE::LESS_THAN_OR_EQUAL)
{
    DistanceSettings s;
    s.threshold        = threshold;
    s.check_type       = check_type;
    s.ref_min_accuracy = 10.0;
    return s;
}

/// the reference lookups of each report, in call order, as "<report> <lookup>"
using Calls = std::vector<std::string>;

DistanceEvaluation run(const std::vector<Report>& reports, const DistanceSettings& settings, Calls* calls = nullptr)
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

    return evaluateDistances(tst_indexes,
                             [ & ] (const DataID& id) { return tstPos(reports.at(id.index().idx_internal)); },
                             ref, settings);
}

std::vector<std::string> commentsOf(const DistanceEvaluation& evaluation)
{
    std::vector<std::string> comments;

    for (const auto& detail : evaluation.details)
        comments.push_back(detail.comment);

    return comments;
}

} // namespace

TEST_CASE("evaluateDistances classifies each test report", "[eval][position_distance]")
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
        const auto counts = run(reports, settings(30.0)).counts;

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
        const auto evaluation = run(reports, settings(30.0));

        REQUIRE(commentsOf(evaluation) == std::vector<std::string>{
                    "No reference data", "No reference position", "Outside sector",
                    "Inaccurate reference position", "Passed", "Failed", "Passed"});

        for (unsigned int i = 0; i < reports.size(); ++i)
        {
            CHECK(evaluation.details[ i ].timestamp == at(reports[ i ].t_s));
            CHECK(evaluation.details[ i ].tst_pos.latitude_ == tstPos(reports[ i ]).latitude_);
        }
    }

    SECTION("detail values")
    {
        const auto& d = run(reports, settings(30.0)).details;

        for (unsigned int i : {0, 1, 3})
        {
            CHECK(!d[ i ].ref_pos.has_value());
            CHECK(!d[ i ].pos_inside);
            CHECK(!d[ i ].value);
            CHECK(!d[ i ].check_passed);
        }

        // outside the sector: reference position, but no distance
        CHECK(d[ 2 ].ref_pos.has_value());
        CHECK(d[ 2 ].pos_inside == false);
        CHECK(!d[ 2 ].value);

        for (unsigned int i : {4, 5, 6})
        {
            REQUIRE(d[ i ].ref_pos.has_value());
            CHECK(d[ i ].ref_pos->latitude_ == RefLat);
            CHECK(d[ i ].pos_inside == true);
            REQUIRE(d[ i ].value);
            CHECK(*d[ i ].value == Approx(reports[ i ].error_m).epsilon(0.005));
        }

        CHECK(d[ 4 ].check_passed);
        CHECK(!d[ 5 ].check_passed);
        CHECK(d[ 6 ].check_passed);
    }

    SECTION("skipped no-data details: only the details of the comparisons, same counts")
    {
        auto s = settings(30.0);
        s.skip_no_data_details = true;

        const auto evaluation = run(reports, s);
        const auto all = run(reports, settings(30.0));

        CHECK(commentsOf(evaluation) == std::vector<std::string>{"Passed", "Failed", "Passed"});
        CHECK(evaluation.counts.num_no_ref == all.counts.num_no_ref);
        CHECK(evaluation.counts.num_pos_outside == all.counts.num_pos_outside);
        CHECK(evaluation.counts.num_ref_inaccurate == all.counts.num_ref_inaccurate);
        CHECK(evaluation.counts.num_comp_passed == all.counts.num_comp_passed);
    }

    SECTION("no test reports: nothing counted")
    {
        const auto evaluation = run({}, settings(30.0));

        CHECK(evaluation.counts.num_pos == 0);
        CHECK(evaluation.details.empty());
    }
}

TEST_CASE("evaluateDistances reads the reference data only as far as needed", "[eval][position_distance]")
{
    Report no_ref_data;   no_ref_data.has_ref_data = false;
    Report no_ref_pos;    no_ref_pos.t_s = 1;  no_ref_pos.has_ref_pos = false;
    Report outside;       outside.t_s = 2;     outside.inside = false;
    Report compared;      compared.t_s = 3;

    Calls calls;
    run({no_ref_data, no_ref_pos, outside, compared}, settings(30.0), &calls);

    CHECK(calls == Calls{"0 data",
                         "1 data", "1 pos",
                         "2 data", "2 pos", "2 inside",
                         "3 data", "3 pos", "3 inside", "3 acc"});
}

TEST_CASE("evaluateDistances compares the distance with the threshold", "[eval][position_distance]")
{
    Report report;
    report.error_m = 25;

    CHECK(run({report}, settings(30.0, COMPARISON_TYPE::LESS_THAN_OR_EQUAL)).counts.num_comp_passed == 1);
    CHECK(run({report}, settings(20.0, COMPARISON_TYPE::LESS_THAN_OR_EQUAL)).counts.num_comp_failed == 1);
    CHECK(run({report}, settings(20.0, COMPARISON_TYPE::GREATER_THAN)).counts.num_comp_passed == 1);

    // a test position south of the reference: the distance is positive
    report.error_m = -25;
    const auto evaluation = run({report}, settings(30.0));

    REQUIRE(evaluation.details.at(0).value);
    CHECK(*evaluation.details.at(0).value == Approx(25.0).epsilon(0.005));
    CHECK(evaluation.counts.num_comp_passed == 1);
}

TEST_CASE("evaluateDistances with averaging: one comparison per window", "[eval][position_distance]")
{
    auto s = settings(30.0);
    s.use_averaging      = true;
    s.averaging_window_s = 5.0;

    std::vector<Report> reports(5);
    reports[ 0 ].t_s = 0;  reports[ 0 ].error_m = 10;
    reports[ 1 ].t_s = 1;  reports[ 1 ].error_m = -10;
    reports[ 2 ].t_s = 2;  reports[ 2 ].error_m = 10;
    reports[ 3 ].t_s = 3;  reports[ 3 ].has_ref_data = false;
    reports[ 4 ].t_s = 20; reports[ 4 ].error_m = 40;

    const auto evaluation = run(reports, s);

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
        REQUIRE(commentsOf(evaluation) == std::vector<std::string>{
                    "No reference data",
                    "Passed (mean position error of 3 reports over 2.0 s)",
                    "Failed (mean position error of 1 reports over 0.0 s)"});

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
