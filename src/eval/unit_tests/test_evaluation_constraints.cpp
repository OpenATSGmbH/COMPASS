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

#include "eval/evaluationconstraints.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

using boost::posix_time::ptime;
using boost::posix_time::seconds;
using boost::posix_time::time_from_string;

namespace
{
ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }

/// UTN 5 not used, UTN 7 with an excluded window 100-200 s, UTN 9 without constraints
EvaluationConstraints makeConstraints(bool global_filter)
{
    dbContent::TargetEvalConstraints not_used;
    not_used.use_in_eval_ = false;

    dbContent::TargetEvalConstraints with_window;
    with_window.excluded_time_windows_.add(Utils::TimeWindow(t0() + seconds(100), t0() + seconds(200), "test"));
    with_window.excluded_requirements_.insert("PD");

    nlohmann::json targets = nlohmann::json::object();
    targets[ "5" ] = not_used.toJSON();
    targets[ "7" ] = with_window.toJSON();

    // global time window 0-1000 s, global exclusion window 500-600 s
    Utils::TimeWindowCollection global_excl;
    global_excl.add(Utils::TimeWindow(t0() + seconds(500), t0() + seconds(600)));

    EvaluationConstraints c;
    c.globalTimeFilterEnabled(global_filter);
    c.globalTimeWindowFromJSON(Utils::TimeWindow(t0(), t0() + seconds(1000)).getAsJson());
    c.globalExclusionTimeWindowsFromJSON(global_excl.asJSON());
    c.targetConstraintsFromJSON(targets);

    return c;
}
}

TEST_CASE("EvaluationConstraints - empty snapshot excludes nothing", "[eval_constraints]")
{
    EvaluationConstraints c;

    REQUIRE(c.useTarget(1));
    REQUIRE(c.isTimeStampNotExcluded(t0()));
    REQUIRE(c.isTimeStampNotExcluded(1, t0()));
    REQUIRE(c.targetConstraint(1) == nullptr);
    REQUIRE(c.numExcludedTargets() == 0);
}

TEST_CASE("EvaluationConstraints - target use flag", "[eval_constraints]")
{
    auto c = makeConstraints(false);

    REQUIRE_FALSE(c.useTarget(5));
    REQUIRE(c.useTarget(7));
    REQUIRE(c.useTarget(9));   // no constraint
    REQUIRE(c.numExcludedTargets() == 1);
}

TEST_CASE("EvaluationConstraints - per target time windows", "[eval_constraints]")
{
    auto c = makeConstraints(false);

    REQUIRE(c.isTimeStampNotExcluded(7, t0() + seconds(50)));
    REQUIRE_FALSE(c.isTimeStampNotExcluded(7, t0() + seconds(150)));
    REQUIRE(c.isTimeStampNotExcluded(7, t0() + seconds(250)));

    // the window of UTN 7 does not apply to other targets or to the global rule
    REQUIRE(c.isTimeStampNotExcluded(9, t0() + seconds(150)));
    REQUIRE(c.isTimeStampNotExcluded(t0() + seconds(150)));
}

TEST_CASE("EvaluationConstraints - global time filter", "[eval_constraints]")
{
    SECTION("off: global window and exclusion windows not applied")
    {
        auto c = makeConstraints(false);

        REQUIRE(c.isTimeStampNotExcluded(t0() + seconds(550)));
        REQUIRE(c.isTimeStampNotExcluded(t0() + seconds(2000)));
        REQUIRE(c.isTimeStampNotExcluded(9, t0() + seconds(550)));
    }

    SECTION("on: outside the global window and inside an exclusion window excluded")
    {
        auto c = makeConstraints(true);

        REQUIRE(c.isTimeStampNotExcluded(t0() + seconds(50)));
        REQUIRE_FALSE(c.isTimeStampNotExcluded(t0() + seconds(550)));
        REQUIRE_FALSE(c.isTimeStampNotExcluded(t0() + seconds(2000)));

        // the global rule applies to every target
        REQUIRE_FALSE(c.isTimeStampNotExcluded(9, t0() + seconds(550)));
        REQUIRE_FALSE(c.isTimeStampNotExcluded(7, t0() + seconds(150)));
        REQUIRE(c.isTimeStampNotExcluded(7, t0() + seconds(50)));
    }
}

TEST_CASE("EvaluationConstraints - JSON round trip", "[eval_constraints]")
{
    auto c = makeConstraints(true);

    EvaluationConstraints c2;
    c2.globalTimeFilterEnabled(c.globalTimeFilterEnabled());
    c2.globalTimeWindowFromJSON(c.globalTimeWindowAsJSON());
    c2.globalExclusionTimeWindowsFromJSON(c.globalExclusionTimeWindowsAsJSON());
    c2.targetConstraintsFromJSON(c.targetConstraintsAsJSON());

    REQUIRE(c2.targetConstraints().size() == 2);
    REQUIRE(c2.globalExclusionTimeWindows().size() == 1);
    REQUIRE(c2.globalTimeWindow().valid());

    REQUIRE_FALSE(c2.useTarget(5));
    REQUIRE_FALSE(c2.isTimeStampNotExcluded(7, t0() + seconds(150)));
    REQUIRE_FALSE(c2.isTimeStampNotExcluded(t0() + seconds(550)));
    REQUIRE(c2.isTimeStampNotExcluded(9, t0() + seconds(50)));

    REQUIRE(c2.targetConstraint(7) != nullptr);
    REQUIRE(c2.targetConstraint(7)->excluded_requirements_.count("PD") == 1);

    // an empty snapshot gives a null global window
    EvaluationConstraints empty;
    REQUIRE(empty.globalTimeWindowAsJSON().is_null());
    REQUIRE(empty.targetConstraintsAsJSON().empty());
}

TEST_CASE("EvaluationConstraints - clear", "[eval_constraints]")
{
    auto c = makeConstraints(true);
    c.clear();

    REQUIRE_FALSE(c.globalTimeFilterEnabled());
    REQUIRE(c.targetConstraints().empty());
    REQUIRE(c.useTarget(5));
    REQUIRE(c.isTimeStampNotExcluded(t0() + seconds(550)));
}
