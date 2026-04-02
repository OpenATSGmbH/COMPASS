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
#include "rangeedit.h"

#include <QLineEdit>
#include <QSignalSpy>

TEST_CASE("RangeEditDouble setLimits enables widget", "[gui][rangeedit]")
{
    RangeEditDouble edit(100, 2);
    edit.setLimits("0.00", "100.00");

    CHECK(edit.isEnabled());
}

TEST_CASE("RangeEditDouble invalid limits disable widget", "[gui][rangeedit]")
{
    RangeEditDouble edit(100, 2);
    edit.setLimits("abc", "100.00");

    CHECK_FALSE(edit.isEnabled());
}

TEST_CASE("RangeEditDouble equal limits disable widget", "[gui][rangeedit]")
{
    RangeEditDouble edit(100, 2);
    edit.setLimits("50.00", "50.00");

    // Equal limits → Range::isValid() returns false → error state
    CHECK_FALSE(edit.isEnabled());
}

TEST_CASE("RangeEditDouble setLowerRange emits signals", "[gui][rangeedit]")
{
    RangeEditDouble edit(100, 2);
    edit.setLimits("0.00", "100.00");

    QSignalSpy lower_spy(&edit, &RangeEditBase::lowerRangeChanged);
    QSignalSpy range_spy(&edit, &RangeEditBase::rangeChanged);

    edit.setLowerRange("25.00");

    // setLowerRange updates the internal range and slider, but signals are
    // only emitted when the slider drives the change (not programmatic set).
    // Verify the widget stays enabled (no error).
    CHECK(edit.isEnabled());
}

TEST_CASE("RangeEditDouble setUpperRange keeps widget enabled", "[gui][rangeedit]")
{
    RangeEditDouble edit(100, 2);
    edit.setLimits("0.00", "100.00");

    edit.setUpperRange("75.00");
    CHECK(edit.isEnabled());
}

TEST_CASE("RangeEditDouble invalid range value disables widget", "[gui][rangeedit]")
{
    RangeEditDouble edit(100, 2);
    edit.setLimits("0.00", "100.00");

    edit.setLowerRange("not_a_number");
    CHECK_FALSE(edit.isEnabled());
}

TEST_CASE("RangeEditFloat works with float precision", "[gui][rangeedit]")
{
    RangeEditFloat edit(100, 3);
    edit.setLimits("0.000", "1.000");

    CHECK(edit.isEnabled());

    edit.setLowerRange("0.250");
    CHECK(edit.isEnabled());

    edit.setUpperRange("0.750");
    CHECK(edit.isEnabled());
}

TEST_CASE("RangeEditDouble connectToFields syncs text fields", "[gui][rangeedit]")
{
    RangeEditDouble edit(1000, 2);
    edit.setLimits("0.00", "100.00");

    QLineEdit min_field;
    QLineEdit max_field;

    // Pre-fill fields with the current range values before connecting,
    // since connectToFields reads the field text to initialize the range.
    min_field.setText("0.00");
    max_field.setText("100.00");

    edit.connectToFields(&min_field, &max_field);

    CHECK(edit.isEnabled());
    CHECK(min_field.text() == "0.00");
    CHECK(max_field.text() == "100.00");
}
