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
#include "rangeslider.h"

#include <QSignalSpy>

TEST_CASE("RangeSlider initial state", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    CHECK(slider.lowerValue() == 0);
    CHECK(slider.upperValue() == 0);
    CHECK(slider.handleMovementMode() == RangeSlider::FreeMovement);
}

TEST_CASE("RangeSlider setSpan sets both values", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    slider.setSpan(20, 80);
    CHECK(slider.lowerValue() == 20);
    CHECK(slider.upperValue() == 80);
}

TEST_CASE("RangeSlider setSpan clamps to min/max", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    slider.setSpan(-10, 150);
    CHECK(slider.lowerValue() == 0);
    CHECK(slider.upperValue() == 100);
}

TEST_CASE("RangeSlider setSpan reorders swapped values", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    // Even if lower > upper, setSpan normalizes via qMin/qMax
    slider.setSpan(80, 20);
    CHECK(slider.lowerValue() == 20);
    CHECK(slider.upperValue() == 80);
}

TEST_CASE("RangeSlider setLowerValue and setUpperValue", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    slider.setSpan(10, 90);

    slider.setLowerValue(30);
    CHECK(slider.lowerValue() == 30);
    CHECK(slider.upperValue() == 90);

    slider.setUpperValue(60);
    CHECK(slider.lowerValue() == 30);
    CHECK(slider.upperValue() == 60);
}

TEST_CASE("RangeSlider positions track values", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    slider.setSpan(25, 75);
    CHECK(slider.lowerPosition() == 25);
    CHECK(slider.upperPosition() == 75);
}

TEST_CASE("RangeSlider handle movement modes", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    SECTION("FreeMovement allows any values")
    {
        slider.setHandleMovementMode(RangeSlider::FreeMovement);
        CHECK(slider.handleMovementMode() == RangeSlider::FreeMovement);

        slider.setSpan(50, 50);
        CHECK(slider.lowerValue() == 50);
        CHECK(slider.upperValue() == 50);
    }

    SECTION("NoCrossing mode")
    {
        slider.setHandleMovementMode(RangeSlider::NoCrossing);
        CHECK(slider.handleMovementMode() == RangeSlider::NoCrossing);
    }

    SECTION("NoOverlapping mode")
    {
        slider.setHandleMovementMode(RangeSlider::NoOverlapping);
        CHECK(slider.handleMovementMode() == RangeSlider::NoOverlapping);
    }
}

TEST_CASE("RangeSlider spanChanged signal", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    QSignalSpy spy(&slider, &RangeSlider::spanChanged);

    slider.setSpan(20, 80);
    CHECK(spy.count() == 1);

    auto args = spy.takeFirst();
    CHECK(args.at(0).toInt() == 20);
    CHECK(args.at(1).toInt() == 80);
}

TEST_CASE("RangeSlider lowerValueChanged signal", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    QSignalSpy spy(&slider, &RangeSlider::lowerValueChanged);

    slider.setSpan(30, 70);
    CHECK(spy.count() == 1);
    CHECK(spy.takeFirst().at(0).toInt() == 30);
}

TEST_CASE("RangeSlider upperValueChanged signal", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    QSignalSpy spy(&slider, &RangeSlider::upperValueChanged);

    slider.setSpan(30, 70);
    CHECK(spy.count() == 1);
    CHECK(spy.takeFirst().at(0).toInt() == 70);
}

TEST_CASE("RangeSlider no signal when value unchanged", "[gui][rangeslider]")
{
    RangeSlider slider;
    slider.setMinimum(0);
    slider.setMaximum(100);

    slider.setSpan(20, 80);

    QSignalSpy spy(&slider, &RangeSlider::spanChanged);
    slider.setSpan(20, 80);  // same values
    CHECK(spy.count() == 0);
}

TEST_CASE("RangeSlider orientation", "[gui][rangeslider]")
{
    RangeSlider h_slider(Qt::Horizontal);
    CHECK(h_slider.orientation() == Qt::Horizontal);

    RangeSlider v_slider(Qt::Vertical);
    CHECK(v_slider.orientation() == Qt::Vertical);
}
