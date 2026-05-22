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
#include "colorscaleselection.h"
#include "colormap_defs.h"

#include <QSignalSpy>

TEST_CASE("ColorScaleSelection construction populates scales", "[gui][colorscale]")
{
    ColorScaleSelection widget;

    // The widget should have a valid initial selection from UsedColorScales
    auto scale = widget.selectedScale();

    // Verify it's one of the used scales
    bool found = false;
    for (auto s : colorscale::UsedColorScales)
    {
        if (s == scale)
        {
            found = true;
            break;
        }
    }
    CHECK(found);
}

TEST_CASE("ColorScaleSelection setSelectedScale round-trip", "[gui][colorscale]")
{
    ColorScaleSelection widget;

    widget.setSelectedScale(colorscale::ColorScale::Gray);
    CHECK(widget.selectedScale() == colorscale::ColorScale::Gray);

    widget.setSelectedScale(colorscale::ColorScale::Red);
    CHECK(widget.selectedScale() == colorscale::ColorScale::Red);

    widget.setSelectedScale(colorscale::ColorScale::Viridis_Cut);
    CHECK(widget.selectedScale() == colorscale::ColorScale::Viridis_Cut);
}

TEST_CASE("ColorScaleSelection setSelectedScale with non-listed scale is no-op", "[gui][colorscale]")
{
    ColorScaleSelection widget;
    auto original = widget.selectedScale();

    // Custom is not in UsedColorScales, so findData returns -1 → no change
    widget.setSelectedScale(colorscale::ColorScale::Custom);
    CHECK(widget.selectedScale() == original);
}

TEST_CASE("ColorScaleSelection scaleChanged signal", "[gui][colorscale]")
{
    ColorScaleSelection widget;

    QSignalSpy spy(&widget, &ColorScaleSelection::scaleChanged);

    // Change to a different scale
    auto first = widget.selectedScale();
    colorscale::ColorScale target = (first == colorscale::ColorScale::Gray)
        ? colorscale::ColorScale::Red
        : colorscale::ColorScale::Gray;

    widget.setSelectedScale(target);

    CHECK(spy.count() == 1);
}
