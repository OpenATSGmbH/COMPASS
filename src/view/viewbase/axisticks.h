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

#pragma once

#include "property.h"
#include "representation.h"

#include <string>
#include <vector>

/**
 * Tick positions and tick labels for a numerical axis.
 *
 * Views flatten every value to double before drawing, so the data type and the
 * variable representation have to be passed in explicitly. Without them a tick
 * label cannot be formatted the way the rest of COMPASS presents that value.
 */
namespace axis_ticks
{

/**
 * Result of a tick generation run.
 */
struct Ticks
{
    std::vector<double> values;       //tick positions, ascending
    double              step     = 0; //distance between two ticks
    int                 decimals = 0; //decimals a STANDARD label needs at this step
};

/**
 * Default number of ticks a generation run aims for.
 */
extern const int TargetTickCountDefault;

/**
 * True for the data types which carry whole numbers only.
 */
bool isIntegralDataType(PropertyDataType dtype);

/**
 * True if an axis for this variable needs tick labels of its own, because Qt
 * would present the value in a way COMPASS does not use elsewhere.
 */
bool needsCustomLabels(PropertyDataType dtype, dbContent::Representation repr);

/**
 * Next step from {1, 2, 5} x 10^n at or above raw_step.
 * An integral step is never smaller than 1.
 */
double niceStep(double raw_step, bool integral);

/**
 * Next step from the time step table at or above raw_step, in seconds.
 * The table runs from 1 second up to 1 day, so that a Time of Day axis breaks
 * at whole minutes and hours.
 */
double niceTimeStep(double raw_step);

/**
 * Next step of {1, 2, 4} x 8^n at or above raw_step.
 * A decimal step turns into a ragged octal label, e.g. a step of 1000 reads as
 * 1750. This keeps a Mode 3/A axis at round octal codes.
 */
double niceOctalStep(double raw_step);

/**
 * Next step of {1, 2, 4, 8} x 16^n at or above raw_step, for the same reason as
 * niceOctalStep.
 */
double niceHexStep(double raw_step);

/**
 * Tick positions covering [vmin, vmax].
 */
Ticks generate(double vmin,
               double vmax,
               PropertyDataType dtype,
               dbContent::Representation repr,
               int target_count = TargetTickCountDefault);

/**
 * Label for a single tick value. The value is cast back to the data type before
 * the representation is applied, since e.g. an octal stream base does nothing
 * to a double.
 *
 * sub_second only affects a time label. At a step of a second or more the
 * milliseconds are noise, so they are left out.
 */
std::string label(double value,
                  PropertyDataType dtype,
                  dbContent::Representation repr,
                  int decimals,
                  bool sub_second = true);

/**
 * Labels for all ticks of a generation run.
 */
std::vector<std::string> labels(const Ticks& ticks,
                                PropertyDataType dtype,
                                dbContent::Representation repr);

}
