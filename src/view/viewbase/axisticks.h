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

#include <boost/date_time/posix_time/posix_time_types.hpp>

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
 * Smallest date time part a timestamp label needs to stay unambiguous over the
 * range it belongs to.
 */
enum class TimeFields
{
    Full = 0,  //date and time, the range crosses a year
    DateTime,  //month, day and time, the range crosses a day
    Time,      //time only
    TimeMs     //time with milliseconds, the range covers seconds
};

/**
 * What a label needs beyond its own value. Derived once per range, so that all
 * labels of one axis read the same way.
 */
struct LabelStyle
{
    int        decimals    = 0;                 //decimals a STANDARD label needs, -1 for the default precision
    bool       sub_second  = true;              //a Time of Day label keeps its milliseconds
    TimeFields time_fields = TimeFields::Full;  //how much of a timestamp a label shows
};

/**
 * Result of a tick generation run.
 */
struct Ticks
{
    std::vector<double> values;   //tick positions, ascending
    double              step = 0; //distance between two ticks
    LabelStyle          style;    //how the labels of these ticks should read
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
 *
 * A timestamp counts, since on a plain value axis it reads as milliseconds since
 * epoch. A view that draws it on a QDateTimeAxis decides that before asking.
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
 * Smallest date time part the labels over [t0, t1] need.
 */
TimeFields timeFieldsForSpan(const boost::posix_time::ptime& t0,
                             const boost::posix_time::ptime& t1);

/**
 * Formats a timestamp down to the given date time part.
 */
std::string timeLabel(const boost::posix_time::ptime& value, TimeFields fields);

/**
 * Label for a single tick value. The value is cast back to the data type before
 * the representation is applied, since e.g. an octal stream base does nothing
 * to a double.
 */
std::string label(double value,
                  PropertyDataType dtype,
                  dbContent::Representation repr,
                  const LabelStyle& style = LabelStyle());

/**
 * Labels for all ticks of a generation run.
 */
std::vector<std::string> labels(const Ticks& ticks,
                                PropertyDataType dtype,
                                dbContent::Representation repr);

}
