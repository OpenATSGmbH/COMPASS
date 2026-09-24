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

#include "axisticks.h"
#include "property_templates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace axis_ticks
{

const int TargetTickCountDefault = 8;

//maximum number of ticks a single axis may get, a guard against a tiny step on
//a huge range
const size_t MaxTicks = 1000;

//decimals a STANDARD label never exceeds
const int MaxDecimals = 6;

//step table for Time of Day axes, in seconds, from a millisecond up to one day
const std::vector<double> TimeSteps = {  0.001, 0.002, 0.005,  0.01,  0.02,  0.05,
                                           0.1,   0.2,   0.5,     1,     2,     5,
                                            10,    15,    30,    60,   120,   300,
                                           600,   900,  1800,  3600,  7200, 10800,
                                         21600, 43200, 86400 };

/**
 */
bool isIntegralDataType(PropertyDataType dtype)
{
    switch (dtype)
    {
        case PropertyDataType::BOOL:
        case PropertyDataType::CHAR:
        case PropertyDataType::UCHAR:
        case PropertyDataType::INT:
        case PropertyDataType::UINT:
        case PropertyDataType::LONGINT:
        case PropertyDataType::ULONGINT:
            return true;
        default:
            return false;
    }
}

/**
 */
bool needsCustomLabels(PropertyDataType dtype, dbContent::Representation repr)
{
    //on a plain value axis a timestamp reads as milliseconds since epoch. A view
    //that puts it on a date time axis instead decides that before asking here.
    if (dtype == PropertyDataType::TIMESTAMP)
        return true;

    //a representation always presents the value differently from a plain number
    if (repr != dbContent::Representation::STANDARD)
        return true;

    //whole numbers should not be broken into fractional ticks
    return isIntegralDataType(dtype);
}

/**
 */
double niceStep(double raw_step, bool integral)
{
    if (!std::isfinite(raw_step) || raw_step <= 0)
        return integral ? 1.0 : std::numeric_limits<double>::min();

    const double pow10 = std::pow(10.0, std::floor(std::log10(raw_step)));
    const double n     = raw_step / pow10;
    const double nice  = (n <= 1.0) ? 1.0 : (n <= 2.0) ? 2.0 : (n <= 5.0) ? 5.0 : 10.0;

    double step = nice * pow10;

    if (integral)
        step = std::max(1.0, std::round(step));

    return step;
}

/**
 */
double niceTimeStep(double raw_step)
{
    if (!std::isfinite(raw_step) || raw_step <= 0)
        return TimeSteps.front();

    for (double s : TimeSteps)
        if (s >= raw_step)
            return s;

    //beyond a day fall back to whole days
    return niceStep(raw_step / TimeSteps.back(), true) * TimeSteps.back();
}

/**
 * Next step of mults x base^n at or above raw_step.
 */
namespace
{
    double niceBaseStep(double raw_step, double base, const std::vector<double>& mults)
    {
        if (!std::isfinite(raw_step) || raw_step <= 1.0)
            return 1.0;

        double magnitude = 1.0;

        //a code range never needs more magnitudes than this
        const int MaxMagnitudes = 24;

        for (int i = 0; i < MaxMagnitudes; ++i)
        {
            for (double m : mults)
                if (magnitude * m >= raw_step)
                    return magnitude * m;

            magnitude *= base;
        }

        return magnitude;
    }
}

/**
 */
double niceOctalStep(double raw_step)
{
    static const std::vector<double> Mults = { 1, 2, 4 };
    return niceBaseStep(raw_step, 8.0, Mults);
}

/**
 */
double niceHexStep(double raw_step)
{
    static const std::vector<double> Mults = { 1, 2, 4, 8 };
    return niceBaseStep(raw_step, 16.0, Mults);
}

/**
 */
TimeFields timeFieldsForSpan(const boost::posix_time::ptime& t0,
                             const boost::posix_time::ptime& t1)
{
    if (t0.is_not_a_date_time() || t1.is_not_a_date_time())
        return TimeFields::Full;

    if (t0.date().year() != t1.date().year())
        return TimeFields::Full;

    if (t0.date() != t1.date())
        return TimeFields::DateTime;

    if ((t1 - t0).total_milliseconds() < 2000)
        return TimeFields::TimeMs;

    return TimeFields::Time;
}

/**
 */
std::string timeLabel(const boost::posix_time::ptime& value, TimeFields fields)
{
    if (value.is_not_a_date_time())
        return "";

    const std::string time = Utils::Time::toString(value.time_of_day(),
                                                   fields == TimeFields::TimeMs ? 3 : 0);

    if (fields == TimeFields::Time || fields == TimeFields::TimeMs)
        return time;

    const std::string date = Utils::Time::toDateString(value); //YYYY-MM-DD

    //drop the year if the range stays inside one
    if (fields == TimeFields::DateTime && date.size() > 5)
        return date.substr(5) + " " + time;

    return date + " " + time;
}

/**
 */
Ticks generate(double vmin,
               double vmax,
               PropertyDataType dtype,
               dbContent::Representation repr,
               int target_count)
{
    Ticks ticks;

    if (!std::isfinite(vmin) || !std::isfinite(vmax) || vmax < vmin)
        return ticks;

    const bool integral = isIntegralDataType(dtype);

    const bool is_time_stamp = (dtype == PropertyDataType::TIMESTAMP);

    //a single value, or a range too narrow to hold a whole number
    if (vmax - vmin < 1e-12 || (integral && vmax - vmin < 1.0))
    {
        const double v = integral ? std::round((vmin + vmax) / 2.0) : (vmin + vmax) / 2.0;

        ticks.values.push_back(v);
        ticks.step           = 0.0;
        ticks.style.decimals = (integral || is_time_stamp) ? 0 : MaxDecimals;

        if (is_time_stamp)
            ticks.style.time_fields = TimeFields::Full;

        return ticks;
    }

    if (target_count < 2)
        target_count = 2;

    const double raw_step = (vmax - vmin) / (double) target_count;

    double step;

    if (is_time_stamp)
    {
        //a timestamp axis carries milliseconds since epoch, the step table is
        //in seconds
        step = niceTimeStep(raw_step / 1000.0) * 1000.0;
    }
    else
    {
        switch (repr)
        {
            case dbContent::Representation::SECONDS_TO_TIME:
                step = niceTimeStep(raw_step);
                break;
            case dbContent::Representation::DEC_TO_OCTAL:
                step = niceOctalStep(raw_step);
                break;
            case dbContent::Representation::DEC_TO_HEX:
                step = niceHexStep(raw_step);
                break;
            default:
                step = niceStep(raw_step, integral);
                break;
        }
    }

    //guard against a step so small that the axis would be flooded
    if ((vmax - vmin) / step > (double) MaxTicks)
        step = niceStep((vmax - vmin) / (double) MaxTicks, integral);

    ticks.step = step;

    //a representation brings its own formatting, a plain number needs as many
    //decimals as the step has
    if (integral || is_time_stamp || repr != dbContent::Representation::STANDARD)
        ticks.style.decimals = 0;
    else
        ticks.style.decimals = std::min(MaxDecimals, std::max(0, (int) std::ceil(-std::log10(step))));

    //a Time of Day label only needs its milliseconds below a second
    ticks.style.sub_second = (step > 0.0 && step < 1.0);

    //snap the first tick to a multiple of the step, so that the labels read as
    //round numbers instead of following the data minimum
    const double eps   = step * 1e-9;
    double       first = std::ceil((vmin - eps) / step) * step;

    for (double v = first; v <= vmax + eps; v += step)
    {
        //re-snap on every step, adding the step up repeatedly drifts
        double snapped = std::round(v / step) * step;

        //the snap can produce a negative zero, which would read as "-0.00"
        if (snapped == 0.0)
            snapped = 0.0;

        ticks.values.push_back(snapped);

        if (ticks.values.size() >= MaxTicks)
            break;
    }

    //a range that holds no multiple of the step still deserves one tick
    if (ticks.values.empty())
        ticks.values.push_back(integral ? std::round((vmin + vmax) / 2.0) : (vmin + vmax) / 2.0);

    //a timestamp label only shows the date time parts that differ across the range
    if (is_time_stamp)
        ticks.style.time_fields = timeFieldsForSpan(Utils::Time::fromLong((long) ticks.values.front()),
                                                    Utils::Time::fromLong((long) ticks.values.back()));

    return ticks;
}

/**
 * Casts the axis value back to the native data type, then formats it.
 */
struct TickLabelFunctor
{
    template <typename T, PropertyDataType DType>
    bool operator()()
    {
        const T v = property_templates::fromDouble<T>(value);

        //a timestamp only shows the date time parts the range needs
        if constexpr (std::is_same<T, boost::posix_time::ptime>::value)
        {
            label = timeLabel(v, style.time_fields);
            return true;
        }
        else
        {
            //at a step of a second or more the milliseconds are noise
            if (repr == dbContent::Representation::SECONDS_TO_TIME && !style.sub_second)
            {
                label = Utils::String::timeStringFromDouble((double) value, false);
                return true;
            }

            if (dbContent::representationString(label, repr, v))
                return true;

            //a truth value reads better as a word than as 0 or 1
            if constexpr (std::is_same<T, bool>::value)
            {
                label = v ? "true" : "false";
                return true;
            }
            else
            {
                label = property_templates::toString<T>(v, style.decimals);
                return true;
            }
        }
    }

    void error(PropertyDataType dtype)
    {
        label = std::to_string(value);
    }

    double                    value = 0.0;
    dbContent::Representation repr  = dbContent::Representation::STANDARD;
    LabelStyle                style;
    std::string               label;
};

/**
 */
std::string label(double value,
                  PropertyDataType dtype,
                  dbContent::Representation repr,
                  const LabelStyle& style)
{
    TickLabelFunctor func;
    func.value = value;
    func.repr  = repr;
    func.style = style;

    property_templates::invokeFunctor(dtype, func);

    return func.label;
}

/**
 */
std::vector<std::string> labels(const Ticks& ticks,
                                PropertyDataType dtype,
                                dbContent::Representation repr)
{
    std::vector<std::string> l;
    l.reserve(ticks.values.size());

    for (double v : ticks.values)
        l.push_back(label(v, dtype, repr, ticks.style));

    return l;
}

}
