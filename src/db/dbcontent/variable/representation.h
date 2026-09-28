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

#include "stringconv.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>

namespace dbContent
{

/**
 * How a variable value is presented to the user.
 * Defined here, and not inside Variable, so that the context-free part of the
 * formatting can be used without an application instance. Variable aliases it
 * as Variable::Representation, so all existing call sites keep working.
 */
enum class Representation
{  // TODO rework to m3a/ta
    STANDARD,
    SECONDS_TO_TIME,
    DEC_TO_OCTAL,
    DEC_TO_HEX,
    FEET_TO_FLIGHTLEVEL,
    DATA_SRC_NAME,
    CLIMB_DESCENT,
    FLOAT_PREC0,
    FLOAT_PREC1,
    FLOAT_PREC2,
    FLOAT_PREC4,
    LINE_NAME,
    MLAT_RUS  // JSON int-array of contributing receiver indices, formatted via DBContextManager RU names
};

/**
 * True if the representation can be resolved from the value alone.
 * DATA_SRC_NAME and MLAT_RUS need the DBContextManager, so they are resolved by
 * Variable and not here. STANDARD is not a special representation at all.
 */
bool representationIsContextFree(Representation repr);

namespace representation_helpers
{
    /**
     * Type an integral value is promoted to before it goes into a stream.
     * Without this a char sized value is written as a character.
     */
    template <typename T>
    struct StreamType { typedef T type; };

    template <> struct StreamType<bool>          { typedef unsigned int type; };
    template <> struct StreamType<char>          { typedef int          type; };
    template <> struct StreamType<signed char>   { typedef int          type; };
    template <> struct StreamType<unsigned char> { typedef unsigned int type; };
}

/**
 * Formats value according to repr and writes the result to str.
 *
 * The value must already be cast to the variable's native data type. Formatting
 * a double as octal or hexadecimal prints garbage, since the stream base only
 * applies to integers.
 *
 * Returns false, leaving str untouched, if repr is STANDARD, needs an
 * application context, or the data type carries no number.
 */
template <typename T>
bool representationString(std::string& str, Representation repr, const T& value)
{
    if (!representationIsContextFree(repr))
        return false;

    if constexpr (std::is_arithmetic<T>::value)
    {
        typedef typename representation_helpers::StreamType<T>::type SType;

        const SType v = (SType) value;

        std::ostringstream out;

        switch (repr)
        {
            case Representation::SECONDS_TO_TIME:
                str = Utils::String::timeStringFromDouble((double) value);
                return true;
            case Representation::LINE_NAME:
            {
                //lineStrFrom asserts on anything outside the four lines, and an
                //axis tick can land outside the data range
                const long line = (long) value;
                if (line < 0 || line > 3)
                    str = std::to_string(line);
                else
                    str = Utils::String::lineStrFrom((unsigned int) line);
                return true;
            }
            case Representation::CLIMB_DESCENT:
            {
                const int numeric_value = (int) value;
                if (numeric_value == 0)
                    str = "LVL";
                else if (numeric_value == 1)
                    str = "CLB";
                else if (numeric_value == 2)
                    str = "DSC";
                else
                    str = "UDF";
                return true;
            }
            //a stream base only applies to integers, so a code is always cast
            case Representation::DEC_TO_OCTAL:
                out << std::oct << std::setfill('0') << std::setw(4) << (long long) value;
                break;
            case Representation::DEC_TO_HEX:
                out << std::uppercase << std::hex << std::setfill('0') << std::setw(6) << (long long) value;
                break;
            case Representation::FEET_TO_FLIGHTLEVEL:
                out << (double) value / 100.0;
                break;
            case Representation::FLOAT_PREC0:
                out << std::fixed << std::setprecision(0) << v;
                break;
            case Representation::FLOAT_PREC1:
                out << std::fixed << std::setprecision(1) << v;
                break;
            case Representation::FLOAT_PREC2:
                out << std::fixed << std::setprecision(2) << v;
                break;
            case Representation::FLOAT_PREC4:
                out << std::fixed << std::setprecision(4) << v;
                break;
            default:
                return false;
        }

        str = out.str();
        return true;
    }
    else
    {
        //no number, nothing a numeric representation could be applied to
        return false;
    }
}

}
