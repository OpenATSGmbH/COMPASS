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

#include "format.h"
#include "logger.h"

#include <algorithm>
#include "traced_assert.h"
#include <initializer_list>



const std::vector<std::string> no_format = {""};
const std::vector<std::string> bool_format = {"", "invert"};
const std::vector<std::string> integer_formats{"", "decimal", "hexadecimal", "octal"};

const std::map<PropertyDataType, std::vector<std::string>> Format::format_options_{
    {PropertyDataType::BOOL, bool_format},
    {PropertyDataType::CHAR, integer_formats},
    {PropertyDataType::UCHAR, integer_formats},
    {PropertyDataType::INT, integer_formats},
    {PropertyDataType::UINT, integer_formats},
    {PropertyDataType::LONGINT, integer_formats},
    {PropertyDataType::ULONGINT, integer_formats},
    {PropertyDataType::FLOAT, no_format},
    {PropertyDataType::DOUBLE, no_format},
    {PropertyDataType::STRING,
        {"", "bool", "bool_invert", "decimal", "hexadecimal", "octal", "epoch_tod_ms", "epoch_tod_s"}},
    {PropertyDataType::JSON, no_format},
    {PropertyDataType::TIMESTAMP, no_format}};

void Format::set(PropertyDataType data_type, const std::string& value)
{
    logdbg << "data type '" << Property::asString(data_type) << "' value '" << value
           << "'";

    traced_assert(format_options_.count(data_type) > 0);
    traced_assert(std::find(format_options_.at(data_type).begin(), format_options_.at(data_type).end(),
                     value) != format_options_.at(data_type).end());
    std::string::operator=(value);
}
