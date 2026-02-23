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

#include "buffer_value_string.h"
#include "buffer.h"
#include "dbcontent/variable/variable.h"

#include "json.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

namespace
{

template <typename T>
std::string getTypedValue(Buffer& buffer, const std::string& property_name,
                          unsigned int buffer_index,
                          dbContent::Variable& variable,
                          bool use_presentation, bool has_repr, bool& is_null)
{
    if (!buffer.has<T>(property_name))
    {
        is_null = true;
        return "";
    }

    auto& vec = buffer.get<T>(property_name);

    if (vec.isNull(buffer_index))
    {
        is_null = true;
        return "";
    }

    std::string raw = vec.getAsString(buffer_index);

    if (use_presentation && has_repr)
        return variable.getRepresentationStringFromValue(raw);

    return raw;
}

} // anonymous namespace

namespace buffer_utils
{

std::string getValueString(dbContent::Variable& variable,
                           Buffer& buffer,
                           unsigned int buffer_index,
                           bool use_presentation,
                           bool& is_null)
{
    PropertyDataType data_type = variable.dataType();
    std::string property_name = variable.name();
    is_null = false;

    switch (data_type)
    {
        case PropertyDataType::BOOL:
            return getTypedValue<bool>(buffer, property_name, buffer_index,
                                      variable, use_presentation, true, is_null);
        case PropertyDataType::CHAR:
            return getTypedValue<char>(buffer, property_name, buffer_index,
                                      variable, use_presentation, true, is_null);
        case PropertyDataType::UCHAR:
            return getTypedValue<unsigned char>(buffer, property_name, buffer_index,
                                               variable, use_presentation, true, is_null);
        case PropertyDataType::INT:
            return getTypedValue<int>(buffer, property_name, buffer_index,
                                     variable, use_presentation, true, is_null);
        case PropertyDataType::UINT:
            return getTypedValue<unsigned int>(buffer, property_name, buffer_index,
                                              variable, use_presentation, true, is_null);
        case PropertyDataType::LONGINT:
            return getTypedValue<long int>(buffer, property_name, buffer_index,
                                          variable, use_presentation, true, is_null);
        case PropertyDataType::ULONGINT:
            return getTypedValue<unsigned long int>(buffer, property_name, buffer_index,
                                                   variable, use_presentation, true, is_null);
        case PropertyDataType::FLOAT:
            return getTypedValue<float>(buffer, property_name, buffer_index,
                                       variable, use_presentation, true, is_null);
        case PropertyDataType::DOUBLE:
            return getTypedValue<double>(buffer, property_name, buffer_index,
                                        variable, use_presentation, true, is_null);
        case PropertyDataType::STRING:
            return getTypedValue<std::string>(buffer, property_name, buffer_index,
                                             variable, use_presentation, false, is_null);
        case PropertyDataType::JSON:
            return getTypedValue<nlohmann::json>(buffer, property_name, buffer_index,
                                                variable, use_presentation, false, is_null);
        case PropertyDataType::TIMESTAMP:
            return getTypedValue<boost::posix_time::ptime>(buffer, property_name, buffer_index,
                                                          variable, use_presentation, false, is_null);
        default:
            throw std::domain_error("buffer_utils::getValueString: unknown property data type");
    }
}

} // namespace buffer_utils
