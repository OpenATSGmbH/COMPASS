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

#include <map>
#include <string>
#include <vector>

#include "property.h"

class Format : public std::string
{
  public:
    Format() = default;
    Format(PropertyDataType data_type, const std::string& value) { set(data_type, value); }

    void set(PropertyDataType data_type, const std::string& value);

    const std::vector<std::string>& getFormatOptions(PropertyDataType data_type)
    {
        return format_options_.at(data_type);
    }

    const std::map<PropertyDataType, std::vector<std::string>>& getAllFormatOptions()
    {
        return format_options_;
    }

  private:
    static const std::map<PropertyDataType, std::vector<std::string>> format_options_;
};
