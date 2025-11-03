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

#include <string>
#include <stdexcept>

namespace EvaluationRequirement
{

enum COMPARISON_TYPE {
    LESS_THAN=0, // <
    LESS_THAN_OR_EQUAL, // <=
    GREATER_THAN, // >
    GREATER_THAN_OR_EQUAL // >=
};

extern std::string comparisonTypeString(COMPARISON_TYPE type);
extern std::string comparisonTypeLongString(COMPARISON_TYPE type);

}
