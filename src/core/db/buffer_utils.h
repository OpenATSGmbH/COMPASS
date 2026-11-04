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

class Buffer;

namespace dbContent {
class Variable;
class VariableSet;
}

namespace buffer_utils
{

/// Renames Buffer properties from DB column names to dbContent variable names (or vice versa).
/// @param dbcol2dbcontvar  true: DB column → variable name; false: variable name → DB column
void transformVariables(Buffer& buffer, dbContent::VariableSet& list, bool dbcol2dbcontvar);

/// Returns the string representation of a variable's value from a buffer row.
/// Sets is_null to true and returns "" if the property is not present in the buffer
/// or the value is null at the given index.
std::string getValueString(dbContent::Variable& variable,
                           Buffer& buffer,
                           unsigned int buffer_index,
                           bool use_presentation,
                           bool& is_null);

} // namespace buffer_utils
