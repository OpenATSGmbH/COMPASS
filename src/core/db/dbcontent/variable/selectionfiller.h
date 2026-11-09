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
#include <memory>
#include <string>

class Buffer;
class DBContentManager;

namespace dbContent
{

class Variable;

/// unit conversion factor from the candidate (or input) variable to the selection (or output)
/// variable: 1 when the units agree or one of them has no unit, false with an error when the
/// dimensions differ or a unit is unknown. Used by SelectionFiller and CalculationPlan
bool unitFactor(DBContentManager& dbcont_man, const Variable& candidate, const Variable& selection,
                double& factor);

/**
 * Fills the selection variables of imported buffers from their candidate lists, see
 * readme_dbcontent.md "Selection variables".
 *
 * Runs once per buffer before any calculation and before the time shift: for every row the
 * first candidate with a value wins, its value is copied into the selection column and its
 * 1-based position into the companion source variable. Selections sharing a source variable are
 * chosen together, a candidate position counts only when every member has a value there,
 * latitude and longitude for example. A calculation candidate ends the search for a row, the
 * code running that calculation fills the rows still null and sets their index.
 *
 * A candidate of another data type or dimension than the selection is skipped with an
 * error. A candidate with another unit of the same dimension is converted on copy, the
 * factor comes from the UnitManager.
 */
class SelectionFiller
{
public:
    static void fill(DBContentManager& dbcont_man,
                     std::map<std::string, std::shared_ptr<Buffer>>& buffers);
    static void fill(DBContentManager& dbcont_man, const std::string& dbcontent_name, Buffer& buffer);
};

}
