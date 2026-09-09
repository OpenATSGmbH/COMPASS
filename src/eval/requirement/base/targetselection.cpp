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

#include "eval/requirement/base/targetselection.h"

#include "logger.h"

namespace EvaluationRequirement
{

std::string targetSelectionString(TargetSelection selection)
{
    if (selection == TargetSelection::Cooperative)
        return "cooperative";
    else if (selection == TargetSelection::NonCooperative)
        return "non_cooperative";
    else
        return "all";
}

std::string targetSelectionLongString(TargetSelection selection)
{
    if (selection == TargetSelection::Cooperative)
        return "Cooperative Only";
    else if (selection == TargetSelection::NonCooperative)
        return "Non-cooperative Only";
    else
        return "All Targets";
}

TargetSelection targetSelectionFromString(const std::string& str)
{
    if (str == "cooperative")
        return TargetSelection::Cooperative;
    else if (str == "non_cooperative")
        return TargetSelection::NonCooperative;
    else if (str == "all")
        return TargetSelection::All;

    logwrn << "unknown target selection '" << str << "', using all targets";

    return TargetSelection::All;
}

}
