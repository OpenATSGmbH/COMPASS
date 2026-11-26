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

namespace EvaluationRequirement
{

/**
 * Which target class a requirement is evaluated for. Standards state some
 * requirements for cooperative targets only, others for every target. EUROCAE
 * ED-87E Table 3-1 NOTE 19 and NOTE 22 are examples.
 *
 * A target outside the selection gets an ignored result with the reason, the
 * same way a target ignored by the standard does. It stays visible in the
 * per-target table and does not enter the sector sum.
 */
enum class TargetSelection
{
    All = 0,        // every target
    Cooperative,    // has a Mode 3/A code, a Mode C code or a Mode S attribute
    NonCooperative  // has none of those, primary-only
};

extern std::string targetSelectionString(TargetSelection selection);
extern std::string targetSelectionLongString(TargetSelection selection);
extern TargetSelection targetSelectionFromString(const std::string& str);

}
