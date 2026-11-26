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

#include <cstdint>

namespace EvaluationRequirement
{

/// result of comparing a test value with the reference value
enum ValueComparisonResult
{
    Unknown_NoRefData=0,
    Unknown_NoTstData,
    Same,
    Different
};

/// detail code bit of a compare result: differs_bit if the values differ, missing_bit if the test value is missing, else 0
inline std::uint32_t comparisonCode(ValueComparisonResult cmp_res, std::uint32_t differs_bit, std::uint32_t missing_bit)
{
    if (cmp_res == ValueComparisonResult::Different)
        return differs_bit;
    if (cmp_res == ValueComparisonResult::Unknown_NoTstData)
        return missing_bit;
    return 0;
}

} // namespace EvaluationRequirement
