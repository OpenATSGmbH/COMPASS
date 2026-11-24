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

#include "requirement/base/correcthelpers.h"

namespace EvaluationRequirement
{
namespace CorrectHelpers
{

/**
 */
ReportClass classify(const std::vector<ValueComparisonResult>& cmp_results, bool require_all_correct)
{
    bool all_no_ref  = true;
    bool any_correct = false;
    bool all_correct = true;

    for (auto cmp_res : cmp_results)
    {
        all_no_ref &= cmp_res == ValueComparisonResult::Unknown_NoRefData;

        bool failed = cmp_res == ValueComparisonResult::Unknown_NoTstData
                   || cmp_res == ValueComparisonResult::Different;

        any_correct |= !failed;
        all_correct &= !failed;
    }

    if (all_no_ref)
        return ReportClass::NoRefValue;

    bool is_correct = require_all_correct ? all_correct : any_correct;

    return is_correct ? ReportClass::Correct : ReportClass::NotCorrect;
}

} // namespace CorrectHelpers
} // namespace EvaluationRequirement
