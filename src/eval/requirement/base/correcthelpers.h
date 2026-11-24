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

#include "requirement/base/valuecomparison.h"

#include <vector>

namespace EvaluationRequirement
{

/**
 * Classification of test reports inside the sector for the Correct requirements (e.g.
 * Identification Correct), free of the evaluation data access, so it can be unit tested on
 * plain compare results.
 */
namespace CorrectHelpers
{

enum class ReportClass
{
    NoRefValue, // no used value has a reference value: not counted in the probability
    Correct,
    NotCorrect
};

/**
 * Class of a test report from the compare results of the used values (e.g. ACID, ACAD,
 * Mode A). A value fails if the test value is missing or different. The report is correct
 * if one value (or all of them, if require_all_correct) does not fail. Without any reference
 * value the report is NoRefValue.
 */
ReportClass classify(const std::vector<ValueComparisonResult>& cmp_results, bool require_all_correct);

} // namespace CorrectHelpers

} // namespace EvaluationRequirement
