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

#include "requirement/base/falsehelpers.h"
#include "results/base/detailcode.h"

#include <stdexcept>

namespace EvaluationRequirement
{
namespace FalseHelpers
{

/**
 */
ReportClass classify(ValueComparisonResult cmp_res)
{
    switch (cmp_res)
    {
        case ValueComparisonResult::Unknown_NoRefData: return ReportClass::NoRefValue;
        case ValueComparisonResult::Unknown_NoTstData: return ReportClass::Unknown;
        case ValueComparisonResult::Same:              return ReportClass::Correct;
        case ValueComparisonResult::Different:         return ReportClass::False;
    }

    throw std::runtime_error("FalseHelpers: classify: unknown compare result");
}

/**
 */
ReportClass classify(const std::vector<ValueComparisonResult>& cmp_results, bool require_all_false)
{
    unsigned int num_compared  = 0;
    unsigned int num_different = 0;
    bool         any_no_tst    = false;

    for (auto cmp_res : cmp_results)
    {
        if (cmp_res == ValueComparisonResult::Same || cmp_res == ValueComparisonResult::Different)
        {
            ++num_compared;

            if (cmp_res == ValueComparisonResult::Different)
                ++num_different;
        }
        else if (cmp_res == ValueComparisonResult::Unknown_NoTstData)
        {
            any_no_tst = true;
        }
    }

    if (num_compared == 0)
        return any_no_tst ? ReportClass::Unknown : ReportClass::NoRefValue;

    bool is_false = require_all_false ? num_different == num_compared : num_different > 0;

    return is_false ? ReportClass::False : ReportClass::Correct;
}

/**
 */
std::uint32_t code(ReportClass c)
{
    namespace Value = EvaluationRequirementResult::DetailCode::Value;

    switch (c)
    {
        case ReportClass::NoRefValue: return Value::NoRefValue;
        case ReportClass::Unknown:    return Value::NoTstValue;
        case ReportClass::Correct:
        case ReportClass::False:      return 0;
    }

    return 0;
}

/**
 */
void Counts::add(ReportClass c)
{
    switch (c)
    {
        case ReportClass::NoRefValue: ++num_no_ref_val; break;
        case ReportClass::Unknown:    ++num_unknown;    break;
        case ReportClass::Correct:    ++num_correct;    break;
        case ReportClass::False:      ++num_false;      break;
    }
}

/**
 */
boost::optional<double> Counts::probabilityFalse() const
{
    unsigned int num_total = num_correct + num_false;

    if (num_total == 0)
        return {};

    return (double)num_false / (double)num_total;
}

} // namespace FalseHelpers
} // namespace EvaluationRequirement
