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
#include "results/base/detailcode.h"

#include <vector>

#include <boost/optional.hpp>

namespace EvaluationRequirement
{

/**
 * Classification of test reports inside the sector for the False requirements (e.g. Mode C
 * False, Mode A False, Identification False), free of the evaluation data access, so it can
 * be unit tested on plain compare results.
 */
namespace FalseHelpers
{

enum class ReportClass
{
    NoRefValue, // reference has no value: not counted in the probability
    Unknown,    // test has no value: not counted in the probability
    Correct,
    False
};

/// class of a test report from the compare result of one value
ReportClass classify(ValueComparisonResult cmp_res);

/**
 * Class of a test report from the compare results of several used values (e.g. ACID, ACAD,
 * Mode A). Only the values with a test and a reference value are compared: the report is
 * false if one of them (or all of them, if require_all_false) is different. Without any
 * compared value the report is Unknown if a used value has no test value, else NoRefValue.
 */
ReportClass classify(const std::vector<ValueComparisonResult>& cmp_results, bool require_all_false);

/// detail code bit of a class without a comparison (DetailCode::Value::NoRefValue, NoTstValue), else 0
std::uint32_t code(ReportClass c);

/// counts of the classified test reports
struct Counts
{
    unsigned int num_no_ref_val = 0;
    unsigned int num_unknown    = 0;
    unsigned int num_correct    = 0;
    unsigned int num_false      = 0;

    void add(ReportClass c);

    /// false / (correct + false), no value without correct or false reports
    boost::optional<double> probabilityFalse() const;
};

} // namespace FalseHelpers

} // namespace EvaluationRequirement
