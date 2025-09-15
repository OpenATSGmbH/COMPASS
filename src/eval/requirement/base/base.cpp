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

#include "eval/requirement/base/base.h"
#include "evaluationdata.h"
#include "stringconv.h"
#include "number.h"
#include "compass.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

// acid
std::function<boost::optional<std::string>(const dbContent::TargetReport::Chain&,
                                           const dbContent::TargetReport::Chain::DataID&)> Base::getACID =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { 
        auto tmp_opt = chain.acid(id);

        if (tmp_opt) // rempve emptry strings with spaces only
        {
            std::string tmp = String::trim(*tmp_opt);

            if (tmp.size())
                return boost::optional<std::string>(tmp);
        }

        return boost::optional<std::string>();
    };

std::function<bool(const std::string&, const std::string&)> Base::cmpACID =
    [] (const std::string& val1, const std::string& val2) { return val1 == val2; };

std::function<std::string(const std::string&)> Base::printACID =
    [] (const std::string& val) { return val; };

// acad
std::function<boost::optional<unsigned int>(const dbContent::TargetReport::Chain&,
                                            const dbContent::TargetReport::Chain::DataID&)> Base::getACAD =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.acad(id); };

std::function<bool(const unsigned int&, const unsigned int&)> Base::cmpACAD =
    [] (const unsigned int& val1, const unsigned int& val2) { return val1 == val2; };

std::function<std::string(const unsigned int&)> Base::printACAD =
    [] (const unsigned int& val) { return String::hexStringFromInt(val); };

// mode a
std::function<boost::optional<unsigned int>(const dbContent::TargetReport::Chain&,
                                            const dbContent::TargetReport::Chain::DataID&)> Base::getModeA =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.modeA(id); };

std::function<bool(const unsigned int&, const unsigned int&)> Base::cmpModeA =
    [] (const unsigned int& val1, const unsigned int& val2) { return val1 == val2; };

std::function<std::string(const unsigned int&)> Base::printModeA =
    [] (const unsigned int& val) { return String::octStringFromInt(val, 4, '0'); };

// mode c
std::function<boost::optional<float>(const dbContent::TargetReport::Chain&,
                                     const dbContent::TargetReport::Chain::DataID&)> Base::getModeC =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.modeC(id); };

std::function<bool(const float&, const float&, const float&)> Base::cmpModeC =
    [] (const float& val1, const float& val2, const float& max_val_diff) { return fabs(val1 - val2) <= max_val_diff; };

std::function<std::string(const float&)> Base::printModeC =
    [] (const unsigned int& val) { return String::doubleToStringPrecision(val, 2); };

// moms

std::function<boost::optional<unsigned char>(const dbContent::TargetReport::Chain&,
                                             const dbContent::TargetReport::Chain::DataID&)> Base::getMomLongAcc =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.momLongAcc(id); };

std::function<boost::optional<unsigned char>(const dbContent::TargetReport::Chain&,
                                             const dbContent::TargetReport::Chain::DataID&)> Base::getMomTransAcc  =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.momTransAcc(id); };

std::function<boost::optional<unsigned char>(const dbContent::TargetReport::Chain&,
                                             const dbContent::TargetReport::Chain::DataID&)> Base::getMomVertRate  =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.momVertRate(id); };

std::function<bool(const unsigned char&, const unsigned char&)> Base::cmpMomAny =
    [] (const unsigned char& val1, const unsigned char& val2) { return val1 == val2; };

std::function<std::string(const unsigned char&)> Base::printMomTransAcc =
    [] (const unsigned char& val) {

        if (val == 0)
            return "ConstantCourse";
        else if (val == 1)
            return "RightTurn";
        else if (val == 2)
            return "LeftTurn";
        else if (val == 3)
            return "Undetermined";

        traced_assert(false);
    };

std::function<std::string(const unsigned char&)> Base::printMomLongAcc =
    [] (const unsigned char& val) {

        if (val == 0)
            return "ConstantGroundspeed";
        else if (val == 1)
            return "IncreasingGroundspeed";
        else if (val == 2)
            return "DecreasingGroundspeed";
        else if (val == 3)
            return "Undetermined";

        traced_assert(false);
    };

std::function<std::string(const unsigned char&)> Base::printMomVertRate =
    [] (const unsigned char& val) {

        if (val == 0)
            return "Level";
        else if (val == 1)
            return "Climb";
        else if (val == 2)
            return "Descent";
        else if (val == 3)
            return "Undetermined";

        traced_assert(false);
    };

// rocd

std::function<boost::optional<float>(const dbContent::TargetReport::Chain&,
                                     const dbContent::TargetReport::Chain::DataID&)> Base::getROCD =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.rocd(id); };

std::function<bool(const float&, const float&, const float&)> Base::cmpROCD =
    [] (const float& val1, const float& val2, const float& max_val_diff) { return fabs(val1 - val2) <= max_val_diff; };

std::function<std::string(const float&)> Base::printROCD =
    [] (const unsigned int& val) { return String::doubleToStringPrecision(val, 2); };


// acceleration

std::function<boost::optional<double>(const dbContent::TargetReport::Chain&,
                                      const dbContent::TargetReport::Chain::DataID&)> Base::getAcceleration =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.acceleration(id); };

std::function<bool(const double&, const double&, const double&)> Base::cmpAcceleration =
    [] (const float& val1, const float& val2, const float& max_val_diff) { return fabs(val1 - val2) <= max_val_diff; };

std::function<std::string(const double&)> Base::printAcceleration =
    [] (const unsigned int& val) { return String::doubleToStringPrecision(val, 2); };

// coasting

std::function<boost::optional<unsigned char>(const dbContent::TargetReport::Chain&,
                                     const dbContent::TargetReport::Chain::DataID&)> Base::getCoasting =
    [] (const dbContent::TargetReport::Chain& chain,
       const dbContent::TargetReport::Chain::DataID& id) { return chain.trackCoasting(id); };

std::function<bool(const unsigned char&, const unsigned char&)> Base::cmpCoasting =
    [] (const float& val1, const float& val2) { return val1 == val2; };

std::function<std::string(const unsigned char&)> Base::printCoasting =
    [] (const unsigned int& val) { return to_string((unsigned int) val); };

/**
*/
Base::Base(const std::string& name, 
           const std::string& short_name, 
           const std::string& group_name,
           double threshold,
           COMPARISON_TYPE check_type, 
           EvaluationCalculator& calculator,
           const boost::optional<bool>& must_hold_for_any_target)
:   name_                    (name      )
,   short_name_              (short_name)
,   group_name_              (group_name)
,   calculator_              (calculator)
,   threshold_               (threshold )
,   check_type_              (check_type)
,   must_hold_for_any_target_(must_hold_for_any_target)
{
}

/**
*/
Base::~Base() = default;

/**
*/
std::string Base::name() const
{
    return name_;
}

/**
*/
std::string Base::shortname() const
{
    return short_name_;
}

/**
*/
std::string Base::groupName() const
{
    return group_name_;
}

/**
*/
double Base::threshold() const
{
    return threshold_;
}

/**
*/
COMPARISON_TYPE Base::conditionCheckType() const
{
    return check_type_;
}

/**
*/
const boost::optional<bool>& Base::mustHoldForAnyTarget() const
{
    return must_hold_for_any_target_;
}

/**
*/
Qt::SortOrder Base::resultSortOrder() const
{
    bool lt = check_type_ == COMPARISON_TYPE::LESS_THAN ||
              check_type_ == COMPARISON_TYPE::LESS_THAN_OR_EQUAL;

    return (lt ? Qt::DescendingOrder : Qt::AscendingOrder);
}

/**
*/
double Base::convertValueToResult(double value) const
{
    return value;
}

/**
*/
std::string Base::getResultValueString(double value, int dec) const
{
    const double value_conv = convertValueToResult(value);

    //by default use one more digit than the threshold value needs
    return String::doubleToStringPrecision(value_conv, dec < 0 ? getNumThresholdDecimals() : (unsigned)dec);
}

/**
*/
std::string Base::getConditionStr() const
{
    if (check_type_ == COMPARISON_TYPE::LESS_THAN)
        return comparisonTypeString(COMPARISON_TYPE::LESS_THAN) + " " + getResultValueString(threshold_);
    else if (check_type_ == COMPARISON_TYPE::LESS_THAN_OR_EQUAL)
        return comparisonTypeString(COMPARISON_TYPE::LESS_THAN_OR_EQUAL) + " " + getResultValueString(threshold_);
    else if (check_type_ == COMPARISON_TYPE::GREATER_THAN)
        return comparisonTypeString(COMPARISON_TYPE::GREATER_THAN) + " " + getResultValueString(threshold_);
    else if (check_type_ == COMPARISON_TYPE::GREATER_THAN_OR_EQUAL)
        return comparisonTypeString(COMPARISON_TYPE::GREATER_THAN_OR_EQUAL) + " " + getResultValueString(threshold_);
    else
        throw std::runtime_error("Base: getConditionStr: unknown type '" + to_string(check_type_) + "'");
}

/**
*/
std::string Base::getConditionResultNameShort(bool with_units) const
{
    std::string name  = getConditionResultNameShort();
    std::string units = getConditionUnits();

    if (with_units && !units.empty())
        name += " [" + units + "]";

    return name;
}

/**
*/
bool Base::conditionPassed(double value) const
{
    return compareValue(value, threshold_, check_type_);
}

/**
*/
std::string Base::getConditionResultStr(double value) const
{
    return conditionPassed(value) ? "Passed" : "Failed";
}

/**
*/
unsigned int Base::getNumThresholdDecimals() const
{
    const auto thres_conv     = convertValueToResult(threshold_);
    const auto num_dec_needed = Utils::Number::numDecimals(thres_conv, NumThresholdDecimalsMax);

    // use one more than needed and at least dec min
    return std::max(NumThresholdDecimalsMin, num_dec_needed + 1); 
}

/**
*/
bool Base::matchesResultID(const Evaluation::RequirementResultID& id) const
{
    return id.req_group_name == group_name_ &&
           id.req_name == name_;
}

/**
*/
bool Base::compareValue(double val, double threshold, COMPARISON_TYPE check_type) const
{
    if (check_type == COMPARISON_TYPE::LESS_THAN)
        return val < threshold;
    else if (check_type == COMPARISON_TYPE::LESS_THAN_OR_EQUAL)
        return val <= threshold;
    else if (check_type == COMPARISON_TYPE::GREATER_THAN)
        return val > threshold;
    else if (check_type == COMPARISON_TYPE::GREATER_THAN_OR_EQUAL)
        return val >= threshold;
    else
        throw std::runtime_error("Base: compareValue: unknown type '" + to_string(check_type) + "'");
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareTi (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    time_duration max_ref_time_diff) const
{
    return compare<std::string>(id, target_data, max_ref_time_diff, getACID, cmpACID, printACID);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareTa (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    time_duration max_ref_time_diff) const
{
    return compare<unsigned int>(id, target_data, max_ref_time_diff, getACAD, cmpACAD, printACAD);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareModeA (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    time_duration max_ref_time_diff) const
{
    return compare<unsigned int>(id, target_data, max_ref_time_diff, getModeA, cmpModeA, printModeA);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareModeC (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    time_duration max_ref_time_diff, float max_val_diff) const
{

    std::function<bool(const float&, const float&)> tmp_cmpModeC =
        [max_val_diff] (const float& val1, const float& val2) { return fabs(val1 - val2) <= max_val_diff; };

    return compare<float>(id, target_data, max_ref_time_diff, getModeC, tmp_cmpModeC, printModeC);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareMomLongAcc (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    boost::posix_time::time_duration max_ref_time_diff) const
{
    return compare<unsigned char>(id, target_data, max_ref_time_diff, getMomLongAcc, cmpMomAny, printMomLongAcc);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareMomTransAcc (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    boost::posix_time::time_duration max_ref_time_diff) const
{
    return compare<unsigned char>(id, target_data, max_ref_time_diff, getMomTransAcc, cmpMomAny, printMomTransAcc);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareMomVertRate (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    boost::posix_time::time_duration max_ref_time_diff) const
{
    return compare<unsigned char>(id, target_data, max_ref_time_diff, getMomVertRate, cmpMomAny, printMomVertRate);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareROCD (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    time_duration max_ref_time_diff, float max_val_diff) const
{

    std::function<bool(const float&, const float&)> tmp_cmpROCD =
        [max_val_diff] (const float& val1, const float& val2) { return fabs(val1 - val2) <= max_val_diff; };

    return compare<float>(id, target_data, max_ref_time_diff, getROCD, tmp_cmpROCD, printROCD);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareAcceleration (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    time_duration max_ref_time_diff, float max_val_diff) const
{

    std::function<bool(const double&, const double&)> tmp_cmpAcceleration =
        [max_val_diff] (const double& val1, const double& val2) { return fabs(val1 - val2) <= max_val_diff; };

    return compare<double>(id, target_data, max_ref_time_diff, getAcceleration, tmp_cmpAcceleration, printAcceleration);
}

/**
*/
std::pair<ValueComparisonResult, std::string> Base::compareCoasting (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    boost::posix_time::time_duration max_ref_time_diff) const
{
    return compare<unsigned char>(id, target_data, max_ref_time_diff, getCoasting, cmpCoasting, printCoasting);
}

}
