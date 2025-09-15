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

#include "dbcontent/target/targetreportchain.h"
#include "evaluationdata.h"
#include "util/timeconv.h"
#include "eval/requirement/base/comparisontype.h"
#include "evaluationdefs.h"

#include "boost/optional.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/time_duration.hpp"

#include <string>
#include <memory>

class EvaluationTargetData;
class EvaluationCalculator;
class SectorLayer;

namespace EvaluationRequirementResult 
{
    class Single;
}

namespace Evaluation
{
    class DataID;
    class RequirementResultID;
}

namespace EvaluationRequirement
{

enum ValueComparisonResult
{
    Unknown_NoRefData=0,
    Unknown_NoTstData,
    Same,
    Different
};

class Base
{
public:
    explicit Base(const std::string& name, 
                  const std::string& short_name, 
                  const std::string& group_name,
                  double threshold,
                  COMPARISON_TYPE check_type, 
                  EvaluationCalculator& calculator,
                  const boost::optional<bool>& must_hold_for_any_target = boost::optional<bool>());
    virtual ~Base();

    virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate (
        const EvaluationTargetData& target_data,
        std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer) = 0;
    // instance is the self-reference for the result (we want to pass the shared pointer to the result)

    std::string name() const;
    std::string shortname() const;
    std::string groupName() const;

    double threshold() const;
    COMPARISON_TYPE conditionCheckType() const;
    const boost::optional<bool>& mustHoldForAnyTarget() const;

    Qt::SortOrder resultSortOrder() const;

    bool conditionPassed(double value) const;
    std::string getConditionStr() const;
    std::string getConditionResultStr(double value) const;
    std::string getResultValueString(double value, int dec = -1) const;
    
    unsigned int getNumThresholdDecimals() const;

    std::string getConditionResultNameShort(bool with_units) const;

    bool matchesResultID(const Evaluation::RequirementResultID& id) const;

    virtual std::string getConditionResultNameShort() const = 0;
    virtual std::string getConditionResultName() const = 0;
    virtual std::string getConditionUnits() const = 0;

    static const unsigned int NumThresholdDecimalsMin = 2;
    static const unsigned int NumThresholdDecimalsMax = 6;
    
protected:
    std::string name_;
    std::string short_name_;
    std::string group_name_;

    EvaluationCalculator& calculator_;

    virtual double convertValueToResult(double value) const;

    bool compareValue (double val, double threshold, COMPARISON_TYPE check_type) const;

    static std::function<boost::optional<std::string>(const dbContent::TargetReport::Chain&,
                                                      const dbContent::TargetReport::Chain::DataID&)> getACID;
    static std::function<bool(const std::string&, const std::string&)> cmpACID;
    static std::function<std::string(const std::string&)> printACID;

    static std::function<boost::optional<unsigned int>(const dbContent::TargetReport::Chain&,
                                                       const dbContent::TargetReport::Chain::DataID&)> getACAD;
    static std::function<bool(const unsigned int&, const unsigned int&)> cmpACAD;
    static std::function<std::string(const unsigned int&)> printACAD;

    static std::function<boost::optional<unsigned int>(const dbContent::TargetReport::Chain&,
                                                       const dbContent::TargetReport::Chain::DataID&)> getModeA;
    static std::function<bool(const unsigned int&, const unsigned int&)> cmpModeA;
    static std::function<std::string(const unsigned int&)> printModeA;

    static std::function<boost::optional<float>(const dbContent::TargetReport::Chain&,
                                                const dbContent::TargetReport::Chain::DataID&)> getModeC;
    static std::function<bool(const float&, const float&, const float&)> cmpModeC;
    static std::function<std::string(const float&)> printModeC;

    // moms

    static std::function<boost::optional<unsigned char>(const dbContent::TargetReport::Chain&,
                                                       const dbContent::TargetReport::Chain::DataID&)> getMomLongAcc;

    static std::function<boost::optional<unsigned char>(const dbContent::TargetReport::Chain&,
                                                       const dbContent::TargetReport::Chain::DataID&)> getMomTransAcc;

    static std::function<boost::optional<unsigned char>(const dbContent::TargetReport::Chain&,
                                                       const dbContent::TargetReport::Chain::DataID&)> getMomVertRate;

    static std::function<bool(const unsigned char&, const unsigned char&)> cmpMomAny;
    static std::function<std::string(const unsigned char&)> printMomTransAcc;
    static std::function<std::string(const unsigned char&)> printMomLongAcc;
    static std::function<std::string(const unsigned char&)> printMomVertRate;

    // rocd

    static std::function<boost::optional<float>(const dbContent::TargetReport::Chain&,
                                                      const dbContent::TargetReport::Chain::DataID&)> getROCD;
    static std::function<bool(const float&, const float&, const float&)> cmpROCD;
    static std::function<std::string(const float&)> printROCD;

    // acceleration

    static std::function<boost::optional<double>(const dbContent::TargetReport::Chain&,
                                                 const dbContent::TargetReport::Chain::DataID&)> getAcceleration;
    static std::function<bool(const double&, const double&, const double&)> cmpAcceleration;
    static std::function<std::string(const double&)> printAcceleration;

    // coasting
    static std::function<boost::optional<unsigned char>(const dbContent::TargetReport::Chain&,
                                                const dbContent::TargetReport::Chain::DataID&)> getCoasting;
    static std::function<bool(const unsigned char&, const unsigned char&)> cmpCoasting;
    static std::function<std::string(const unsigned char&)> printCoasting;

    template <typename T>
    std::pair<ValueComparisonResult, std::string> compare (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff,
        std::function<boost::optional<T>(const dbContent::TargetReport::Chain& chain,
                                                    const dbContent::TargetReport::Chain::DataID& id)>& getter,
        std::function<bool(const T& val1, const T& val2)>& cmp,
        std::function<std::string(const T& val)>& to_str
        ) const;

    std::pair<ValueComparisonResult, std::string> compareTi (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff) const;
    std::pair<ValueComparisonResult, std::string> compareTa (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff) const;
    std::pair<ValueComparisonResult, std::string> compareModeA (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff) const;
    std::pair<ValueComparisonResult, std::string> compareModeC (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff, float max_val_diff) const; // tod tst

    std::pair<ValueComparisonResult, std::string> compareMomLongAcc (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff) const;
    std::pair<ValueComparisonResult, std::string> compareMomTransAcc (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff) const;
    std::pair<ValueComparisonResult, std::string> compareMomVertRate (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff) const;

    std::pair<ValueComparisonResult, std::string> compareROCD (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff, float max_val_diff) const; // tod tst

    std::pair<ValueComparisonResult, std::string> compareAcceleration (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff, float max_val_diff) const; // tod tst

    std::pair<ValueComparisonResult, std::string> compareCoasting (
        const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff) const;

private:
    double                threshold_;
    COMPARISON_TYPE       check_type_ = COMPARISON_TYPE::GREATER_THAN_OR_EQUAL;
    boost::optional<bool> must_hold_for_any_target_;
};

template <typename T>
std::pair<ValueComparisonResult, std::string> Base::compare (
    const dbContent::TargetReport::Chain::DataID& id, const EvaluationTargetData& target_data,
    boost::posix_time::time_duration max_ref_time_diff,
    std::function<boost::optional<T>(const dbContent::TargetReport::Chain& chain,
                                     const dbContent::TargetReport::Chain::DataID& id)>& getter,
    std::function<bool(const T& val1, const T& val2)>& cmp,
    std::function<std::string(const T& val)>& to_str
    ) const // tst timestamp
{
    boost::posix_time::ptime ref_lower, ref_upper;
    std::tie(ref_lower, ref_upper) = target_data.mappedRefTimes(id, max_ref_time_diff);

    boost::optional<T> tst_value, ref_value_lower, ref_value_upper;

    tst_value = getter(target_data.tstChain(), id);

    if (!ref_lower.is_not_a_date_time())
        ref_value_lower = getter(target_data.refChain(), ref_lower);

    if (!ref_upper.is_not_a_date_time())
        ref_value_upper = getter(target_data.refChain(), ref_upper);

    bool has_ref_data = ref_value_lower.has_value() || ref_value_upper.has_value();

    if (!has_ref_data)
    {
//        if (tst_value.has_value()) TODO really?????
//            return {ValueComparisonResult::Different, "Tst data without ref value"};
//        else
            return {ValueComparisonResult::Unknown_NoRefData, "No ref value"};
    }

    if (tst_value.has_value())
    {
        bool value_ok;
        bool lower_nok, upper_nok;

        traced_assert(has_ref_data); // ref times possible

        value_ok = false;

        lower_nok = false;
        upper_nok = false;

        if (ref_value_lower.has_value())
        {
            value_ok = cmp(*ref_value_lower, *tst_value);
            lower_nok = !value_ok;
        }

        if (!value_ok && ref_value_upper.has_value())
        {
            value_ok = cmp(*ref_value_upper, *tst_value);
            upper_nok = !value_ok;
        }

        if (value_ok)
            return {ValueComparisonResult::Same, "OK"};
        else
        {
            std::string comment = "Not OK:";

            if (lower_nok)
            {
                comment += " tst '" + to_str(*tst_value)
                           +"' ref at " + Utils::Time::toString(ref_lower)
                           + " '" + to_str(*ref_value_lower) + "'";
            }
            else
            {
                traced_assert(upper_nok);
                comment += " tst '" + to_str(*tst_value)
                           +"' ref at " + Utils::Time::toString(ref_upper)
                           + " '" + to_str(*ref_value_upper)
                           + "'";
            }

            return {ValueComparisonResult::Different, comment};
        }

    }
    else
    {
        std::string ref_tmp;

        if (ref_value_lower)
            ref_tmp = "'"+to_str(*ref_value_lower)+"'";
        
        if (ref_value_upper && ref_tmp != "'"+to_str(*ref_value_upper)+"'")
        {
            assert (ref_value_upper);
            if (ref_tmp.size())
                ref_tmp += ",";
            
            ref_tmp += "'"+to_str(*ref_value_upper)+"'";
        }
        assert (ref_tmp.size());

        return {ValueComparisonResult::Unknown_NoTstData, "No test value, ref "+ref_tmp};
    }
}

}
