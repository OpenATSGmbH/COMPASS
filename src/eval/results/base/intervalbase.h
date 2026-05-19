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

#include "eval/results/base/probabilitybase.h"

#include "timeperiod.h"

#include <boost/optional.hpp>

#include <QVariant>

namespace EvaluationRequirementResult
{

/**
*/
class IntervalBase
{
public:
    IntervalBase() = default;
    IntervalBase(int sum_uis, 
                 int missed_uis)
    :   sum_uis_   (sum_uis)
    ,   missed_uis_(missed_uis)
    {}
    virtual ~IntervalBase() = default;

    int sumUIs() const { return sum_uis_; }
    int missedUIs() const { return missed_uis_; }

protected:
    int sum_uis_    {0};
    int missed_uis_ {0};
};

/**
*/
class SingleIntervalBase : public IntervalBase, public SingleProbabilityBase
{
public:
    SingleIntervalBase(const std::string& result_type, 
                       const std::string& result_id, 
                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                       const SectorLayer& sector_layer, 
                       unsigned int utn, 
                       const EvaluationTargetData* target,
                       EvaluationCalculator& calculator,
                       const EvaluationDetails& details,
                       int sum_uis, 
                       int missed_uis, 
                       TimePeriodCollection ref_periods);

    enum DetailKey
    {
        MissOccurred,        //bool
        DiffTOD,             //float
        RefExists,           //bool
        MissedUIs,           //unsigned int
        RefUpdateStartIndex, //unsigned int
        RefUpdateEndIndex    //unsigned int
    };

protected:
    virtual boost::optional<double> computeResult_impl() const override;
    virtual unsigned int numIssues() const override;

    virtual std::vector<std::string> targetTableHeadersCustom() const override;
    virtual nlohmann::json::array_t targetTableValuesCustom() const override;
    virtual std::string targetTableCustomSortColumn() const override { return "#MUIs"; }
    virtual Qt::SortOrder targetTableSortOrder() const override { return Qt::SortOrder::DescendingOrder; }
    virtual std::vector<TargetInfo> targetInfos() const override;
    virtual std::vector<std::string> detailHeaders() const override;
    virtual nlohmann::json::array_t detailValues(const EvaluationDetail& detail,
                                                 const EvaluationDetail* parent_detail) const override;

    virtual bool detailIsOk(const EvaluationDetail& detail) const override;
    virtual void addAnnotationForDetail(nlohmann::json& annotations_json, 
                                        const EvaluationDetail& detail, 
                                        TargetAnnotationType type,
                                        bool is_ok) const override;
    
    TimePeriodCollection ref_periods_;
};

/**
*/
class JoinedIntervalBase : public IntervalBase, public JoinedProbabilityBase
{
public:
    JoinedIntervalBase(const std::string& result_type, 
                       const std::string& result_id, 
                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                       const SectorLayer& sector_layer, 
                       EvaluationCalculator& calculator);
protected:
    virtual unsigned int numIssues() const override;
    virtual unsigned int numUpdates() const override;

    virtual void clearResults_impl() override;
    virtual void accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last) override;
    virtual boost::optional<double> computeResult_impl() const override;

    virtual std::vector<SectorInfo> sectorInfos() const override;

    virtual FeatureDefinitions getCustomAnnotationDefinitions() const override;
};

} // EvaluationRequirementResult
