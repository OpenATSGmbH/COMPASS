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

#include <memory>
#include <string>

#include <boost/optional.hpp>

class EvaluationDetailComments;

namespace EvaluationRequirement
{
    class Base;
}

namespace EvaluationRequirementResult
{

/**
*/
class PresentBase
{
public:
    PresentBase(const std::string& no_ref_value_name);
    PresentBase(int num_updates, 
                int num_no_ref_pos, 
                int num_pos_outside, 
                int num_pos_inside,
                int num_no_ref_val, 
                int num_present, 
                int num_missing,
                const std::string& no_ref_value_name);
    virtual ~PresentBase() = default;

    int numUpdates() const;
    int numNoRefPos() const;
    int numPosOutside() const;
    int numPosInside() const;
    int numNoRefValue() const;
    int numPresent() const;
    int numMissing() const;

protected:
    int num_updates_     {0};
    int num_no_ref_pos_  {0};
    int num_pos_outside_ {0};
    int num_pos_inside_  {0};
    int num_no_ref_val_  {0}; // !ref
    int num_present_     {0}; // ref + tst
    int num_missing_     {0}; // ref + !tst

    std::string no_ref_value_name_;
};

/**
*/
class SinglePresentBase : public PresentBase, public SingleProbabilityBase
{
public:
    SinglePresentBase(const std::string& result_type,
                      const std::string& result_id, 
                      std::shared_ptr<EvaluationRequirement::Base> requirement,
                      const SectorLayer& sector_layer,
                      unsigned int utn, 
                      const EvaluationTargetData* target, 
                      EvaluationCalculator& calculator,
                      const EvaluationDetails& details,
                      int num_updates, 
                      int num_no_ref_pos, 
                      int num_pos_outside, 
                      int num_pos_inside,
                      int num_no_ref_val, 
                      int num_present, 
                      int num_missing,
                      const std::string& no_ref_value_name);

    enum DetailKey
    {
        RefExists,    //bool
        PosInside,    //bool
        IsNotOk,      //bool
        NumUpdates,   //unsigned int
        NumNoRef,     //unsigned int
        NumInside,    //unsigned int
        NumOutside,   //unsigned int
        NumNoRefVal,  //unsigned int
        NumPresent,   //unsigned int
        NumMissing   //unsigned int
    };

protected:
    virtual boost::optional<double> computeResult_impl() const override;
    virtual unsigned int numIssues() const override;

    virtual std::vector<std::string> targetTableHeadersCustom() const override;
    virtual nlohmann::json::array_t targetTableValuesCustom() const override;
    virtual std::string targetTableCustomSortColumn() const override { return "#Missing"; }
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
private:
    std::string no_ref_value_name_;
};

/**
*/
class JoinedPresentBase : public PresentBase, public JoinedProbabilityBase
{
public:
    JoinedPresentBase(const std::string& result_type,
                      const std::string& result_id, 
                      std::shared_ptr<EvaluationRequirement::Base> requirement,
                      const SectorLayer& sector_layer, 
                      EvaluationCalculator& calculator,
                      const std::string& no_ref_value_name);

protected:
    virtual unsigned int numIssues() const override;
    virtual unsigned int numUpdates() const override;

    virtual void clearResults_impl() override;
    virtual void accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last) override;
    virtual boost::optional<double> computeResult_impl() const override;

    virtual std::vector<SectorInfo> sectorInfos() const override;

    virtual FeatureDefinitions getCustomAnnotationDefinitions() const override;
};

}
