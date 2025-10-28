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
#include "valueaccumulator.h"

#include <memory>

#include <boost/optional.hpp>

namespace EvaluationRequirement
{
    class Speed;
}

namespace EvaluationRequirementResult
{

/**
*/
class SpeedBase
{
public:
    SpeedBase();
    SpeedBase(unsigned int num_pos, 
              unsigned int num_no_ref,
              unsigned int num_pos_outside, 
              unsigned int num_pos_inside, 
              unsigned int num_no_tst_value,
              unsigned int num_comp_failed, 
              unsigned int num_comp_passed);

    unsigned int numPos() const;
    unsigned int numNoRef() const;
    unsigned int numPosOutside() const;
    unsigned int numPosInside() const;
    unsigned int numNoTstValues() const;
    unsigned int numCompFailed() const;
    unsigned int numCompPassed() const;

    const ValueAccumulator& accumulator() const;

protected:
    unsigned int num_pos_          {0};
    unsigned int num_no_ref_       {0};
    unsigned int num_pos_outside_  {0};
    unsigned int num_pos_inside_   {0};
    unsigned int num_no_tst_value_ {0};
    unsigned int num_comp_failed_  {0};
    unsigned int num_comp_passed_  {0};

    mutable ValueAccumulator accumulator_;
};

/**
*/
class SingleSpeed : public SpeedBase, public SingleProbabilityBase
{
public:
    SingleSpeed(const std::string& result_id, 
                std::shared_ptr<EvaluationRequirement::Base> requirement,
                const SectorLayer& sector_layer,
                unsigned int utn, 
                const EvaluationTargetData* target, 
                EvaluationCalculator& calculator,
                const EvaluationDetails& details,
                unsigned int num_pos, 
                unsigned int num_no_ref,
                unsigned int num_pos_outside, 
                unsigned int num_pos_inside, 
                unsigned int num_no_tst_value,
                unsigned int num_comp_failed, 
                unsigned int num_comp_passed);

    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) override;

    enum DetailKey
    {
        Offset,         //float
        CheckPassed,    //bool
        PosInside,      //bool
        NumPos,         //unsigned int
        NumNoRef,       //unsigned int
        NumInside,      //unsigned int
        NumOutside,     //unsigned int
        NumCheckFailed, //unsigned int
        NumCheckPassed  //unsigned int
    };

protected:
    virtual boost::optional<double> computeResult_impl() const override;
    virtual unsigned int numIssues() const override;

    virtual std::vector<std::string> targetTableHeadersCustom() const override;
    virtual nlohmann::json::array_t targetTableValuesCustom() const override;
    virtual std::string targetTableCustomSortColumn() const override;
    virtual Qt::SortOrder targetTableSortOrder() const override  { return  Qt::SortOrder::DescendingOrder; }
    virtual std::vector<TargetInfo> targetInfos() const override;
    virtual std::vector<std::string> detailHeaders() const override;
    virtual nlohmann::json::array_t detailValues(const EvaluationDetail& detail,
                                                 const EvaluationDetail* parent_detail) const override;

    virtual bool detailIsOk(const EvaluationDetail& detail) const override;
    virtual void addAnnotationForDetail(nlohmann::json& annotations_json, 
                                        const EvaluationDetail& detail, 
                                        TargetAnnotationType type,
                                        bool is_ok) const override;
};

/**
*/
class JoinedSpeed : public SpeedBase, public JoinedProbabilityBase
{
public:
    JoinedSpeed(const std::string& result_id, 
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

    virtual bool exportAsCSV(std::ofstream& strm) const override;

    virtual FeatureDefinitions getCustomAnnotationDefinitions() const override;
};

}
