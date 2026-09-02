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
#include "eval/results/base/valueaccumulator.h"

namespace EvaluationRequirementResult
{

/**
*/
class ADSBLatencyBase
{
public:
    ADSBLatencyBase() = default;
    ADSBLatencyBase(unsigned int num_pos,
                    unsigned int num_no_data,
                    unsigned int num_pos_outside,
                    unsigned int num_pos_inside,
                    unsigned int num_value_ok,
                    unsigned int num_value_nok);

    unsigned int numPos() const { return num_pos_; }
    unsigned int numNoData() const { return num_no_data_; }
    unsigned int numPosOutside() const { return num_pos_outside_; }
    unsigned int numPosInside() const { return num_pos_inside_; }
    unsigned int numValueOk() const { return num_value_ok_; }
    unsigned int numValueNok() const { return num_value_nok_; }

    const ValueAccumulator& accumulator() const { return accumulator_; }

protected:
    unsigned int num_pos_         {0};
    unsigned int num_no_data_     {0}; // reports without TOMR and/or TORT
    unsigned int num_pos_outside_ {0};
    unsigned int num_pos_inside_  {0};
    unsigned int num_value_ok_    {0};
    unsigned int num_value_nok_   {0};

    ValueAccumulator accumulator_; // latency values in seconds
};

/**
*/
class SingleADSBLatency : public ADSBLatencyBase, public SingleProbabilityBase
{
public:
    SingleADSBLatency(const std::string& result_id,
                      std::shared_ptr<EvaluationRequirement::Base> requirement,
                      const SectorLayer& sector_layer,
                      unsigned int utn,
                      const EvaluationTargetData* target,
                      EvaluationCalculator& calculator,
                      const EvaluationDetails& details,
                      unsigned int num_pos,
                      unsigned int num_no_data,
                      unsigned int num_pos_outside,
                      unsigned int num_pos_inside,
                      unsigned int num_value_ok,
                      unsigned int num_value_nok,
                      const std::vector<double>& latencies);

    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) override;

    enum DetailKey
    {
        PosInside,       //bool
        Value,           //float, latency in seconds
        CheckPassed,     //bool
        NumPos,          //unsigned int
        NumNoData,       //unsigned int
        NumInside,       //unsigned int
        NumOutside,      //unsigned int
        NumCheckPassed,  //unsigned int
        NumCheckFailed,  //unsigned int
    };

protected:
    virtual boost::optional<double> computeResult_impl() const override;
    virtual unsigned int numIssues() const override;

    virtual std::vector<std::string> targetTableHeadersCustom() const override;
    virtual nlohmann::json::array_t targetTableValuesCustom() const override;
    virtual std::string targetTableCustomSortColumn() const override { return "#LTNOK"; }
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
};

/**
*/
class JoinedADSBLatency : public ADSBLatencyBase, public JoinedProbabilityBase
{
public:
    JoinedADSBLatency(const std::string& result_id,
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
};

}
