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
class IdentificationChangeDelayBase
{
public:
    IdentificationChangeDelayBase() = default;
    IdentificationChangeDelayBase(unsigned int num_events,
                                  unsigned int num_not_assessable,
                                  unsigned int num_passed,
                                  unsigned int num_failed);

    unsigned int numEvents() const { return num_events_; }
    unsigned int numNotAssessable() const { return num_not_assessable_; }
    unsigned int numPassed() const { return num_passed_; }
    unsigned int numFailed() const { return num_failed_; }

    const ValueAccumulator& accumulator() const { return accumulator_; }

protected:
    unsigned int num_events_         {0}; // detected identity change events
    unsigned int num_not_assessable_ {0}; // events without sufficient data to judge
    unsigned int num_passed_         {0};
    unsigned int num_failed_         {0};

    ValueAccumulator accumulator_; // change delays in seconds
};

/**
*/
class SingleIdentificationChangeDelay : public IdentificationChangeDelayBase, public SingleProbabilityBase
{
public:
    SingleIdentificationChangeDelay(const std::string& result_id,
                                    std::shared_ptr<EvaluationRequirement::Base> requirement,
                                    const SectorLayer& sector_layer,
                                    unsigned int utn,
                                    const EvaluationTargetData* target,
                                    EvaluationCalculator& calculator,
                                    const EvaluationDetails& details,
                                    unsigned int num_events,
                                    unsigned int num_not_assessable,
                                    unsigned int num_passed,
                                    unsigned int num_failed,
                                    const std::vector<double>& delays);

    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) override;

    enum DetailKey
    {
        Value,           //float, change delay in seconds
        CheckPassed,     //bool
        NumEvents,       //unsigned int
        NumNotAssessable,//unsigned int
        NumCheckPassed,  //unsigned int
        NumCheckFailed,  //unsigned int
    };

protected:
    virtual boost::optional<double> computeResult_impl() const override;
    virtual unsigned int numIssues() const override;

    virtual std::vector<std::string> targetTableHeadersCustom() const override;
    virtual nlohmann::json::array_t targetTableValuesCustom() const override;
    virtual std::string targetTableCustomSortColumn() const override { return "#CDNOK"; }
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
class JoinedIdentificationChangeDelay : public IdentificationChangeDelayBase, public JoinedProbabilityBase
{
public:
    JoinedIdentificationChangeDelay(const std::string& result_id,
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
