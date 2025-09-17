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

#include "eval/results/base/single.h"
#include "eval/results/base/joined.h"
#include "eval/results/base/probabilitybase.h"

#include "valueaccumulator.h"

#include "json_fwd.hpp"

#include <boost/optional.hpp>

namespace EvaluationRequirementResult
{

/**
 * Common contents for position results.
*/
class PositionBase
{
public:
    PositionBase();
    PositionBase(unsigned int num_pos,
                 unsigned int num_no_ref,
                 unsigned int num_pos_outside,
                 unsigned int num_pos_inside,
                 unsigned int num_ref_inaccurate,
                 unsigned int num_passed,
                 unsigned int num_failed);

    unsigned int numPos() const;
    unsigned int numNoRef() const;
    unsigned int numPosOutside() const;
    unsigned int numPosInside() const;
    unsigned int numRefInaccurate() const;
    unsigned int numPassed() const;
    unsigned int numFailed() const;

    const ValueAccumulator& accumulator() const;

protected:
    unsigned int num_pos_         {0};
    unsigned int num_no_ref_      {0};
    unsigned int num_pos_outside_ {0};
    unsigned int num_pos_inside_  {0};
    unsigned int num_ref_inaccurate_ {0};
    unsigned int num_passed_      {0};
    unsigned int num_failed_      {0};

    mutable ValueAccumulator accumulator_;
};

/**
 * Common functionality for single position results.
*/
class SinglePositionBaseCommon : public PositionBase
{
public:
    SinglePositionBaseCommon(unsigned int num_pos,
                             unsigned int num_no_ref,
                             unsigned int num_pos_outside,
                             unsigned int num_pos_inside,
                             unsigned int num_ref_inaccurate,
                             unsigned int num_passed,
                             unsigned int num_failed);

    enum DetailKey
    {
        Value,          //float
        CheckPassed,    //bool
        PosInside,      //bool
        NumPos,         //unsigned int
        NumNoRef,       //unsigned int
        NumInside,      //unsigned int
        NumOutside,     //unsigned int
        NumRefInaccurate,     //unsigned int
        NumCheckPassed, //unsigned int
        NumCheckFailed, //unsigned int
        PositionBaseMax
    };

protected:
    boost::optional<double> common_computeResult(const Single* single_result) const;
    unsigned int common_numIssues() const;
    bool common_detailIsOk(const EvaluationDetail& detail,
                           const std::shared_ptr<EvaluationRequirement::Base>& requirement) const;

    FeatureDefinitions common_getCustomAnnotationDefinitions(const Single& single,
                                                             const EvaluationCalculator& calculator) const;

    virtual boost::optional<double> computeFinalResultValue() const = 0;
};

/**
 * Single position results with a final probabilistic value.
*/
class SinglePositionProbabilityBase : public SingleProbabilityBase, public SinglePositionBaseCommon
{
public:
    SinglePositionProbabilityBase(const std::string& result_type,
                                  const std::string& result_id,
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
                                  unsigned int num_ref_inaccurate,
                                  unsigned int num_passed,
                                  unsigned int num_failed);
    virtual ~SinglePositionProbabilityBase() = default;

protected:
    boost::optional<double> computeFinalResultValue() const override final;

    boost::optional<double> computeResult_impl() const override final;
    unsigned int numIssues() const override final;

    bool detailIsOk(const EvaluationDetail& detail) const override final;
    void addAnnotationForDetail(nlohmann::json& annotations_json, 
                                const EvaluationDetail& detail, 
                                TargetAnnotationType type,
                                bool is_ok) const override final;

    virtual FeatureDefinitions getCustomAnnotationDefinitions() const override;
};

/**
 * Single position results with a final general value.
*/
class SinglePositionValueBase : public Single, public SinglePositionBaseCommon
{
public:
    SinglePositionValueBase(const std::string& result_type,
                       const std::string& result_id,
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
                       unsigned int num_ref_inaccurate,
                       unsigned int num_passed,
                       unsigned int num_failed);
    virtual ~SinglePositionValueBase() = default;

    nlohmann::json resultValue(double value) const override final;

protected:
    virtual boost::optional<double> computeResult_impl() const override;
    virtual unsigned int numIssues() const override;

    virtual bool detailIsOk(const EvaluationDetail& detail) const override;
    virtual void addAnnotationForDetail(nlohmann::json& annotations_json, 
                                        const EvaluationDetail& detail, 
                                        TargetAnnotationType type,
                                        bool is_ok) const override;

    virtual FeatureDefinitions getCustomAnnotationDefinitions() const override;
};

/**
*/
class JoinedPositionBase : public PositionBase
{
public:
    JoinedPositionBase(const std::string& csv_header);

protected:
    unsigned int common_numIssues() const;
    unsigned int common_numUpdates() const;

    bool common_exportAsCSV(std::ofstream& strm,
                            const Joined* result) const;

    void common_clearResults();
    void common_accumulateSingleResult(unsigned int utn, const PositionBase& single_result,
                                       bool last);

    boost::optional<double> common_computeResult() const;
    virtual boost::optional<double> computeFinalResultValue() const = 0;

    FeatureDefinitions common_getCustomAnnotationDefinitions(const Joined& joined,
                                                             const EvaluationCalculator& calculator) const;

private:
    std::string csv_header_;
};

/**
*/
class JoinedPositionProbabilityBase : public JoinedPositionBase, public JoinedProbabilityBase
{
public:
    JoinedPositionProbabilityBase(const std::string& result_type,
                                  const std::string& result_id,
                                  std::shared_ptr<EvaluationRequirement::Base> requirement,
                                  const SectorLayer& sector_layer,
                                  EvaluationCalculator& calculator,
                                  const std::string& csv_header);
protected:
    boost::optional<double> computeFinalResultValue() const override final;

    virtual unsigned int numIssues() const override;
    virtual unsigned int numUpdates() const override;

    virtual void clearResults_impl() override;
    virtual void accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last) override;
    virtual boost::optional<double> computeResult_impl() const override;

    virtual bool exportAsCSV(std::ofstream& strm) const override;

    virtual FeatureDefinitions getCustomAnnotationDefinitions() const override;
};

/**
*/
class JoinedPositionValueBase : public JoinedPositionBase, public Joined
{
public:
    JoinedPositionValueBase(const std::string& result_type,
                            const std::string& result_id,
                            std::shared_ptr<EvaluationRequirement::Base> requirement,
                            const SectorLayer& sector_layer,
                            EvaluationCalculator& calculator,
                            const std::string& csv_header);

    nlohmann::json resultValue(double value) const override final;

protected:
    virtual unsigned int numIssues() const override;
    virtual unsigned int numUpdates() const override;

    virtual void clearResults_impl() override;
    virtual void accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last) override;
    virtual boost::optional<double> computeResult_impl() const override;

    virtual bool exportAsCSV(std::ofstream& strm) const override;

    virtual FeatureDefinitions getCustomAnnotationDefinitions() const override;
};

}
