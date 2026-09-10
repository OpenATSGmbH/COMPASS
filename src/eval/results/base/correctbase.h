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

#include <boost/optional.hpp>

namespace EvaluationRequirementResult
{

/**
*/
class CorrectBase
{
public:
    CorrectBase(const std::string& correct_value_name,
                const std::string& correct_short_name,
                const std::string& not_correct_short_name);
    CorrectBase(unsigned int num_updates, 
                unsigned int num_no_ref_pos, 
                unsigned int num_no_ref_id,
                unsigned int num_pos_outside, 
                unsigned int num_pos_inside,
                unsigned int num_correct, 
                unsigned int num_not_correct,
                const std::string& correct_value_name,
                const std::string& correct_short_name,
                const std::string& not_correct_short_name);
    virtual ~CorrectBase() = default;

    unsigned int numUpdates() const;
    unsigned int numNoRefPos() const;
    unsigned int numNoRefId() const;
    unsigned int numPosOutside() const;
    unsigned int numPosInside() const;
    unsigned int numCorrect() const;
    unsigned int numNotCorrect() const;

protected:
    unsigned int num_updates_     {0};
    unsigned int num_no_ref_pos_  {0};
    unsigned int num_no_ref_id_   {0};
    unsigned int num_pos_outside_ {0};
    unsigned int num_pos_inside_  {0};
    unsigned int num_correct_     {0};
    unsigned int num_not_correct_ {0};

    std::string correct_value_name_;
    std::string correct_short_name_;
    std::string not_correct_short_name_;
};

/**
*/
class SingleCorrectBase : public CorrectBase, public SingleProbabilityBase
{
public:
    SingleCorrectBase(const std::string& result_type,
                      const std::string& result_id, 
                      std::shared_ptr<EvaluationRequirement::Base> requirement,
                      const SectorLayer& sector_layer,
                      unsigned int utn, 
                      const EvaluationTargetData* target, 
                      EvaluationCalculator& calculator,
                      const EvaluationDetails& details,
                      unsigned int num_updates, 
                      unsigned int num_no_ref_pos, 
                      unsigned int num_no_ref_id,
                      unsigned int num_pos_outside, 
                      unsigned int num_pos_inside,
                      unsigned int num_correct, 
                      unsigned int num_not_correct,
                      const std::string& correct_value_name,
                      const std::string& correct_short_name,
                      const std::string& not_correct_short_name);

    enum DetailKey
    {
        RefExists,     //bool
        PosInside,     //bool
        IsNotCorrect,  //bool
        NumUpdates,    //unsigned int
        NumNoRef,      //unsigned int
        NumInside,     //unsigned int
        NumOutside,    //unsigned int
        NumCorrect,    //unsigned int
        NumNotCorrect //unsigned int
    };

protected:
    virtual boost::optional<double> computeResult_impl() const override final;
    virtual unsigned int numIssues() const override final;

    virtual std::vector<std::string> targetTableHeadersCustom() const override final;
    virtual nlohmann::json::array_t targetTableValuesCustom() const override final;
    virtual std::vector<TargetInfo> targetInfos() const override final;

    virtual bool detailIsOk(const EvaluationDetail& detail) const override final;
    void addReportTableColumns(ReportTableDefinition& def) const override;
    void fillReportTableRow(ReportTableRows& rows,
                            const EvaluationDetail& detail,
                            const EvaluationDetail* parent_detail,
                            const EvaluationDetail* prev_detail) const override;
    void fillDetailFromReportTableRow(EvaluationDetail& detail,
                                      const Buffer& buffer,
                                      unsigned int row,
                                      const EvaluationDetail* prev_detail) const override;
    virtual void addAnnotationForDetail(nlohmann::json& annotations_json, 
                                        const EvaluationDetail& detail, 
                                        TargetAnnotationType type,
                                        bool is_ok) const override final;

    virtual std::vector<TargetInfo> additionalTargetInfos() const { return {}; }
};

/**
*/
class JoinedCorrectBase : public CorrectBase, public JoinedProbabilityBase
{
public:
    JoinedCorrectBase(const std::string& result_type,
                      const std::string& result_id, 
                      std::shared_ptr<EvaluationRequirement::Base> requirement,
                      const SectorLayer& sector_layer, 
                      EvaluationCalculator& calculator,
                      const std::string& correct_value_name,
                      const std::string& correct_short_name,
                      const std::string& not_correct_short_name);
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
