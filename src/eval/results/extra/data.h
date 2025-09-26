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
class ExtraDataBase
{
public:
    ExtraDataBase();
    ExtraDataBase(unsigned int num_extra, 
                  unsigned int num_ok);

    unsigned int numExtra() const;
    unsigned int numOK() const;

protected:
    unsigned int num_extra_{0};
    unsigned int num_ok_   {0};
};

/**
*/
class SingleExtraData : public ExtraDataBase, public SingleProbabilityBase
{
public:
    SingleExtraData(const std::string& result_id, 
                    std::shared_ptr<EvaluationRequirement::Base> requirement,
                    const SectorLayer& sector_layer, 
                    unsigned int utn, 
                    const EvaluationTargetData* target,
                    EvaluationCalculator& calculator,
                    const EvaluationDetails& details,
                    unsigned int num_extra, 
                    unsigned int num_ok, 
                    bool has_extra_test_data);

    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) override;

    bool hasExtraTestData() const;

    enum DetailKey
    {
        Inside,    //bool
        Extra,     //bool
        RefExists //bool
    };

protected:
    virtual boost::optional<double> computeResult_impl() const override;
    virtual unsigned int numIssues() const override;

    virtual std::vector<std::string> targetTableHeadersCustom() const override;
    virtual nlohmann::json::array_t targetTableValuesCustom() const override;
    virtual std::vector<TargetInfo> targetInfos() const override;
    virtual std::vector<std::string> detailHeaders() const override;
    virtual nlohmann::json::array_t detailValues(const EvaluationDetail& detail,
                                                 const EvaluationDetail* parent_detail) const override;

    virtual bool detailIsOk(const EvaluationDetail& detail) const override;
    virtual void addAnnotationForDetail(nlohmann::json& annotations_json, 
                                        const EvaluationDetail& detail, 
                                        TargetAnnotationType type,
                                        bool is_ok) const override;

    bool has_extra_test_data_ {false};
};

/**
*/
class JoinedExtraData : public ExtraDataBase, public JoinedProbabilityBase
{
public:
    JoinedExtraData(const std::string& result_id, 
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

}
