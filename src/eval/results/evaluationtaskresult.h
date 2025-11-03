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

#include "taskresult.h"
#include "evaluationtarget.h"
#include "evaluationdefs.h"

#include <memory>

#include <QObject>

class QMenu;
class QCheckBox;

class EvaluationCalculator;

/**
 */
class EvaluationTaskResult : public QObject, public TaskResult
{
public:
    EvaluationTaskResult(unsigned int id, 
                         TaskManager& task_man);
    virtual ~EvaluationTaskResult();

    typedef std::map<unsigned int, EvaluationTarget> TargetMap;
    typedef std::map<std::string, bool>              InterestSwitches;

    void setTargets(const TargetMap& targets);
    void injectCalculator(EvaluationCalculator* calculator);
    
    task::TaskResultType type() const override final { return task::TaskResultType::Evaluation; }

    void showUTN (unsigned int utn) const;
    void showFullUTN (unsigned int utn) const;
    void showSurroundingData (unsigned int utn) const;

    const InterestSwitches& interestSwitches() const;

    std::string startSection() const override final;

    static const std::string FieldTargets;
    static const std::string FieldTargetUTN;
    static const std::string FieldTargetInfo;

protected:
    Result update_impl(UpdateState state) override final;
    Result canUpdate_impl(UpdateState state) const override final;
    Result updateContents_impl(const std::vector<ContentID>& contents) override final;

    Result initResult_impl() override final;
    Result prepareResult_impl() override final;
    Result finalizeResult_impl() override final;

    bool loadOnDemandFigure_impl(ResultReport::SectionContentFigure* figure) const override;
    bool loadOnDemandTable_impl(ResultReport::SectionContentTable* table) const override;
    bool loadOnDemandViewable_impl(const ResultReport::SectionContent& content,
                                   ResultReport::SectionContentViewable& viewable, 
                                   const QVariant& index,
                                   unsigned int row) const override;

    void toJSON_impl(nlohmann::json& root_node) const override final;
    bool fromJSON_impl(const nlohmann::json& j) override final;

    bool customContextMenu_impl(QMenu& menu, 
                                ResultReport::SectionContentTable* table, 
                                unsigned int row) override final;
    bool customMenu_impl(QMenu& menu, 
                         ResultReport::SectionContent* content) override final;
    void postprocessTable_impl(ResultReport::SectionContentTable* table) override final;
    bool hasCustomTooltip_impl(const ResultReport::SectionContentTable* table, 
                               unsigned int row,
                               unsigned int col) const override final;
    std::string customTooltip_impl(const ResultReport::SectionContentTable* table, 
                                   unsigned int row,
                                   unsigned int col) const override final;
private:
    Result createCalculator();

    void updateTargets();
    void updateInterestSwitches();
    void updateInterestMenu();

    void setInterestFactorEnabled(const Evaluation::RequirementSumResultID& id, bool ok);
    void setInterestFactorEnabled(const std::string& req_name, bool ok);
    void setInterestFactorsEnabled(bool ok);
    bool interestFactorEnabled(const Evaluation::RequirementSumResultID& id) const;
    EvaluationTarget::InterestMap activeInterestFactors(unsigned int utn) const;

    void jumpToRequirement(const Evaluation::RequirementSumResultID& id, unsigned int utn, bool show_image);

    void createInterestMenu(QMenu& menu);
    void createRequirementLinkMenu(unsigned int utn, QMenu& menu);

    void informUpdateEvalResult(int update_type);

    mutable std::unique_ptr<EvaluationCalculator> calculator_;
    TargetMap                                     targets_;
    mutable InterestSwitches                      interest_factor_enabled_; //req sum result id => enabled

    std::unique_ptr<QMenu>            interest_menu_;
    std::map<std::string, QCheckBox*> interest_boxes_;
};
