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

#include "evaluationtaskresult.h"
#include "evaluationcalculator.h"
#include "evaluationmanager.h"
#include "evalsectionid.h"
#include "eval/results/base/single.h"
#include "eval/results/base/joined.h"

#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/section.h"
#include "task/taskdefs.h"
#include "taskmanager.h"
#include "radarplotpositioncalculatortask.h"

#include "compass.h"
#include "dbcontentmanager.h"
#include "targetmodel.h"
#include "targetlistwidget.h"

#include "logger.h"

#include <QMenu>
#include <QWidgetAction>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QPushButton>

const std::string EvaluationTaskResult::FieldTargets    = "targets";
const std::string EvaluationTaskResult::FieldTargetUTN  = "utn";
const std::string EvaluationTaskResult::FieldTargetInfo = "info";

/**
 */
EvaluationTaskResult::EvaluationTaskResult(unsigned int id, 
                                           TaskManager& task_man)
:   TaskResult(id, task_man)
{
}

/**
 */
EvaluationTaskResult::~EvaluationTaskResult() = default;

/**
 */
std::string EvaluationTaskResult::startSection() const
{
    return EvalSectionID::prependReportResults(EvaluationRequirementResult::Base::RequirementOverviewSectionName);
}

/**
 */
void EvaluationTaskResult::setTargets(const TargetMap& targets)
{
    targets_ = targets;
}

/**
 * !Handle with care!
 */
void EvaluationTaskResult::injectCalculator(EvaluationCalculator* calculator)
{
    traced_assert(calculator);
    calculator_.reset(calculator);
}

/**
 */
Result EvaluationTaskResult::createCalculator()
{
    //clone calculator from config
    auto res = EvaluationCalculator::clone(config_);
    if (!res.ok())
        return res;

    calculator_.reset(res.result());
    
    //created calculator should be properly configured
    auto can_eval = calculator_->canEvaluate();
    if (!can_eval.ok())
        return can_eval;

    updateInterestSwitches();

    return Result::succeeded();
}

/**
 */
Result EvaluationTaskResult::initResult_impl()
{
    //create calculator
    auto res = createCalculator();
    if (!res.ok())
        return res;

    //connect to eval manager
    auto& eval_manager = COMPASS::instance().evaluationManager();
    connect(&eval_manager, &EvaluationManager::resultsNeedUpdate, this, &EvaluationTaskResult::informUpdateEvalResult);

    return Result::succeeded();
}

/**
 */
Result EvaluationTaskResult::prepareResult_impl()
{
    return Result::succeeded();
}

/**
 */
Result EvaluationTaskResult::finalizeResult_impl()
{
    //connect to eval manager
    auto& eval_manager = COMPASS::instance().evaluationManager();
    connect(&eval_manager, &EvaluationManager::resultsNeedUpdate, this, &EvaluationTaskResult::informUpdateEvalResult);

    return Result::succeeded();
}

/**
 */
Result EvaluationTaskResult::update_impl(UpdateState state)
{
    Result res = Result::succeeded();

    if (state == UpdateState::FullUpdateNeeded ||
        state == UpdateState::Locked)
    {
        loginf << "running full update";
        res = calculator_->evaluate(true);
    }
    else if (state == UpdateState::PartialUpdateNeeded)
    {
        bool needs_recompute = !calculator_->evaluated() || 
                                calculator_->hasConstraints();

        if (needs_recompute)
        {
            loginf << "running initial full update";
            res = calculator_->evaluate(true);
        }
        else
        {
            loginf << "running partial update";
            calculator_->updateResultsToChanges();
        }
    }
    
    return res;
}

/**
 */
Result EvaluationTaskResult::canUpdate_impl(UpdateState state) const
{
    //true for all kinds of updates
    if (!calculator_)
        return Result::failed("Calculator not initialized");

    auto r = calculator_->canEvaluate();
    if (!r.ok())
        return r;

    return Result::succeeded();
}

/**
 */
Result EvaluationTaskResult::updateContents_impl(const std::vector<ContentID>& contents)
{
    //first update eval targets from db targets
    updateTargets();

    //run base's default contents update
    auto r = TaskResult::updateContents_impl(contents);
    if (!r.ok())
        return r;

    //update result content in db (eval targets changed)
    syncContent();

    return Result::succeeded();
}

namespace helpers
{
    /**
     */
    std::pair<unsigned int, Evaluation::RequirementResultID> singleResultContentProperties(const ResultReport::SectionContent* content)
    {
        traced_assert(content);

        auto info = EvaluationRequirementResult::Single::singleContentProperties(*content);
        traced_assert(info.has_value());

        return std::make_pair(info->first, info->second);
    }

    /**
     */
    Evaluation::RequirementResultID joinedResultContentProperties(const ResultReport::SectionContent* content)
    {
        traced_assert(content);

        auto info = EvaluationRequirementResult::Joined::joinedContentProperties(*content);
        traced_assert(info.has_value());

        return info.value();
    }

    /**
     */
    EvaluationRequirementResult::Single* obtainSingleResult(const ResultReport::SectionContent* content,
                                                            EvaluationCalculator* calculator)
    {
        auto info = singleResultContentProperties(content);

        loginf << "obtaining result for" 
               << " utn " << info.first 
               << " layer " << info.second.sec_layer_name
               << " group " << info.second.req_group_name
               << " req " << info.second.req_name;

        //result already present?
        auto r = calculator->singleResult(info.second, info.first);
        if (r)
            return r;

        //otherwise evaluate for specified utn and requirement
        //note: if eval fails a nullptr is returned in the next step
        calculator->evaluate(false, { info.first }, { info.second });
        
        //then return result
        return calculator->singleResult(info.second, info.first);
    }

    /**
     */
    EvaluationRequirementResult::Joined* obtainJoinedResult(const ResultReport::SectionContent* content,
                                                            EvaluationCalculator* calculator)
    {
        auto info = joinedResultContentProperties(content);

        loginf << "obtaining result for" 
               << " layer " << info.sec_layer_name
               << " group " << info.req_group_name
               << " req " << info.req_name;

        //result already present?
        auto r = calculator->joinedResult(info);
        if (r)
            return r;

        //otherwise evaluate for specified requirement
        //note: if eval fails a nullptr is returned in the next step
        calculator->evaluate(false, {}, { info });

        //then return result
        return calculator->joinedResult(info);
    }

    /**
     */
    unsigned int utnFromTable(const ResultReport::SectionContentTable* table, unsigned int row)
    {
        traced_assert(table);
        traced_assert(table->hasColumn("UTN"));
        const auto& d = table->getData(row, "UTN");
        traced_assert(d.is_number_unsigned());

        unsigned int utn = d;

        return utn;
    }
}

/**
 */
bool EvaluationTaskResult::loadOnDemandFigure_impl(ResultReport::SectionContentFigure* figure) const
{
    if (!calculator_)
        return false;

    try
    {
        if (figure->name() == EvaluationRequirementResult::Single::TargetOverviewID)
        {
            //get result for section
            auto result = helpers::obtainSingleResult(figure, calculator_.get());
            if (!result)
            {
                logerr << "result could not be obtained";
                return false;
            }

            //add overview to figure
            if (!result->addOverviewToFigure(*figure))
            {
                logerr << "error configuring content";
                return false;
            }

            return true;
        }
    }
    catch(...)
    {
        logerr << "critical error during load";
    }
    
    return false;
}

/**
 */
bool EvaluationTaskResult::loadOnDemandTable_impl(ResultReport::SectionContentTable* table) const
{
    if (!calculator_)
        return false;

    try
    {
        if (table->name() == EvaluationRequirementResult::Single::TRDetailsTableName)
        {
            //target reports details table in single result section

            //get result for section
            auto result = helpers::obtainSingleResult(table, calculator_.get());
            if (!result)
            {
                logerr << "result could not be obtained";
                return false;
            }

            //add table details
            if (!result->addDetailsToTable(*table))
            {
                logerr << "error configuring content";
                return false;
            }

            return true;
        }
        else if (table->name() == EvaluationData::TargetsTableName)
        {
            //evaluation targets table

            //fill table with target info
            calculator_->data().fillTargetsTable(targets_, *table, 
                [ this ] (const Evaluation::RequirementSumResultID& id) { return this->interestFactorEnabled(id); });

            return true;
        }
    }
    catch(...)
    {
        logerr << "critical error during load";
    }
    
    return false;
}

/**
 */
bool EvaluationTaskResult::loadOnDemandViewable_impl(const ResultReport::SectionContent& content,
                                                     ResultReport::SectionContentViewable& viewable, 
                                                     const QVariant& index,
                                                     unsigned int row) const
{
    if (!calculator_)
        return false;

    if (content.contentType() == ResultReport::SectionContent::ContentType::Table)
    {
        if (content.name() == EvaluationRequirementResult::Single::TRDetailsTableName)
        {
            //get result for section
            auto result = helpers::obtainSingleResult(&content, calculator_.get());
            if (!result)
            {
                logerr << "result could not be obtained";
                return false;
            }

            //configure detail highlight viewable 
            if (!result->addHighlightToViewable(viewable, index))
            {
                logerr << "error configuring content";
                return false;
            }

            return true;
        }
        else if (content.name() == EvaluationData::TargetsTableName)
        {
            const ResultReport::SectionContentTable* table = dynamic_cast<const ResultReport::SectionContentTable*>(&content);
            traced_assert(table);

            //obtain utn
            auto utn = helpers::utnFromTable(table, row);

            //configure viewable
            auto content = calculator_->getViewableForUTN(utn);
            nlohmann::json j_content = *content;
            viewable.setCallback(j_content);

            return true;
        }
    }

    return false;
}

/**
 */
bool EvaluationTaskResult::customContextMenu_impl(QMenu& menu, 
                                                  ResultReport::SectionContentTable* table, 
                                                  unsigned int row)
{
    logdbg;

    if (table->name() == EvaluationRequirementResult::Single::TRDetailsTableName)
    {
        //target report details table in single result section
    }
    else if (table->name() == EvaluationRequirementResult::Joined::SectorTargetsTableName)
    {
        //target table in joined result section
        auto info = helpers::joinedResultContentProperties(table);
        auto utn  = helpers::utnFromTable(table, row);
        
        loginf << "context menu requested for utn " << utn;

        if (calculator_ && !isLocked())
        {
            auto action_show_utn = menu.addAction("Show Full UTN");
            QObject::connect (action_show_utn, &QAction::triggered, [ = ] () { this->showFullUTN(utn); });

            auto action_show_data = menu.addAction("Show Surrounding Data");
            QObject::connect (action_show_data, &QAction::triggered, [ = ] () { this->showSurroundingData(utn); });

            auto usage_menu = menu.addMenu("Target Usage");

            const auto& target             = targets_.at(utn);
            auto        target_list_widget = COMPASS::instance().dbContentManager().targetListWidget();

            target_list_widget->createTargetEvalMenu(*usage_menu, 
                                                     target,
                                                     info.req_name);
        }

        //@TODO: jump to requirement

        return true;
    }
    else if (table->name() == EvaluationRequirementResult::Base::RequirementOverviewTableName)
    {
        //requirement table in overview section
    }
    else if (table->name() == EvaluationData::TargetsTableName)
    {
        //evaluation target table
        auto utn  = helpers::utnFromTable(table, row);

        loginf << "context menu requested for utn " << utn;

        if (calculator_ && !isLocked())
        {
            auto action_show_utn = menu.addAction("Show Full UTN");
            QObject::connect (action_show_utn, &QAction::triggered, [ = ] () { this->showFullUTN(utn); });

            auto action_show_data = menu.addAction("Show Surrounding Data");
            QObject::connect (action_show_data, &QAction::triggered, [ = ] () { this->showSurroundingData(utn); });
        }

        //no harm showing this one in locked state
        createRequirementLinkMenu(utn, menu);

        if (calculator_ && !isLocked())
        {
            auto usage_menu = menu.addMenu("Target Usage");
            COMPASS::instance().dbContentManager().targetListWidget()->createTargetEvalMenu(*usage_menu, { utn }, true);
        }

        return true;
    }

    return false;
}

/**
 */
bool EvaluationTaskResult::customMenu_impl(QMenu& menu, 
                                           ResultReport::SectionContent* content)
{
    if (!calculator_)
        return false;

    if (content->contentType() == ResultReport::SectionContent::ContentType::Table)
    {
        if (content->name() == EvaluationData::TargetsTableName)
        {
            const ResultReport::SectionContentTable* table = dynamic_cast<const ResultReport::SectionContentTable*>(content);
            traced_assert(table);

            createInterestMenu(menu);

            return true;
        }
    }

    return false;
}

/**
 */
void EvaluationTaskResult::postprocessTable_impl(ResultReport::SectionContentTable* table)
{
    if (table->name() == EvaluationRequirementResult::Single::TRDetailsTableName)
    {
        //target report details table in single result section
    }
    else if (table->name() == EvaluationRequirementResult::Joined::SectorTargetsTableName)
    {
        //target table in joined result section
    }
    else if (table->name() == EvaluationRequirementResult::Base::RequirementOverviewTableName)
    {
        //requirement table in overview section
    }
    else if (table->name() == EvaluationData::TargetsTableName)
    {
        //evaluation target table
        calculator_->data().postprocessTargetsTable(*table);
    }
}

/**
 */
bool EvaluationTaskResult::hasCustomTooltip_impl(const ResultReport::SectionContentTable* table, 
                                                 unsigned int row,
                                                 unsigned int col) const
{
    if (table->name() == EvaluationData::TargetsTableName)
    {
        //evaluation target table
        return calculator_->data().hasTargetTableTooltip(col);
    }

    return false;
}

/**
 */
std::string EvaluationTaskResult::customTooltip_impl(const ResultReport::SectionContentTable* table, 
                                                     unsigned int row,
                                                     unsigned int col) const
{
    if (table->name() == EvaluationData::TargetsTableName)
    {
        //evaluation target table
        auto utn = helpers::utnFromTable(table, row);

        const auto& target = targets_.at(utn);

        return calculator_->data().targetTableToolTip(target, col,
            [ this ] (const Evaluation::RequirementSumResultID& id) { return this->interestFactorEnabled(id); });
    }

    return "";
}

/**
 */
void EvaluationTaskResult::updateTargets()
{
    auto& dbcontent_man = COMPASS::instance().dbContentManager();

    for (auto& t : targets_)
        EvaluationTargetData::updateTarget(dbcontent_man, t.second);
}

/**
 */
void EvaluationTaskResult::showUTN(unsigned int utn) const
{
    if (!calculator_)
        return;

    calculator_->showUTN(utn);
}

/**
 */
void EvaluationTaskResult::showFullUTN(unsigned int utn) const
{
    if (!calculator_)
        return;

    calculator_->showFullUTN(utn);
}

/**
 */
void EvaluationTaskResult::showSurroundingData(unsigned int utn) const
{
    if (targets_.count(utn) == 0)
    {
        logerr << "utn " << utn << " not found in targets";
        return;
    }

    if (!calculator_)
        return;

    calculator_->showSurroundingData(targets_.at(utn));
}

/**
 */
void EvaluationTaskResult::updateInterestSwitches()
{
    interest_factor_enabled_.clear();

    if (!calculator_)
        return;

    auto req_names = calculator_->currentRequirementNames();

    for (const auto& req : req_names)
    {
        interest_factor_enabled_[ req ] = true;
    }
}

/**
 */
const std::map<std::string, bool>& EvaluationTaskResult::interestSwitches() const
{
    return interest_factor_enabled_;
}

/**
 */
bool EvaluationTaskResult::interestFactorEnabled(const Evaluation::RequirementSumResultID& id) const
{
    auto it = interest_factor_enabled_.find(id.req_name);
    if (it == interest_factor_enabled_.end())
        return false;

    return it->second;
}

/**
 */
void EvaluationTaskResult::setInterestFactorEnabled(const Evaluation::RequirementSumResultID& id, bool ok)
{
    traced_assert(calculator_);

    interest_factor_enabled_.at(id.req_name) = ok;

    updateContent(TaskResultContentID(EvaluationData::SectionID, EvaluationData::TargetsTableName, ResultReport::SectionContentType::Table));
}

/**
 */
void EvaluationTaskResult::setInterestFactorEnabled(const std::string& req_name, bool ok)
{
    traced_assert(calculator_);

    interest_factor_enabled_.at(req_name) = ok;

    updateContent(TaskResultContentID(EvaluationData::SectionID, EvaluationData::TargetsTableName, ResultReport::SectionContentType::Table));
}

/**
 */
void EvaluationTaskResult::setInterestFactorsEnabled(bool ok)
{
    for (auto& it : interest_factor_enabled_)
        it.second = ok;

    updateContent(TaskResultContentID(EvaluationData::SectionID, EvaluationData::TargetsTableName, ResultReport::SectionContentType::Table));
}

/**
 */
EvaluationTarget::InterestMap EvaluationTaskResult::activeInterestFactors(unsigned int utn) const
{
    EvaluationTarget::InterestMap interest_factors;

    traced_assert(targets_.count(utn));

    const auto& target = targets_.at(utn);

    auto ifactors = target.interestFactors();

    for (const auto& ifactor : ifactors)
    {
        const auto& id = ifactor.first;
        if (!interestFactorEnabled(id))
            continue;

        interest_factors[ id ] = ifactor.second;
    }

    return interest_factors;
}

/**
 */
void EvaluationTaskResult::updateInterestMenu()
{
    if (!interest_menu_)
        return;

    for (const auto& ife : interestSwitches())
    {
        auto cb = interest_boxes_.at(ife.first);

        cb->blockSignals(true);
        cb->setChecked(ife.second);
        cb->blockSignals(false);
    }
}

/**
 */
void EvaluationTaskResult::createInterestMenu(QMenu& menu)
{
    if (interest_factor_enabled_.empty())
        return;

    if (!interest_menu_)
    {
        interest_menu_.reset(new QMenu("Edit Shown Interest Factors"));

        auto w      = new QWidget;
        auto layout = new QHBoxLayout;

        w->setLayout(layout);

        auto button_all  = new QPushButton("All");
        auto button_none = new QPushButton("None");

        auto buttonCB = [ this ] (bool ok)
        {
            this->setInterestFactorsEnabled(ok);
            this->updateInterestMenu();
        };

        QObject::connect(button_all , &QPushButton::pressed, [ = ] () { buttonCB(true);  });
        QObject::connect(button_none, &QPushButton::pressed, [ = ] () { buttonCB(false); });

        layout->addWidget(button_all);
        layout->addWidget(button_none);
        layout->addStretch(1);

        auto wa = new QWidgetAction(interest_menu_.get());
        wa->setDefaultWidget(w);

        interest_menu_->addAction(wa);

        for (const auto& ife : interestSwitches())
        {
            std::string req_name = ife.first;

            auto wa = new QWidgetAction(interest_menu_.get());
            auto cb = new QCheckBox(QString::fromStdString(req_name));

            wa->setDefaultWidget(cb);
            interest_menu_->addAction(wa);

            auto clickCB = [ this, req_name ] (bool ok) 
            { 
                this->setInterestFactorEnabled(req_name, ok);
            };

            QObject::connect(cb, &QCheckBox::toggled, clickCB);

            interest_boxes_[ req_name ] = cb;
        }
    }

    updateInterestMenu();

    menu.addMenu(interest_menu_.get());
}

/**
 */
void EvaluationTaskResult::createRequirementLinkMenu(unsigned int utn, QMenu& menu)
{
    auto ifactors = activeInterestFactors(utn);

    if (!ifactors.empty())
    {
        //menu.addSeparator();

        auto req_menu = menu.addMenu("Jump to Requirement");

        for (const auto& ifactor : ifactors)
        {
            const auto& id = ifactor.first;

            QAction* action = EvaluationTargetData::interestFactorAction(id, ifactor.second);

            req_menu->addAction(action);

            QObject::connect(action, &QAction::triggered, [ this, id, utn ] () { this->jumpToRequirement(id, utn, true); });
        }
    }
}

/**
 */
void EvaluationTaskResult::jumpToRequirement(const Evaluation::RequirementSumResultID& id, 
                                             unsigned int utn, 
                                             bool show_image)
{
    if (!calculator_ || !report_)
        return;

    std::string sum_id = EvalSectionID::requirementResultSumID(id);

    loginf << "sum id: " << sum_id;

    std::string utn_id = EvalSectionID::createForTargetResult(utn, id);

    loginf << "utn id: " << utn_id;

    report_->setCurrentSection(utn_id, show_image);
}

/**
 */
void EvaluationTaskResult::informUpdateEvalResult(int update_type)
{
    //update_type = task::Locked;

    TaskResult::ContentID content_id;
    if (update_type == task::ContentUpdateNeeded)
    {
        content_id = TaskResult::ContentID(EvaluationData::SectionID, EvaluationData::TargetsTableName, ResultReport::SectionContentType::Table);
    }

    //inform update
    informUpdate((task::UpdateState)update_type, content_id);
}

/**
 */
void EvaluationTaskResult::toJSON_impl(nlohmann::json& root_node) const
{
    //write evaluation targets
    auto j_targets = nlohmann::json::array();

    for (const auto& target_it : targets_)
    {
        nlohmann::json j_target;
        j_target[ FieldTargetUTN  ] = target_it.first;
        j_target[ FieldTargetInfo ] = target_it.second.info();

        j_targets.push_back(j_target);
    }

    root_node[ FieldTargets ] = j_targets;
}

/**
 */
bool EvaluationTaskResult::fromJSON_impl(const nlohmann::json& j)
{
    if (!j.contains(FieldTargets))
        return false;

    // read evaluation targets
    const auto& j_targets = j[ FieldTargets ];
    if (!j_targets.is_array())
        return false;

    for (const auto& j_target : j_targets)
    {
        if (!j_target.is_object()              ||
            !j_target.contains(FieldTargetUTN) ||
            !j_target.contains(FieldTargetInfo))
            return false;

        unsigned int utn  = j_target[ FieldTargetUTN  ];
        const auto&  info = j_target[ FieldTargetInfo ];

        if(!info.is_object())
            return false;

        targets_.emplace(utn, EvaluationTarget(utn, info));
    }

    return true;
}
