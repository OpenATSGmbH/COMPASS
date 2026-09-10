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
#include "eval/results/reporttablecontent.h"

#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/section.h"
#include "task/taskdefs.h"
#include "taskmanager.h"
#include "radarplotpositioncalculatortask.h"

#include "compass.h"
#include "dbinterface.h"
#include "dbcontentmanager.h"
#include "buffer.h"
#include "viewpoint.h"
#include "viewpointgenerator.h"
#include "timeconv.h"
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
                                           TaskManager& task_man,
                                           COMPASS& compass)
:   TaskResult(id, task_man)
,   compass_(compass)
{
    //connected here and not during init or finalization: a result whose content is not read yet
    //must still follow the update and lock state of the evaluation manager
    connect(&compass_.evaluationManager(), &EvaluationManager::resultsNeedUpdate,
            this, &EvaluationTaskResult::informUpdateEvalResult);
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
    calculator_failed_ = false;
}

/**
 * Creates the calculator on first use. Building it clones the stored configuration and runs a
 * sector check, so it is deferred until the result is actually used.
 */
EvaluationCalculator* EvaluationTaskResult::calculator() const
{
    if (calculator_)
        return calculator_.get();

    if (calculator_failed_)
        return nullptr;

    //set upfront, so neither a failed nor a re-entered creation is run twice
    calculator_failed_ = true;

    auto res = const_cast<EvaluationTaskResult*>(this)->createCalculator();

    if (!res.ok())
    {
        logerr << "could not create calculator for result '" << name() << "': " << res.error();
        return nullptr;
    }

    calculator_failed_ = false;

    return calculator_.get();
}

/**
 */
Result EvaluationTaskResult::createCalculator()
{
    //the configuration is part of the result content
    ensureContentLoaded();

    calculator_.reset();

    loginf << "creating calculator for result '" << name() << "' with config: " << config_.dump();

    //clone calculator from config
    auto res = EvaluationCalculator::clone(compass_.evaluationManager(),
                                              compass_.dbContentManager(),
                                              config_);
    if (!res.ok())
        return res;

    calculator_.reset(res.result());

    //always preserve report name
    calculator_->setCustomReportName(name());
    
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
    //the calculator is created on first use, see calculator()
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
    //the eval manager connection is made in the constructor
    return Result::succeeded();
}

/**
 */
Result EvaluationTaskResult::update_impl(UpdateState state)
{
    Result res = Result::succeeded();

    auto calc = calculator();

    if (!calc)
        return Result::failed("Calculator not initialized");

    if (state == UpdateState::FullUpdateNeeded ||
        state == UpdateState::Locked)
    {
        // sync: run full evaluation with updated constraints (also needed to remove lock)
        loginf << "running full update";
        res = calc->evaluate();

        loginf << calc->constraintsAsString();
    }
    else if (state == UpdateState::PartialUpdateNeeded)
    {
        // partial update: decide if full update is needed anyways
        bool needs_recompute = !calc->evaluated();

        if (needs_recompute)
        {
            // full update needed, because result is yet uninitialized
            loginf << "running initial full update";
            res = calc->evaluate();
        }
        else
        {
            // only partial update needed
            loginf << "running partial update";
            calc->updateResultsToChanges();
        }
    }

    return res;
}

/**
 */
Result EvaluationTaskResult::canUpdate_impl(UpdateState state) const
{
    //true for all kinds of updates
    auto calc = calculator();

    if (!calc)
        return Result::failed("Calculator not initialized");

    auto r = calc->canEvaluate();
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
    Evaluation::RequirementResultID joinedResultContentProperties(const ResultReport::SectionContent* content)
    {
        traced_assert(content);

        auto info = EvaluationRequirementResult::Joined::joinedContentProperties(*content);
        traced_assert(info.has_value());

        return info.value();
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

        //the result exists in the session, a stored report has no joined results
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
 * Locates the Report Table of an on-demand content of a single result. The content carries the
 * table key, a report written before the key existed falls back to the key the definition builds
 * from layer, group and requirement.
 */
const ReportTableInfo* EvaluationTaskResult::reportTableFor(const ResultReport::SectionContent& content,
                                                            EvaluationRequirementResult::Single::ContentInfo& info) const
{
    auto content_info = EvaluationRequirementResult::Single::singleContentProperties(content);
    if (!content_info.has_value())
        return nullptr;

    info = content_info.value();

    if (!info.report_table_key.empty() && hasReportTable(info.report_table_key))
        return &reportTable(info.report_table_key);

    auto key = ReportTableDefinition::identifierFrom(info.id.sec_layer_name + "_"
                                                     + info.id.req_group_name + "_"
                                                     + info.id.req_name);

    if (hasReportTable(key))
        return &reportTable(key);

    logerr << "no report table for"
           << " utn " << info.utn
           << " layer " << info.id.sec_layer_name
           << " group " << info.id.req_group_name
           << " req " << info.id.req_name;

    return nullptr;
}

/**
 * Reads the rows of one target from a Report Table, ordered by time.
 */
std::shared_ptr<Buffer> EvaluationTaskResult::loadReportTableRows(const ReportTableInfo& table,
                                                                   unsigned int utn) const
{
    const std::string table_name = table.tableName(id());

    const std::string filter = "\"" + ReportTableDefinition::UTNColumnName + "\" = " + std::to_string(utn)
                             + " ORDER BY \"" + ReportTableDefinition::TimestampColumnName + "\"";

    auto res = compass_.dbInterface().select(table_name, table.propertyList(), filter);

    if (!res.ok())
    {
        logerr << "could not read table '" << table_name << "': " << res.error();
        return nullptr;
    }

    return res.result();
}

/**
 * Viewable of one target, built from the rows of its Report Table. The base viewable loads the
 * target, the annotations show the rows, and one row is highlighted on demand.
 */
std::shared_ptr<nlohmann::json::object_t> EvaluationTaskResult::createTargetViewable(
    const ReportTableInfo& table,
    const Buffer& buffer,
    const EvaluationRequirementResult::Single::ContentInfo& info,
    boost::optional<unsigned int> highlight_row) const
{
    auto calc = calculator();
    if (!calc)
        return nullptr;

    auto viewable = calc->getViewableForUTN(info.utn);
    if (!viewable)
        return nullptr;

    const double zoom = calc->settings().result_detail_zoom_;

    if (highlight_row.has_value())
    {
        auto positions = ReportTableContent::rowPositions(table, buffer, highlight_row.value());

        if (positions.valid())
        {
            (*viewable)[ ViewPoint::VP_POS_LAT_KEY     ] = positions.event->first;
            (*viewable)[ ViewPoint::VP_POS_LON_KEY     ] = positions.event->second;
            (*viewable)[ ViewPoint::VP_POS_WIN_LAT_KEY ] = zoom;
            (*viewable)[ ViewPoint::VP_POS_WIN_LON_KEY ] = zoom;
        }

        auto timestamp = ReportTableContent::rowTimestamp(buffer, highlight_row.value());
        if (timestamp.has_value())
            (*viewable)[ ViewPoint::VP_TIMESTAMP_KEY ] = Utils::Time::toString(timestamp.value());
    }
    else
    {
        auto bounds = ReportTableContent::failedRowBounds(table, buffer);

        if (!bounds.isEmpty() && !bounds.isNull())
        {
            (*viewable)[ ViewPoint::VP_POS_LAT_KEY ] = bounds.center().x();
            (*viewable)[ ViewPoint::VP_POS_LON_KEY ] = bounds.center().y();

            (*viewable)[ ViewPoint::VP_POS_WIN_LAT_KEY ] = std::max(bounds.width() , zoom);
            (*viewable)[ ViewPoint::VP_POS_WIN_LON_KEY ] = std::max(bounds.height(), zoom);
        }
    }

    //root annotation of the requirement, the same id the result objects use
    (*viewable)[ ViewPoint::VP_ANNOTATION_KEY ] = nlohmann::json::array();
    auto& annotations = (*viewable)[ ViewPoint::VP_ANNOTATION_KEY ];

    ViewPointGenAnnotation root_annotation("Evaluation:" + info.id.req_name + ":UTN" + std::to_string(info.utn));

    nlohmann::json root_annotation_json;
    root_annotation.toJSON(root_annotation_json);

    annotations.push_back(root_annotation_json);

    auto& result_annotations = ViewPointGenAnnotation::getChildrenJSON(annotations.at(0));

    ReportTableContent::createOverviewAnnotations(result_annotations, table, buffer);

    if (highlight_row.has_value())
        ReportTableContent::createHighlightAnnotations(result_annotations, table, buffer, highlight_row.value());

    return std::make_shared<nlohmann::json::object_t>(*viewable);
}

/**
 */
bool EvaluationTaskResult::loadOnDemandFigure_impl(ResultReport::SectionContentFigure* figure) const
{
    try
    {
        if (figure->name() == EvaluationRequirementResult::Single::TargetOverviewID)
        {
            EvaluationRequirementResult::Single::ContentInfo info;

            auto table = reportTableFor(*figure, info);
            if (!table)
                return false;

            auto buffer = loadReportTableRows(*table, info.utn);
            if (!buffer)
                return false;

            auto viewable = createTargetViewable(*table, *buffer, info, boost::optional<unsigned int>());
            if (!viewable)
                return false;

            figure->setViewableFunc([ viewable ] () { return viewable; });

            return true;
        }
    }
    catch(const std::exception& ex)
    {
        logerr << "critical error during load: " << ex.what();
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
    try
    {
        if (table->name() == EvaluationRequirementResult::Single::TRDetailsTableName)
        {
            //target report details table in single result section, read from the report table

            EvaluationRequirementResult::Single::ContentInfo info;

            auto report_table = reportTableFor(*table, info);
            if (!report_table)
                return false;

            auto buffer = loadReportTableRows(*report_table, info.utn);
            if (!buffer)
                return false;

            for (unsigned int row = 0; row < buffer->size(); ++row)
            {
                auto values = ReportTableContent::detailsTableValues(*report_table, *buffer, row);

                if (values.size() != table->numColumns())
                {
                    logerr << "table '" << report_table->key << "' columns " << values.size()
                           << " expected " << table->numColumns();
                    return false;
                }

                table->addRow(values, ResultReport::SectionContentViewable().setOnDemand(), "", "",
                              QPoint((int)row, -1));
            }

            return true;
        }
        else if (table->name() == EvaluationData::TargetsTableName)
        {
            //evaluation targets table
            auto calc = calculator();

            if (!calc)
                return false;

            //fill table with target info
            calc->data().fillTargetsTable(targets_, *table,
                [ this ] (const Evaluation::RequirementSumResultID& id) { return this->interestFactorEnabled(id); });

            return true;
        }
    }
    catch(const std::exception& ex)
    {
        logerr << "critical error during load: " << ex.what();
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
    if (content.contentType() == ResultReport::SectionContent::ContentType::Table)
    {
        if (content.name() == EvaluationRequirementResult::Single::TRDetailsTableName)
        {
            //highlight of one row, read from the report table

            if (!index.isValid())
                return false;

            const QPoint row_index = index.toPoint();
            if (row_index.x() < 0)
                return false;

            EvaluationRequirementResult::Single::ContentInfo info;

            auto report_table = reportTableFor(content, info);
            if (!report_table)
                return false;

            auto buffer = loadReportTableRows(*report_table, info.utn);
            if (!buffer || (unsigned int)row_index.x() >= buffer->size())
                return false;

            auto v = createTargetViewable(*report_table, *buffer, info, (unsigned int)row_index.x());
            if (!v)
                return false;

            viewable.setCallback(*v);

            return true;
        }
        else if (content.name() == EvaluationData::TargetsTableName)
        {
            auto calc = calculator();

            if (!calc)
                return false;

            const ResultReport::SectionContentTable* table = dynamic_cast<const ResultReport::SectionContentTable*>(&content);
            traced_assert(table);

            //obtain utn
            auto utn = helpers::utnFromTable(table, row);

            //configure viewable
            auto content = calc->getViewableForUTN(utn);
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

        if (calculator() && !isLocked())
        {
            auto action_show_utn = menu.addAction("Show Full UTN");
            QObject::connect (action_show_utn, &QAction::triggered, [ = ] () { this->showFullUTN(utn); });

            auto action_show_data = menu.addAction("Show Surrounding Data");
            QObject::connect (action_show_data, &QAction::triggered, [ = ] () { this->showSurroundingData(utn); });

            auto usage_menu = menu.addMenu("Target Usage");

            const auto& target             = targets_.at(utn);
            auto        target_list_widget = compass_.dbContentManager().targetListWidget();

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

        if (calculator() && !isLocked())
        {
            auto action_show_utn = menu.addAction("Show Full UTN");
            QObject::connect (action_show_utn, &QAction::triggered, [ = ] () { this->showFullUTN(utn); });

            auto action_show_data = menu.addAction("Show Surrounding Data");
            QObject::connect (action_show_data, &QAction::triggered, [ = ] () { this->showSurroundingData(utn); });
        }

        //no harm showing this one in locked state
        createRequirementLinkMenu(utn, menu);

        if (calculator() && !isLocked())
        {
            auto usage_menu = menu.addMenu("Target Usage");
            compass_.dbContentManager().targetListWidget()->createTargetEvalMenu(*usage_menu, { utn }, true);
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
    if (!calculator())
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
        auto calc = calculator();

        if (calc)
            calc->data().postprocessTargetsTable(*table);
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
        auto calc = calculator();

        return calc ? calc->data().hasTargetTableTooltip(col) : false;
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

        auto calc = calculator();

        if (!calc)
            return "";

        return calc->data().targetTableToolTip(target, col,
            [ this ] (const Evaluation::RequirementSumResultID& id) { return this->interestFactorEnabled(id); });
    }

    return "";
}

/**
 */
void EvaluationTaskResult::updateTargets()
{
    auto& dbcontent_man = compass_.dbContentManager();

    for (auto& t : targets_)
        EvaluationTargetData::updateTarget(dbcontent_man, t.second);
}

/**
 */
void EvaluationTaskResult::showUTN(unsigned int utn) const
{
    auto calc = calculator();

    if (!calc)
        return;

    calc->showUTN(utn);
}

/**
 */
void EvaluationTaskResult::showFullUTN(unsigned int utn) const
{
    auto calc = calculator();

    if (!calc)
        return;

    calc->showFullUTN(utn);
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

    auto calc = calculator();

    if (!calc)
        return;

    calc->showSurroundingData(targets_.at(utn));
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
    //the switches are filled when the calculator is created
    calculator();

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

    updateContent(TaskResultContentID(EvaluationData::TargetsSectionID, EvaluationData::TargetsTableName, ResultReport::SectionContentType::Table));
}

/**
 */
void EvaluationTaskResult::setInterestFactorEnabled(const std::string& req_name, bool ok)
{
    traced_assert(calculator_);

    interest_factor_enabled_.at(req_name) = ok;

    updateContent(TaskResultContentID(EvaluationData::TargetsSectionID, EvaluationData::TargetsTableName, ResultReport::SectionContentType::Table));
}

/**
 */
void EvaluationTaskResult::setInterestFactorsEnabled(bool ok)
{
    for (auto& it : interest_factor_enabled_)
        it.second = ok;

    updateContent(TaskResultContentID(EvaluationData::TargetsSectionID, EvaluationData::TargetsTableName, ResultReport::SectionContentType::Table));
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
    if (!calculator() || !report_)
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
        content_id = TaskResult::ContentID(EvaluationData::TargetsSectionID, EvaluationData::TargetsTableName, ResultReport::SectionContentType::Table);
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
