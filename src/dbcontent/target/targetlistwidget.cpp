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


#include "targetlistwidget.h"
#include "targetmodel.h"
#include "evaluationtargetfilterdialog.h"

#include "compass.h"
#include "taskmanager.h"
#include "evaluationmanager.h"
#include "dbcontentmanager.h"
#include "evaluationtimestampconditionsdialog.h"
#include "evaluationtargetexcludedtimewindowsdialog.h"
#include "evaluationtargetexcludedrequirementsdialog.h"
#include "evaluationcalculator.h"
#include "evaluationstandard.h"

#include "files.h"
#include "logger.h"
#include "stringconv.h"

#include <QTableView>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QSortFilterProxyModel>
#include <QMenu>
#include <QApplication>
#include <QThread>
#include <QToolBar>
#include <QMessageBox>

using namespace std;
using namespace Utils;

namespace dbContent 
{

TargetListWidget::TargetListWidget(TargetModel& model, DBContentManager& dbcont_manager)
    : ToolBoxWidget(), model_(model), dbcont_manager_(dbcont_manager)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    // table
    proxy_model_ = new QSortFilterProxyModel();
    proxy_model_->setSourceModel(&model_);

    table_view_ = new QTableView();
    table_view_->setModel(proxy_model_);
    table_view_->setSortingEnabled(true);
    table_view_->sortByColumn(0, Qt::AscendingOrder);
    table_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_view_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    table_view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
    table_view_->horizontalHeader()->setMaximumSectionSize(300);
    //table_view_->setIconSize(QSize(24, 24));
    table_view_->setContextMenuPolicy(Qt::CustomContextMenu);
    table_view_->setWordWrap(true);
    table_view_->reset();
    // update done later

    connect(table_view_, &QTableView::customContextMenuRequested,
            this, &TargetListWidget::customContextMenuSlot);

    // connect(table_view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
    //         this, &TargetListWidget::currentRowChanged);

    connect(table_view_->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &TargetListWidget::selectionChanged);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
    main_layout->addWidget(table_view_);

    setLayout(main_layout);

    showMainColumns(model_.showMainColumns());
    showEvalColumns(model_.showEvalColumns());
    showDurationColumns(model_.showDurationColumns());
    showModeSColumns(model_.showModeSColumns());
    showModeACColumns(model_.showModeACColumns());
    showADSBColumns(model_.showADSBColumns());
}

/**
 */
QIcon TargetListWidget::toolIcon() const
{
    return QIcon(Utils::Files::getIconFilepath("targets.png").c_str());
}

/**
 */
std::string TargetListWidget::toolName() const 
{
    return "Targets";
}

/**
 */
std::string TargetListWidget::toolInfo() const 
{
    return "Targets";
}

/**
 */
std::vector<std::string> TargetListWidget::toolLabels() const 
{
    return { "Targets" };
}

/**
 */
toolbox::ScreenRatio TargetListWidget::defaultScreenRatio() const 
{
    return ToolBoxWidget::defaultScreenRatio();
}

/**
 */
void TargetListWidget::addToConfigMenu(QMenu* menu) 
{
    QAction* clear_action = menu->addAction("Clear All Comments");
    connect (clear_action, &QAction::triggered, this, &TargetListWidget::clearAllCommentsSlot);

    QMenu* column_menu = menu->addMenu("Edit Columns");

    // auto main_cols_action = column_menu->addAction("Main");
    // main_cols_action->setCheckable(true);
    // main_cols_action->setChecked(model_.showMainColumns());

    // connect(main_cols_action, &QAction::toggled, this, &TargetListWidget::showMainColumns);
    // connect(main_cols_action, &QAction::toggled, this, &TargetListWidget::toolsChangedSignal);

    auto eval_cols_action = column_menu->addAction("Eval Details");
    eval_cols_action->setCheckable(true);
    eval_cols_action->setChecked(model_.showEvalColumns());

    connect(eval_cols_action, &QAction::toggled, this, &TargetListWidget::showEvalColumns);
    connect(eval_cols_action, &QAction::toggled, this, &TargetListWidget::toolsChangedSignal);

    auto dur_cols_action = column_menu->addAction("Duration");
    dur_cols_action->setCheckable(true);
    dur_cols_action->setChecked(model_.showDurationColumns());

    connect(dur_cols_action, &QAction::toggled, this, &TargetListWidget::showDurationColumns);
    connect(dur_cols_action, &QAction::toggled, this, &TargetListWidget::toolsChangedSignal);

    auto mode_s_cols_action = column_menu->addAction("Mode S");
    mode_s_cols_action->setCheckable(true);
    mode_s_cols_action->setChecked(model_.showModeSColumns());

    connect(mode_s_cols_action, &QAction::toggled, this, &TargetListWidget::showModeSColumns);
    connect(mode_s_cols_action, &QAction::toggled, this, &TargetListWidget::toolsChangedSignal);

    auto mode_ac_cols_action = column_menu->addAction("Mode A/C");
    mode_ac_cols_action->setCheckable(true);
    mode_ac_cols_action->setChecked(model_.showModeACColumns());

    connect(mode_ac_cols_action, &QAction::toggled, this, &TargetListWidget::showModeACColumns);
    connect(mode_ac_cols_action, &QAction::toggled, this, &TargetListWidget::toolsChangedSignal);

    auto mode_adsb_action = column_menu->addAction("ADS-B");
    mode_adsb_action->setCheckable(true);
    mode_adsb_action->setChecked(model_.showModeACColumns());

    connect(mode_adsb_action, &QAction::toggled, this, &TargetListWidget::showADSBColumns);
    connect(mode_adsb_action, &QAction::toggled, this, &TargetListWidget::toolsChangedSignal);

    menu->addSeparator();

    QMenu* eval_menu = menu->addMenu("Evaluation");

    QAction* all_action = eval_menu->addAction("Use All Targets");
    connect (all_action, &QAction::triggered, this, &TargetListWidget::evalUseAllSlot);

    QAction* none_action = eval_menu->addAction("Disable All Targets");
    connect (none_action, &QAction::triggered, this, &TargetListWidget::evalUseNoneSlot);

    QAction* filter_action = eval_menu->addAction("Filter Usage ...");
    connect (filter_action, &QAction::triggered, this, &TargetListWidget::evalFilterSlot);

    eval_menu->addSeparator();

    QAction* tw_action = eval_menu->addAction("Clear All Targets Excluded Time Windows");
    connect (tw_action, &QAction::triggered, this, &TargetListWidget::evalClearAllExcludeTimeWindowsSlot);

    QAction* req_action = eval_menu->addAction("Clear All Targets Excluded Requirements");
    connect (req_action, &QAction::triggered, this, &TargetListWidget::evalClearAllExcludeRequirementsSlot);

    eval_menu->addSeparator();

    QAction* exlcude_tw_action = eval_menu->addAction("Edit Global Excluded Time Windows ...");
    connect (exlcude_tw_action, &QAction::triggered, this, &TargetListWidget::evalEditGlobalExcludeTimeWindowsSlot);
}

/**
 */
void TargetListWidget::addToToolBar(QToolBar* tool_bar)
{
    // auto main_cols_action = tool_bar->addAction("M");
    // main_cols_action->setCheckable(true);
    // main_cols_action->setChecked(model_.showMainColumns());
    // main_cols_action->setToolTip("Show Main Information");

    // connect(main_cols_action, &QAction::toggled, this, &TargetListWidget::showMainColumns);

    auto eval_cols_action = tool_bar->addAction("Eval Details");
    eval_cols_action->setCheckable(true);
    eval_cols_action->setChecked(model_.showEvalColumns());
    eval_cols_action->setToolTip("Show Evaluation Columns");

    connect(eval_cols_action, &QAction::toggled, this, &TargetListWidget::showEvalColumns);

    auto dur_cols_action = tool_bar->addAction("Duration");
    dur_cols_action->setCheckable(true);
    dur_cols_action->setChecked(model_.showDurationColumns());
    dur_cols_action->setToolTip("Show Duration Columns");

    connect(dur_cols_action, &QAction::toggled, this, &TargetListWidget::showDurationColumns);

    auto mode_s_cols_action = tool_bar->addAction("Mode S");
    mode_s_cols_action->setCheckable(true);
    mode_s_cols_action->setChecked(model_.showModeSColumns());
    mode_s_cols_action->setToolTip("Show Mode S Columns");

    connect(mode_s_cols_action, &QAction::toggled, this, &TargetListWidget::showModeSColumns);

    auto mode_ac_cols_action = tool_bar->addAction("Mode A/C");
    mode_ac_cols_action->setCheckable(true);
    mode_ac_cols_action->setChecked(model_.showModeACColumns());
    mode_ac_cols_action->setToolTip("Show Mode A/C Columns");

    connect(mode_ac_cols_action, &QAction::toggled, this, &TargetListWidget::showModeACColumns);

    auto mode_adsb_action = tool_bar->addAction("ADS-B");
    mode_adsb_action->setCheckable(true);
    mode_adsb_action->setChecked(model_.showModeACColumns());
    mode_adsb_action->setToolTip("Show ADS-B Columns");

    connect(mode_adsb_action, &QAction::toggled, this, &TargetListWidget::showADSBColumns);
}

/**
 */
void TargetListWidget::loadingStarted()
{
    table_view_->setEnabled(false);
}

/**
 */
void TargetListWidget::loadingDone()
{
    table_view_->setEnabled(true);
}

void TargetListWidget::resizeColumnsToContents()
{
    loginf;

    table_view_->resizeColumnsToContents();
}

void TargetListWidget::evalUseAllSlot()
{
    model_.setAllUseTargets(true);
}

void TargetListWidget::evalUseNoneSlot()
{
    model_.setAllUseTargets(false);
}

void TargetListWidget::evalFilterSlot()
{
    loginf;

    EvaluationTargetFilterDialog dialog (COMPASS::instance().evaluationManager().targetFilter(), model_);
    dialog.exec();
}

void TargetListWidget::clearAllCommentsSlot()
{
    model_.clearAllTargetComments();

    resizeColumnsToContents();
}

void TargetListWidget::evalClearAllExcludeTimeWindowsSlot()
{
    loginf;

    model_.clearAllEvalExcludeTimeWindows();

    resizeColumnsToContents();
}

void TargetListWidget::evalClearAllExcludeRequirementsSlot()
{
    loginf;

    model_.clearAllEvalExcludeRequirements();

    resizeColumnsToContents();
}

void TargetListWidget::evalEditGlobalExcludeTimeWindowsSlot()
{
    loginf;

    EvaluationTimestampConditionsDialog dialog (COMPASS::instance().evaluationManager());
    dialog.exec();

    if (dialog.somethingChangedFlag())
    {
        COMPASS::instance().evaluationManager().saveTimeConstraints();

        emit model_.targetEvalFullChangeSignal();

        model_.updateEvalUseColumn();
    }
}

void TargetListWidget::customContextMenuSlot(const QPoint& p)
{
    logdbg;

    traced_assert(table_view_);

    QMenu menu;

    QAction* clear_action = menu.addAction("Clear Comment(s)");
    connect (clear_action, &QAction::triggered, this, &TargetListWidget::clearSelectedTargetsCommentsSlot);

    QAction* show_action = menu.addAction("Show Surrounding Data");
    connect (show_action, &QAction::triggered, this, &TargetListWidget::showSurroundingDataSlot);

    QMenu* eval_menu = menu.addMenu("Evaluation");

    const auto& selected_utns = selectedUTNs();
    createTargetEvalMenu(*eval_menu, selected_utns);

    menu.exec(table_view_->viewport()->mapToGlobal(p));
}

void TargetListWidget::showSurroundingDataSlot ()
{
    auto& dbcont_man = COMPASS::instance().dbContentManager();

    while (dbcont_man.loadInProgress())
    {
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        QThread::msleep(10);
    }

    std::set<unsigned int> selected_utns = selectedUTNs();

    logdbg << "utns " << String::compress(selected_utns,',');

    dbcont_man.showSurroundingData(selected_utns);
}

void TargetListWidget::createTargetEvalMenu(QMenu& menu, 
                                            const std::set<unsigned int>& utns,
                                            bool check_utns)
{
    bool targets_ok = true;
    if (check_utns)
    {
        for (auto utn : utns)
        {
            if (!model_.existsTarget(utn))
            {
                targets_ok = false;
                break;
            }
        }
    }

    QAction* use_action = menu.addAction("Use Target(s)");
    use_action->setEnabled(targets_ok);
    connect (use_action, &QAction::triggered, [ this, utns ] () { this->evalUseSelectedTargets(utns); });

    QAction* nouse_action = menu.addAction("Disable Target(s)");
    nouse_action->setEnabled(targets_ok);
    connect (nouse_action, &QAction::triggered, [ this, utns ] () { this->evalDisableSelectedTargets(utns); });

    menu.addSeparator();

    QAction* tw_action = menu.addAction("Edit Excluded Time Windows");
    tw_action->setEnabled(targets_ok);
    connect (tw_action, &QAction::triggered, [ this, utns ] () { this->evalExcludeTimeWindowsTarget(utns); });

    QAction* tw_clear_action = menu.addAction("Clear Excluded Time Windows");
    tw_clear_action->setEnabled(targets_ok);
    connect (tw_clear_action, &QAction::triggered, [ this, utns ] () { this->evalClearTargetsExcludeTimeWindows(utns); });

    QAction* req_action = menu.addAction("Edit Excluded Requirements");
    req_action->setEnabled(targets_ok);
    connect (req_action, &QAction::triggered, [ this, utns ] () { this->evalExcludeRequirementsTarget(utns); });

    QAction* req_clear_action = menu.addAction("Clear Excluded Requirements");
    req_clear_action->setEnabled(targets_ok);
    connect (req_clear_action, &QAction::triggered, [ this, utns ] () { this->evalClearTargetsExcludeRequirements(utns); });
}

void TargetListWidget::createTargetEvalMenu(QMenu& menu, 
                                            const Target& target,
                                            const std::string& req_name)
{
    bool  target_ok        = model_.existsTarget(target.utn_);
    auto  target_ptr       = &target;
    auto& eval_man         = COMPASS::instance().evaluationManager();
    auto  all_requirements = eval_man.hasCurrentStandard() ? eval_man.currentStandard().getAllRequirementNames() : std::set<std::string>();
    bool  has_requirement  = all_requirements.count(req_name) > 0;
    bool  has_timewin      = !target.timeBegin().is_not_a_date_time() &&
                             !target.timeEnd().is_not_a_date_time();

    QAction* tw_action = menu.addAction("Exclude Time Window");
    tw_action->setEnabled(target_ok && has_timewin);
    connect (tw_action, &QAction::triggered, 
        [ this, target_ptr ] () { this->evalExcludeTimeWindowTarget(*target_ptr); });

    QAction* req_action = menu.addAction("Exclude From Requirement '" + QString::fromStdString(req_name) + "'");
    req_action->setEnabled(target_ok && has_requirement);
    connect (req_action, &QAction::triggered, 
        [ this, target_ptr, req_name ] () { this->evalExcludeRequirementTarget(*target_ptr, req_name); });

    QAction* req_all_action = menu.addAction("Exclude From All Requirements");
    req_all_action->setEnabled(target_ok);
    connect (req_all_action, &QAction::triggered, 
        [ this, target_ptr ] () { this->evalExcludeAllRequirementsTarget(*target_ptr); });
}

void TargetListWidget::clearSelectedTargetsComments(const std::set<unsigned int>& utns)
{
    loginf;

    model_.setTargetComment(utns, "");

    resizeColumnsToContents();
}

void TargetListWidget::evalUseSelectedTargets(const std::set<unsigned int>& utns)
{
    loginf;

    model_.setEvalUseTarget(utns, true);
}

void TargetListWidget::evalDisableSelectedTargets(const std::set<unsigned int>& utns)
{
    loginf;

    model_.setEvalUseTarget(utns, false);
}

void TargetListWidget::evalClearTargetsExcludeTimeWindows(const std::set<unsigned int>& utns)
{
    loginf;

    model_.clearEvalExcludeTimeWindows(utns);

    resizeColumnsToContents();
}

void TargetListWidget::evalClearTargetsExcludeRequirements(const std::set<unsigned int>& utns)
{
    loginf;

    model_.clearEvalExcludeRequirements(utns);

    resizeColumnsToContents();
}

void TargetListWidget::evalExcludeTimeWindowsTarget(const std::set<unsigned int>& utns,
                                                    const Utils::TimeWindowCollection* exclude_windows)
{
    loginf;

    auto& dbcont_man = COMPASS::instance().dbContentManager();

    Utils::TimeWindowCollection filtered_time_windows;
    set<string> comments;

    // collect all time windows from all targets
    for (auto utn : utns)
    {
        traced_assert(dbcont_man.existsTarget(utn));

        auto& target = dbcont_man.target(utn);

        for (auto& tw : target.evalExcludedTimeWindows())
        {
            if (!filtered_time_windows.contains(tw))
                filtered_time_windows.add(tw);
        }

        if (target.comment().size() && !comments.count(target.comment()))
            comments.insert(target.comment());
    }

    // add externally provided time windows
    if (exclude_windows)
    {
        for (const auto& tw : *exclude_windows)
        {
            if (!filtered_time_windows.contains(tw))
                filtered_time_windows.add(tw);
        }
    }

    EvaluationTargetExcludedTimeWindowsDialog dialog (String::compress(utns, ','),
                                                     filtered_time_windows,
                                                     String::compress(comments, '\n'));
    int result = dialog.exec();

    if (result == QDialog::Rejected)
        return;

    string comment = dialog.comment();

    // set time windows for all targets
    model_.setTargetComment(utns, comment);
    model_.setEvalExcludeTimeWindows(utns, filtered_time_windows);

    resizeColumnsToContents();
}

void TargetListWidget::evalExcludeRequirementsTarget(const std::set<unsigned int>& utns,
                                                     const std::set<std::string>* exclude_requirements)
{
    loginf;

    auto& dbcont_man = COMPASS::instance().dbContentManager();
    auto& eval_man = COMPASS::instance().evaluationManager();

    if (!eval_man.hasCurrentStandard())
    {
        QMessageBox m_warning(QMessageBox::Information, "Exclude Requirements",
                              "Please select a current evaluation standard to disable requirements.",
                              QMessageBox::Ok);
        m_warning.exec();

        return;
    }

    set<string> selected_requirements;
    set<string> all_requirements = eval_man.currentStandard().getAllRequirementNames();
    set<string> comments;

    // collect all requirements from all targets
    for (auto utn : utns)
    {
        traced_assert(dbcont_man.existsTarget(utn));

        auto& target = dbcont_man.target(utn);

        for (auto& req_name : target.evalExcludedRequirements())
        {
            if (!selected_requirements.count(req_name))
                selected_requirements.insert(req_name);
        }

        if (target.comment().size() && !comments.count(target.comment()))
            comments.insert(target.comment());
    }

    // add externally provided requirements
    if (exclude_requirements)
    {
        for (const auto& req_name : *exclude_requirements)
        {
            if (!selected_requirements.count(req_name))
                selected_requirements.insert(req_name);
        }
    }

    EvaluationTargetExcludedRequirementsDialog dialog(String::compress(utns, ','),
                                                      selected_requirements, 
                                                      all_requirements,
                                                      String::compress(comments, '\n'));
    int result = dialog.exec();

    if (result == QDialog::Rejected)
        return;

    selected_requirements = dialog.selectedRequirements();
    string comment = dialog.comment();

    // set reqs for all targets
    model_.setTargetComment(utns, comment);
    model_.setEvalExcludeRequirements(utns, selected_requirements);

    resizeColumnsToContents();
}

void TargetListWidget::evalExcludeTimeWindowTarget(const Target& target)
{
    Utils::TimeWindowCollection twc;
    twc.add(Utils::TimeWindow(target.timeBegin(), target.timeEnd()));

    evalExcludeTimeWindowsTarget({ target.utn_ }, &twc);
}

void TargetListWidget::evalExcludeRequirementTarget(const Target& target,
                                                    const std::string& req_name)
{
    std::set<std::string> requirements;
    requirements.insert(req_name);

    evalExcludeRequirementsTarget({ target.utn_ }, &requirements);
}

void TargetListWidget::evalExcludeAllRequirementsTarget(const Target& target)
{
    auto& eval_man         = COMPASS::instance().evaluationManager();
    auto  all_requirements = eval_man.currentStandard().getAllRequirementNames();

    evalExcludeRequirementsTarget({ target.utn_ }, &all_requirements);
}

void TargetListWidget::clearSelectedTargetsCommentsSlot()
{
    clearSelectedTargetsComments(selectedUTNs());
}

void TargetListWidget::evalUseSelectedTargetsSlot()
{
    evalUseSelectedTargets(selectedUTNs());
}

void TargetListWidget::evalDisableSelectedTargetsSlot()
{
    evalDisableSelectedTargets(selectedUTNs());
}

void TargetListWidget::evalClearTargetsExcludeTimeWindowsSlot()
{
    evalClearTargetsExcludeTimeWindows(selectedUTNs());
}

void TargetListWidget::evalClearTargetsExcludeRequirementsSlot()
{
    evalClearTargetsExcludeRequirements(selectedUTNs());
}

void TargetListWidget::evalExcludeTimeWindowsTargetSlot()
{
    evalExcludeTimeWindowsTarget(selectedUTNs());
}

void TargetListWidget::evalExcludeRequirementsTargetSlot()
{
    evalExcludeRequirementsTarget(selectedUTNs());
}

void TargetListWidget::currentRowChanged(const QModelIndex& current, const QModelIndex& previous)
{
    if (!current.isValid())
    {
        loginf << "invalid index";
        return;
    }

    auto const source_index = proxy_model_->mapToSource(current);
    traced_assert(source_index.isValid());

    const dbContent::Target& target = model_.getTargetOf(source_index);

    loginf << "current target " << target.utn_;

    dbcont_manager_.showUTN(target.utn_);
}

void TargetListWidget::selectionChanged(const QItemSelection& selected, const QItemSelection& deselected)
{
    std::set<unsigned int> selected_utns = selectedUTNs();

    dbcont_manager_.showUTNs(selected_utns);
}

void TargetListWidget::showMainColumns(bool show)
{
    model_.showMainColumns(show);

    for (int c : model_.mainColumns())
        table_view_->setColumnHidden(c, !show);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

void TargetListWidget::showEvalColumns(bool show)
{
    model_.showEvalColumns(show);

    for (int c : model_.evalColumns())
        table_view_->setColumnHidden(c, !show);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

void TargetListWidget::showDurationColumns(bool show)
{
    model_.showDurationColumns(show);

    for (int c : model_.durationColumns())
        table_view_->setColumnHidden(c, !show);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

void TargetListWidget::showModeSColumns(bool show)
{
    model_.showModeSColumns(show);

    for (int c : model_.modeSColumns())
        table_view_->setColumnHidden(c, !show);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

void TargetListWidget::showADSBColumns(bool show)
{
    model_.showADSBColumns(show);

    for (int c : model_.adsbColumns())
        table_view_->setColumnHidden(c, !show);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

void TargetListWidget::showModeACColumns(bool show)
{
    model_.showModeACColumns(show);

    for (int c : model_.modeACColumns())
        table_view_->setColumnHidden(c, !show);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

std::set<unsigned int> TargetListWidget::selectedUTNs() const
{
    traced_assert(table_view_);

    std::set<unsigned int> selected_indexes;

    QItemSelectionModel* selection_model = table_view_->selectionModel();

    QModelIndexList selected_indexes_list = selection_model->selectedIndexes();

    for (const QModelIndex& index : selected_indexes_list)
    {
        if (!index.isValid())
            continue;

        auto const source_index = proxy_model_->mapToSource(index);
        traced_assert(source_index.isValid());

        const dbContent::Target& target = model_.getTargetOf(source_index);

        selected_indexes.insert(target.utn_);
    }

    loginf << "start" << String::compress(selected_indexes, ',');

    return selected_indexes;
}

}
