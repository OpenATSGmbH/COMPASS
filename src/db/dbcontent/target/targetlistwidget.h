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

#include "toolboxwidget.h"

#include <QItemSelection>

#include <set>

#include <boost/date_time/posix_time/ptime.hpp>

class DBContentManager;

class QTableView;
class QSortFilterProxyModel;

class QMenu;

namespace Utils
{
    class TimeWindowCollection;
}

namespace dbContent 
{

class Target;
class TargetModel;

/**
 */
class TargetListWidget : public ToolBoxWidget
{
    Q_OBJECT

public slots:
    void evalUseAllSlot();
    void evalUseNoneSlot();
    void evalFilterSlot();

    void evalEditGlobalExcludeTimeWindowsSlot();

    void clearAllCommentsSlot();
    void evalClearAllExcludeTimeWindowsSlot();
    void evalClearAllExcludeRequirementsSlot();

    void customContextMenuSlot(const QPoint& p);
    void showSurroundingDataSlot ();

    //per-target
    void clearSelectedTargetsCommentsSlot();
    void evalUseSelectedTargetsSlot();
    void evalDisableSelectedTargetsSlot();
    void evalClearTargetsExcludeTimeWindowsSlot();
    void evalClearTargetsExcludeRequirementsSlot();
    void evalExcludeTimeWindowsTargetSlot();
    void evalExcludeRequirementsTargetSlot();

    //void currentRowChanged(const QModelIndex& current, const QModelIndex& previous);
    void selectionChanged(const QItemSelection& selected, const QItemSelection& deselected);

    void loadingDoneSlot();

public:
    TargetListWidget(TargetModel& model, DBContentManager& dbcont_manager);
    virtual ~TargetListWidget() {};

    //ToolBoxWidget
    QIcon toolIcon() const override final;
    std::string toolName() const override final;
    std::string toolInfo() const override final;
    std::vector<std::string> toolLabels() const override final;
    toolbox::ScreenRatio defaultScreenRatio() const override final;
    void addToConfigMenu(QMenu* menu) override final;
    void addToToolBar(QToolBar* tool_bar) override final;
    void loadingStarted() override final;
    void loadingDone() override final;

    void resizeColumnsToContents();

    void createTargetEvalMenu(QMenu& menu, 
                              const std::set<unsigned int>& utns,
                              bool check_utns = false);
    void createTargetEvalMenu(QMenu& menu, 
                              const Target& target,
                              const std::string& req_name);
protected:
    void showMainColumns(bool show);
    void showEvalColumns(bool show);
    void showDurationColumns(bool show);
    void showModeACColumns(bool show);
    void showModeSColumns(bool show);
    void showADSBColumns(bool show);

    void clearSelectedTargetsComments(const std::set<unsigned int>& utns);
    void evalUseSelectedTargets(const std::set<unsigned int>& utns);
    void evalDisableSelectedTargets(const std::set<unsigned int>& utns);
    void evalClearTargetsExcludeTimeWindows(const std::set<unsigned int>& utns);
    void evalClearTargetsExcludeRequirements(const std::set<unsigned int>& utns);
    void evalExcludeTimeWindowsTarget(const std::set<unsigned int>& utns,
                                      const Utils::TimeWindowCollection* exclude_windows = nullptr);
    void evalExcludeRequirementsTarget(const std::set<unsigned int>& utns,
                                       const std::set<std::string>* exclude_requirements = nullptr);

    void evalExcludeTimeWindowTarget(const Target& target);
    void evalExcludeRequirementTarget(const Target& target,
                                      const std::string& req_name);
    void evalExcludeAllRequirementsTarget(const Target& target);

    TargetModel& model_;
    DBContentManager& dbcont_manager_;

    QTableView* table_view_{nullptr};
    QSortFilterProxyModel* proxy_model_{nullptr};

    std::set<unsigned int> selectedUTNs() const;
};

};
