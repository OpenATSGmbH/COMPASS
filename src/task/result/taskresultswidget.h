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
#include "reportdefs.h"

#include <boost/optional.hpp>

#include <QWidget>

#include "json.hpp"

class TaskManager;

namespace ResultReport
{
    class ReportWidget;
}

class QComboBox;
class QPushButton;

class TaskResultsWidget : public ToolBoxWidget
{
    Q_OBJECT

public slots:
    void updateResultsSlot();

public:
    TaskResultsWidget(TaskManager& task_man);
    virtual ~TaskResultsWidget();

    void setReport(const std::string name);
    void selectID(const std::string id,
                  bool show_figure = false);

    void storeBackupSection();
    void restoreBackupSection();

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

    std::string currentReportName() const;

protected:
    void updateResults(const std::string& selected_result = "");
    void resultHeaderChanged(const QString& name);
    void updateResultUI(const std::string& name);

    void removeCurrentResult();
    bool removeResult(const std::string& name);
    void exportCurrentResult(ResultReport::ReportExportMode mode);
    void refreshCurrentResult();

    TaskManager& task_man_;

    QComboBox*   report_combo_          {nullptr};
    QPushButton* remove_result_button_  {nullptr};
    QPushButton* export_result_button_  {nullptr};
    QPushButton* refresh_result_button_ {nullptr};

    std::string    current_report_name_;
    std::string    current_report_name_backup_;
    std::string    current_section_name_backup_;
    nlohmann::json current_section_config_backup_;

    ResultReport::ReportWidget* report_widget_;
};
