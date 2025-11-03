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

#include <memory>

#include <boost/optional.hpp>

#include "json_fwd.hpp"

class EvaluationManager;
class EvaluationManagerSettings;
class EvaluationTargetsTabWidget;
class EvaluationResultsTabWidget;

class QVBoxLayout;
class QTabWidget;
class QPushButton;
class QLabel;

/**
 */
class EvaluationManagerWidget : public ToolBoxWidget
{
    Q_OBJECT

private slots:
    void generateReportSlot();

public:
    EvaluationManagerWidget(EvaluationManager& eval_man);
    virtual ~EvaluationManagerWidget();

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

    void updateButtons();
    void expandResults();

    void showResultId (const std::string& id, 
                       bool select_tab = false,
                       bool show_figure = false);
    void reshowLastResultId();
    
protected:
    EvaluationManager& eval_man_;

    QTabWidget* tab_widget_{nullptr};

    std::unique_ptr<EvaluationTargetsTabWidget> targets_tab_widget_;
    std::unique_ptr<EvaluationResultsTabWidget> results_tab_widget_;

    QPushButton* gen_report_button_ {nullptr};
};
