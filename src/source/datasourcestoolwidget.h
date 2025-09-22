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

class DataSourcesWidget;
class DataSourceManager;


class QLabel;

class DataSourcesToolWidget : public ToolBoxWidget
{
    Q_OBJECT

public:
    DataSourcesToolWidget(DataSourceManager& ds_man);
    virtual ~DataSourcesToolWidget();

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

    void updateContent(bool recreate_required = false);

private:
    DataSourceManager& ds_man_;
    DataSourcesWidget* ds_widget_{nullptr};

    QLabel* ts_min_label_{nullptr};
    QLabel* ts_max_label_{nullptr};
    QLabel* associations_label_{nullptr}; 

    void createUI();

    void updateAdditionalInfo();

};