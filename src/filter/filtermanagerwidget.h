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

class FilterManager;
class FilterGeneratorWidget;

class QVBoxLayout;
class QCheckBox;
class QMenu;
class QScrollArea;

/**
 */
class FilterManagerWidget : public ToolBoxWidget
{
    Q_OBJECT

  public slots:
    void filterWidgetActionSlot(bool generated);

public:
    explicit FilterManagerWidget(FilterManager& manager, 
                                 QWidget* parent = nullptr,
                                 Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~FilterManagerWidget();

    //ToolBoxWidget
    QIcon toolIcon() const override final;
    std::string toolName() const override final;
    std::string toolInfo() const override final;
    std::vector<std::string> toolLabels() const override final;
    toolbox::ScreenRatio defaultScreenRatio() const override final;
    void addToConfigMenu(QMenu* menu) override final;
    void addToToolBar(QToolBar* tool_bar) override final; 
    void rightClicked() override final;
    void loadingStarted() override final;
    void loadingDone() override final;

    QCheckBox* filtersCheckBox() const;

    void updateFilters();
    void updateUseFilters();

    void addMenuEntries(QMenu* menu);

protected:
    void toggleUseFilters();

    void addFilter();

    void databaseOpened();

    void expandAll();
    void collapseAll();
    void collapseUnused();

    void syncFilterLayouts();

    FilterManager& filter_manager_;
    std::unique_ptr<FilterGeneratorWidget> filter_generator_widget_;

    QCheckBox*   filters_check_    {nullptr};
    QVBoxLayout* ds_filter_layout_ {nullptr};
    QScrollArea* scroll_area_      {nullptr};
};
