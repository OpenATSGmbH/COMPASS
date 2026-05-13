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

#include "viewconfigwidget.h"

namespace dbContent
{
    class VariableSelectionWidget;
}

class ViewWidget;
class VariableView;

class QVBoxLayout;
class QToolButton;
class QWidget;

/**
 */
class VariableViewConfigWidget : public TabStyleViewConfigWidget
{
    Q_OBJECT

public slots:
    void selectedVariableChangedSlot(int idx);

public:
    VariableViewConfigWidget(ViewWidget* view_widget,
                             VariableView* view,
                             QWidget* parent = nullptr);
    virtual ~VariableViewConfigWidget();

    virtual void configChanged() override final;

    void updateConfig();

protected:
    void updateSelectedVariables();
    void updateSelectedVariables(size_t idx);

    void showSwitch(int var0, bool ok);

    const dbContent::VariableSelectionWidget* variableSelection(size_t idx) const;

    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;
    virtual void configChanged_impl() {};

    virtual void variableChangedEvent(int idx) {}
    virtual void dataSourceChangedEvent() {}

    QVBoxLayout* configLayout() { return config_layout_; }

private:
    void switchVariables(int idx0, int idx1);

    VariableView* var_view_      = nullptr;
    QVBoxLayout*  config_layout_ = nullptr;

    std::vector<dbContent::VariableSelectionWidget*> var_selection_widgets_;
    std::vector<QToolButton*> var_switches_;
};
