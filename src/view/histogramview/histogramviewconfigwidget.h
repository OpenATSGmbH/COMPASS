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

#include "variableviewconfigwidget.h"

class HistogramView;
class HistogramViewWidget;

class ViewLayerPanelWidget;

namespace dbContent {
class VariableOrderedSetWidget;
class VariableSelectionWidget;
}

class QCheckBox;
class QRadioButton;
class QLineEdit;
class QPushButton;
class QLabel;
class GroupBox;

/**
 * @brief Widget with configuration elements for a HistogramView
 *
 */
class HistogramViewConfigWidget : public VariableViewConfigWidget
{
    Q_OBJECT

public:
    HistogramViewConfigWidget(HistogramViewWidget* view_widget, QWidget* parent = nullptr);
    virtual ~HistogramViewConfigWidget();

protected:
    void updateLogScale();

    void toggleLogScale();

    virtual void onDisplayChange_impl() override;
    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;

    virtual void configChanged_impl() override;

    HistogramView* view_ = nullptr;

    // general
    QCheckBox* log_check_{nullptr};

    ViewLayerPanelWidget*  layer_panel_      {nullptr};
};
