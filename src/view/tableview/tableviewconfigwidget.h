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
//#include "dbcontent/variable/variable.h"
//#include "appmode.h"

namespace dbContent {
    class VariableOrderedSetWidget;
}

class TableView;
class TableViewWidget;

class QCheckBox;
class QPushButton;

/**
 * @brief Widget with configuration elements for a TableView
 *
 */
class TableViewConfigWidget : public TabStyleViewConfigWidget
{
    Q_OBJECT
public:
    TableViewConfigWidget(TableViewWidget* view_widget, QWidget* parent = nullptr);
    virtual ~TableViewConfigWidget();

    virtual void configChanged() override;

public slots:
    void toggleShowOnlySeletedSlot();
    void toggleUsePresentation();
    void toggleIgnoreNonTargetReports();
    void exportSlot();
    void exportDoneSlot(bool cancelled);

signals:
    void exportSignal();

protected:
    void viewInfoJSON_impl(nlohmann::json& info) const override;

    TableView*    view_ = nullptr;

    QCheckBox*      only_selected_check_{nullptr};
    QCheckBox*      presentation_check_{nullptr};
    QCheckBox*      ignore_non_target_reports_check_{nullptr};

    QPushButton*    export_button_{nullptr};

    dbContent::VariableOrderedSetWidget* set_config_widget_ {nullptr};
};
