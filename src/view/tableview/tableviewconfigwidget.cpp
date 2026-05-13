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

#include "tableviewconfigwidget.h"
#include "tableviewwidget.h"
#include "tableviewdatawidget.h"
#include "tableview.h"
//#include "tableviewsetconfigwidget.h"

#include "ui_test_common.h"

#include "compass.h"
#include "logger.h"
#include "viewwidget.h"
#include "dbcontent/variable/variableorderedsetwidget.h"

#include "dbcontentlayer.h"
#include "viewlayerpanelwidget.h"
#include "viewlayertreemodel.h"

#include <QCheckBox>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTreeView>
#include <QVBoxLayout>
#include <QTabWidget>
#include <QListWidget>

using namespace Utils;
using namespace std;

TableViewConfigWidget::TableViewConfigWidget(TableViewWidget* view_widget, QWidget* parent)
:   TabStyleViewConfigWidget(view_widget, parent)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    // config
    {
        QFont font_bold;
        font_bold.setBold(true);

        QWidget* cfg_widget = new QWidget();
        QVBoxLayout* cfg_layout = new QVBoxLayout();

        set_config_widget_ = view_->getDataSource()->getSet()->createWidget();
        set_config_widget_->setObjectName("variables");
        //set_config_widget_->updateFromDataSource();

        cfg_layout->addWidget(set_config_widget_);

        QFrame* line = new QFrame();
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        cfg_layout->addWidget(line);

        only_selected_check_ = new QCheckBox("Show Only Selected");
        UI_TEST_OBJ_NAME(only_selected_check_, only_selected_check_->text())
        only_selected_check_->setChecked(view_->showOnlySelected());
        connect(only_selected_check_, &QCheckBox::clicked, this, &TableViewConfigWidget::toggleShowOnlySeletedSlot);
        cfg_layout->addWidget(only_selected_check_);

        presentation_check_ = new QCheckBox("Use Presentation");
        UI_TEST_OBJ_NAME(presentation_check_, presentation_check_->text())
        presentation_check_->setChecked(view_->usePresentation());
        connect(presentation_check_, &QCheckBox::clicked, this, &TableViewConfigWidget::toggleUsePresentation);
        cfg_layout->addWidget(presentation_check_);

        ignore_non_target_reports_check_ = new QCheckBox("Ignore Non-Target Reports");
        UI_TEST_OBJ_NAME(ignore_non_target_reports_check_, presentation_check_->text())
        ignore_non_target_reports_check_->setChecked(view_->ignoreNonTargetReports());
        connect(ignore_non_target_reports_check_, &QCheckBox::clicked,
                this, &TableViewConfigWidget::toggleIgnoreNonTargetReports);
        cfg_layout->addWidget(ignore_non_target_reports_check_);

        color_mode_label_ = new QLabel(this);
        color_mode_label_->setText("Color Mode: " + colorModeText(view_->compass().colorMode()));
        cfg_layout->addWidget(color_mode_label_);

        connect(&view_->compass(), &COMPASS::colorModeChangedSignal,
                this, &TableViewConfigWidget::colorModeChangedSlot);

        // Layer panel - fills remaining vertical space below the checkboxes.
        // No addStretch() anymore: the panel's tree view is the stretchy child.
        layer_panel_     = new ViewLayerPanelWidget({}, false, this);
        db_content_root_ = layer_panel_->model()->dbContentRootItem();

        auto* data_widget = view_widget->getViewDataWidget();
        data_widget->attachLayerPanel(db_content_root_, layer_panel_->model());

        connect(data_widget, &TableViewDataWidget::layerTreeRebuiltSignal,
                this, &TableViewConfigWidget::applyDefaultExpansionSlot);

        cfg_layout->addWidget(layer_panel_);

        cfg_widget->setLayout(cfg_layout);

        getTabWidget()->addTab(cfg_widget, "Config");
    }

    export_button_ = new QPushButton("Export");
    connect(export_button_, SIGNAL(clicked(bool)), this, SLOT(exportSlot()));
    getMainLayout()->addWidget(export_button_);
}

TableViewConfigWidget::~TableViewConfigWidget() = default;

void TableViewConfigWidget::toggleShowOnlySeletedSlot()
{
    traced_assert(only_selected_check_);
    bool checked = only_selected_check_->checkState() == Qt::Checked;
    loginf << "setting to " << checked;
    view_->showOnlySelected(checked);
}

void TableViewConfigWidget::toggleUsePresentation()
{
    traced_assert(presentation_check_);
    bool checked = presentation_check_->checkState() == Qt::Checked;
    logdbg << "setting use presentation to "
           << checked;
    view_->usePresentation(checked);
}

void TableViewConfigWidget::toggleIgnoreNonTargetReports()
{
    traced_assert(ignore_non_target_reports_check_);
    bool checked = ignore_non_target_reports_check_->checkState() == Qt::Checked;
    logdbg << "setting to "
           << checked;
    view_->ignoreNonTargetReports(checked);
}

void TableViewConfigWidget::exportSlot()
{
    logdbg;
    traced_assert(export_button_);

    export_button_->setDisabled(true);
    emit exportSignal();
}

void TableViewConfigWidget::exportDoneSlot(bool cancelled)
{
    traced_assert(export_button_);

    export_button_->setDisabled(false);

    if (!cancelled)
    {
        QMessageBox msgBox(this);
        msgBox.setText("Export complete.");
        msgBox.exec();
    }
}

void TableViewConfigWidget::colorModeChangedSlot(unsigned int mode)
{
    traced_assert(color_mode_label_);
    color_mode_label_->setText("Color Mode: " + colorModeText(mode));

    applyDefaultExpansionSlot();
}

void TableViewConfigWidget::applyDefaultExpansionSlot()
{
    if (!db_content_root_ || !layer_panel_)
        return;
    db_content_root_->applyDefaultExpansionForColorMode(
        layer_panel_->treeView(), view_->compass().colorMode());
}

QString TableViewConfigWidget::colorModeText(unsigned int mode)
{
    switch (mode)
    {
        case 0: return "DSType";
        case 1: return "DBContent";
        case 2: return "Data Source";
        case 3: return "Data Source + Line";
        default: return "Unknown";
    }
}

void TableViewConfigWidget::configChanged()
{
    traced_assert(view_);

    //update ui for var set
    //set_config_widget_->updateFromDataSource();
    set_config_widget_->updateVariableListSlot();

    //other ui elements
    only_selected_check_->setChecked(view_->showOnlySelected());
    presentation_check_->setChecked(view_->usePresentation());
    ignore_non_target_reports_check_->setChecked(view_->ignoreNonTargetReports());
}

void TableViewConfigWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    std::vector<std::string> variables;
    for (int i = 0; i < set_config_widget_->listWidget()->count(); ++i)
        variables.push_back(set_config_widget_->listWidget()->item(i)->text().toStdString());

    info[ "variables"          ] = variables;
    info[ "show_only_selected" ] = only_selected_check_->isChecked();
    info[ "use_presentation"   ] = presentation_check_->isChecked();
    info[ "ignore_non_target_reports"   ] = ignore_non_target_reports_check_->isChecked();
}

