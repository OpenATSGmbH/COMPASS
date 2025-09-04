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
#include "tableview.h"
//#include "tableviewsetconfigwidget.h"

#include "ui_test_common.h"

#include "logger.h"
#include "viewwidget.h"
#include "dbcontent/variable/variableorderedsetwidget.h"

#include <QCheckBox>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
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

        cfg_layout->addStretch();
        
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
        QMessageBox msgBox;
        msgBox.setText("Export complete.");
        msgBox.exec();
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
