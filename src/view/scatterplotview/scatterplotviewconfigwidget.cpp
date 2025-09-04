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

#include "scatterplotviewconfigwidget.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "scatterplotviewwidget.h"
#include "scatterplotview.h"
#include "logger.h"
#include "variable.h"
#include "metavariable.h"
#include "ui_test_common.h"
#include "scatterseriestreeitem.h"
#include "scatterplotviewdatawidget.h"

#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTabWidget>
#include <QTreeView>
#include <QHeaderView>

using namespace Utils;
using namespace dbContent;

/**
*/
ScatterPlotViewConfigWidget::ScatterPlotViewConfigWidget(ScatterPlotViewWidget* view_widget, 
                                                         QWidget* parent)
:   VariableViewConfigWidget(view_widget, view_widget->getView(), parent)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    auto layout = configLayout();

    {
        loginf << "creating lay view";

        layer_view_ = new QTreeView(this);
        ScatterSeriesTreeItemDelegate* delegate = new ScatterSeriesTreeItemDelegate(this);
        layer_view_->setItemDelegate(delegate);

        layer_view_->setModel(&view_widget->getViewDataWidget()->dataModel());

        layer_view_->header()->resizeSection(0 /*column index*/, 300 /*width*/);

        connect(&view_widget->getViewDataWidget()->dataModel(), &ScatterSeriesModel::visibilityChangedSignal,
                this, &ScatterPlotViewConfigWidget::updateToVisibilitySlot);

        //getTabWidget()->addTab(layer_view_, "Layers");
        layout->addWidget(layer_view_);

        QPushButton* push_button = new QPushButton("Deselect All");
        push_button->setObjectName("deselect_all");
        layout->addWidget(push_button);
        QObject::connect(push_button, &QPushButton::pressed,
                         this, &ScatterPlotViewConfigWidget::deselectAllSlot);
    }


    use_connection_lines_ = new QCheckBox("Use Connection Lines");
    use_connection_lines_->setChecked(view_->useConnectionLines());
    UI_TEST_OBJ_NAME(use_connection_lines_, use_connection_lines_->text())

    connect(use_connection_lines_, &QCheckBox::clicked,
            this, &ScatterPlotViewConfigWidget::useConnectionLinesSlot);
    
    layout->addWidget(use_connection_lines_);

    //showSwitch(0, true);
}

/**
*/
ScatterPlotViewConfigWidget::~ScatterPlotViewConfigWidget() = default;

/**
*/
void ScatterPlotViewConfigWidget::useConnectionLinesSlot()
{
    loginf;

    traced_assert(use_connection_lines_);
    view_->useConnectionLines(use_connection_lines_->checkState() == Qt::Checked);
}

/**
*/
void ScatterPlotViewConfigWidget::updateToVisibilitySlot()
{
    traced_assert(layer_view_);
    layer_view_->expandToDepth(3);
    layer_view_->header()->resizeSection(0 /*column index*/, 300 /*width*/);
}

/**
*/
void ScatterPlotViewConfigWidget::deselectAllSlot()
{
    view_->getDataWidget()->dataModel().deselectAll();
}

/**
*/
void ScatterPlotViewConfigWidget::onDisplayChange_impl()
{
    traced_assert(use_connection_lines_);
    use_connection_lines_->setChecked(view_->useConnectionLines());
}

/**
*/
void ScatterPlotViewConfigWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableViewConfigWidget::viewInfoJSON_impl(info);

    info[ "use_connection_lines" ] = use_connection_lines_->isChecked();
}

/**
*/
void ScatterPlotViewConfigWidget::configChanged_impl()
{
    use_connection_lines_->blockSignals(true);
    use_connection_lines_->setChecked(view_->useConnectionLines());
    use_connection_lines_->blockSignals(false);
}

//void ScatterPlotViewConfigWidget::exportSlot()
//{
//    logdbg << "start";
//    //assert(overwrite_check_);
//    traced_assert(export_button_);

//    export_button_->setDisabled(true);
//    //emit exportSignal(overwrite_check_->checkState() == Qt::Checked);
//}

//void ScatterPlotViewConfigWidget::exportDoneSlot(bool cancelled)
//{
//    traced_assert(export_button_);

//    export_button_->setDisabled(false);

//    if (!cancelled)
//    {
//        QMessageBox msgBox;
//        msgBox.setText("Export complete.");
//        msgBox.exec();
//    }
//}
