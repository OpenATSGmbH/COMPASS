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

#include "histogramviewconfigwidget.h"
#include "histogramviewwidget.h"
#include "histogramviewdatawidget.h"
#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "histogramview.h"
//#include "histogramviewdatasource.h"
#include "logger.h"
//#include "stringconv.h"
#include "groupbox.h"
#include "ui_test_common.h"
#include "metavariable.h"

#include "dbcontentlayer.h"
#include "viewlayerpanelwidget.h"
#include "annotationsrootitem.h"
#include "layertreemodel.h"

#include <QCheckBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTreeView>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QTabWidget>
#include <QRadioButton>

using namespace Utils;

/**
 */
HistogramViewConfigWidget::HistogramViewConfigWidget(HistogramViewWidget* view_widget, 
                                                     QWidget* parent)
:   VariableViewConfigWidget(view_widget, view_widget->getView(), parent)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    //log scale
    {
        auto config_layout = configLayout();

        log_check_ = new QCheckBox("Logarithmic Y Scale");
        UI_TEST_OBJ_NAME(log_check_, log_check_->text())

        updateLogScale();

        connect(log_check_, &QCheckBox::clicked, this,
                &HistogramViewConfigWidget::toggleLogScale);

        config_layout->addWidget(log_check_);
    }

    {
        auto config_layout = configLayout();

        color_mode_label_ = new QLabel(this);
        color_mode_label_->setText("Color Mode: " + colorModeText(view_->compass().colorMode()));
        config_layout->addWidget(color_mode_label_);

        connect(&view_->compass(), &COMPASS::colorModeChangedSignal,
                this, &HistogramViewConfigWidget::colorModeChangedSlot);

        layer_panel_     = new ViewLayerPanelWidget({}, view_->canShowAnnotations(), this);
        db_content_root_ = layer_panel_->dbContentRootItem();

        auto* data_widget = view_widget->getViewDataWidget();
        data_widget->attachLayerPanel(db_content_root_, layer_panel_->model());

        connect(data_widget, &HistogramViewDataWidget::layerTreeRebuiltSignal,
                this, &HistogramViewConfigWidget::applyDefaultExpansionSlot);

        config_layout->addWidget(layer_panel_);
    }
}

/**
 */
HistogramViewConfigWidget::~HistogramViewConfigWidget() = default;

/**
 */
void HistogramViewConfigWidget::toggleLogScale()
{
    traced_assert(log_check_);
    bool checked = log_check_->checkState() == Qt::Checked;
    logdbg << "setting overwrite to " << checked;
    view_->useLogScale(checked, true);
}

/**
 */
void HistogramViewConfigWidget::onDisplayChange_impl()
{
    updateLogScale();
}

/**
 */
void HistogramViewConfigWidget::configChanged_impl()
{
    updateLogScale();
}

/**
 */
void HistogramViewConfigWidget::updateLogScale()
{
    log_check_->setChecked(view_->useLogScale());
}

/**
 */
void HistogramViewConfigWidget::colorModeChangedSlot(unsigned int mode)
{
    traced_assert(color_mode_label_);
    color_mode_label_->setText("Color Mode: " + colorModeText(mode));

    applyDefaultExpansionSlot();
}

/**
 */
void HistogramViewConfigWidget::applyDefaultExpansionSlot()
{
    if (!db_content_root_ || !layer_panel_)
        return;
    db_content_root_->applyDefaultExpansionForColorMode(
        layer_panel_->treeView(), view_->compass().colorMode());
}

/**
 */
QString HistogramViewConfigWidget::colorModeText(unsigned int mode)
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

/**
 */
void HistogramViewConfigWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableViewConfigWidget::viewInfoJSON_impl(info);

    info[ "log_enabled" ] = log_check_->isChecked();
}

/**
 */
//void HistogramViewConfigWidget::exportSlot()
//{
//    logdbg;
//    //assert(overwrite_check_);
//    traced_assert(export_button_);

//    export_button_->setDisabled(true);
//    //emit exportSignal(overwrite_check_->checkState() == Qt::Checked);
//}

/**
 */
//void HistogramViewConfigWidget::exportDoneSlot(bool cancelled)
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
