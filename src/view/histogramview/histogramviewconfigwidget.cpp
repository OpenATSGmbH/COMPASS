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
//#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "histogramview.h"
//#include "histogramviewdatasource.h"
#include "logger.h"
//#include "stringconv.h"
#include "groupbox.h"
#include "ui_test_common.h"
#include "metavariable.h"

#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
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
