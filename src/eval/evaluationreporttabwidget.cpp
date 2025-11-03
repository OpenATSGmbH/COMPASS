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

#include "evaluationreporttabwidget.h"
#include "evaluationcalculator.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGridLayout>
#include <QFormLayout>
#include <QCheckBox>
#include <QLineEdit>
#include <QDoubleValidator>
#include <QSpinBox>

/**
 */
EvaluationReportTabWidget::EvaluationReportTabWidget(EvaluationCalculator& calculator)
:   calculator_(calculator)
{
    QHBoxLayout* main_layout = new QHBoxLayout();
    QFormLayout* layout = new QFormLayout();

    //int row = 0;

    // skip no data details
    //++row;

    const auto& eval_settings = calculator.settings();

    skip_no_data_details_check_ = new QCheckBox ();
    skip_no_data_details_check_->setChecked(eval_settings.report_skip_no_data_details_);
    connect(skip_no_data_details_check_, &QCheckBox::clicked,
            this, &EvaluationReportTabWidget::toggleSkipNoDataDetailsSlot);
    layout->addRow("Skip No Data Details", skip_no_data_details_check_);

    // split results by MOPS
    //++row;

    split_results_by_mops_check_ = new QCheckBox ();
    split_results_by_mops_check_->setChecked(eval_settings.report_split_results_by_mops_);
    connect(split_results_by_mops_check_, &QCheckBox::clicked,
            this, &EvaluationReportTabWidget::toggleSplitResultsByMOPSSlot);
    layout->addRow("Split Results by MOPS Version", split_results_by_mops_check_);

    // split results by mode a/c only and mode s

    split_results_by_mac_ms_check_ = new QCheckBox ();
    split_results_by_mac_ms_check_->setChecked(eval_settings.report_split_results_by_aconly_ms_);
    connect(split_results_by_mac_ms_check_, &QCheckBox::clicked,
            this, &EvaluationReportTabWidget::toggleSplitResultsByMACMSSlot);
    layout->addRow("Split Results by Mode A/C Only and Mode S", split_results_by_mac_ms_check_);

    // show ok

    show_ok_joined_target_reports_check_ = new QCheckBox ();
    show_ok_joined_target_reports_check_->setChecked(eval_settings.show_ok_joined_target_reports_);
    connect(show_ok_joined_target_reports_check_, &QCheckBox::clicked,
            this, &EvaluationReportTabWidget::toggleShowOKJoinedSlot);
    layout->addRow("Show OK data in Sector Results", show_ok_joined_target_reports_check_);

    result_detail_zoom_edit_ = new QLineEdit(QString::number(eval_settings.result_detail_zoom_));
    result_detail_zoom_edit_->setValidator(new QDoubleValidator(0.000001, 1.0, 7, this));
    connect(result_detail_zoom_edit_, &QLineEdit::textEdited,
            this, &EvaluationReportTabWidget::resultDetailZoomEditSlot);
    layout->addRow("Result Detail WGS84 Zoom Factor [deg]", result_detail_zoom_edit_);

    EvaluationSettings* settings_ptr = &calculator.settings();

    // grid generation
    grid_num_cells_x_box_  = new QSpinBox;
    grid_num_cells_x_box_->setMinimum(1);
    grid_num_cells_x_box_->setMaximum(2000);
    grid_num_cells_x_box_->setValue(eval_settings.grid_num_cells_x);
    connect(grid_num_cells_x_box_, QOverload<int>::of(&QSpinBox::valueChanged), [ settings_ptr ] (int v) { settings_ptr->grid_num_cells_x = v; });
    layout->addRow("#Grid Cells X", grid_num_cells_x_box_);

    grid_num_cells_y_box_  = new QSpinBox;
    grid_num_cells_y_box_->setMinimum(1);
    grid_num_cells_y_box_->setMaximum(2000);
    grid_num_cells_y_box_->setValue(eval_settings.grid_num_cells_y);
    connect(grid_num_cells_y_box_, QOverload<int>::of(&QSpinBox::valueChanged), [ settings_ptr ] (int v) { settings_ptr->grid_num_cells_y = v; });
    layout->addRow("#Grid Cells Y", grid_num_cells_y_box_);

    setContentsMargins(0, 0, 0, 0);

    main_layout->addLayout(layout);
    main_layout->addStretch();

    setLayout(main_layout);
}

/**
 */
EvaluationReportTabWidget::~EvaluationReportTabWidget() = default;

/**
 */
void EvaluationReportTabWidget::updateFromSettings()
{
    const auto& eval_settings = calculator_.settings();

    skip_no_data_details_check_->setChecked(eval_settings.report_skip_no_data_details_);
    split_results_by_mops_check_->setChecked(eval_settings.report_split_results_by_mops_);
    split_results_by_mac_ms_check_->setChecked(eval_settings.report_split_results_by_aconly_ms_);
    show_ok_joined_target_reports_check_->setChecked(eval_settings.show_ok_joined_target_reports_);
    result_detail_zoom_edit_->setText(QString::number(eval_settings.result_detail_zoom_));

    grid_num_cells_x_box_->setValue(eval_settings.grid_num_cells_x);
    grid_num_cells_y_box_->setValue(eval_settings.grid_num_cells_y);
}

/**
 */
void EvaluationReportTabWidget::toggleSplitResultsByMOPSSlot()
{
    traced_assert(split_results_by_mops_check_);
    calculator_.settings().report_split_results_by_mops_ = split_results_by_mops_check_->checkState() == Qt::Checked;
}

/**
 */
void EvaluationReportTabWidget::toggleSplitResultsByMACMSSlot()
{
    traced_assert(split_results_by_mac_ms_check_);
    calculator_.settings().report_split_results_by_aconly_ms_ = split_results_by_mac_ms_check_->checkState() == Qt::Checked;
}

/**
 */
void EvaluationReportTabWidget::toggleShowOKJoinedSlot()
{
    traced_assert(show_ok_joined_target_reports_check_);
    calculator_.settings().show_ok_joined_target_reports_ = show_ok_joined_target_reports_check_->checkState() == Qt::Checked;
}

/**
 */
void EvaluationReportTabWidget::toggleSkipNoDataDetailsSlot()
{
    traced_assert(skip_no_data_details_check_);
    calculator_.settings().report_skip_no_data_details_ = skip_no_data_details_check_->checkState() == Qt::Checked;
}

/**
 */
void EvaluationReportTabWidget::resultDetailZoomEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        calculator_.settings().result_detail_zoom_ = val;
    else
        loginf << "axvalid value";
}
