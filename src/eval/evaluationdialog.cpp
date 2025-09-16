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


#include "evaluationdialog.h"
#include "evaluationcalculator.h"
#include "evaluationsettings.h"
#include "evaluationmaintabwidget.h"
//#include "evaluationfiltertabwidget.h"
#include "evaluationstandardtabwidget.h"
#include "evaluationreporttabwidget.h"
#include "evaluationdatasourcewidget.h"
#include "evaluationsectorwidget.h"
#include "evaluationstandardcombobox.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QFormLayout>
#include <QComboBox>
#include <QStandardItemModel>

/**
 */
EvaluationDialog::EvaluationDialog(EvaluationCalculator& calculator)
:   calculator_(calculator)
{
    setWindowTitle("Evaluation");
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);

    setMinimumSize(QSize(1000, 800));

    QVBoxLayout* main_layout = new QVBoxLayout();

    QTabWidget* tab_widget = new QTabWidget();

    main_tab_widget_.reset(new EvaluationMainTabWidget(calculator_, *this));
    tab_widget->addTab(main_tab_widget_.get(), "Main");

    // filter_widget_.reset(new EvaluationFilterTabWidget(calculator_));
    // tab_widget->addTab(filter_widget_.get(), "Filter");

    std_tab_widget_.reset(new EvaluationStandardTabWidget(calculator_));
    tab_widget->addTab(std_tab_widget_.get(), "Standard");

    report_tab_widget_.reset(new EvaluationReportTabWidget(calculator_));
    tab_widget->addTab(report_tab_widget_.get(), "Results Config");

    main_layout->addWidget(tab_widget);
    main_layout->addSpacing(20);

    // not evaluate comment
    not_eval_comment_label_ = new QLabel();
    QPalette palette = not_eval_comment_label_->palette();
    palette.setColor(not_eval_comment_label_->foregroundRole(), Qt::red);
    not_eval_comment_label_->setPalette(palette);

    main_layout->addWidget(not_eval_comment_label_);

    QHBoxLayout* button_layout = new QHBoxLayout();

    cancel_button_ = new QPushButton("Cancel");
    connect(cancel_button_, &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_button_);

    button_layout->addStretch();

    run_button_ = new QPushButton("Run");
    connect(run_button_, &QPushButton::clicked, this, &QDialog::accept);
    button_layout->addWidget(run_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

    //updateFromSettings();

    updateDataSources();
    updateSectors();
    updateButtons();
    //updateFilterWidget();
}

/**
 */
EvaluationDialog::~EvaluationDialog() = default;

/**
 */
void EvaluationDialog::updateDataSources()
{
    if (main_tab_widget_)
        main_tab_widget_->updateDataSources();
}

/**
 */
void EvaluationDialog::updateSectors()
{
    if (main_tab_widget_)
        main_tab_widget_->updateSectors();

    updateButtons();
}

/**
 */
// void EvaluationDialog::updateFilterWidget()
// {
//     traced_assert(filter_widget_);
//     filter_widget_->update();
// }

/**
 */
void EvaluationDialog::updateButtons()
{
    traced_assert(run_button_);

    auto r = calculator_.canEvaluate();

    if (r.ok())
    {
        not_eval_comment_label_->setText("");
        not_eval_comment_label_->setHidden(true);
    }
    else
    {
        not_eval_comment_label_->setText(r.error().c_str());
        not_eval_comment_label_->setHidden(false);
    }

    run_button_->setEnabled(r.ok());
}
