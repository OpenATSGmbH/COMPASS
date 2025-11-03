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

#include "eval/results/report/pdfgeneratordialog.h"
#include "eval/results/report/pdfgenerator.h"
#include "evaluationmanager.h"
//#include "textfielddoublevalidator.h"
#include "logger.h"
#include "compass.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QTextEdit>
#include <QLineEdit>
#include <QCheckBox>
#include <QFileDialog>

using namespace std;

namespace EvaluationResultsReport
{

PDFGeneratorDialog::PDFGeneratorDialog(PDFGenerator& generator, EvaluationManager& eval_man,
                                       EvaluationManagerSettings& eval_settings,
                                       QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f), generator_(generator), eval_man_(eval_man), eval_settings_(eval_settings)
{
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setWindowTitle("Export Evaluation Results as PDF");

    setModal(true);

    setMinimumSize(QSize(800, 600));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    QVBoxLayout* main_layout = new QVBoxLayout();

    int row = 0;
    {
        config_container_ = new QWidget();

        QGridLayout* config_grid = new QGridLayout();

        // report path

        config_grid->addWidget(new QLabel("Report Path"), row, 0);

        directory_edit_ = new QLineEdit ();
        directory_edit_->setText(generator_.reportPath().c_str());
        connect(directory_edit_, &QLineEdit::editingFinished, this, &PDFGeneratorDialog::pathEditedSlot);
        config_grid->addWidget(directory_edit_, row, 1);

        ++row;
        config_grid->addWidget(new QLabel("Report Filename"), row, 0);

        filename_edit_ = new QLineEdit();
        filename_edit_->setText(generator_.reportFilename().c_str());
        connect(filename_edit_, &QLineEdit::editingFinished, this, &PDFGeneratorDialog::filenameEditedSlot);
        config_grid->addWidget(filename_edit_, row, 1);

        ++row;

        QPushButton* set_path_but = new QPushButton("Change Location");
        connect(set_path_but, &QPushButton::clicked, this, &PDFGeneratorDialog::setPathSlot);
        config_grid->addWidget(set_path_but, row, 1);

        // latex content

        ++row;
        config_grid->addWidget(new QLabel("Author"), row, 0);

        author_edit_ = new QLineEdit();
        author_edit_->setText(eval_settings_.report_author_.c_str());
        connect(author_edit_, &QLineEdit::textEdited, this, &PDFGeneratorDialog::authorEditedSlot);
        config_grid->addWidget(author_edit_, row, 1);

        ++row;
        config_grid->addWidget(new QLabel("Abstract"), row, 0);

        abstract_edit_ = new QLineEdit();
        abstract_edit_->setText(eval_settings_.report_abstract_.c_str());
        connect(abstract_edit_, &QLineEdit::textEdited, this, &PDFGeneratorDialog::abstractEditedSlot);
        config_grid->addWidget(abstract_edit_, row, 1);

        // target details
        ++row;
        config_grid->addWidget(new QLabel("Include Per-Target Details"), row, 0);

        include_target_details_check_ = new QCheckBox();
        include_target_details_check_->setChecked(eval_settings_.report_include_target_details_);
        connect(include_target_details_check_, &QCheckBox::clicked, this,
                &PDFGeneratorDialog::includeTargetDetailsEditedSlot);
        config_grid->addWidget(include_target_details_check_, row, 1);

        // skip target details w/o issues
        ++row;
        config_grid->addWidget(new QLabel("Skip Per-Target Details Without Issues"), row, 0);

        skip_target_details_wo_issues_check_ = new QCheckBox();
        skip_target_details_wo_issues_check_->setChecked(eval_settings_.report_skip_targets_wo_issues_);
        connect(skip_target_details_wo_issues_check_, &QCheckBox::clicked, this,
                &PDFGeneratorDialog::skipTargetDetailsWOIssuesEditedSlot);
        config_grid->addWidget(skip_target_details_wo_issues_check_, row, 1);

        // target details
        ++row;
        config_grid->addWidget(new QLabel("Include Per-Target Target Report Details"), row, 0);

        include_target_tr_details_check_ = new QCheckBox();
        include_target_tr_details_check_->setChecked(eval_settings_.report_include_target_tr_details_);
        connect(include_target_tr_details_check_, &QCheckBox::clicked, this,
                &PDFGeneratorDialog::includeTargetTRDetailsEditedSlot);
        config_grid->addWidget(include_target_tr_details_check_, row, 1);

        // table rows
        ++row;
        config_grid->addWidget(new QLabel("Maximum Table Rows"), row, 0);

        num_max_table_rows_edit_ = new QLineEdit();
        num_max_table_rows_edit_->setText(QString::number(eval_settings_.report_num_max_table_rows_));
        connect(num_max_table_rows_edit_, &QLineEdit::textEdited, this, &PDFGeneratorDialog::numMaxTableRowsEditedSlot);
        config_grid->addWidget(num_max_table_rows_edit_, row, 1);

        ++row;
        config_grid->addWidget(new QLabel("Maximum Table Column Width"), row, 0);

        num_max_table_col_width_edit_ = new QLineEdit();
        num_max_table_col_width_edit_->setText(QString::number(eval_settings_.report_num_max_table_col_width_));
        connect(num_max_table_col_width_edit_, &QLineEdit::textEdited,
                this, &PDFGeneratorDialog::numMaxTableColWidthEditedSlot);
        config_grid->addWidget(num_max_table_col_width_edit_, row, 1);

        // wait
        ++row;
        config_grid->addWidget(new QLabel("Wait On Map Loading"), row, 0);

        wait_on_map_loading_check_ = new QCheckBox();
        wait_on_map_loading_check_->setChecked(eval_settings_.report_wait_on_map_loading_);
        connect(wait_on_map_loading_check_, &QCheckBox::clicked, this,
                &PDFGeneratorDialog::waitOnMapLoadingEditedSlot);
        config_grid->addWidget(wait_on_map_loading_check_, row, 1);

        // run pdflatex
        ++row;
        config_grid->addWidget(new QLabel("Run PDFLatex"), row, 0);

        pdflatex_check_ = new QCheckBox();
        pdflatex_check_->setChecked(eval_settings_.report_run_pdflatex_);

        if (!generator_.pdfLatexFound())
            pdflatex_check_->setDisabled(true);

        connect(pdflatex_check_, &QCheckBox::clicked, this,
                &PDFGeneratorDialog::runPDFLatexChangedSlot);
        config_grid->addWidget(pdflatex_check_, row, 1);

        // open
        ++row;
        config_grid->addWidget(new QLabel("Open Created PDF"), row, 0);

        open_pdf_check_ = new QCheckBox();
        open_pdf_check_->setChecked(eval_settings_.report_open_created_pdf_);

        if (!generator_.pdfLatexFound())
            open_pdf_check_->setDisabled(true);

        connect(open_pdf_check_, &QCheckBox::clicked, this,
                &PDFGeneratorDialog::openPDFChangedSlot);
        config_grid->addWidget(open_pdf_check_, row, 1);

        config_container_->setLayout(config_grid);

        //main_layout->addLayout(config_grid);
        main_layout->addWidget(config_container_);

    }

    run_button_ = new QPushButton("Run");
    connect(run_button_, &QPushButton::clicked, this, &PDFGeneratorDialog::runSlot);
    main_layout->addWidget(run_button_);

    main_layout->addStretch();

    row = 0;
    {
        QGridLayout* status_grid = new QGridLayout();

        status_grid->addWidget(new QLabel("Elapsed Time"), row, 0);
        elapsed_time_label_ = new QLabel();
        elapsed_time_label_->setAlignment(Qt::AlignRight);
        status_grid->addWidget(elapsed_time_label_, row, 1);

        ++row;
        status_grid->addWidget(new QLabel("Progress"), row, 0);
        progress_bar_ = new QProgressBar();
        status_grid->addWidget(progress_bar_, row, 1);

        ++row;
        status_grid->addWidget(new QLabel("Status"), row, 0);
        status_label_ = new QLabel();
        status_label_->setAlignment(Qt::AlignRight);
        status_grid->addWidget(status_label_, row, 1);

        ++row;
        status_grid->addWidget(new QLabel("Remaining Time"), row, 0);
        remaining_time_label_ = new QLabel();
        remaining_time_label_->setAlignment(Qt::AlignRight);
        status_grid->addWidget(remaining_time_label_, row, 1);

        main_layout->addLayout(status_grid);
    }

    //main_layout->addStretch();

    quit_button_ = new QPushButton("Cancel");
    connect(quit_button_, &QPushButton::clicked, this, &PDFGeneratorDialog::cancelSlot);
    main_layout->addWidget(quit_button_);

    setLayout(main_layout);
}

void PDFGeneratorDialog::updateFileInfo ()
{
    traced_assert(directory_edit_);
    directory_edit_->setText(generator_.reportPath().c_str());
    traced_assert(filename_edit_);
    filename_edit_->setText(generator_.reportFilename().c_str());
}

void PDFGeneratorDialog::setRunning (bool value)
{
    traced_assert(config_container_);
    config_container_->setDisabled(value);

    traced_assert(run_button_);
    run_button_->setDisabled(value);
}

void PDFGeneratorDialog::setPathSlot ()
{
    QFileDialog dialog(nullptr);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDirectory(COMPASS::instance().lastUsedPath().c_str());
    dialog.setNameFilter("TEX Files (*.tex)");
    dialog.setDefaultSuffix("tex");
    dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);

    if (dialog.exec())
    {
        QStringList file_names = dialog.selectedFiles();
        traced_assert(file_names.size() == 1);

        generator_.reportPathAndFilename(file_names.at(0).toStdString());
    }
}

void PDFGeneratorDialog::pathEditedSlot ()
{
    traced_assert(directory_edit_);

    string tmp = directory_edit_->text().toStdString();
    if (*(tmp.rbegin()) != '/')
    {
        tmp += "/";
        directory_edit_->setText(tmp.c_str());
    }

    generator_.reportPath(tmp);
}

void PDFGeneratorDialog::filenameEditedSlot()
{
    traced_assert(filename_edit_);
    generator_.reportFilename(filename_edit_->text().toStdString());
}

void PDFGeneratorDialog::authorEditedSlot (const QString& text)
{
    eval_settings_.report_author_ = text.toStdString();
}

void PDFGeneratorDialog::abstractEditedSlot(const QString& text)
{
    eval_settings_.report_abstract_ = text.toStdString();
}

void PDFGeneratorDialog::waitOnMapLoadingEditedSlot(bool checked)
{
    eval_settings_.report_wait_on_map_loading_ = checked;
}

void PDFGeneratorDialog::includeTargetDetailsEditedSlot(bool checked)
{
    eval_settings_.report_include_target_details_ = checked;
}

void PDFGeneratorDialog::skipTargetDetailsWOIssuesEditedSlot(bool checked)
{
    eval_settings_.report_skip_targets_wo_issues_ = checked;
}

void PDFGeneratorDialog::includeTargetTRDetailsEditedSlot(bool checked)
{
    eval_settings_.report_include_target_tr_details_ = checked;
}

void PDFGeneratorDialog::numMaxTableRowsEditedSlot(const QString& text)
{
    string value_str = text.toStdString();
    loginf << "value '" << value_str << "'";

    bool ok;

    unsigned int value = text.toUInt(&ok);

    if (ok)
        eval_settings_.report_num_max_table_rows_ = value;
    else
        logwrn << "unable to parse '" << value_str << "'";

}

void PDFGeneratorDialog::numMaxTableColWidthEditedSlot(const QString& text)
{
    string value_str = text.toStdString();
    loginf << "value '" << value_str << "'";

    bool ok;

    unsigned int value = text.toUInt(&ok);

    if (ok)
        eval_settings_.report_num_max_table_col_width_ = value;
    else
        logwrn << "unable to parse '" << value_str << "'";

}


void PDFGeneratorDialog::runPDFLatexChangedSlot (bool checked)
{
    eval_settings_.report_run_pdflatex_ = checked;
}

void PDFGeneratorDialog::openPDFChangedSlot (bool checked)
{
    eval_settings_.report_open_created_pdf_ = checked;
}

void PDFGeneratorDialog::runSlot()
{
    loginf;

    generator_.run();
}

void PDFGeneratorDialog::cancelSlot()
{
    loginf;
    generator_.cancel();
}

void PDFGeneratorDialog::setElapsedTime (const std::string& time_str)
{
    traced_assert(elapsed_time_label_);
    elapsed_time_label_->setText(time_str.c_str());
}

void PDFGeneratorDialog::setProgress (unsigned int min, unsigned int max, unsigned int value)
{
    traced_assert(progress_bar_);
    traced_assert(max >= min);

    progress_bar_->setRange(min, max);
    progress_bar_->setValue(value);
}

void PDFGeneratorDialog::setStatus (const std::string& status)
{
    traced_assert(status_label_);
    status_label_->setText(status.c_str());
}

void PDFGeneratorDialog::setRemainingTime (const std::string& time_str)
{
    traced_assert(remaining_time_label_);
    remaining_time_label_->setText(time_str.c_str());
}

}
