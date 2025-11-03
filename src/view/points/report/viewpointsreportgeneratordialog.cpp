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

#include "viewpointsreportgeneratordialog.h"
#include "viewpointsreportgenerator.h"
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

ViewPointsReportGeneratorDialog::ViewPointsReportGeneratorDialog(ViewPointsReportGenerator& generator,
                                                                 QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f), generator_(generator)
{
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setWindowTitle("Export View Points as PDF");

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
        connect(directory_edit_, &QLineEdit::editingFinished, this, &ViewPointsReportGeneratorDialog::pathEditedSlot);
        config_grid->addWidget(directory_edit_, row, 1);

        ++row;
        config_grid->addWidget(new QLabel("Report Filename"), row, 0);

        filename_edit_ = new QLineEdit();
        filename_edit_->setText(generator_.reportFilename().c_str());
        connect(filename_edit_, &QLineEdit::editingFinished, this, &ViewPointsReportGeneratorDialog::filenameEditedSlot);
        config_grid->addWidget(filename_edit_, row, 1);

        ++row;

        QPushButton* set_path_but = new QPushButton("Change Location");
        connect(set_path_but, &QPushButton::clicked, this, &ViewPointsReportGeneratorDialog::setPathSlot);
        config_grid->addWidget(set_path_but, row, 1);

        // latex content

        ++row;
        config_grid->addWidget(new QLabel("Author"), row, 0);

        author_edit_ = new QLineEdit();
        author_edit_->setText(generator_.author().c_str());
        connect(author_edit_, &QLineEdit::textEdited, this, &ViewPointsReportGeneratorDialog::authorEditedSlot);
        config_grid->addWidget(author_edit_, row, 1);

        ++row;
        config_grid->addWidget(new QLabel("Abstract"), row, 0);

        abstract_edit_ = new QLineEdit();
        abstract_edit_->setText(generator_.abstract().c_str());
        connect(abstract_edit_, &QLineEdit::textEdited, this, &ViewPointsReportGeneratorDialog::abstractEditedSlot);
        config_grid->addWidget(abstract_edit_, row, 1);

        // export all
        ++row;
        config_grid->addWidget(new QLabel("Export All (Sorted by id)"), row, 0);


        all_unsorted_check_ = new QCheckBox();
        all_unsorted_check_->setChecked(generator_.exportAllUnsorted());
        connect(all_unsorted_check_, &QCheckBox::clicked, this,
                &ViewPointsReportGeneratorDialog::allUnsortedChangedSlot);
        config_grid->addWidget(all_unsorted_check_, row, 1);

        // group by type
        ++row;
        config_grid->addWidget(new QLabel("Group by Type"), row, 0);


        group_types_check_ = new QCheckBox();
        group_types_check_->setChecked(generator_.groupByType());
        connect(group_types_check_, &QCheckBox::clicked, this,
                &ViewPointsReportGeneratorDialog::groupByTypeChangedSlot);
        config_grid->addWidget(group_types_check_, row, 1);

        // add overview table
        ++row;
        config_grid->addWidget(new QLabel("Add Overview Table"), row, 0);

        add_overview_table_check_ = new QCheckBox();
        add_overview_table_check_->setChecked(generator_.addOverviewTable());
        connect(add_overview_table_check_, &QCheckBox::clicked, this,
                &ViewPointsReportGeneratorDialog::addOverviewTableChangedSlot);
        config_grid->addWidget(add_overview_table_check_, row, 1);

        // wait
        ++row;
        config_grid->addWidget(new QLabel("Wait On Map Loading"), row, 0);

        wait_on_map_loading_check_ = new QCheckBox();
        wait_on_map_loading_check_->setChecked(generator_.waitOnMapLoading());
        connect(wait_on_map_loading_check_, &QCheckBox::clicked, this,
                &ViewPointsReportGeneratorDialog::waitOnMapLoadingEditedSlot);
        config_grid->addWidget(wait_on_map_loading_check_, row, 1);

        // add overview screenshot
        ++row;
        config_grid->addWidget(new QLabel("Add Overview Screenshot"), row, 0);

        add_overview_screenshot_check_ = new QCheckBox();
        add_overview_screenshot_check_->setChecked(generator_.addOverviewScreenshot());
        connect(add_overview_screenshot_check_, &QCheckBox::clicked, this,
                &ViewPointsReportGeneratorDialog::addOverviewScreenshotChangedSlot);
        config_grid->addWidget(add_overview_screenshot_check_, row, 1);

        // run pdflatex
        ++row;
        config_grid->addWidget(new QLabel("Run PDFLatex"), row, 0);

        pdflatex_check_ = new QCheckBox();
        pdflatex_check_->setChecked(generator_.runPDFLatex());

        if (!generator_.pdfLatexFound())
            pdflatex_check_->setDisabled(true);

        connect(pdflatex_check_, &QCheckBox::clicked, this,
                &ViewPointsReportGeneratorDialog::runPDFLatexChangedSlot);
        config_grid->addWidget(pdflatex_check_, row, 1);

        // open
        ++row;
        config_grid->addWidget(new QLabel("Open Created PDF"), row, 0);

        open_pdf_check_ = new QCheckBox();
        open_pdf_check_->setChecked(generator_.openCreatedPDF());

        if (!generator_.pdfLatexFound())
            open_pdf_check_->setDisabled(true);

        connect(open_pdf_check_, &QCheckBox::clicked, this,
                &ViewPointsReportGeneratorDialog::openPDFChangedSlot);
        config_grid->addWidget(open_pdf_check_, row, 1);

        config_container_->setLayout(config_grid);

        //main_layout->addLayout(config_grid);
        main_layout->addWidget(config_container_);

    }

    run_button_ = new QPushButton("Run");
    connect(run_button_, &QPushButton::clicked, this, &ViewPointsReportGeneratorDialog::runSlot);
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
    connect(quit_button_, &QPushButton::clicked, this, &ViewPointsReportGeneratorDialog::cancelSlot);
    main_layout->addWidget(quit_button_);

    setLayout(main_layout);
}

void ViewPointsReportGeneratorDialog::updateFileInfo ()
{
    traced_assert(directory_edit_);
    directory_edit_->setText(generator_.reportPath().c_str());
    traced_assert(filename_edit_);
    filename_edit_->setText(generator_.reportFilename().c_str());
}

void ViewPointsReportGeneratorDialog::setRunning (bool value)
{
    traced_assert(config_container_);
    config_container_->setDisabled(value);

    traced_assert(run_button_);
    run_button_->setDisabled(value);
}

void ViewPointsReportGeneratorDialog::setPathSlot ()
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

void ViewPointsReportGeneratorDialog::pathEditedSlot ()
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

void ViewPointsReportGeneratorDialog::filenameEditedSlot()
{
    traced_assert(filename_edit_);
    generator_.reportFilename(filename_edit_->text().toStdString());
}

void ViewPointsReportGeneratorDialog::authorEditedSlot (const QString& text)
{
    generator_.author(text.toStdString());
}

void ViewPointsReportGeneratorDialog::abstractEditedSlot(const QString& text)
{
    generator_.abstract(text.toStdString());
}

void ViewPointsReportGeneratorDialog::waitOnMapLoadingEditedSlot(bool checked)
{
    generator_.waitOnMapLoading(checked);
}

void ViewPointsReportGeneratorDialog::addOverviewScreenshotChangedSlot (bool checked)
{
    generator_.addOverviewScreenshot(checked);
}

void ViewPointsReportGeneratorDialog::groupByTypeChangedSlot (bool checked)
{
    generator_.groupByType(checked);
}

void ViewPointsReportGeneratorDialog::addOverviewTableChangedSlot (bool checked)
{
    generator_.addOverviewTable(checked);
}

void ViewPointsReportGeneratorDialog::allUnsortedChangedSlot (bool checked)
{
    generator_.exportAllUnsorted(checked);
}

void ViewPointsReportGeneratorDialog::runPDFLatexChangedSlot (bool checked)
{
    generator_.runPDFLatex(checked);
}

void ViewPointsReportGeneratorDialog::openPDFChangedSlot (bool checked)
{
    generator_.openCreatedPDF(checked);
}

void ViewPointsReportGeneratorDialog::runSlot()
{
    loginf;
    generator_.run();
}

void ViewPointsReportGeneratorDialog::cancelSlot()
{
    loginf;
    generator_.cancel();
}

void ViewPointsReportGeneratorDialog::setElapsedTime (const std::string& time_str)
{
    traced_assert(elapsed_time_label_);
    elapsed_time_label_->setText(time_str.c_str());
}

void ViewPointsReportGeneratorDialog::setProgress (unsigned int min, unsigned int max, unsigned int value)
{
    traced_assert(progress_bar_);
    traced_assert(max >= min);

    progress_bar_->setRange(min, max);
    progress_bar_->setValue(value);
}

void ViewPointsReportGeneratorDialog::setStatus (const std::string& status)
{
    traced_assert(status_label_);
    status_label_->setText(status.c_str());
}

void ViewPointsReportGeneratorDialog::setRemainingTime (const std::string& time_str)
{
    traced_assert(remaining_time_label_);
    remaining_time_label_->setText(time_str.c_str());
}
