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

#include "reportexportdialog.h"
#include "reportexport.h"

#include "taskresult.h"

#include "logger.h"
#include "compass.h"
#include "files.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QTextEdit>
#include <QLineEdit>
#include <QCheckBox>
#include <QSpinBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QApplication>

#include <boost/filesystem.hpp>

namespace ResultReport
{

/**
 */
ReportExportDialog::ReportExportDialog(TaskResult& task_result,
                                       ReportExport& report_export, 
                                       ReportExportMode export_mode,
                                       bool no_interaction_mode,
                                       const boost::optional<std::string>& export_dir,
                                       const std::string& section,
                                       QWidget* parent, 
                                       Qt::WindowFlags f)
:   QDialog             (parent, f          )
,   task_result_        (task_result        )
,   report_export_      (report_export      )
,   export_mode_        (export_mode        )
,   no_interaction_mode_(no_interaction_mode)
,   section_            (section            )
{
    format_str_ = QString::fromStdString(reportExportMode2String(export_mode));

    setWindowTitle("Export Report as " + format_str_);
    setMinimumWidth(800);

    createUI();
    configureUI(export_dir);

    connect(&report_export, &ReportExport::progressChanged, this, &ReportExportDialog::updateProgress);
}

/**
 */
void ReportExportDialog::showEvent(QShowEvent *event)
{
    if (no_interaction_mode_)
        QTimer::singleShot(10, this, &ReportExportDialog::exportReport);
}

/**
 */
QLabel* ReportExportDialog::configLabel(QWidget* w)
{
    auto label = dynamic_cast<QLabel*>(config_layout_->labelForField(w));
    traced_assert(label);
    return label;
}

/**
 */
QLabel* ReportExportDialog::configLabel(QLayout* l)
{
    auto label = dynamic_cast<QLabel*>(config_layout_->labelForField(l));
    traced_assert(label);
    return label;
}

/**
 */
void ReportExportDialog::setRowVisible(QWidget* w, bool ok)
{
    w->setVisible(ok);
    configLabel(w)->setVisible(ok);
}

/**
 */
void ReportExportDialog::setRowVisible(QLayout* l, bool ok)
{
    for (int i = 0; i < l->count(); ++i) 
    {
        QLayoutItem* item = l->itemAt(i);
        if (!item)
            continue;

        QWidget* widget = item->widget();
        if (widget) 
            widget->setVisible(ok);
    }

    configLabel(l)->setVisible(ok);
}

/**
 */
void ReportExportDialog::createUI()
{
    auto main_layout = new QVBoxLayout;
    setLayout(main_layout);

    config_widget_ = new QWidget;

    config_layout_ = new QFormLayout;
    config_widget_->setLayout(config_layout_);

    main_layout->addWidget(config_widget_);
    
    base_dir_button_ = new QPushButton;
    base_dir_button_->setIcon(Utils::Files::getIcon("folder.png"));

    connect(base_dir_button_, &QPushButton::pressed, this, &ReportExportDialog::editBaseDir);

    base_dir_edit_ = new QLineEdit;
    base_dir_edit_->setReadOnly(true);
    base_dir_edit_->setFrame(false);

    base_dir_layout_ = new QHBoxLayout;
    base_dir_layout_->setSpacing(0);
    base_dir_layout_->setContentsMargins(0, 0, 0, 0);

    base_dir_layout_->addWidget(base_dir_button_);
    base_dir_layout_->addWidget(base_dir_edit_);

    config_layout_->addRow("Report Directory", base_dir_layout_);

    // res_dir_name_edit_ = new QLineEdit;
    // config_layout_->addRow("Report Directory", res_dir_name_edit_);

    //connect(res_dir_name_edit_, &QLineEdit::textChanged, this, &ReportExportDialog::checkExport);

    report_name_edit_ = new QLineEdit;
    config_layout_->addRow("Report Name", report_name_edit_);

    connect(report_name_edit_, &QLineEdit::textChanged, this, &ReportExportDialog::checkExport);

    author_edit_ = new QLineEdit;
    config_layout_->addRow("Author", author_edit_);

    comment_edit_ = new QTextEdit;
    config_layout_->addRow("Comments", comment_edit_);

    latex_max_rows_edit_ = new QSpinBox;
    latex_max_rows_edit_->setMinimum(0);
    latex_max_rows_edit_->setMaximum(std::numeric_limits<int>::max());
    config_layout_->addRow("Maximum Table Rows", latex_max_rows_edit_);

    latex_max_colw_edit_ = new QSpinBox;
    latex_max_colw_edit_->setMinimum(0);
    latex_max_colw_edit_->setMaximum(std::numeric_limits<int>::max());
    config_layout_->addRow("Maximum Table Column Width", latex_max_colw_edit_);

    open_file_box_ = new QCheckBox;
    config_layout_->addRow("Open Created " + format_str_ + " File", open_file_box_);

    config_spacer_ = new QWidget;
    config_spacer_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    main_layout->addWidget(config_spacer_);

    elapsed_label_   = new QLabel;
    remaining_label_ = new QLabel;
    status_label_    = new QLabel;

    progress_bar_ = new QProgressBar;
    progress_bar_->setMinimum(0);
    progress_bar_->setMaximum(100);

    auto progress_layout = new QFormLayout;
    progress_layout->addRow("Progress" , progress_bar_   );
    progress_layout->addRow("Status"   , status_label_   );
    //progress_layout->addRow("Elapsed"  , elapsed_label_  );
    //progress_layout->addRow("Remaining", remaining_label_);

    main_layout->addLayout(progress_layout);

    export_button_  = new QPushButton("Export");
    cancel_button_  = new QPushButton("Cancel");

    connect(export_button_, &QPushButton::pressed, this, &ReportExportDialog::exportReport);
    connect(cancel_button_, &QPushButton::pressed, this, &ReportExportDialog::cancel);

    auto button_layout = new QHBoxLayout;
    button_layout->addWidget(cancel_button_);
    button_layout->addStretch(1);
    button_layout->addWidget(export_button_);

    main_layout->addLayout(button_layout);

    checkExport();
}

/**
 */
void ReportExportDialog::configureUI(const boost::optional<std::string>& export_dir)
{
    auto result_name   = task_result_.name();
    
    std::string report_dir;
    auto report_name = Utils::Files::normalizeFilename(result_name, false) + 
                       reportExportMode2Extension(export_mode_);

    //auto exp_folder = reportExportMode2Folder(export_mode_);
    //traced_assert(!exp_folder.empty());

    if (export_dir)
        report_dir = *export_dir;
    else
    {
        auto db_fn = COMPASS::instance().lastDbFilename();
        auto db_dir = Utils::Files::getDirectoryFromPath(db_fn);
        // auto db_dir     = export_dir.has_value() ? export_dir.value() :
        // Utils::Files::getDirectoryFromPath(db_fn);
        //auto base_name = boost::filesystem::path(db_fn).stem().string();

        report_dir = db_dir + "/" + Utils::Files::normalizeFilename(result_name, false);

        loginf << "db_fn " << db_fn << " db_dir " << db_dir
            << " result_name " << result_name << " report_dir " << report_dir;
    }
    
    loginf << "report_dir " << report_dir << " report_name " << report_name;

    base_dir_edit_->setText(QString::fromStdString(report_dir));
    //res_dir_name_edit_->setText(QString::fromStdString(report_dir));
    report_name_edit_->setText(QString::fromStdString(report_name));

    bool is_latex_mode = export_mode_ == ReportExportMode::Latex ||
                         export_mode_ == ReportExportMode::LatexPDF;
    bool is_pdf_mode   = export_mode_ == ReportExportMode::LatexPDF;

    config_widget_->setVisible(export_mode_ != ReportExportMode::JSONBlob);
    config_spacer_->setVisible(export_mode_ != ReportExportMode::JSONBlob);

    setRowVisible(latex_max_rows_edit_, is_latex_mode);
    setRowVisible(latex_max_colw_edit_, is_latex_mode);
    setRowVisible(open_file_box_      , is_pdf_mode  );

    loadSettings();
}

/**
 */
void ReportExportDialog::loadSettings()
{
    const auto& settings = report_export_.settings();

    author_edit_->setText(QString::fromStdString(settings.author));

    open_file_box_->setChecked(settings.open_created_file);

    latex_max_rows_edit_->setValue(settings.latex_table_max_rows);
    latex_max_colw_edit_->setValue(settings.latex_table_max_col_width);
}

/**
 */
void ReportExportDialog::writeSettings()
{
    auto& settings = report_export_.settings();

    settings.author   = author_edit_->text().toStdString();
    settings.abstract = comment_edit_->toPlainText().toStdString();

    settings.open_created_file = open_file_box_->isChecked();

    settings.latex_table_max_rows      = latex_max_rows_edit_->value();
    settings.latex_table_max_col_width = latex_max_colw_edit_->value();
}

/**
 */
void ReportExportDialog::checkExport()
{
    bool can_export = !base_dir_edit_->text().isEmpty() &&
                      !report_name_edit_->text().isEmpty();

    export_button_->setEnabled(can_export);
}

/**
 */
void ReportExportDialog::exportReport()
{
    writeSettings();

    progress_bar_->setValue(0);
    status_label_->setText("");

    export_button_->setEnabled(false);
    cancel_button_->setEnabled(false);
    config_widget_->setEnabled(false);

    auto dir = base_dir_edit_->text().toStdString() + "/";
    auto fn  = report_name_edit_->text().toStdString();

    QApplication::processEvents();

    export_result_ = report_export_.exportReport(task_result_, export_mode_, fn, dir, section_, !no_interaction_mode_);

    QApplication::processEvents();

    if (!export_result_.ok())
    {
        logerr << export_result_.error();

        progress_bar_->setValue(0);
        status_label_->setText("Exporting report failed");
    }

    export_button_->setEnabled(true);
    cancel_button_->setEnabled(true);
    config_widget_->setEnabled(true);

    if (export_result_.ok() || no_interaction_mode_)
        accept();
}

/**
 */
void ReportExportDialog::cancel()
{
    writeSettings();
    reject();
}

/**
 */
void ReportExportDialog::updateProgress()
{
    status_label_->setText(QString::fromStdString(report_export_.status()));
    progress_bar_->setValue(progress_bar_->maximum() * report_export_.progress());
}

/**
 */
void ReportExportDialog::editBaseDir()
{
    auto dir = QFileDialog::getExistingDirectory(this, "Select Base Directory", base_dir_edit_->text());
    if(dir.isEmpty())
        return;

    base_dir_edit_->setText(dir);

    checkExport();
}

}
