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

#pragma once

#include <QDialog>

class ViewPointsReportGenerator;

class QPushButton;
class QLabel;
class QProgressBar;
class QLineEdit;
class QCheckBox;

class ViewPointsReportGeneratorDialog : public QDialog
{
    Q_OBJECT

public slots:
    void setPathSlot ();
    void pathEditedSlot ();
    void filenameEditedSlot();

    void authorEditedSlot (const QString& text);
    void abstractEditedSlot(const QString& text);

    void allUnsortedChangedSlot (bool checked);
    void groupByTypeChangedSlot (bool checked);
    void addOverviewTableChangedSlot (bool checked);

    void waitOnMapLoadingEditedSlot(bool checked);
    void addOverviewScreenshotChangedSlot (bool checked);

    void runPDFLatexChangedSlot (bool checked);
    void openPDFChangedSlot (bool checked);

    void runSlot();
    void cancelSlot();

public:
    ViewPointsReportGeneratorDialog(ViewPointsReportGenerator& generator,
                                    QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());

    void updateFileInfo ();

    void setRunning (bool value);

    void setElapsedTime (const std::string& time_str);
    void setProgress (unsigned int min, unsigned int max, unsigned int value);
    void setStatus (const std::string& status);
    void setRemainingTime (const std::string& time_str);

protected:
    ViewPointsReportGenerator& generator_;

    QWidget* config_container_ {nullptr};

    QLineEdit* directory_edit_ {nullptr};
    QLineEdit* filename_edit_ {nullptr};

    QLineEdit* author_edit_ {nullptr};
    QLineEdit* abstract_edit_ {nullptr};

    QCheckBox* all_unsorted_check_ {nullptr};
    QCheckBox* group_types_check_ {nullptr};
    QCheckBox* add_overview_table_check_ {nullptr};

    QCheckBox* wait_on_map_loading_check_ {nullptr};
    QCheckBox* add_overview_screenshot_check_ {nullptr};

    QCheckBox* pdflatex_check_ {nullptr};
    QCheckBox* open_pdf_check_ {nullptr};

    QPushButton* run_button_{nullptr};

    QLabel* elapsed_time_label_{nullptr};
    QProgressBar* progress_bar_{nullptr};
    QLabel* status_label_{nullptr};
    QLabel* remaining_time_label_{nullptr};

    QPushButton* quit_button_{nullptr};
};
