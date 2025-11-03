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

class QPushButton;
class QStackedWidget;
class QComboBox;
class QLabel;

class ReconstructorTask;

class ReconstructorTaskDialog : public QDialog
{
    Q_OBJECT
    
public slots:
    void reconstructorMethodChangedSlot(const QString& value);

public:
    ReconstructorTaskDialog(ReconstructorTask& task);
    virtual ~ReconstructorTaskDialog();

    void showCurrentReconstructorWidget();
    void updateButtons();
    void checkValidity();

protected:
    void updateReconstructorInfo();
    void updateTimeframe();

    std::pair<bool, std::string> configValid() const;

    ReconstructorTask& task_;

    // order in stack has to be the same as in box
    QComboBox*      reconstructor_box_  {nullptr};
    QLabel*         reconstructor_info_  {nullptr};
    QStackedWidget* reconstructor_widget_stack_ {nullptr};

    QPushButton* cancel_button_{nullptr};
    QPushButton* run_button_{nullptr};
};

