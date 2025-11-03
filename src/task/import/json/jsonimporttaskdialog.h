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

class JSONImportTask;
class JSONImportTaskWidget;

class QPushButton;

class JSONImportTaskDialog : public QDialog
{
    Q_OBJECT

signals:
    void testTmportSignal();
    void importSignal();
    void cancelSignal();

public slots:
    void testImportClickedSlot();
    void importClickedSlot();
    void cancelClickedSlot();

public:
    explicit JSONImportTaskDialog(JSONImportTask& task);

    void updateSource();
    void updateButtons();

protected:
    JSONImportTask& task_;

    JSONImportTaskWidget* task_widget_ {nullptr}; // owned here

    QPushButton* cancel_button_{nullptr};
    QPushButton* import_button_{nullptr};
    QPushButton* test_button_{nullptr};
};
