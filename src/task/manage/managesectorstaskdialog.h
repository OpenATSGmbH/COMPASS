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

class ManageSectorsTask;
class ManageSectorsTaskWidget;

class ManageSectorsTaskDialog : public QDialog
{
    Q_OBJECT

signals:
    void doneSignal();

public slots:
    void doneClickedSlot();

public:
    ManageSectorsTaskDialog(ManageSectorsTask& task);

    void updateFileList();
    void updateParseMessage();

protected:
    ManageSectorsTask& task_;

    ManageSectorsTaskWidget* task_widget_ {nullptr}; // owned here

    QPushButton* done_button_{nullptr};
};
