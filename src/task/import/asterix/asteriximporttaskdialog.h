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

class ASTERIXImportTask;
class ASTERIXImportTaskWidget;

class QPushButton;

class ASTERIXImportTaskDialog : public QDialog
{
    Q_OBJECT

public slots:
    void importClickedSlot();
    void cancelClickedSlot();

public:
    explicit ASTERIXImportTaskDialog(ASTERIXImportTask& task, QWidget* parent = nullptr);

    void updateSourcesInfo();
    void updateButtons();
    void updateTitle();

protected:
    void configChanged();
    void decodingStateChangedSlot();

    ASTERIXImportTask& task_;

    ASTERIXImportTaskWidget* task_widget_ {nullptr}; // owned here

    QPushButton* cancel_button_{nullptr};
    QPushButton* import_button_{nullptr};
};
