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

#include <taskwidget.h>

class ViewPointsImportTask;

class QPushButton;
class QListWidget;
class QTextEdit;

class ViewPointsImportTaskWidget : public QWidget
{
    Q_OBJECT

public slots:

signals:

public:
    ViewPointsImportTaskWidget(ViewPointsImportTask& task, QWidget* parent = nullptr);
    virtual ~ViewPointsImportTaskWidget();

    void updateText ();

protected:
    ViewPointsImportTask& task_;

    QTextEdit* context_edit_ {nullptr};

};
