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

#include "viewpointsimporttaskwidget.h"
#include "viewpointsimporttask.h"
//#include "compass.h"
//#include "global.h"
#include "logger.h"
//#include "stringconv.h"

#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QStackedWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QListWidget>
#include <QMessageBox>

using namespace std;

ViewPointsImportTaskWidget::ViewPointsImportTaskWidget(ViewPointsImportTask& task, QWidget* parent)
    : QWidget(parent), task_(task)
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout_ = new QVBoxLayout();

    context_edit_ = new QTextEdit ();
    context_edit_->setReadOnly(true);
    main_layout_->addWidget(context_edit_);

    if (task_.importFilename().size())
        updateText();

    setLayout(main_layout_);
}

ViewPointsImportTaskWidget::~ViewPointsImportTaskWidget()
{
}

void ViewPointsImportTaskWidget::updateText ()
{
    loginf;

    traced_assert(context_edit_);
    context_edit_->setText("");

    if (task_.currentError().size())
    {
        context_edit_->setText(QString("Error: ")+task_.currentError().c_str());
    }
    else
    {
        const nlohmann::json& data = task_.currentData();

        string text;

        if (data.contains("view_point_context"))
            text = data.at("view_point_context").dump(4).c_str();;

        if (data.contains("view_points"))
        {
            if (text.size())
                text += "\n\n";

            text = "View Points: " + to_string(data.at("view_points").size());
        }

        context_edit_->setText(text.c_str());
    }
}

