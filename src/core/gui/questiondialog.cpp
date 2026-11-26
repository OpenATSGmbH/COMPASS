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

#include "questiondialog.h"

#include <QAbstractButton>
#include <QIcon>
#include <QMessageBox>
#include <QPushButton>

bool QuestionDialog::ask(QWidget* parent, const QString& title, const QString& text)
{
    QMessageBox msg(parent);
    msg.setObjectName("question_dialog");
    msg.setWindowTitle(title);
    msg.setText(text);
    msg.setIcon(QMessageBox::NoIcon);
    msg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msg.setDefaultButton(QMessageBox::No);

    // strip platform-default Yes/No button icons (project rule: no colored icons)
    for (auto* btn : msg.buttons())
        if (auto* push = qobject_cast<QPushButton*>(btn))
            push->setIcon(QIcon());

    // object names for ui testing - QMessageBox::exec returns the clicked
    // button, so tests must click Yes/No instead of accepting/rejecting
    if (auto* yes = qobject_cast<QPushButton*>(msg.button(QMessageBox::Yes)))
        yes->setObjectName("question_yes_button");
    if (auto* no = qobject_cast<QPushButton*>(msg.button(QMessageBox::No)))
        no->setObjectName("question_no_button");

    return msg.exec() == QMessageBox::Yes;
}
