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

#include <QLineEdit>

class InvalidQLineEdit : public QLineEdit
{
  public:
    explicit InvalidQLineEdit(QWidget* parent = Q_NULLPTR) : QLineEdit(parent) {}
    explicit InvalidQLineEdit(const QString& text, QWidget* parent = Q_NULLPTR)
        : QLineEdit(text, parent)
    {
    }

    void setValid(bool value)
    {
        if (value)
            setStyleSheet(
                "QLineEdit { background: rgb(255, 255, 255); selection-background-color:"
                " rgb(200, 200, 200); }");
        else
            setStyleSheet(
                "QLineEdit { background: rgb(255, 100, 100); selection-background-color:"
                " rgb(255, 200, 200); }");
    }
};
