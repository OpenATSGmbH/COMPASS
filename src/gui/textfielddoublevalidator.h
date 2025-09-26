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

#include "compass.h"

#include <QDoubleValidator>
#include <QLineEdit>

class TextFieldDoubleValidator : public QDoubleValidator
{
  public:
    TextFieldDoubleValidator(QObject* parent = nullptr) : QDoubleValidator(parent)
    {
        setNotation(QDoubleValidator::StandardNotation);
    }

    TextFieldDoubleValidator(double bottom, double top, int decimals, QObject* parent = nullptr)
        : QDoubleValidator(bottom, top, decimals, parent)
    {
        setNotation(QDoubleValidator::StandardNotation);
    }

    QValidator::State validate(QString& s, int& pos) const
    {
        if (s.isEmpty() || (s.startsWith("-") && s.length() == 1))  //||
            return QValidator::Intermediate;

        if (decimals())
        {
            if ((s.startsWith("0") && s.size() == 1) || (s.startsWith("0.") && s.size() == 2))
            {
                //loginf << "UGA dec inter";
                return QValidator::Acceptable;
            }

            // check length of decimal places
            QChar point = '.';
            if (s.indexOf(point) != -1)
            {
                int lengthDecimals = s.length() - s.indexOf(point) - 1;

                if (lengthDecimals > decimals())
                    return QValidator::Invalid;
            }

            // check range of value
            bool ok;
            double value = s.toDouble(&ok);

            //loginf << "UGA dec ok " << ok << " bot " << (bottom() <= value) << " top " << (value <= top());

            if (ok && bottom() <= value && value <= top())
                return QValidator::Acceptable;

            return QValidator::Invalid;
        }
        else  // int
        {
            // check range of value
            bool ok;
            int value = s.toInt(&ok);

            //loginf << "UGA int ok " << ok << " bot " << (bottom() <= value) << " top " << (value <= top());

            if (ok && bottom() <= value && value <= top())
                return QValidator::Acceptable;

            return QValidator::Invalid;
        }
    }

    static void displayValidityAsColor(QLineEdit* line_edit)
    {
        traced_assert(line_edit);

        if (line_edit->hasAcceptableInput())
            line_edit->setStyleSheet("");
        else
            line_edit->setStyleSheet(COMPASS::instance().lineEditInvalidStyle());
    }
};

