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

#include "dbcontent/variable/variable.h"
//#include "global.h"
#include "logger.h"

#include <QComboBox>

#include <stdexcept>

class StringRepresentationComboBox : public QComboBox
{
    Q_OBJECT

  signals:
    void changedRepresentation();

  public slots:
    void changedSlot()
    {
        loginf << currentText().toStdString();

        representation_ = representation();
        representation_str_ = dbContent::Variable::representationToString(representation_);
    }

  public:
    StringRepresentationComboBox(dbContent::Variable::Representation& representation, std::string& representation_str,
                                 QWidget* parent = nullptr)
        : QComboBox(parent), representation_(representation), representation_str_(representation_str)
    {
        for (auto it : dbContent::Variable::Representations())
            addItem(it.second.c_str());

        update();

        connect(this, SIGNAL(currentTextChanged(const QString&)), this, SLOT(changedSlot()));
    }

    virtual ~StringRepresentationComboBox() {}

    dbContent::Variable::Representation representation()
    {
        std::string text = currentText().toStdString();
        return dbContent::Variable::stringToRepresentation(text);
    }

    void setRepresentation(dbContent::Variable::Representation& representation,
                           std::string& representation_str)
    {
        representation_ = representation;
        representation_str_ = representation_str;

        update();
    }

  protected:
    dbContent::Variable::Representation& representation_;
    std::string& representation_str_;

    void update()
    {
        setCurrentText(dbContent::Variable::representationToString(representation_).c_str());
    }
};
