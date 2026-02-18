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

//#include "dbcontent/variable/variable.h"
//#include "global.h"
#include "property.h"
#include "logger.h"

#include <QComboBox>
#include <stdexcept>

namespace dbContent
{

class VariableDataTypeComboBox : public QComboBox
{
    Q_OBJECT

  public slots:
    /// @brief Sets the data type
    void changed()
    {
        loginf << "start" << currentText().toStdString();

        data_type_str_ = currentText().toStdString();
        data_type_ = Property::asDataType(data_type_str_);
    }

  public:
    /// @brief Constructor
    VariableDataTypeComboBox(PropertyDataType& data_type, std::string& data_type_str, QWidget* parent = nullptr)
        : QComboBox(parent), data_type_(data_type), data_type_str_(data_type_str)
    {
        for (auto& type_it : Property::dataTypes2Strings())
        {
            addItem(type_it.second.c_str());
        }

        update();

        //connect(this, SIGNAL(activated(const QString&)), this, SIGNAL(changedType()));
        connect(this, SIGNAL(activated(const QString&)), this, SLOT(changed()));
    }
    /// @brief Destructor
    virtual ~VariableDataTypeComboBox() {}

    /// @brief Sets the currently selected data type
    void setType(PropertyDataType& type, std::string& data_type_str)
    {
        data_type_ = type;
        data_type_str_ = data_type_str;

        update();
    }

  protected:
    /// Used variable
    PropertyDataType& data_type_;
    std::string& data_type_str_;

    void update()
    {
        int index = findText(QString(Property::asString(data_type_).c_str()));
        traced_assert(index >= 0);
        setCurrentIndex(index);
    }
};

}
