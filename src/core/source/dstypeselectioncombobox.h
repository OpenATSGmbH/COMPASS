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

#include "datasourcemanager.h"
#include "logger.h"

#include <QComboBox>

#include <algorithm>

class DSTypeSelectionComboBox : public QComboBox
{
    Q_OBJECT

signals:
    void changedTypeSignal(const QString& type);

public slots:
    void changedSlot()
    {
        if (doing_update_)
            return;

        loginf << currentText().toStdString();

        ds_type_ = currentText().toStdString();

        emit changedTypeSignal(ds_type_.c_str());
    }

public:
    DSTypeSelectionComboBox(QWidget* parent = nullptr)
        : QComboBox(parent)
    {
        addItem(""); // to show none

        for (auto it : DataSourceManager::data_source_types_)
            addItem(it.c_str());

        updateCurrentText();

        connect(this, SIGNAL(currentTextChanged(const QString&)), this, SLOT(changedSlot()));
    }

    virtual ~DSTypeSelectionComboBox() {}

    std::string type()
    {
        if (ds_type_.size())
            traced_assert(std::find(DataSourceManager::data_source_types_.begin(),
                              DataSourceManager::data_source_types_.end(), ds_type_)
                    != DataSourceManager::data_source_types_.end());

        return ds_type_;
    }

    void setType(const std::string& type)
    {
        ds_type_ = type;

        if (ds_type_.size())
            traced_assert(std::find(DataSourceManager::data_source_types_.begin(),
                              DataSourceManager::data_source_types_.end(), ds_type_)
                    != DataSourceManager::data_source_types_.end());

        updateCurrentText();
    }

protected:
    std::string ds_type_;
    bool doing_update_ {false};

    void updateCurrentText()
    {
        doing_update_ = true;

        if (ds_type_.size())
            traced_assert(std::find(DataSourceManager::data_source_types_.begin(),
                              DataSourceManager::data_source_types_.end(), ds_type_)
                    != DataSourceManager::data_source_types_.end());

        setCurrentText(ds_type_.c_str());

        doing_update_ = false;
    }

};
