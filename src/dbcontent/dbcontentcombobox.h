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

#include <QComboBox>

#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "global.h"

class DBContentComboBox : public QComboBox
{
    Q_OBJECT

  signals:
    void changedObject();

  public:
    DBContentComboBox(bool allow_meta, bool no_status_content, QWidget* parent = nullptr)
        : QComboBox(parent), allow_meta_(allow_meta)
    {
        traced_assert(COMPASS::instance().dbContentManager().size());
        if (allow_meta_)
            addItem(META_OBJECT_NAME.c_str());

        for (auto& dbcont_it : COMPASS::instance().dbContentManager())
        {
            if (no_status_content && dbcont_it.second->containsStatusContent())
                continue;

            addItem(dbcont_it.first.c_str());
        }

        setCurrentIndex(0);
        connect(this, SIGNAL(activated(const QString&)), this, SIGNAL(changedObject()));
    }
    virtual ~DBContentComboBox() {}

    std::string getObjectName() { return currentText().toStdString(); }

    void setObjectName(const std::string& object_name)
    {
        int index = findText(QString(object_name.c_str()));
        traced_assert(index >= 0);
        setCurrentIndex(index);
    }

  protected:
    bool allow_meta_ {false};
};
