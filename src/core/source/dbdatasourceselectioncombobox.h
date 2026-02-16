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
#include "datasourcemanager.h"
#include "source/dbdatasource.h"
#include "logger.h"

#include <QComboBox>

namespace dbContent
{

class DBDataSourceComboBox : public QComboBox
{
    Q_OBJECT

  signals:
    void changedSource();

  public:
    DBDataSourceComboBox(QWidget* parent = nullptr)
        : QComboBox(parent)
    {
        updateBox();

        connect(this, SIGNAL(activated(const QString&)), this, SIGNAL(changedSource()));
    }
    virtual ~DBDataSourceComboBox() {}

    void showDSTypeOnly(const std::string& only_dstype_name)
    {
        only_dstype_name_ = only_dstype_name;
        show_dstype_only_ = true;

        updateBox();
    }

    void disableShowDSTypeOnly()
    {
        only_dstype_name_ = "";
        show_dstype_only_ = false;
    }

    void showDBContentOnly(const std::string& only_dbcontent_name)
    {
        only_dbcontent_name_ = only_dbcontent_name;
        show_dbcontent_only_ = true;

        updateBox();
    }

    void disableShowDBContOnly()
    {
        only_dbcontent_name_ = "";
        show_dbcontent_only_ = false;
    }

    std::string getDSName() { return currentText().toStdString(); }

    bool hasDSName(const std::string& ds_name)
    {
        int index = findText(QString(ds_name.c_str()));
        return index >= 0;
    }

    void setDSName(const std::string& ds_name)
    {
        int index = findText(QString(ds_name.c_str()));
        traced_assert(index >= 0);
        setCurrentIndex(index);
    }

  protected:
    bool show_dstype_only_{false};
    std::string only_dstype_name_;

    bool show_dbcontent_only_{false};
    std::string only_dbcontent_name_;

    void updateBox()
    {
        clear();

        if (show_dstype_only_)
            loginf << "show_dstype_only " << show_dstype_only_ << " only_dstype_name '" << only_dstype_name_ << "'";

        if (show_dbcontent_only_)
            loginf << "show_dbcontent_only " << show_dbcontent_only_ << " only_dbcontent_name '" << only_dbcontent_name_ << "'";

        for (auto& ds_it : COMPASS::instance().dataSourceManager().dbDataSources())
        {
            if (show_dstype_only_ && ds_it->dsType() != only_dstype_name_)
            {
                loginf << "skip " << ds_it->name() << " dsType " << ds_it->dsType();
                continue;
            }

            if (show_dbcontent_only_ && !ds_it->hasNumInserted(only_dbcontent_name_))
            {
                loginf << "skip " << ds_it->name() << " hasNumInserted " << ds_it->hasNumInserted(only_dbcontent_name_);
                continue;
            }

            addItem(ds_it->name().c_str());
        }

        setCurrentIndex(0);
    }
};

}
