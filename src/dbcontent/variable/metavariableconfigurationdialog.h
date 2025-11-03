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

#include <QDialog>
#include <QListWidget>

class DBContentManager;

class QSplitter;
class QPushButton;

namespace dbContent
{

class MetaVariableDetailWidget;

class MetaVariableConfigurationDialog : public QDialog
{
    Q_OBJECT

signals:
    void okSignal();

public slots:
    void okClickedSlot();
    void itemSelectedSlot(const QString& text);

    void addAllMetaVariablesSlot();

public:
    MetaVariableConfigurationDialog(DBContentManager& dbcont_man);
    virtual ~MetaVariableConfigurationDialog();

    void updateList();
    void selectMetaVariable (const std::string& name);
    void clearDetails();

protected:
    DBContentManager& dbcont_man_;

    QSplitter* splitter_{nullptr};

    QListWidget* list_widget_ {nullptr};

    MetaVariableDetailWidget* detail_widget_{nullptr};

};

}
