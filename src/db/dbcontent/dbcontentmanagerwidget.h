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

#include <QWidget>

#include <map>

class DBContent;
class DBContentWidget;
class DBContentManager;

namespace dbContent
{
class MetaVariable;
class MetaVariableWidget;
}

class QGridLayout;
class QScrollArea;
class QPushButton;
class QLineEdit;
class QComboBox;

class DBContentManagerWidget : public QWidget
{
    Q_OBJECT

  public slots:
    void addDBContSlot();
    void changedDBContSlot();
    void editDBContSlot();
    void deleteDBContSlot();
    void updateDBContentsSlot();

    void addMetaVariableSlot();
    void editMetaVariableSlot();
    void deleteMetaVariableSlot();
    void addAllMetaVariablesSlot();
    void updateMetaVariablesSlot();

  public:
    DBContentManagerWidget(DBContentManager& object_manager);
    virtual ~DBContentManagerWidget();

  private:
    DBContentManager& object_manager_;

    QGridLayout* dbconts_grid_{nullptr};
    QGridLayout* meta_variables_grid_{nullptr};

    QPushButton* add_dbcont_button_{nullptr};
    QPushButton* add_metavar_button_{nullptr};

    std::map<QPushButton*, DBContent*> edit_dbcont_buttons_;
    std::map<QPushButton*, DBContent*> delete_dbcont_buttons_;

    std::map<DBContent*, DBContentWidget*> edit_dbcont_widgets_;

    std::map<QPushButton*, dbContent::MetaVariable*> edit_meta_buttons_;
    std::map<QPushButton*, dbContent::MetaVariable*> delete_meta_buttons_;
    std::map<dbContent::MetaVariable*, dbContent::MetaVariableWidget*> edit_meta_widgets_;
};
