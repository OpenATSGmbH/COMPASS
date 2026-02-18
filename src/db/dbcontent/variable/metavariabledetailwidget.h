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

class DBContentManager;


class QLineEdit;
class QTextEdit;
class QPushButton;

namespace dbContent
{

class MetaVariable;
class VariableSelectionWidget;

class MetaVariableDetailWidget : public QWidget
{
    Q_OBJECT

public slots:
    void nameEditedSlot();
    void variableChangedSlot();

    void deleteVariableSlot();

public:
    MetaVariableDetailWidget(DBContentManager& dbcont_man, QWidget* parent = nullptr);

    void show (MetaVariable& meta_var);
    void clear();

private:
    DBContentManager& dbcont_man_;

    bool has_current_entry_ {false};
    MetaVariable* meta_var_ {nullptr};

    QLineEdit* name_edit_{nullptr};
    QTextEdit* description_edit_{nullptr};

    std::map<std::string, VariableSelectionWidget*> selection_widgets_; // db content -> var select

    QPushButton* delete_button_ {nullptr};
};

}
