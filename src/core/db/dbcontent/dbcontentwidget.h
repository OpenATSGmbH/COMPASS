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

class QLineEdit;
class QComboBox;
class QCheckBox;
class QGridLayout;
class QPushButton;
class QTextEdit;

class DBContent;


/**
 * @brief Edit widget for a DBContent
 */
class DBContentWidget : public QWidget
{
    Q_OBJECT

  signals:
    void changedDBContSignal();

  public slots:
    void updateDataSourcesGridSlot();

    void editNameSlot();
    void editInfoSlot();

    void editDBContentVariableNameSlot();
    void editDBContentVariableDescriptionSlot();
    void editDBContentVariableDBColumnSlot(const QString& text);
    void deleteDBContVarSlot();

    void updateDBContVarsGridSlot();

  public:
    DBContentWidget(DBContent* object, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~DBContentWidget();

  private:
    DBContent* object_{nullptr};

    QLineEdit* name_edit_{nullptr};
    QLineEdit* info_edit_{nullptr};

    QGridLayout* ds_grid_{nullptr};

    /// @brief grid with all meta tables per schema
    QGridLayout* meta_table_grid_{nullptr};

    QPushButton* new_meta_button_{nullptr};

    QGridLayout* dbcontvars_grid_{nullptr};
};
