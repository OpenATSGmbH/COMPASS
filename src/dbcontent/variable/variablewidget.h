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
class QPushButton;
class QTextEdit;

class StringRepresentationComboBox;
class UnitSelectionWidget;
class QGridLayout;

namespace dbContent
{

class Variable;
class VariableDataTypeComboBox;

class VariableWidget : public QWidget
{
    Q_OBJECT

  signals:
    void dbcontVariableChangedSignal();

  public slots:
    /// @brief Changes DBCont name
    void editNameSlot();
    /// @brief Changes DBCont info
    void editDescriptionSlot();
    //void editDataTypeSlot();

  public:
    /// @brief Constructor
    VariableWidget(Variable& variable, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    /// @brief Destructor
    virtual ~VariableWidget();

    void lock();
    void unlock();

    void setVariable(Variable& variable);
    void update();

  private:
    /// @brief DBContent to be managed
    Variable* variable_{nullptr};
    QGridLayout* properties_layout_{nullptr};

    bool locked_{false};

    /// @brief DBContVariable name
    QLineEdit* name_edit_{nullptr};
    /// @brief DBContVariable info
    QLineEdit* description_edit_{nullptr};
    VariableDataTypeComboBox* type_combo_{nullptr};
    StringRepresentationComboBox* representation_box_{nullptr};
    UnitSelectionWidget* unit_sel_{nullptr};
};

}
