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
#include <vector>

//#include "configuration.h"

class FilterConditionOperatorComboBox;
class FilterConditionResetValueComboBox;
class QGridLayout;
class QLineEdit;
class QCheckBox;
class QComboBox;
class QPushButton;
class DBFilterCondition;
class DBFilter;

namespace dbContent
{
class VariableSelectionWidget;
}

class FilterEditWidget : public QWidget
{
    Q_OBJECT
  public slots:
    //  void loadMin ();
    //  void loadMax ();
    void addCondition();
    void deleteCondition();
    void changedName();
    void changedConditionVariable();
    void changedABS();
    void changedOperator();
    void changedResetValue();

  public:
    FilterEditWidget(DBFilter* filter, QWidget* parent = nullptr);
    virtual ~FilterEditWidget();

  protected:
    DBFilter* filter_;
    QLineEdit* filter_name_;
    dbContent::VariableSelectionWidget* condition_variable_widget_;
    QCheckBox* condition_absolute_;
    FilterConditionOperatorComboBox* condition_combo_;
    FilterConditionResetValueComboBox* condition_reset_combo_;
    QLineEdit* condition_value_;

    QGridLayout* conditions_grid_;
    std::map<QPushButton*, DBFilterCondition*> conditions_delete_buttons_;
    std::map<dbContent::VariableSelectionWidget*, DBFilterCondition*> conditions_variable_selects_;
    std::map<QCheckBox*, DBFilterCondition*> conditions_abs_checkboxes_;
    std::map<FilterConditionOperatorComboBox*, DBFilterCondition*> conditions_operator_combos_;
    std::map<FilterConditionResetValueComboBox*, DBFilterCondition*> conditions_reset_value_combos_;

    void updateConditionsGrid();
};
