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
#include <vector>

//#include "configuration.h"

namespace dbContent
{
class VariableSelectionWidget;
}

class FilterManager;
class QLineEdit;
class QCheckBox;
class QComboBox;
class QListWidget;
class QPushButton;

typedef struct
{
    std::string variable_name_;
    std::string variable_dbcont_name_;
    std::string operator_;
    std::string value_;
    std::string reset_value_;
    bool absolute_value_;
} ConditionTemplate;

class FilterGeneratorDialog : public QDialog
{
    Q_OBJECT

  public slots:
//    void loadMin();
//    void loadMax();
    void updateAddConditionButton();
    void updateAddButton();
    void addCondition();
    void accept() override;
    void cancel();

  public:
    FilterGeneratorDialog(FilterManager& filter_man, QWidget* parent = nullptr);
    virtual ~FilterGeneratorDialog();

  protected:
    FilterManager& filter_man_;

    QLineEdit* filter_name_{nullptr};
    dbContent::VariableSelectionWidget* condition_variable_widget_{nullptr};
    QPushButton* add_condition_button_{nullptr};
    QPushButton* add_button_{nullptr};
    QComboBox* condition_combo_{nullptr};
    QCheckBox* condition_absolute_{nullptr};
    QLineEdit* condition_value_{nullptr};
    //QComboBox* condition_reset_combo_{nullptr};
    
    QListWidget* conditions_list_{nullptr};

    std::vector<ConditionTemplate> data_conditions_;

    void createGUIElements();
    void updateWidgetList();
};
