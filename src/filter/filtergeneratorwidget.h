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

class QLineEdit;
class QCheckBox;
class QComboBox;
class QListWidget;

typedef struct
{
    std::string variable_name_;
    std::string variable_dbcont_name_;
    std::string operator_;
    std::string value_;
    std::string reset_value_;
    bool absolute_value_;
} ConditionTemplate;

class FilterGeneratorWidget : public QWidget
{
    Q_OBJECT
  signals:
    void filterWidgetAction(bool generated);

  public slots:
//    void loadMin();
//    void loadMax();
    void addCondition();
    void accept();
    void cancel();

  public:
    FilterGeneratorWidget(QWidget* parent = nullptr);
    virtual ~FilterGeneratorWidget();

  protected:
    QLineEdit* filter_name_;
    dbContent::VariableSelectionWidget* condition_variable_widget_;
    QComboBox* condition_combo_;
    QCheckBox* condition_absolute_;
    QLineEdit* condition_value_;
    QComboBox* condition_reset_combo_;
    QListWidget* conditions_list_;

    std::vector<ConditionTemplate> data_conditions_;

    void createGUIElements();
    void updateWidgetList();
    virtual void closeEvent(QCloseEvent* event);
};
