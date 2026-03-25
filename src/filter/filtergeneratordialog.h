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

#include "json_fwd.hpp"

namespace dbContent
{
class VariableSelectionWidget;
}

class DBFilter;
class FilterManager;
class QLineEdit;
class QCheckBox;
class QComboBox;
class QHBoxLayout;
class QVBoxLayout;
class QPushButton;

struct ConditionTemplate
{
    std::string variable_name_;
    std::string variable_dbcont_name_;
    std::string operator_;
    std::string value_;
    std::string value2_; // second value for BETWEEN
    std::string reset_value_;
    bool absolute_value_{false};
    bool include_null_{false};
};

class FilterGeneratorDialog : public QDialog
{
    Q_OBJECT

  public slots:
    void updateOperatorCombo();
    void updateValueField();
    void updateAddConditionButton();
    void updateAddButton();
    void addOrUpdateCondition();
    void editCondition(int index);
    void removeCondition(int index);
    void cancelEditCondition();
    void accept() override;
    void cancel();

  public:
    // create mode
    FilterGeneratorDialog(FilterManager& filter_man, QWidget* parent = nullptr);
    // edit mode
    FilterGeneratorDialog(FilterManager& filter_man, DBFilter& filter, QWidget* parent = nullptr);
    virtual ~FilterGeneratorDialog();

  protected:
    FilterManager& filter_man_;
    DBFilter* edit_filter_{nullptr};

    QLineEdit* filter_name_{nullptr};
    dbContent::VariableSelectionWidget* condition_variable_widget_{nullptr};
    QPushButton* add_condition_button_{nullptr};
    QPushButton* cancel_edit_button_{nullptr};
    QPushButton* add_button_{nullptr};
    QComboBox* condition_combo_{nullptr};
    QHBoxLayout* condition_combo_layout_{nullptr};
    QCheckBox* condition_absolute_{nullptr};
    QCheckBox* condition_include_null_{nullptr};

    // value field area — swapped based on operator
    QHBoxLayout* condition_value_layout_{nullptr};
    QLineEdit* condition_value_{nullptr};       // default single value
    QLineEdit* condition_value_min_{nullptr};    // BETWEEN min
    QLineEdit* condition_value_max_{nullptr};    // BETWEEN max
    QComboBox* condition_value_is_{nullptr};     // IS / IS NOT dropdown

    QVBoxLayout* conditions_list_layout_{nullptr};

    std::vector<ConditionTemplate> data_conditions_;
    int editing_condition_index_{-1}; // -1 = adding new, >= 0 = editing existing

    void init();
    void createGUIElements();
    void loadConditionsFromFilter();
    void updateConditionsList();
    void clearValueFields();
    void resetConditionFields();
    ConditionTemplate collectConditionFromUI();
    void populateUIFromCondition(const ConditionTemplate& cond);
    bool validateValue();
    void createConditionConfigs(nlohmann::json& parent_json, const std::string& filter_name);
};
