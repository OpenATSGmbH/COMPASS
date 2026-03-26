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

#include "filtergeneratordialog.h"
#include "compass.h"
#include "configuration.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "filterconditionoperatorcombobox.h"
#include "filtermanager.h"
#include "dbfilter.h"
#include "dbfiltercondition.h"
#include "global.h"
#include "json.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

using namespace Utils;

// create mode
FilterGeneratorDialog::FilterGeneratorDialog(FilterManager& filter_man, QWidget* parent)
:   QDialog(parent),
    filter_man_(filter_man)
{
    init();
}

// edit mode
FilterGeneratorDialog::FilterGeneratorDialog(FilterManager& filter_man, DBFilter& filter, QWidget* parent)
:   QDialog(parent),
    filter_man_(filter_man),
    edit_filter_(&filter)
{
    loadConditionsFromFilter();
    init();
}

FilterGeneratorDialog::~FilterGeneratorDialog() {}

void FilterGeneratorDialog::init()
{
    if (edit_filter_)
        setWindowTitle(tr("Edit Filter"));
    else
        setWindowTitle(tr("Add New Filter"));

    setMinimumSize(800, 500);

    createGUIElements();

    if (edit_filter_)
    {
        filter_name_->setText(QString::fromStdString(edit_filter_->getName()));
        filter_name_->setReadOnly(true);
        add_button_->setText(tr("Save"));

        // load condition logic
        if (edit_filter_->conditionLogic() == "OR")
            condition_logic_combo_->setCurrentIndex(static_cast<int>(ConditionLogic::OR));
        else
            condition_logic_combo_->setCurrentIndex(static_cast<int>(ConditionLogic::AND));
    }

    updateConditionsList();
    updateAddButton();
}

void FilterGeneratorDialog::createGUIElements()
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* layout = new QVBoxLayout();

    // filter name
    QHBoxLayout* name_layout = new QHBoxLayout();
    QLabel* name_label = new QLabel(tr("Filter name"));
    name_layout->addWidget(name_label);
    filter_name_ = new QLineEdit(tr("Filter0"));
    connect(filter_name_, &QLineEdit::textChanged, this, &FilterGeneratorDialog::updateAddButton);
    name_layout->addWidget(filter_name_);
    layout->addLayout(name_layout);

    // condition editor section
    QLabel* condition_label = new QLabel(tr("Condition"));
    condition_label->setFont(font_bold);
    layout->addWidget(condition_label);

    QGridLayout* condition_layout = new QGridLayout();

    // Variable (col 0)
    QLabel* label_var = new QLabel(tr("Variable"));
    label_var->setFont(font_bold);
    condition_layout->addWidget(label_var, 0, 0);

    condition_variable_widget_ = new dbContent::VariableSelectionWidget(filter_man_.dbContentManager());
    condition_variable_widget_->setMinimumWidth(200);
    condition_variable_widget_->showMetaVariables(true);
    condition_variable_widget_->showEmptyVariable(true);
    connect(condition_variable_widget_, &dbContent::VariableSelectionWidget::selectionChanged,
            this, &FilterGeneratorDialog::updateOperatorCombo);
    connect(condition_variable_widget_, &dbContent::VariableSelectionWidget::selectionChanged,
            this, &FilterGeneratorDialog::updateAddConditionButton);
    condition_layout->addWidget(condition_variable_widget_, 1, 0);

    // ABS (col 1)
    QLabel* label_abs = new QLabel(tr("ABS"));
    label_abs->setFont(font_bold);
    condition_layout->addWidget(label_abs, 0, 1);

    condition_absolute_ = new QCheckBox("");
    condition_layout->addWidget(condition_absolute_, 1, 1, Qt::AlignHCenter);

    // Operator (col 2)
    QLabel* label_op = new QLabel(tr("Operator"));
    label_op->setFont(font_bold);
    condition_layout->addWidget(label_op, 0, 2);

    condition_combo_layout_ = new QHBoxLayout();
    condition_combo_ = new FilterConditionOperatorComboBox();
    connect(condition_combo_, &QComboBox::currentTextChanged,
            this, &FilterGeneratorDialog::updateValueField);
    connect(condition_combo_, &QComboBox::currentTextChanged,
            this, &FilterGeneratorDialog::updateAddConditionButton);
    condition_combo_layout_->addWidget(condition_combo_);
    condition_layout->addLayout(condition_combo_layout_, 1, 2);

    // Value (col 3)
    QLabel* label_val = new QLabel(tr("Value"));
    label_val->setFont(font_bold);
    condition_layout->addWidget(label_val, 0, 3);

    condition_value_layout_ = new QHBoxLayout();

    condition_value_ = new QLineEdit();
    connect(condition_value_, &QLineEdit::textChanged,
            this, &FilterGeneratorDialog::updateAddConditionButton);
    condition_value_layout_->addWidget(condition_value_);

    // BETWEEN min/max (created but hidden initially)
    condition_value_min_ = new QLineEdit();
    condition_value_min_->setPlaceholderText(tr("Min"));
    connect(condition_value_min_, &QLineEdit::textChanged,
            this, &FilterGeneratorDialog::updateAddConditionButton);
    condition_value_min_->hide();
    condition_value_layout_->addWidget(condition_value_min_);

    condition_value_max_ = new QLineEdit();
    condition_value_max_->setPlaceholderText(tr("Max"));
    connect(condition_value_max_, &QLineEdit::textChanged,
            this, &FilterGeneratorDialog::updateAddConditionButton);
    condition_value_max_->hide();
    condition_value_layout_->addWidget(condition_value_max_);

    // IS / IS NOT dropdown (created but hidden initially)
    condition_value_is_ = new QComboBox();
    condition_value_is_->addItem("NULL");
    condition_value_is_->addItem("NOT NULL");
    condition_value_is_->hide();
    condition_value_layout_->addWidget(condition_value_is_);

    condition_layout->addLayout(condition_value_layout_, 1, 3);

    // Include NULL (col 4)
    QLabel* label_null = new QLabel(tr("Include\nNULL"));
    label_null->setFont(font_bold);
    condition_layout->addWidget(label_null, 0, 4);

    condition_include_null_ = new QCheckBox("");
    condition_include_null_->setToolTip(tr("Also include rows where this column has no value (NULL)"));
    condition_layout->addWidget(condition_include_null_, 1, 4, Qt::AlignHCenter);

    layout->addLayout(condition_layout);

    // add/update condition button + cancel edit button
    QHBoxLayout* condition_button_layout = new QHBoxLayout();

    add_condition_button_ = new QPushButton(tr("Add condition"));
    add_condition_button_->setEnabled(false);
    connect(add_condition_button_, &QPushButton::clicked,
            this, &FilterGeneratorDialog::addOrUpdateCondition);
    condition_button_layout->addWidget(add_condition_button_);

    cancel_edit_button_ = new QPushButton(tr("Cancel edit"));
    cancel_edit_button_->hide();
    connect(cancel_edit_button_, &QPushButton::clicked,
            this, &FilterGeneratorDialog::cancelEditCondition);
    condition_button_layout->addWidget(cancel_edit_button_);

    condition_button_layout->addStretch();
    layout->addLayout(condition_button_layout);

    layout->addSpacing(20);

    // condition logic selector
    QHBoxLayout* logic_layout = new QHBoxLayout();
    QLabel* logic_label = new QLabel(tr("Join conditions with"));
    logic_label->setFont(font_bold);
    logic_layout->addWidget(logic_label);

    condition_logic_combo_ = new QComboBox();
    condition_logic_combo_->addItem("AND");
    condition_logic_combo_->addItem("OR");
    condition_logic_combo_->setCurrentIndex(0);
    logic_layout->addWidget(condition_logic_combo_);
    logic_layout->addStretch();
    layout->addLayout(logic_layout);

    // conditions list
    QLabel* conditions_label = new QLabel(tr("Current conditions"));
    conditions_label->setFont(font_bold);
    layout->addWidget(conditions_label);

    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    QWidget* scroll_widget = new QWidget();
    conditions_list_layout_ = new QVBoxLayout();
    conditions_list_layout_->setAlignment(Qt::AlignTop);
    scroll_widget->setLayout(conditions_list_layout_);
    scroll_area->setWidget(scroll_widget);
    layout->addWidget(scroll_area);

    // bottom buttons
    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* cancel_btn = new QPushButton(tr("Cancel"));
    connect(cancel_btn, &QPushButton::clicked, this, &FilterGeneratorDialog::cancel);
    button_layout->addWidget(cancel_btn);

    button_layout->addStretch();

    add_button_ = new QPushButton(tr("Add"));
    add_button_->setEnabled(false);
    connect(add_button_, &QPushButton::clicked, this, &FilterGeneratorDialog::accept);
    button_layout->addWidget(add_button_);

    layout->addLayout(button_layout);
    setLayout(layout);
}

void FilterGeneratorDialog::loadConditionsFromFilter()
{
    traced_assert(edit_filter_);

    for (const auto& cond : edit_filter_->getConditions())
    {
        ConditionTemplate ct;
        ct.variable_name_ = cond->getVariableName();
        ct.variable_dbcont_name_ = cond->getVariableDBContentName();
        ct.operator_ = cond->getOperator();
        ct.value_ = cond->getValue();
        ct.value2_ = cond->getValue2();
        ct.reset_value_ = cond->getResetValue();
        ct.absolute_value_ = cond->getAbsoluteValue();
        ct.include_null_ = cond->getIncludeNull();

        data_conditions_.push_back(ct);
    }
}

void FilterGeneratorDialog::updateOperatorCombo()
{
    traced_assert(condition_combo_layout_);

    PropertyDataType dt = PropertyDataType::STRING; // default

    if (condition_variable_widget_->hasVariable())
        dt = condition_variable_widget_->selectedVariable().dataType();
    else if (condition_variable_widget_->hasMetaVariable())
        dt = condition_variable_widget_->selectedMetaVariable().dataType();

    bool numeric_only = false;
    bool string_only = false;

    switch (dt)
    {
        case PropertyDataType::CHAR:
        case PropertyDataType::UCHAR:
        case PropertyDataType::INT:
        case PropertyDataType::UINT:
        case PropertyDataType::LONGINT:
        case PropertyDataType::ULONGINT:
        case PropertyDataType::FLOAT:
        case PropertyDataType::DOUBLE:
            numeric_only = true;
            break;
        case PropertyDataType::STRING:
            string_only = true;
            break;
        default: // BOOL, JSON, TIMESTAMP
            numeric_only = true;
            string_only = true;
            break;
    }

    // replace combo box
    delete condition_combo_;
    condition_combo_ = new FilterConditionOperatorComboBox(numeric_only, string_only);
    connect(condition_combo_, &QComboBox::currentTextChanged,
            this, &FilterGeneratorDialog::updateValueField);
    connect(condition_combo_, &QComboBox::currentTextChanged,
            this, &FilterGeneratorDialog::updateAddConditionButton);
    condition_combo_layout_->addWidget(condition_combo_);

    // ABS only for numeric types
    condition_absolute_->setEnabled(numeric_only && !string_only);
    if (!condition_absolute_->isEnabled())
        condition_absolute_->setChecked(false);

    updateValueField();
}

void FilterGeneratorDialog::updateValueField()
{
    std::string op = condition_combo_ ? condition_combo_->currentText().toStdString() : "";

    bool is_between = (op == "BETWEEN");
    bool is_is_op = (op == "IS" || op == "IS NOT");

    // show/hide appropriate value widgets
    condition_value_->setVisible(!is_between && !is_is_op);
    condition_value_min_->setVisible(is_between);
    condition_value_max_->setVisible(is_between);
    condition_value_is_->setVisible(is_is_op);

    // Include NULL disabled for IS/IS NOT
    condition_include_null_->setEnabled(!is_is_op);
    if (is_is_op)
        condition_include_null_->setChecked(false);

    // tooltip for LIKE/NOT LIKE
    if (op == "LIKE" || op == "NOT LIKE")
        condition_value_->setToolTip(tr("Use % as wildcard (e.g. %pattern%)"));
    else
        condition_value_->setToolTip("");
}

void FilterGeneratorDialog::updateAddConditionButton()
{
    traced_assert(add_condition_button_);

    bool has_variable = condition_variable_widget_ &&
        (condition_variable_widget_->hasVariable() || condition_variable_widget_->hasMetaVariable());

    bool has_value = validateValue();

    add_condition_button_->setEnabled(has_variable && has_value);
}

bool FilterGeneratorDialog::validateValue()
{
    std::string op = condition_combo_ ? condition_combo_->currentText().toStdString() : "";

    // IS / IS NOT always valid (dropdown enforces)
    if (op == "IS" || op == "IS NOT")
        return true;

    if (op == "BETWEEN")
    {
        bool min_ok = condition_value_min_ && !condition_value_min_->text().isEmpty();
        bool max_ok = condition_value_max_ && !condition_value_max_->text().isEmpty();

        // validate both fields
        if (min_ok)
        {
            condition_value_min_->setStyleSheet("");
        }
        else if (condition_value_min_)
        {
            condition_value_min_->setStyleSheet(LINE_EDIT_INVALID_STYLE);
        }

        if (max_ok)
        {
            condition_value_max_->setStyleSheet("");
        }
        else if (condition_value_max_)
        {
            condition_value_max_->setStyleSheet(LINE_EDIT_INVALID_STYLE);
        }

        return min_ok && max_ok;
    }

    // default: single value
    bool has_value = condition_value_ && !condition_value_->text().isEmpty();

    if (has_value)
        condition_value_->setStyleSheet("");
    else if (condition_value_)
        condition_value_->setStyleSheet(LINE_EDIT_INVALID_STYLE);

    return has_value;
}

void FilterGeneratorDialog::updateAddButton()
{
    if (!add_button_)
        return;

    bool has_name = false;

    if (edit_filter_)
    {
        has_name = true; // name is fixed in edit mode
    }
    else
    {
        has_name = filter_name_ && !filter_name_->text().isEmpty()
                   && !filter_man_.hasFilter(filter_name_->text().toStdString());
    }

    bool has_conditions = !data_conditions_.empty();

    add_button_->setEnabled(has_name && has_conditions);

    if (!has_name)
        add_button_->setToolTip(tr("Please set a unique filter name"));
    else if (!has_conditions)
        add_button_->setToolTip(tr("Please add at least one condition"));
    else
        add_button_->setToolTip("");
}

ConditionTemplate FilterGeneratorDialog::collectConditionFromUI()
{
    ConditionTemplate ct;

    if (condition_variable_widget_->hasVariable())
    {
        const dbContent::Variable& var = condition_variable_widget_->selectedVariable();
        ct.variable_name_ = var.name();
        ct.variable_dbcont_name_ = var.dbContentName();
    }
    else
    {
        traced_assert(condition_variable_widget_->hasMetaVariable());
        dbContent::MetaVariable& var = condition_variable_widget_->selectedMetaVariable();
        ct.variable_name_ = var.name();
        ct.variable_dbcont_name_ = META_OBJECT_NAME;
    }

    ct.absolute_value_ = condition_absolute_->checkState() == Qt::Checked;
    ct.operator_ = condition_combo_->currentText().toStdString();
    ct.include_null_ = condition_include_null_->checkState() == Qt::Checked;

    if (ct.operator_ == "IS" || ct.operator_ == "IS NOT")
    {
        ct.value_ = condition_value_is_->currentText().toStdString();
    }
    else if (ct.operator_ == "BETWEEN")
    {
        ct.value_ = condition_value_min_->text().toStdString();
        ct.value2_ = condition_value_max_->text().toStdString();
    }
    else
    {
        ct.value_ = condition_value_->text().toStdString();
    }

    ct.reset_value_ = ct.value_;

    return ct;
}

void FilterGeneratorDialog::addOrUpdateCondition()
{
    traced_assert(condition_variable_widget_);
    traced_assert(condition_combo_);

    ConditionTemplate ct = collectConditionFromUI();

    if (editing_condition_index_ >= 0)
    {
        // update existing
        data_conditions_.at(editing_condition_index_) = ct;
        editing_condition_index_ = -1;
        add_condition_button_->setText(tr("Add condition"));
        cancel_edit_button_->hide();
    }
    else
    {
        data_conditions_.push_back(ct);
    }

    resetConditionFields();
    updateConditionsList();
    updateAddButton();
}

void FilterGeneratorDialog::editCondition(int index)
{
    traced_assert(index >= 0 && index < (int)data_conditions_.size());

    editing_condition_index_ = index;
    add_condition_button_->setText(tr("Update condition"));
    cancel_edit_button_->show();

    populateUIFromCondition(data_conditions_.at(index));
}

void FilterGeneratorDialog::populateUIFromCondition(const ConditionTemplate& cond)
{
    // set variable
    if (cond.variable_dbcont_name_ == META_OBJECT_NAME)
    {
        if (filter_man_.dbContentManager().existsMetaVariable(cond.variable_name_))
        {
            auto& mv = filter_man_.dbContentManager().metaVariable(cond.variable_name_);
            condition_variable_widget_->selectedMetaVariable(mv);
        }
    }
    else
    {
        if (filter_man_.dbContentManager().existsDBContent(cond.variable_dbcont_name_))
        {
            auto& dbc = filter_man_.dbContentManager().dbContent(cond.variable_dbcont_name_);
            if (dbc.hasVariable(cond.variable_name_))
            {
                auto& var = dbc.variable(cond.variable_name_);
                condition_variable_widget_->selectedVariable(var);
            }
        }
    }

    // operator — updateOperatorCombo was triggered by variable change,
    // now set the correct operator
    int op_idx = condition_combo_->findText(QString::fromStdString(cond.operator_));
    if (op_idx >= 0)
        condition_combo_->setCurrentIndex(op_idx);

    // ABS
    condition_absolute_->setChecked(cond.absolute_value_);

    // Include NULL
    condition_include_null_->setChecked(cond.include_null_);

    // values
    if (cond.operator_ == "IS" || cond.operator_ == "IS NOT")
    {
        int val_idx = condition_value_is_->findText(QString::fromStdString(cond.value_));
        if (val_idx >= 0)
            condition_value_is_->setCurrentIndex(val_idx);
    }
    else if (cond.operator_ == "BETWEEN")
    {
        condition_value_min_->setText(QString::fromStdString(cond.value_));
        condition_value_max_->setText(QString::fromStdString(cond.value2_));
    }
    else
    {
        condition_value_->setText(QString::fromStdString(cond.value_));
    }
}

void FilterGeneratorDialog::removeCondition(int index)
{
    traced_assert(index >= 0 && index < (int)data_conditions_.size());

    // if we're editing this condition, cancel the edit
    if (editing_condition_index_ == index)
        cancelEditCondition();
    else if (editing_condition_index_ > index)
        editing_condition_index_--; // adjust index

    data_conditions_.erase(data_conditions_.begin() + index);

    updateConditionsList();
    updateAddButton();
}

void FilterGeneratorDialog::cancelEditCondition()
{
    editing_condition_index_ = -1;
    add_condition_button_->setText(tr("Add condition"));
    cancel_edit_button_->hide();
    resetConditionFields();
}

void FilterGeneratorDialog::resetConditionFields()
{
    condition_value_->clear();
    condition_value_min_->clear();
    condition_value_max_->clear();
    condition_absolute_->setChecked(false);
    condition_include_null_->setChecked(false);
}

void FilterGeneratorDialog::clearValueFields()
{
    condition_value_->clear();
    condition_value_min_->clear();
    condition_value_max_->clear();
}

void FilterGeneratorDialog::updateConditionsList()
{
    // clear existing widgets
    while (conditions_list_layout_->count() > 0)
    {
        QLayoutItem* item = conditions_list_layout_->takeAt(0);
        if (item->widget())
            delete item->widget();
        delete item;
    }

    for (int i = 0; i < (int)data_conditions_.size(); i++)
    {
        const ConditionTemplate& ct = data_conditions_.at(i);

        QHBoxLayout* row_layout = new QHBoxLayout();

        // build label text
        std::string variable = ct.variable_name_;
        if (ct.absolute_value_)
            variable = "ABS(" + variable + ")";

        std::string text = variable + " " + ct.operator_;

        if (ct.operator_ == "BETWEEN")
            text += " " + ct.value_ + " AND " + ct.value2_;
        else
            text += " " + ct.value_;

        if (ct.include_null_)
            text += " [+NULL]";

        QLabel* label = new QLabel(QString::fromStdString(text));
        label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        row_layout->addWidget(label);

        QPushButton* edit_btn = new QPushButton(tr("Edit"));
        edit_btn->setFixedWidth(60);
        int idx = i;
        connect(edit_btn, &QPushButton::clicked, this, [this, idx]() { editCondition(idx); });
        row_layout->addWidget(edit_btn);

        QPushButton* remove_btn = new QPushButton(tr("Remove"));
        remove_btn->setFixedWidth(70);
        connect(remove_btn, &QPushButton::clicked, this, [this, idx]() { removeCondition(idx); });
        row_layout->addWidget(remove_btn);

        QWidget* row_widget = new QWidget();
        row_widget->setLayout(row_layout);
        conditions_list_layout_->addWidget(row_widget);
    }
}

void FilterGeneratorDialog::createConditionConfigs(nlohmann::json& parent_json, const std::string& filter_name)
{
    for (unsigned int cnt = 0; cnt < data_conditions_.size(); cnt++)
    {
        const ConditionTemplate& ct = data_conditions_.at(cnt);
        std::string condition_name = filter_name + "Condition" + std::to_string(cnt);

        loginf << "creating condition with operator '" << ct.operator_ << "'";

        auto& condition_json = Configuration::addSubConfigEntry(
            parent_json, "DBFilterCondition", condition_name);
        condition_json[Configuration::ParameterSection]["operator"] = ct.operator_;
        condition_json[Configuration::ParameterSection]["variable_name"] = ct.variable_name_;
        condition_json[Configuration::ParameterSection]["variable_dbcontent_name"] = ct.variable_dbcont_name_;
        condition_json[Configuration::ParameterSection]["absolute_value"] = ct.absolute_value_;
        condition_json[Configuration::ParameterSection]["value"] = ct.value_;
        condition_json[Configuration::ParameterSection]["value2"] = ct.value2_;
        condition_json[Configuration::ParameterSection]["include_null"] = ct.include_null_;
        condition_json[Configuration::ParameterSection]["reset_value"] = ct.value_;
    }
}

void FilterGeneratorDialog::accept()
{
    loginf;

    std::string filter_name = filter_name_->text().toStdString();
    traced_assert(!data_conditions_.empty());

    std::string logic = condition_logic_combo_->currentIndex() == static_cast<int>(ConditionLogic::OR) ? "OR" : "AND";

    if (edit_filter_)
    {
        // edit mode: update logic and clear existing conditions
        edit_filter_->conditionLogic(logic);
        edit_filter_->clearConditions();
        edit_filter_->removeSubConfigurations("DBFilterCondition");

        for (unsigned int cnt = 0; cnt < data_conditions_.size(); cnt++)
        {
            const ConditionTemplate& ct = data_conditions_.at(cnt);
            std::string condition_name = filter_name + "Condition" + std::to_string(cnt);

            auto& cond_json = edit_filter_->addNewSubConfiguration("DBFilterCondition", condition_name);
            cond_json[Configuration::ParameterSection]["operator"] = ct.operator_;
            cond_json[Configuration::ParameterSection]["variable_name"] = ct.variable_name_;
            cond_json[Configuration::ParameterSection]["variable_dbcontent_name"] = ct.variable_dbcont_name_;
            cond_json[Configuration::ParameterSection]["absolute_value"] = ct.absolute_value_;
            cond_json[Configuration::ParameterSection]["value"] = ct.value_;
            cond_json[Configuration::ParameterSection]["value2"] = ct.value2_;
            cond_json[Configuration::ParameterSection]["include_null"] = ct.include_null_;
            cond_json[Configuration::ParameterSection]["reset_value"] = ct.value_;

            edit_filter_->generateSubConfigurable(cond_json);
        }
    }
    else
    {
        // create mode
        traced_assert(!filter_man_.hasFilter(filter_name));

        auto& child_json = filter_man_.addNewSubConfiguration("DBFilter");
        child_json[Configuration::ParameterSection]["name"] = filter_name;
        child_json[Configuration::ParameterSection]["is_custom"] = true;
        child_json[Configuration::ParameterSection]["condition_logic"] = logic;

        createConditionConfigs(child_json, filter_name);

        filter_man_.generateSubConfigurable(child_json);
    }

    QDialog::accept();
}

void FilterGeneratorDialog::cancel()
{
    reject();
}
