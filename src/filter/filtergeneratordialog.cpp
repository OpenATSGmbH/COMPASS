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
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "filterconditionoperatorcombobox.h"
#include "filtermanager.h"
#include "dbcontent/variable/metavariable.h"
#include "global.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

using namespace Utils;

FilterGeneratorDialog::FilterGeneratorDialog(QWidget* parent) : QDialog(parent)
{
    setWindowTitle(tr("Add New Filter"));

    setMinimumSize(800, 400);

    createGUIElements();
}

FilterGeneratorDialog::~FilterGeneratorDialog() {}

void FilterGeneratorDialog::createGUIElements()
{
    QFont font_bold;
    font_bold.setBold(true);
    QFont font_big;
    font_big.setPointSize(18);

    QVBoxLayout* layout = new QVBoxLayout();

    QHBoxLayout* name_layout = new QHBoxLayout();
    QLabel* name_label = new QLabel(tr("Define unique filter name"));
    name_layout->addWidget(name_label);
    filter_name_ = new QLineEdit(tr("Filter0"));
    connect(filter_name_, &QLineEdit::textChanged, this, &FilterGeneratorDialog::updateAddButton);
    name_layout->addWidget(filter_name_);
    layout->addLayout(name_layout);

    QLabel* condition_label = new QLabel(tr("Define condition"));
    layout->addWidget(condition_label);

    QGridLayout* condition_layout = new QGridLayout();

    // Variable
    QLabel* label_var = new QLabel(tr("Variable"));
    label_var->setFont(font_bold);
    condition_layout->addWidget(label_var, 0, 0);

    condition_variable_widget_ = new dbContent::VariableSelectionWidget();
    condition_variable_widget_->setMinimumWidth(200);
    condition_variable_widget_->showMetaVariables(true);
    condition_variable_widget_->showEmptyVariable(true);
    connect(condition_variable_widget_, &dbContent::VariableSelectionWidget::selectionChanged, this, &FilterGeneratorDialog::updateAddConditionButton);
    condition_layout->addWidget(condition_variable_widget_, 1, 0);

    // ABS
    QLabel* label_abs = new QLabel(tr("ABS"));
    label_abs->setFont(font_bold);
    condition_layout->addWidget(label_abs, 0, 1);

    condition_absolute_ = new QCheckBox("");
    condition_layout->addWidget(condition_absolute_, 1, 1, Qt::AlignHCenter);

    // Operator
    QLabel* label_op = new QLabel(tr("Operator"));
    label_op->setFont(font_bold);
    condition_layout->addWidget(label_op, 0, 2);

    condition_combo_ = new FilterConditionOperatorComboBox();
    condition_layout->addWidget(condition_combo_, 1, 2);

    // Value
    QLabel* label_val = new QLabel(tr("Value"));
    label_val->setFont(font_bold);
    condition_layout->addWidget(label_val, 0, 3);

    condition_value_ = new QLineEdit();
    connect(condition_value_, &QLineEdit::textChanged, this, &FilterGeneratorDialog::updateAddConditionButton);
    condition_layout->addWidget(condition_value_, 1, 3);

    // Reset Value
    QLabel* label_reset = new QLabel(tr("Reset value"));
    label_reset->setFont(font_bold);
    condition_layout->addWidget(label_reset, 0, 4);

    condition_reset_combo_ = new QComboBox();
    condition_reset_combo_->addItem("value");
//    condition_reset_combo_->addItem("MIN");
//    condition_reset_combo_->addItem("MAX");
    condition_layout->addWidget(condition_reset_combo_, 1, 4);

    layout->addLayout(condition_layout);

    add_condition_button_ = new QPushButton(tr("Add condition"));
    add_condition_button_->setEnabled(false);
    connect(add_condition_button_, &QPushButton::clicked, this, &FilterGeneratorDialog::addCondition);
    layout->addWidget(add_condition_button_);
    // layout->addStretch(); 

    // QFrame* line = new QFrame();
    // line->setFrameShape(QFrame::HLine);
    // line->setFrameShadow(QFrame::Sunken);
    // layout->addWidget(line);

    layout->addSpacing(50);

    QLabel* conditions_label = new QLabel(tr("Current conditions"));
    conditions_label->setFont(font_bold);
    layout->addWidget(conditions_label);

    conditions_list_ = new QListWidget();
    layout->addWidget(conditions_list_);

    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* cancel = new QPushButton(tr("Cancel"));
    connect(cancel, &QPushButton::clicked, this, &FilterGeneratorDialog::cancel);
    button_layout->addWidget(cancel);

    button_layout->addStretch();

    add_button_ = new QPushButton(tr("Add"));
    add_button_->setEnabled(false);
    connect(add_button_, &QPushButton::clicked, this, &FilterGeneratorDialog::accept);
    button_layout->addWidget(add_button_);

    layout->addLayout(button_layout);
    setLayout(layout);
}

//void FilterGeneratorWidget::loadMin()
//{
//    traced_assert(condition_variable_widget_);

//    std::string value;
//    if (condition_variable_widget_->hasVariable())
//    {
//        value = condition_variable_widget_->selectedVariable().getMinStringRepresentation();
//    }
//    else if (condition_variable_widget_->hasMetaVariable())
//    {
//        value = condition_variable_widget_->selectedMetaVariable().getMinStringRepresentation();
//    }
//    else
//    {
//        QMessageBox msgBox;
//        msgBox.setText("Error: No variable selected.");
//        msgBox.exec();
//        return;
//    }
//    condition_value_->setText(tr(value.c_str()));
//}
//void FilterGeneratorWidget::loadMax()
//{
//    traced_assert(condition_variable_widget_);

//    std::string value;
//    if (condition_variable_widget_->hasVariable())
//    {
//        value = condition_variable_widget_->selectedVariable().getMaxStringRepresentation();
//    }
//    else if (condition_variable_widget_->hasMetaVariable())
//    {
//        value = condition_variable_widget_->selectedMetaVariable().getMaxStringRepresentation();
//    }
//    else
//    {
//        QMessageBox msgBox;
//        msgBox.setText("Error: No variable selected.");
//        msgBox.exec();
//        return;
//    }
//    condition_value_->setText(tr(value.c_str()));
//}

void FilterGeneratorDialog::updateAddConditionButton()
{
    assert(add_condition_button_);
    
    bool has_variable = false;
    if (condition_variable_widget_)
    {
        has_variable = condition_variable_widget_->hasVariable() || condition_variable_widget_->hasMetaVariable();
    }
    
    bool has_value = false;
    if (condition_value_)
    {
        has_value = !condition_value_->text().isEmpty();
    }

    add_condition_button_->setEnabled(has_variable && has_value);
}

void FilterGeneratorDialog::updateAddButton()
{
    if (!add_button_)
        return;

    auto& filter_man = COMPASS::instance().filterManager();

    bool has_name = filter_name_ && !filter_name_->text().isEmpty()
                    && !filter_man.hasFilter(filter_name_->text().toStdString());
    
    bool has_conditions = !data_conditions_.empty();

    add_button_->setEnabled(has_name && has_conditions);

    if (!has_name)
        add_button_->setToolTip("Please set a (unique) filter name");

    if (!has_conditions)
        add_button_->setToolTip("Please add at least one condition");        
}

void FilterGeneratorDialog::addCondition()
{
    traced_assert(condition_variable_widget_);
    traced_assert(condition_combo_);

    ConditionTemplate data_condition;

    if (condition_variable_widget_->hasVariable())
    {
        const dbContent::Variable& var = condition_variable_widget_->selectedVariable();
        data_condition.variable_name_ = var.name();
        data_condition.variable_dbcont_name_ = var.dbContentName();
    }
    else
    {
        traced_assert(condition_variable_widget_->hasMetaVariable());
        dbContent::MetaVariable& var = condition_variable_widget_->selectedMetaVariable();
        data_condition.variable_name_ = var.name();
        data_condition.variable_dbcont_name_ = META_OBJECT_NAME;
    }

    data_condition.absolute_value_ = condition_absolute_->checkState() == Qt::Checked;
    data_condition.operator_ = condition_combo_->currentText().toStdString();
    data_condition.value_ = condition_value_->text().toStdString();
    data_condition.reset_value_ = condition_reset_combo_->currentText().toStdString();

    data_conditions_.push_back(data_condition);

    updateWidgetList();
    updateAddButton();
}

void FilterGeneratorDialog::updateWidgetList()
{
    traced_assert(conditions_list_);
    conditions_list_->clear();

    for (unsigned int cnt = 0; cnt < data_conditions_.size(); cnt++)
    {
        ConditionTemplate& data_condition = data_conditions_.at(cnt);
        std::string variable = data_condition.variable_name_;
        if (data_condition.absolute_value_)
            variable = "ABS(" + variable + ")";
        std::string text = variable + " " + data_condition.operator_ + " " + data_condition.value_ +
                           ", resets to " + data_condition.reset_value_;
        conditions_list_->addItem(tr(text.c_str()));
    }
}

void FilterGeneratorDialog::accept()
{
    loginf;

    std::string filter_name = filter_name_->text().toStdString();

    auto& filter_man = COMPASS::instance().filterManager();

    traced_assert(!filter_man.hasFilter(filter_name));
    assert (!data_conditions_.empty());

    // if (filter_man.hasSubConfigurable("DBFilter", filter_name))
    // {
    //     QMessageBox::critical(this, "Error", "A filter with this name is already present. Please choose a unique filter name.");
    //     return;
    // }

    auto& configuration = filter_man.addNewSubConfiguration("DBFilter");
    configuration.addParameter<std::string>("name", filter_name);

    for (unsigned int cnt = 0; cnt < data_conditions_.size(); cnt++)
    {
        ConditionTemplate& data_condition = data_conditions_.at(cnt);
        std::string condition_name = filter_name + "Condition" + std::to_string(cnt);

        loginf << "creating condition with operator '"
               << data_condition.operator_ << "'";

        Configuration& condition_configuration = configuration.addNewSubConfiguration("DBFilterCondition", condition_name);
        condition_configuration.addParameter<std::string>("operator", data_condition.operator_);
        condition_configuration.addParameter<std::string>("variable_name", data_condition.variable_name_);
        condition_configuration.addParameter<std::string>("variable_dbcontent_name", data_condition.variable_dbcont_name_);
        condition_configuration.addParameter<bool>("absolute_value", data_condition.absolute_value_);
        condition_configuration.addParameter<std::string>("value", data_condition.value_);

        std::string reset_value;
        if (data_condition.reset_value_.compare("MIN") == 0 ||
            data_condition.reset_value_.compare("MAX") == 0)
            reset_value = data_condition.reset_value_;
        else
            reset_value = data_condition.value_;
        
        condition_configuration.addParameter<std::string>("reset_value", reset_value);

        // configuration.addSubConfigurable ("DBFilterCondition", condition_name,
        // condition_config_name);
    }

    //filter_man.generateSubConfigurableFromConfig(std::move(configuration));
    filter_man.generateSubConfigurable (configuration.getClassId(), configuration.getInstanceId());

    QDialog::accept();
}

void FilterGeneratorDialog::cancel() 
{ 
    reject(); 
}
