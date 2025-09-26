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

#include "filtergeneratorwidget.h"
#include "compass.h"
//#include "configurationmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "filterconditionoperatorcombobox.h"
#include "filtermanager.h"
#include "dbcontent/variable/metavariable.h"
//#include "stringconv.h"
#include "global.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

using namespace Utils;

FilterGeneratorWidget::FilterGeneratorWidget(QWidget* parent) : QWidget(parent)
{
    condition_variable_widget_ = 0;
    setWindowTitle(tr("Add New Filter"));

    setMinimumSize(800, 400);

    createGUIElements();
}

FilterGeneratorWidget::~FilterGeneratorWidget() {}

void FilterGeneratorWidget::createGUIElements()
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
    name_layout->addWidget(filter_name_);
    layout->addLayout(name_layout);

    QLabel* condition_label = new QLabel(tr("Define condition"));
    layout->addWidget(condition_label);

    QGridLayout* condition_layout = new QGridLayout();

    QVBoxLayout* var_layout = new QVBoxLayout();
    condition_variable_widget_ = new dbContent::VariableSelectionWidget();
    condition_variable_widget_->showMetaVariables(true);
    condition_variable_widget_->showEmptyVariable(false);
    var_layout->addWidget(condition_variable_widget_);
    var_layout->addStretch();
    condition_layout->addLayout(var_layout, 0, 0);

    QVBoxLayout* math_layout = new QVBoxLayout();

    condition_absolute_ = new QCheckBox("ABS");

    math_layout->addWidget(condition_absolute_);
    condition_layout->addLayout(math_layout, 0, 1);

    QVBoxLayout* operator_layout = new QVBoxLayout();
    QLabel* label = new QLabel(tr("Operator"));
    label->setFont(font_bold);
    operator_layout->addWidget(label);

    condition_combo_ = new FilterConditionOperatorComboBox();
    operator_layout->addWidget(condition_combo_);
    operator_layout->addStretch();

    condition_layout->addLayout(operator_layout, 0, 2);

    QVBoxLayout* value_layout = new QVBoxLayout();
    QLabel* label2 = new QLabel(tr("Value"));
    label2->setFont(font_bold);
    value_layout->addWidget(label2);

    condition_value_ = new QLineEdit();
    value_layout->addWidget(condition_value_);

//    QPushButton* load_min = new QPushButton(tr("Load min"));
//    connect(load_min, SIGNAL(clicked()), this, SLOT(loadMin()));
//    value_layout->addWidget(load_min);

//    QPushButton* load_max = new QPushButton(tr("Load max"));
//    connect(load_max, SIGNAL(clicked()), this, SLOT(loadMax()));
//    value_layout->addWidget(load_max);

    value_layout->addStretch();

    condition_layout->addLayout(value_layout, 0, 3);

    QVBoxLayout* reset_layout = new QVBoxLayout();
    QLabel* label_reset = new QLabel(tr("Reset value"));
    label_reset->setFont(font_bold);
    reset_layout->addWidget(label_reset);

    condition_reset_combo_ = new QComboBox();
    condition_reset_combo_->addItem("value");
//    condition_reset_combo_->addItem("MIN");
//    condition_reset_combo_->addItem("MAX");
    reset_layout->addWidget(condition_reset_combo_);
    reset_layout->addStretch();

    condition_layout->addLayout(reset_layout, 0, 4);

    layout->addLayout(condition_layout);

    QPushButton* add_condition = new QPushButton(tr("Add condition"));
    connect(add_condition, SIGNAL(clicked()), this, SLOT(addCondition()));
    layout->addWidget(add_condition);
    layout->addStretch();

    QLabel* conditions_label = new QLabel(tr("Current conditions"));
    conditions_label->setFont(font_bold);
    layout->addWidget(conditions_label);

    conditions_list_ = new QListWidget();
    layout->addWidget(conditions_list_);

    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* cancel = new QPushButton(tr("Cancel"));
    connect(cancel, SIGNAL(clicked()), this, SLOT(cancel()));
    button_layout->addWidget(cancel);

    button_layout->addStretch();

    QPushButton* ok = new QPushButton(tr("Add"));
    connect(ok, SIGNAL(clicked()), this, SLOT(accept()));
    button_layout->addWidget(ok);

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

void FilterGeneratorWidget::addCondition()
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
}

void FilterGeneratorWidget::updateWidgetList()
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

void FilterGeneratorWidget::closeEvent(QCloseEvent* event) { emit filterWidgetAction(false); }

void FilterGeneratorWidget::accept()
{
    loginf;

    std::string filter_name = filter_name_->text().toStdString();

    auto configuration = Configuration::create("DBFilter", filter_name);

    for (unsigned int cnt = 0; cnt < data_conditions_.size(); cnt++)
    {
        ConditionTemplate& data_condition = data_conditions_.at(cnt);
        std::string condition_name = filter_name + "Condition" + std::to_string(cnt);

        loginf << "creating condition with operator '"
               << data_condition.operator_ << "'";

        Configuration& condition_configuration = configuration->addNewSubConfiguration("DBFilterCondition", condition_name);
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

    COMPASS::instance().filterManager().generateSubConfigurableFromConfig(std::move(configuration));

    emit filterWidgetAction(true);
}

void FilterGeneratorWidget::cancel() 
{ 
    emit filterWidgetAction(false); 
}
