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

#include "filtereditwidget.h"
//#include "compass.h"
//#include "configurationmanager.h"
#include "dbfilter.h"
#include "dbfiltercondition.h"
#include "dbfilterwidget.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "files.h"
#include "filterconditionoperatorcombobox.h"
#include "filterconditionresetvaluecombobox.h"
//#include "filtermanager.h"
#include "logger.h"
//#include "stringconv.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

using namespace Utils;

FilterEditWidget::FilterEditWidget(DBFilter* filter, QWidget* parent)
    : QWidget(parent), filter_(filter)
{
    condition_variable_widget_ = 0;
    setWindowTitle(tr(("Edit Filter " + filter_->instanceId()).c_str()));
    setAttribute(Qt::WA_DeleteOnClose);

    setMinimumSize(800, 600);

    QFont font_bold;
    font_bold.setBold(true);
    QFont font_big;
    font_big.setPointSize(18);

    QVBoxLayout* layout = new QVBoxLayout();

    QHBoxLayout* name_layout = new QHBoxLayout();
    QLabel* name_label = new QLabel(tr("Filter name"));
    name_layout->addWidget(name_label);

    filter_name_ = new QLineEdit(tr(filter_->getName().c_str()));
    connect(filter_name_, SIGNAL(returnPressed()), this, SLOT(changedName()));
    name_layout->addWidget(filter_name_);
    layout->addLayout(name_layout);

    QLabel* condition_label = new QLabel(tr("Define condition"));
    layout->addWidget(condition_label);

    QGridLayout* condition_layout = new QGridLayout();

    condition_variable_widget_ = new dbContent::VariableSelectionWidget();
    condition_layout->addWidget(condition_variable_widget_, 0, 0);

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

    QPushButton* load_min = new QPushButton(tr("Load min"));
    connect(load_min, SIGNAL(clicked()), this, SLOT(loadMin()));
    value_layout->addWidget(load_min);

    QPushButton* load_max = new QPushButton(tr("Load max"));
    connect(load_max, SIGNAL(clicked()), this, SLOT(loadMax()));
    value_layout->addWidget(load_max);

    value_layout->addStretch();

    condition_layout->addLayout(value_layout, 0, 3);

    QVBoxLayout* reset_layout = new QVBoxLayout();
    QLabel* label_reset = new QLabel(tr("Reset value"));
    label_reset->setFont(font_bold);
    reset_layout->addWidget(label_reset);

    condition_reset_combo_ = new FilterConditionResetValueComboBox();
    //  condition_reset_combo_->addItem("MIN");
    //  condition_reset_combo_->addItem("MAX");
    //  condition_reset_combo_->addItem("value");
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

    //  conditions_list_ = new QListWidget();
    //  layout->addWidget(conditions_list_);

    conditions_grid_ = new QGridLayout();
    updateConditionsGrid();
    layout->addLayout(conditions_grid_);

    QHBoxLayout* button_layout = new QHBoxLayout();

    //  QPushButton *cancel = new QPushButton(tr("Cancel"));
    //  connect(cancel, SIGNAL( clicked() ), this, SLOT( reject() ));
    //  button_layout->addWidget(cancel);

    button_layout->addStretch();

    QPushButton* ok = new QPushButton(tr("Close"));
    connect(ok, SIGNAL(clicked()), this, SLOT(close()));
    button_layout->addWidget(ok);

    layout->addLayout(button_layout);
    setLayout(layout);
}

FilterEditWidget::~FilterEditWidget() 
{
     logdbg; 
}

void FilterEditWidget::addCondition()
{
    traced_assert(filter_);
    traced_assert(filter_name_);
    traced_assert(condition_variable_widget_);
    traced_assert(condition_combo_);
    traced_assert(condition_absolute_);
    traced_assert(condition_value_);
    traced_assert(condition_reset_combo_);

    std::string filter_name = filter_name_->text().toStdString();
    std::string operator_str = condition_combo_->currentText().toStdString();
    std::string absolute_value = condition_absolute_->text().toStdString();
    std::string value = condition_value_->text().toStdString();
    std::string reset_value_str = condition_reset_combo_->currentText().toStdString();

    std::string condition_name =
        filter_name + condition_variable_widget_->selectedVariable().name() + "Condition" +
        std::to_string(filter_->getNumConditions());  // TODO not the best way

    auto condition_configuration = Configuration::create("DBFilterCondition", condition_name);

    condition_configuration->addParameter<std::string>("operator", operator_str);
    condition_configuration->addParameter<std::string>("variable_name", condition_variable_widget_->selectedVariable().name());
    condition_configuration->addParameter<std::string>("variable_dbcontent_name", condition_variable_widget_->selectedVariable().dbContentName());
    condition_configuration->addParameter<bool>("absolute_value", condition_absolute_->checkState() == Qt::Checked);
    condition_configuration->addParameter<std::string>("value", value);
    
    std::string reset_value;
    if (reset_value_str.compare("MIN") == 0 || reset_value_str.compare("MAX") == 0)
        reset_value = reset_value_str;
    else
        reset_value = value;

    condition_configuration->addParameter<std::string>("reset_value", reset_value);

    filter_->generateSubConfigurableFromConfig(std::move(condition_configuration));

    updateConditionsGrid();
}

void FilterEditWidget::deleteCondition()
{
    QPushButton* button = (QPushButton*)sender();
    traced_assert(conditions_delete_buttons_.find(button) != conditions_delete_buttons_.end());
    traced_assert(filter_);
    filter_->deleteCondition(conditions_delete_buttons_[button]);
    filter_->widget()->updateChildWidget();

    updateConditionsGrid();
}

void FilterEditWidget::updateConditionsGrid()
{
    traced_assert(filter_);
    traced_assert(conditions_grid_);

    QLayoutItem* child;
    while (!conditions_grid_->isEmpty() && (child = conditions_grid_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }
    conditions_delete_buttons_.clear();
    conditions_variable_selects_.clear();
    conditions_abs_checkboxes_.clear();
    conditions_operator_combos_.clear();

    QFont font_bold;
    font_bold.setBold(true);

    QLabel* name_label = new QLabel("Variable");
    name_label->setFont(font_bold);
    conditions_grid_->addWidget(name_label, 0, 1);

    QLabel* abs_label = new QLabel("ABS");
    abs_label->setFont(font_bold);
    conditions_grid_->addWidget(abs_label, 0, 2);

    QLabel* operator_label = new QLabel("Operator");
    operator_label->setFont(font_bold);
    conditions_grid_->addWidget(operator_label, 0, 3);

    QLabel* reset_label = new QLabel("Reset value");
    reset_label->setFont(font_bold);
    conditions_grid_->addWidget(reset_label, 0, 4);

    //  QLabel *unit_label = new QLabel ("Unit");
    //  unit_label->setFont (font_bold);
    //  dbcontvars_grid_->addWidget (unit_label, 0, 5);

    std::vector<DBFilterCondition*>& conditions = filter_->getConditions();

    // std::map <std::string, DBSchema *> &schemas  = DBSchemaManager::getInstance().getSchemas ();
    std::vector<DBFilterCondition*>::iterator it;

    unsigned int row = 1;
    for (it = conditions.begin(); it != conditions.end(); it++)
    {
        QPushButton* del = new QPushButton();
        del->setIcon(Files::IconProvider::getIcon("delete.png"));
        del->setFixedSize(20, 20);
        del->setFlat(true);
        connect(del, SIGNAL(clicked()), this, SLOT(deleteCondition()));
        conditions_delete_buttons_[del] = (*it);
        conditions_grid_->addWidget(del, row, 0);

        TODO_ASSERT

//        DBContVariableSelectionWidget* var_select = new DBContVariableSelectionWidget(true);
//        var_select->selectedVariable(*(*it)->getVariable());
//        conditions_grid_->addWidget(var_select, row, 1);
//        connect(var_select, SIGNAL(selectionChanged()), this, SLOT(changedConditionVariable()));
//        conditions_variable_selects_[var_select] = (*it);

        QCheckBox* absolute = new QCheckBox();
        absolute->setChecked((*it)->getAbsoluteValue());
        connect(absolute, SIGNAL(stateChanged(int)), this, SLOT(changedABS()));
        conditions_abs_checkboxes_[absolute] = (*it);
        conditions_grid_->addWidget(absolute, row, 2);

        FilterConditionOperatorComboBox* operatorbox = new FilterConditionOperatorComboBox();
        conditions_operator_combos_[operatorbox] = (*it);
        int index = operatorbox->findText(tr((*it)->getOperator().c_str()));
        if (index != -1)
            operatorbox->setCurrentIndex(index);
        connect(operatorbox, SIGNAL(currentIndexChanged(int)), this, SLOT(changedOperator()));
        conditions_grid_->addWidget(operatorbox, row, 3);

        FilterConditionResetValueComboBox* resetbox = new FilterConditionResetValueComboBox();
        conditions_reset_value_combos_[resetbox] = (*it);
        index = resetbox->findText(tr((*it)->getResetValue().c_str()));
        if (index != -1)
            resetbox->setCurrentIndex(index);
        connect(resetbox, SIGNAL(currentIndexChanged(int)), this, SLOT(changedResetValue()));
        conditions_grid_->addWidget(resetbox, row, 4);

        //    PropertyDataTypeComboBox *type = new PropertyDataTypeComboBox (it->second);
        //    dbcont_vars_grid_data_type_boxes_[type] = it->second;
        //    dbcontvars_grid_->addWidget (type, row, 3);
        //
        //    StringRepresentationComboBox *repr = new StringRepresentationComboBox (it->second);
        //    dbcont_vars_grid_representation_boxes_[repr] = it->second;
        //    dbcontvars_grid_->addWidget (repr, row, 4);
        //
        //    UnitSelectionWidget *unit_sel = new UnitSelectionWidget
        //    (it->second->getUnitDimension(), it->second->getUnitUnit()); dbcontvars_grid_->addWidget
        //    (unit_sel, row, 5);
        //
        //    unsigned col=6;
        //    for (sit = schemas.begin(); sit != schemas.end(); sit++)
        //    {
        //      if (metas.find (sit->second->getName()) == metas.end())
        //        continue;
        //
        //      DBTableColumnComboBox *box = new DBTableColumnComboBox (sit->second->getName(),
        //      metas[sit->second->getName()], it->second); dbcontvars_grid_->addWidget (box, row,
        //      col); col++;
        //    }
        row++;
    }
}

void FilterEditWidget::changedName()
{
    traced_assert(filter_name_);
    filter_->setName(filter_name_->text().toStdString());
}

void FilterEditWidget::changedConditionVariable()
{
//    DBContVariableSelectionWidget* source = (DBContVariableSelectionWidget*)sender();
//    traced_assert(conditions_variable_selects_.find(source) != conditions_variable_selects_.end());
//    conditions_variable_selects_[source]->setVariable(&source->selectedVariable());

    TODO_ASSERT
}

void FilterEditWidget::changedABS()
{
    QCheckBox* source = (QCheckBox*)sender();
    traced_assert(conditions_abs_checkboxes_.find(source) != conditions_abs_checkboxes_.end());
    conditions_abs_checkboxes_[source]->setAbsoluteValue(source->checkState() == Qt::Checked);
}

void FilterEditWidget::changedOperator()
{
    FilterConditionOperatorComboBox* source = (FilterConditionOperatorComboBox*)sender();
    traced_assert(conditions_operator_combos_.find(source) != conditions_operator_combos_.end());
    conditions_operator_combos_[source]->setOperator(source->currentText().toStdString());
}

void FilterEditWidget::changedResetValue()
{
    FilterConditionResetValueComboBox* source = (FilterConditionResetValueComboBox*)sender();
    traced_assert(conditions_reset_value_combos_.find(source) != conditions_reset_value_combos_.end());
    conditions_reset_value_combos_[source]->setResetValue(source->currentText().toStdString());
}
