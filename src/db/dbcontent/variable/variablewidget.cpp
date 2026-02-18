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

#include "dbcontent/variable/variablewidget.h"
//#include "compass.h"
//#include "configuration.h"
//#include "configurationmanager.h"
//#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variabledatatypecombobox.h"
#include "logger.h"
//#include "stringconv.h"
#include "stringrepresentationcombobox.h"
#include "unitselectionwidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QTextEdit>
#include <QVBoxLayout>

//using namespace Utils;

namespace dbContent
{

VariableWidget::VariableWidget(Variable& variable, QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f), variable_(&variable)
{
    setMinimumSize(QSize(800, 600));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(18);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QLabel* main_label = new QLabel("Edit DB object variable");
    main_label->setFont(font_big);
    main_layout->addWidget(main_label);

    // object parameters
    properties_layout_ = new QGridLayout();

    unsigned int row = 0;
    QLabel* name_label = new QLabel("Name");
    properties_layout_->addWidget(name_label, row, 0);

    name_edit_ = new QLineEdit(variable_->name().c_str());
    connect(name_edit_, SIGNAL(returnPressed()), this, SLOT(editNameSlot()));
    properties_layout_->addWidget(name_edit_, row, 1);
    row++;

    QLabel* description_label = new QLabel("Description");
    properties_layout_->addWidget(description_label, row, 0);

    description_edit_ = new QLineEdit(variable_->description().c_str());
    connect(description_edit_, SIGNAL(returnPressed()), this, SLOT(editDescriptionSlot()));
    properties_layout_->addWidget(description_edit_, row, 1);
    row++;

    QLabel* type_label = new QLabel("Data Type");
    properties_layout_->addWidget(type_label, row, 0);

    type_combo_ = new VariableDataTypeComboBox(variable_->dataTypeRef(), variable_->dataTypeStringRef());
    properties_layout_->addWidget(type_combo_, row, 1);
    row++;

    properties_layout_->addWidget(new QLabel("Representation"), row, 0);

    representation_box_ = new StringRepresentationComboBox(variable_->representationRef(),
                                                           variable_->representationStringRef());
    properties_layout_->addWidget(representation_box_, row, 1);
    row++;

    QLabel* unit_label = new QLabel("Unit");
    properties_layout_->addWidget(unit_label, row, 0);

    unit_sel_ = new UnitSelectionWidget(variable_->dimension(), variable_->unit());
    properties_layout_->addWidget(unit_sel_, row, 1);
    row++;

    main_layout->addLayout(properties_layout_);
    main_layout->addStretch();

    setLayout(main_layout);

    show();
}

VariableWidget::~VariableWidget() {}

void VariableWidget::lock()
{
    if (locked_)
        return;

    name_edit_->setDisabled(true);
    description_edit_->setDisabled(true);
    type_combo_->setDisabled(true);
    representation_box_->setDisabled(true);
    unit_sel_->setDisabled(true);

    locked_ = true;
}

void VariableWidget::unlock()
{
    if (!locked_)
        return;

    name_edit_->setDisabled(false);
    description_edit_->setDisabled(false);
    type_combo_->setDisabled(false);
    representation_box_->setDisabled(false);
    unit_sel_->setDisabled(false);

    locked_ = false;
}

void VariableWidget::setVariable(Variable& variable)
{
    variable_ = &variable;

    update();
}

void VariableWidget::update()
{
    name_edit_->setText(variable_->name().c_str());
    description_edit_->setText(variable_->description().c_str());
    type_combo_->setType(variable_->dataTypeRef(), variable_->dataTypeStringRef());
    representation_box_->setRepresentation(variable_->representationRef(),
                                     variable_->representationStringRef());
    unit_sel_->update(variable_->dimension(), variable_->unit());
}

void VariableWidget::editNameSlot()
{
    logdbg;
    traced_assert(name_edit_);

    std::string text = name_edit_->text().toStdString();
    traced_assert(text.size() > 0);
    variable_->name(text);
    emit dbcontVariableChangedSignal();
}
void VariableWidget::editDescriptionSlot()
{
    logdbg;
    traced_assert(description_edit_);

    std::string text = description_edit_->text().toStdString();
    traced_assert(text.size() > 0);
    variable_->description(text);
    emit dbcontVariableChangedSignal();
}

//void VariableWidget::editDataTypeSlot()
//{
//    logdbg;
//    traced_assert(type_combo_);
//    variable_->dataType(type_combo_->getType());
//    emit dbcontVariableChangedSignal();
//}

}
