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

#include "dbcontent/variable/metavariablewidget.h"
#include "compass.h"
//#include "configuration.h"
//#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "logger.h"
#include "dbcontent/variable/metavariable.h"
//#include "stringconv.h"

#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QVBoxLayout>

//using namespace Utils;

namespace dbContent
{

MetaVariableWidget::MetaVariableWidget(MetaVariable& variable, QWidget* parent,
                                             Qt::WindowFlags f)
    : QWidget(parent, f), variable_(variable)
{
    setMinimumSize(QSize(800, 600));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(18);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QLabel* main_label = new QLabel("Edit Meta DB object variable");
    main_label->setFont(font_big);
    main_layout->addWidget(main_label);

    // object parameters
    QGridLayout* properties_layout = new QGridLayout();

    unsigned int row = 0;
    QLabel* name_label = new QLabel("Name");
    properties_layout->addWidget(name_label, row, 0);

    name_edit_ = new QLineEdit(variable_.name().c_str());
    connect(name_edit_, SIGNAL(returnPressed()), this, SLOT(editNameSlot()));
    properties_layout->addWidget(name_edit_, row, 1);
    row++;

    QLabel* description_label = new QLabel("Description");
    properties_layout->addWidget(description_label, row, 0);

    description_edit_ = new QLineEdit(variable_.description().c_str());
    description_edit_->setReadOnly(true);
    properties_layout->addWidget(description_edit_, row, 1);
    row++;

    main_layout->addLayout(properties_layout);

    grid_layout_ = new QGridLayout();
    main_layout->addLayout(grid_layout_);

    main_layout->addStretch();

    setLayout(main_layout);

    updateSlot();

    show();
}

MetaVariableWidget::~MetaVariableWidget() {}

void MetaVariableWidget::lock()
{
    if (locked_)
        return;

    locked_ = true;

    setDisabled(true);
}

void MetaVariableWidget::unlock()
{
    if (!locked_)
        return;

    locked_ = false;

    setDisabled(false);
}

void MetaVariableWidget::editNameSlot()
{
    logdbg;
    traced_assert(name_edit_);

    std::string text = name_edit_->text().toStdString();
    traced_assert(text.size() > 0);
    variable_.name(text);

    emit metaVariableChangedSignal();
}

void MetaVariableWidget::subVariableChangedSlot()
{
    VariableSelectionWidget* var_sel =
        dynamic_cast<VariableSelectionWidget*>(QObject::sender());
    traced_assert(var_sel);
    traced_assert(!var_sel->hasMetaVariable());
    traced_assert(selection_widgets_.count(var_sel) > 0);

    std::string obj_name = selection_widgets_.at(var_sel);

    if (variable_.existsIn(obj_name))
        variable_.removeVariable(obj_name);

    if (var_sel->hasVariable())
    {
        traced_assert(!variable_.existsIn(obj_name));
        Variable& variable = var_sel->selectedVariable();
        variable_.addVariable(obj_name, variable.name());
    }
}

void MetaVariableWidget::updateSlot()
{
    traced_assert(grid_layout_);

    QLayoutItem* child;
    while (!grid_layout_->isEmpty() && (child = grid_layout_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }

    selection_widgets_.clear();

    unsigned int row = 0;
    for (auto& obj_it : COMPASS::instance().dbContentManager())
    {
        grid_layout_->addWidget(new QLabel(obj_it.first.c_str()), row, 0);

        VariableSelectionWidget* var_sel = new VariableSelectionWidget(true);
        var_sel->showDBContentOnly(obj_it.first);

        if (variable_.existsIn(obj_it.first))
            var_sel->selectedVariable(variable_.getFor(obj_it.first));

        connect(var_sel, SIGNAL(selectionChanged()), this, SLOT(subVariableChangedSlot()));
        selection_widgets_[var_sel] = obj_it.first;

        grid_layout_->addWidget(var_sel, row, 1);
        row++;
    }
}

}
