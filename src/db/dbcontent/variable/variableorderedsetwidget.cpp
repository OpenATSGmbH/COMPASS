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

#include "dbcontent/variable/variableorderedsetwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/variable/variableselectiondialog.h"
#include "files.h"
#include "global.h"
#include "rtcommand.h"
#include "traced_assert.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

using namespace Utils;
using namespace std;

namespace dbContent
{

const std::string VariableOrderedSetWidget::VariableSeparator = ", ";

VariableOrderedSetWidget::VariableOrderedSetWidget(VariableOrderedSet& set,
                                                   QWidget* parent, 
                                                   Qt::WindowFlags f)
:   QWidget(parent, f)
,   set_   (set)
{
    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->setContentsMargins(0, 0, 0, 0);

    QFont font_bold;
    font_bold.setBold(true);

    QHBoxLayout* hvars_layout = new QHBoxLayout();
    QVBoxLayout* vvars_layout = new QVBoxLayout();

    QLabel* var_label = new QLabel("Variables");
    var_label->setFont(font_bold);
    main_layout->addWidget(var_label);

    list_widget_ = new QListWidget();
    updateVariableListSlot();
    vvars_layout->addWidget(list_widget_);
    hvars_layout->addLayout(vvars_layout);

    // up/down buttons
    {
        QVBoxLayout* vupdown_layout = new QVBoxLayout();

        QPushButton* up = new QPushButton();
        up->setIcon(Files::IconProvider::getIcon("up.png"));
        up->setFixedSize(UI_ICON_SIZE);
        up->setFlat(UI_ICON_BUTTON_FLAT);
        up->setToolTip(tr("Move variable up"));
        connect(up, &QPushButton::clicked, this, &VariableOrderedSetWidget::moveUpSlot);

        vupdown_layout->addWidget(up);

        vupdown_layout->addStretch();

        QPushButton* down = new QPushButton();
        down->setIcon(Files::IconProvider::getIcon("down.png"));
        down->setFixedSize(UI_ICON_SIZE);
        down->setFlat(UI_ICON_BUTTON_FLAT);
        down->setToolTip(tr("Move variable down"));
        connect(down, &QPushButton::clicked, this, &VariableOrderedSetWidget::moveDownSlot);

        vupdown_layout->addWidget(down);

        hvars_layout->addLayout(vupdown_layout);
    }

    main_layout->addLayout(hvars_layout);

    // buttons
    {
        QHBoxLayout* button_layout = new QHBoxLayout();

        QPushButton* remove_button = new QPushButton();
        remove_button->setText("Remove");
        connect(remove_button, &QPushButton::clicked, this,
                &VariableOrderedSetWidget::removeSlot);
        button_layout->addWidget(remove_button);

        QPushButton* add_button = new QPushButton();
        add_button->setText("Add");
        connect(add_button, &QPushButton::clicked, this,
                &VariableOrderedSetWidget::showDialogSlot);
        button_layout->addWidget(add_button);

        main_layout->addLayout(button_layout);
    }

    setLayout(main_layout);

    connect(&set, SIGNAL(setChangedSignal()), this, SLOT(updateVariableListSlot()));
}

VariableOrderedSetWidget::~VariableOrderedSetWidget() {}

void VariableOrderedSetWidget::showDialogSlot()
{
    DBContentManager& manager = set_.dbContentManager();

    VariableSelectionDialog::Settings dialog_settings;

    dialog_settings.show_meta_variables = true;
    dialog_settings.multi_select = true;

    VariableSelectionDialog dialog(manager, dialog_settings, this);

    if (dialog.exec() != QDialog::Accepted)
        return;

    for (const auto& sel_it : dialog.selection())
    {
        const std::string& obj_name = sel_it.first;
        const std::string& var_name = sel_it.second;

        if (obj_name == META_OBJECT_NAME)
        {
            traced_assert(manager.existsMetaVariable(var_name));
            set_.add(manager.metaVariable(var_name));
        }
        else
        {
            traced_assert(manager.existsDBContent(obj_name));
            traced_assert(manager.dbContent(obj_name).hasVariable(var_name));
            set_.add(manager.dbContent(obj_name).variable(var_name));
        }
    }
}

void VariableOrderedSetWidget::removeSlot()
{
    traced_assert(list_widget_);
    int index = list_widget_->currentRow();

    loginf << "index " << index;

    if (index < 0)
        return;

    set_.removeVariableAt(index);
    current_index_ = -1;
}

void VariableOrderedSetWidget::moveUpSlot()
{
    traced_assert(list_widget_);
    int index = list_widget_->currentRow();
    loginf << "index " << index;

    if (index <= 0)
        return;

    set_.moveVariableUp(index);

    current_index_ = index - 1;
    list_widget_->setCurrentRow(current_index_);
}

void VariableOrderedSetWidget::moveDownSlot()
{
    traced_assert(list_widget_);
    int index = list_widget_->currentRow();
    loginf << "index " << index;

    if (index < 0 || index == (int)set_.getSize() - 1)
        return;

    set_.moveVariableDown(index);

    current_index_ = index + 1;
    list_widget_->setCurrentRow(current_index_);
}

void VariableOrderedSetWidget::updateVariableListSlot()
{
    logdbg;

    traced_assert(list_widget_);

    list_widget_->clear();

    logdbg << "clear done";

    DBContentManager& manager = set_.dbContentManager();

    string tooltip;

    for (const auto& def_it: set_.definitions())
    {
        if (def_it.first == META_OBJECT_NAME)
        {
            traced_assert(manager.existsMetaVariable(def_it.second));
            tooltip = manager.metaVariable(def_it.second).info();
        }
        else
        {
            traced_assert(manager.existsDBContent(def_it.first));
            traced_assert(manager.dbContent(def_it.first).hasVariable(def_it.second));
            tooltip = manager.dbContent(def_it.first).variable(def_it.second).info();
        }

        QListWidgetItem* item = new QListWidgetItem((def_it.first + VariableSeparator + def_it.second).c_str());
        item->setToolTip(tooltip.c_str());

        list_widget_->addItem(item);
    }

    if (current_index_ != -1)
    {
        logdbg << "current index "
               << current_index_;
        list_widget_->setCurrentRow(current_index_);
        current_index_ = -1;
    }
}

boost::optional<QString> VariableOrderedSetWidget::uiGet(const QString& what) const
{   
    QStringList vars;

    for (int i = 0; i < list_widget_->count(); ++i)
        vars << list_widget_->item(i)->text();
    
    return vars.join(rtcommand::RTCommand::ParameterListSeparator);
}

nlohmann::json VariableOrderedSetWidget::uiGetJSON(const QString& what) const
{
    std::vector<std::string> vars;
    for (int i = 0; i < list_widget_->count(); ++i)
        vars.push_back(list_widget_->item(i)->text().toStdString());

    nlohmann::json ui_info = vars;

    return ui_info;
}

bool VariableOrderedSetWidget::uiSet(const QString& str)
{
    std::vector<std::pair<std::string,std::string>> vars;

    QStringList var_strings = str.split(rtcommand::RTCommand::ParameterListSeparator);

    QString sep  = QString::fromStdString(VariableSeparator);
    int     nsep = sep.count();

    for (const auto& vs : var_strings)
    {
        if (vs.count() < nsep + 2)
            continue;

        int idx = vs.indexOf(sep);
        if (idx < 1 || idx >= vs.count() - nsep)
            return false;

        QString dbcont   = vs.mid(0, idx);
        QString param = vs.mid(idx + nsep);

        if (dbcont.isEmpty() || param.isEmpty())
            return false;

        vars.emplace_back(dbcont.toStdString(), param.toStdString());
    }

    set_.set(vars);

    return true;
}

}

