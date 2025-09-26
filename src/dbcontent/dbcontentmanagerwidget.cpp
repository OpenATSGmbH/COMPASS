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

#include "dbcontent/dbcontentmanagerwidget.h"

//#include "compass.h"
#include "configuration.h"
//#include "configurationmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentwidget.h"
#include "dbcontent/variable/variable.h"
#include "files.h"
#include "global.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/variable/metavariablewidget.h"
//#include "stringconv.h"

#include <QComboBox>
#include <QGridLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

using namespace Utils;
using namespace dbContent;

DBContentManagerWidget::DBContentManagerWidget(DBContentManager& object_manager)
    : object_manager_(object_manager)
{
    // unsigned int frame_width = FRAME_SIZE;
    setContentsMargins(0, 0, 0, 0);

    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout = new QVBoxLayout();

    // database objects
    QLabel* main_label = new QLabel("Database Objects");
    main_label->setFont(font_bold);
    main_layout->addWidget(main_label);

    QFrame* dbcont_frame = new QFrame();
    dbcont_frame->setFrameStyle(QFrame::Panel | QFrame::Raised);
    dbcont_frame->setLineWidth(1);

    dbconts_grid_ = new QGridLayout();
    updateDBContentsSlot();

    dbcont_frame->setLayout(dbconts_grid_);

    QScrollArea* dbcont_scroll = new QScrollArea();
    dbcont_scroll->setWidgetResizable(true);
    dbcont_scroll->setWidget(dbcont_frame);

    main_layout->addWidget(dbcont_scroll);

    // new dbcontbject
    add_dbcont_button_ = new QPushButton("Add");
    connect(add_dbcont_button_, SIGNAL(clicked()), this, SLOT(addDBContSlot()));
    main_layout->addWidget(add_dbcont_button_);

    main_layout->addStretch();

    // meta objects
    QLabel* metavars_label = new QLabel("Meta Variables");
    metavars_label->setFont(font_bold);
    main_layout->addWidget(metavars_label);

    QFrame* meta_frame = new QFrame();
    meta_frame->setFrameStyle(QFrame::Panel | QFrame::Raised);
    meta_frame->setLineWidth(1);

    meta_variables_grid_ = new QGridLayout();
    updateMetaVariablesSlot();

    meta_frame->setLayout(meta_variables_grid_);

    QScrollArea* meta_scroll = new QScrollArea();
    meta_scroll->setWidgetResizable(true);
    meta_scroll->setWidget(meta_frame);

    main_layout->addWidget(meta_scroll);

    add_metavar_button_ = new QPushButton("Add");
    connect(add_metavar_button_, SIGNAL(clicked()), this, SLOT(addAllMetaVariablesSlot()));
    main_layout->addWidget(add_metavar_button_);

    setLayout(main_layout);

    // lock();
}

DBContentManagerWidget::~DBContentManagerWidget()
{
    edit_dbcont_buttons_.clear();
    delete_dbcont_buttons_.clear();
    edit_dbcont_widgets_.clear();
}

// void DBContentManagerWidget::lock ()
//{
////    for (auto it : edit_dbcont_buttons_)
////        it.first->setDisabled (true);

//    for (auto it : delete_dbcont_buttons_)
//        it.first->setDisabled (true);

//    add_dbcont_button_->setDisabled (true);

//    for (auto it : delete_meta_buttons_)
//        it.first->setDisabled (true);

//    add_metavar_button_->setDisabled (true);
//}

// void DBContentManagerWidget::unlock ()
//{
////    for (auto it : edit_dbcont_buttons_)
////        it.first->setDisabled (!it.second->hasCurrentMetaTable());

//    for (auto it : delete_dbcont_buttons_)
//        it.first->setDisabled (false);

//    add_dbcont_button_->setDisabled (false);

//    for (auto it : delete_meta_buttons_)
//        it.first->setDisabled (false);

//    add_metavar_button_->setDisabled (false);
//}

// void DBContentManagerWidget::databaseOpenedSlot ()
//{
//    loginf;
//}

void DBContentManagerWidget::addDBContSlot()
{
//    if (!schema_manager_.hasCurrentSchema())
//    {
//        logerr << "no schema was selected";
//        return;
//    }

    bool ok;
    QString text =
        QInputDialog::getText(this, tr("Database Object Name"),
                              tr("Specify a (unique) DBContent name:"), QLineEdit::Normal, "", &ok);
    if (ok && !text.isEmpty())
    {
        std::string name = text.toStdString();

        if (object_manager_.existsDBContent(name))
        {
            logerr << "DBContent same name already exists";
            return;
        }

        std::string instance = "DBContent" + name + "0";

        auto config = Configuration::create("DBContent", instance);
        config->addParameter<std::string>("name", name);
        //config->addParameter<std::string> ("meta_table", meta_table_name); // TODO add db_table_name

        object_manager_.generateSubConfigurableFromConfig(std::move(config));

        updateDBContentsSlot();
    }
}

void DBContentManagerWidget::changedDBContSlot() { updateDBContentsSlot(); }

void DBContentManagerWidget::editDBContSlot()
{
    traced_assert(edit_dbcont_buttons_.find((QPushButton*)sender()) != edit_dbcont_buttons_.end());

    DBContent* object = edit_dbcont_buttons_[(QPushButton*)sender()];

    if (edit_dbcont_widgets_.find(object) == edit_dbcont_widgets_.end())
    {
        DBContentWidget* widget = object->widget();
        connect(widget, SIGNAL(changedDBContSignal()), this, SLOT(changedDBContSlot()));
        edit_dbcont_widgets_[object] = widget;
    }
    else
        edit_dbcont_widgets_[object]->show();
}

void DBContentManagerWidget::deleteDBContSlot()
{
    traced_assert(delete_dbcont_buttons_.find((QPushButton*)sender()) != delete_dbcont_buttons_.end());

    DBContent* object = delete_dbcont_buttons_[(QPushButton*)sender()];
    object_manager_.deleteDBContent(object->name());

    updateDBContentsSlot();
}

void DBContentManagerWidget::updateDBContentsSlot()
{
    traced_assert(dbconts_grid_);

    QLayoutItem* child;
    while (!dbconts_grid_->isEmpty() && (child = dbconts_grid_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }

    QIcon edit_icon(Files::IconProvider::getIcon("edit.png"));
    QIcon del_icon(Files::IconProvider::getIcon("delete.png"));

    edit_dbcont_buttons_.clear();
    delete_dbcont_buttons_.clear();

    QFont font_bold;
    font_bold.setBold(true);

    QLabel* name_label = new QLabel("Name");
    name_label->setFont(font_bold);
    dbconts_grid_->addWidget(name_label, 0, 0);

    QLabel* numel_label = new QLabel("# columns");
    numel_label->setFont(font_bold);
    dbconts_grid_->addWidget(numel_label, 0, 1);

    unsigned int row = 1;

    for (auto& obj_it : object_manager_)
    {
        QLabel* name = new QLabel(obj_it.second->name().c_str());
        //        if (!obj_it.second->existsInDB())
        //        {
        //            QFont font = name->font();
        //            font.setStrikeOut(true);
        //            name->setFont(font);
        //        }
        dbconts_grid_->addWidget(name, row, 0);

        QLabel* numel = new QLabel((std::to_string(obj_it.second->numVariables())).c_str());
        dbconts_grid_->addWidget(numel, row, 1);

        QPushButton* edit = new QPushButton();
        edit->setIcon(edit_icon);
        edit->setIconSize(UI_ICON_SIZE);
        edit->setMaximumWidth(UI_ICON_BUTTON_MAX_WIDTH);
        edit->setFlat(UI_ICON_BUTTON_FLAT);
        // edit->setDisabled(!active || locked_);
        connect(edit, SIGNAL(clicked()), this, SLOT(editDBContSlot()));
        dbconts_grid_->addWidget(edit, row, 3);
        edit_dbcont_buttons_[edit] = obj_it.second;

        QPushButton* del = new QPushButton();
        del->setIcon(del_icon);
        del->setIconSize(UI_ICON_SIZE);
        del->setMaximumWidth(UI_ICON_BUTTON_MAX_WIDTH);
        del->setFlat(UI_ICON_BUTTON_FLAT);
        // del->setDisabled(locked_);
        connect(del, SIGNAL(clicked()), this, SLOT(deleteDBContSlot()));
        dbconts_grid_->addWidget(del, row, 4);
        delete_dbcont_buttons_[del] = obj_it.second;

        row++;
    }
}

void DBContentManagerWidget::addMetaVariableSlot() {}

void DBContentManagerWidget::editMetaVariableSlot()
{
    traced_assert(edit_meta_buttons_.find((QPushButton*)sender()) != edit_meta_buttons_.end());

    MetaVariable* meta_var = edit_meta_buttons_[(QPushButton*)sender()];

    if (edit_meta_widgets_.find(meta_var) == edit_meta_widgets_.end())
    {
        MetaVariableWidget* widget = meta_var->widget();
        connect(widget, SIGNAL(metaVariableChangedSignal()), this, SLOT(updateMetaVariablesSlot()));
        edit_meta_widgets_[meta_var] = widget;
    }
    else
        edit_meta_widgets_[meta_var]->show();
}

void DBContentManagerWidget::deleteMetaVariableSlot()
{
    traced_assert(delete_meta_buttons_.find((QPushButton*)sender()) != delete_meta_buttons_.end());

    MetaVariable* meta_var = delete_meta_buttons_[(QPushButton*)sender()];
    object_manager_.deleteMetaVariable(meta_var->name());

    updateMetaVariablesSlot();
}

void DBContentManagerWidget::addAllMetaVariablesSlot()
{
    std::vector<std::string> found_dbconts;

    bool changed = false;

    for (auto& obj_it : object_manager_)
    {
        for (auto& var_it : obj_it.second->variables())
        {
            if (object_manager_.usedInMetaVariable(*var_it.second.get()))
            {
                loginf << "not adding dbcontvariable "
                       << var_it.first << " since already used";
                continue;
            }

            found_dbconts.clear();
            found_dbconts.push_back(obj_it.first);  // original object

            for (auto& obj_it2 : object_manager_)
            {
                if (obj_it == obj_it2)
                    continue;

                if (obj_it2.second->hasVariable(var_it.first) &&
                    var_it.second->dataType() == obj_it2.second->variable(var_it.first).dataType())
                {
                    found_dbconts.push_back(obj_it2.first);
                }
            }

            if (found_dbconts.size() > 1)
            {
                if (!object_manager_.existsMetaVariable(var_it.first))
                {
                    loginf << "adding meta variable "
                        << var_it.first;

                    std::string instance = "MetaVariable" + var_it.first + "0";

                    auto config = Configuration::create("MetaVariable", instance);
                    config->addParameter<std::string>("name", var_it.first);

                    object_manager_.generateSubConfigurableFromConfig(std::move(config));
                }

                traced_assert(object_manager_.existsMetaVariable(var_it.first));
                MetaVariable& meta_var = object_manager_.metaVariable(var_it.first);

                for (auto dbcont_it2 = found_dbconts.begin(); dbcont_it2 != found_dbconts.end(); dbcont_it2++)
                {
                    if (!meta_var.existsIn(*dbcont_it2))
                    {
                        loginf << "adding meta "
                                  "variable "
                               << var_it.first << " dbcont variable " << var_it.first;
                        meta_var.addVariable(*dbcont_it2, var_it.first);
                    }
                }

                changed = true;
            }
        }
    }

    if (changed)
        updateMetaVariablesSlot();
}

void DBContentManagerWidget::updateMetaVariablesSlot()
{
    traced_assert(meta_variables_grid_);

    QLayoutItem* child;
    while (!meta_variables_grid_->isEmpty() && (child = meta_variables_grid_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }

    QIcon edit_icon(Files::IconProvider::getIcon("edit.png"));
    QIcon del_icon(Files::IconProvider::getIcon("delete.png"));

    edit_meta_buttons_.clear();
    delete_meta_buttons_.clear();

    QFont font_bold;
    font_bold.setBold(true);

    QLabel* name_label = new QLabel("Name");
    name_label->setFont(font_bold);
    meta_variables_grid_->addWidget(name_label, 0, 0);

    QLabel* type_label = new QLabel("Data type");
    type_label->setFont(font_bold);
    meta_variables_grid_->addWidget(type_label, 0, 1);

    unsigned int row = 1;

    for (auto& var_it : object_manager_.metaVariables())
    {
        traced_assert(var_it.second.get());

        QLabel* name = new QLabel(var_it.first.c_str());
        meta_variables_grid_->addWidget(name, row, 0);

        QLabel* datatype = new QLabel();
        if (var_it.second->hasVariables())
            datatype->setText(var_it.second->dataTypeString().c_str());
        meta_variables_grid_->addWidget(datatype, row, 1);

        QPushButton* edit = new QPushButton();
        edit->setIcon(edit_icon);
        edit->setIconSize(UI_ICON_SIZE);
        edit->setMaximumWidth(UI_ICON_BUTTON_MAX_WIDTH);
        edit->setFlat(UI_ICON_BUTTON_FLAT);
        // edit->setDisabled(!active || !unlocked_);
        connect(edit, SIGNAL(clicked()), this, SLOT(editMetaVariableSlot()));
        meta_variables_grid_->addWidget(edit, row, 2);
        edit_meta_buttons_[edit] = var_it.second.get();

        QPushButton* del = new QPushButton();
        del->setIcon(del_icon);
        del->setIconSize(UI_ICON_SIZE);
        del->setMaximumWidth(UI_ICON_BUTTON_MAX_WIDTH);
        del->setFlat(UI_ICON_BUTTON_FLAT);
        // del->setDisabled(!unlocked_);
        connect(del, SIGNAL(clicked()), this, SLOT(deleteMetaVariableSlot()));
        meta_variables_grid_->addWidget(del, row, 3);
        delete_meta_buttons_[del] = var_it.second.get();

        row++;
    }
}
