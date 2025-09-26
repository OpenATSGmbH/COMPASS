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

#include "viewpointstoolwidget.h"
#include "viewpointswidget.h"

#include "files.h"
#include "logger.h"
#include "traced_assert.h"

#include <QApplication>
#include <QColorDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QToolBar>
#include <QToolButton>


using namespace Utils;

ViewPointsToolWidget::ViewPointsToolWidget(ViewPointsWidget* vp_widget, QWidget* parent)
: QWidget(parent), vp_widget_(vp_widget)
{
    traced_assert(vp_widget_);

    setMaximumHeight(40);

    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    toolbar_ = new QToolBar("Tools");

    // previous
    {
        toolbar_->addAction(Files::IconProvider::getIcon("up.png"), "Select Previous [Up]");
    }

    toolbar_->addSeparator();

    // selected

    {
        toolbar_->addAction(Files::IconProvider::getIcon("not_recommended.png"), "Set Selected Status Open [O]");

        toolbar_->addAction(Files::IconProvider::getIcon("not_todo.png"), "Set Selected Status Closed [C]");

        toolbar_->addAction(Files::IconProvider::getIcon("todo.png"), "Set Selected Status ToDo [T]");

    }

    toolbar_->addSeparator();

    // comment

    {
        toolbar_->addAction(Files::IconProvider::getIcon("comment.png"), "Edit Comment [E]");
    }

    toolbar_->addSeparator();

     // next
     {
         toolbar_->addAction(Files::IconProvider::getIcon("down.png"), "Select Next [Down]");
     }

    QWidget* empty = new QWidget();
    empty->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    toolbar_->addWidget(empty);

    // columns
    {
        toolbar_->addAction("Edit Columns");
    }

    QWidget* empty2 = new QWidget();
    empty2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    toolbar_->addWidget(empty2);

    // filters
    {
        toolbar_->addAction("Filter By Type");
        toolbar_->addAction("Filter By Status");
    }

     connect(toolbar_, &QToolBar::actionTriggered, this, &ViewPointsToolWidget::actionTriggeredSlot);

     layout->addWidget(toolbar_);
}

//void ViewPointsToolWidget::toolChangedSignal(ViewPointsTool selected, QCursor cursor)
//{

//}

void ViewPointsToolWidget::actionTriggeredSlot(QAction* action)
{
    std::string text = action->text().toStdString();

    if (text == "Select Previous [Up]")
    {
        vp_widget_->selectPreviousSlot();
    }
    else if (text == "Set Selected Status Open [O]")
    {
        vp_widget_->setSelectedOpenSlot();
    }
    else if (text == "Set Selected Status Closed [C]")
    {
        vp_widget_->setSelectedClosedSlot();
    }
    else if (text == "Set Selected Status ToDo [T]")
    {
        vp_widget_->setSelectedTodoSlot();
    }
    else if (text == "Edit Comment [E]")
    {
        vp_widget_->editCommentSlot();
    }
    else if (text == "Select Next [Down]")
    {
        vp_widget_->selectNextSlot();
    }
    else if (text == "Edit Columns")
    {
        showColumnsMenu();
    }
    else if (text == "Filter By Type")
    {
        showTypesMenu();
    }
    else if (text == "Filter By Status")
    {
        showStatusesMenu();
    }
    else
        logwrn << "unkown action '" << text << "'";
}

void ViewPointsToolWidget::showTypesMenu ()
{
    QMenu menu;

    QStringList types = vp_widget_->types();
    QStringList filtered_types = vp_widget_->filteredTypes();

    for (auto& type : types)
    {
        QAction* action = new QAction(type, this);
        action->setCheckable(true);
        action->setChecked(!filtered_types.contains(type));
        connect (action, &QAction::triggered, this, &ViewPointsToolWidget::typeFilteredSlot);

        menu.addAction(action);
    }

    menu.addSeparator();

    QAction* all_action = new QAction("Show All", this);
    connect (all_action, &QAction::triggered, this, &ViewPointsToolWidget::typeFilteredSlot);
    menu.addAction(all_action);

    QAction* none_action = new QAction("Show None", this);
    connect (none_action, &QAction::triggered, this, &ViewPointsToolWidget::typeFilteredSlot);
    menu.addAction(none_action);

    menu.exec(QCursor::pos());
}

void ViewPointsToolWidget::showStatusesMenu ()
{
    QMenu menu;

    QStringList statuses = vp_widget_->statuses();
    QStringList filtered_statuses = vp_widget_->filteredStatuses();

    for (auto& status : statuses)
    {
        QAction* action = new QAction(status, this);
        action->setCheckable(true);
        action->setChecked(!filtered_statuses.contains(status));
        connect (action, &QAction::triggered, this, &ViewPointsToolWidget::statusFilteredSlot);

        menu.addAction(action);
    }

    menu.addSeparator();

    QAction* all_action = new QAction("Show All", this);
    connect (all_action, &QAction::triggered, this, &ViewPointsToolWidget::statusFilteredSlot);
    menu.addAction(all_action);

    QAction* none_action = new QAction("Show None", this);
    connect (none_action, &QAction::triggered, this, &ViewPointsToolWidget::statusFilteredSlot);
    menu.addAction(none_action);

    menu.exec(QCursor::pos());
}

void ViewPointsToolWidget::typeFilteredSlot ()
{
    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    loginf << "start" << action->text().toStdString();

    QString type = action->text();

    if (type == "Show All")
        vp_widget_->showAllTypes();
    else if (type == "Show None")
        vp_widget_->showNoTypes();
    else
        vp_widget_->filterType(type);
}

void ViewPointsToolWidget::statusFilteredSlot ()
{
    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    loginf << "start" << action->text().toStdString();

    QString status = action->text();

    if (status == "Show All")
        vp_widget_->showAllStatuses();
    else if (status == "Show None")
        vp_widget_->showNoStatuses();
    else
        vp_widget_->filterStatus(status);
}

void ViewPointsToolWidget::showColumnsMenu()
{

    QMenu menu;

    QStringList columns = vp_widget_->columns();
    QStringList filtered_columns = vp_widget_->filteredColumns();

    for (auto& col : columns)
    {
        QAction* action = new QAction(col, this);
        action->setCheckable(true);
        action->setChecked(!filtered_columns.contains(col));
        connect (action, &QAction::triggered, this, &ViewPointsToolWidget::columnFilteredSlot);

        menu.addAction(action);
    }

    menu.addSeparator();

    QAction* main_action = new QAction("Show Only Main", this);
    connect (main_action, &QAction::triggered, this, &ViewPointsToolWidget::columnFilteredSlot);
    menu.addAction(main_action);

    QAction* all_action = new QAction("Show All", this);
    connect (all_action, &QAction::triggered, this, &ViewPointsToolWidget::columnFilteredSlot);
    menu.addAction(all_action);

    QAction* none_action = new QAction("Show None", this);
    connect (none_action, &QAction::triggered, this, &ViewPointsToolWidget::columnFilteredSlot);
    menu.addAction(none_action);

    menu.exec(QCursor::pos());
}

void ViewPointsToolWidget::columnFilteredSlot ()
{
    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    loginf << "start" << action->text().toStdString();

    QString col = action->text();

    if (col == "Show Only Main")
        vp_widget_->showOnlyMainColumns();
    else if (col == "Show All")
        vp_widget_->showAllColumns();
    else if (col == "Show None")
        vp_widget_->showNoColumns();
    else
        vp_widget_->filterColumn(col);
}
