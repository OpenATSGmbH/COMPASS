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

#include "viewmanagerwidget.h"
//#include "global.h"
#include "jobmanager.h"
#include "logger.h"
//#include "viewcontainer.h"
//#include "viewcontainerwidget.h"
#include "viewmanager.h"


#include <QAction>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QVBoxLayout>
#include <QVariant>


ViewManagerWidget::ViewManagerWidget(ViewManager& view_manager)
    : view_manager_(view_manager), layout_(nullptr), cont_layout_(nullptr), add_button_(nullptr)
{
    logdbg << "start";

    QFont font_bold;
    font_bold.setBold(true);

    layout_ = new QVBoxLayout();

    QLabel* head = new QLabel(tr("Views"));
    head->setFont(font_bold);
    layout_->addWidget(head);

    cont_layout_ = new QVBoxLayout();
    cont_layout_->setSpacing(0);
    cont_layout_->setMargin(0);
    layout_->addLayout(cont_layout_);

    layout_->addStretch();

    add_button_ = new QPushButton(tr("Add View"));
    connect(add_button_, SIGNAL(clicked()), this, SLOT(addViewMenuSlot()));
    layout_->addWidget(add_button_);

    setLayout(layout_);

    connect(&JobManager::instance(), SIGNAL(databaseBusy()), this, SLOT(databaseBusy()));
    connect(&JobManager::instance(), SIGNAL(databaseIdle()), this, SLOT(databaseIdle()));

    update();

    logdbg << "end";
}

ViewManagerWidget::~ViewManagerWidget() {}

void ViewManagerWidget::databaseBusy()
{
    traced_assert(add_button_);
    add_button_->setDisabled(true);
}

void ViewManagerWidget::databaseIdle()
{
    traced_assert(add_button_);
    add_button_->setDisabled(false);
}

//void ViewManagerWidget::addViewMenuSlot()
//{
//    add_template_actions_.clear();

//    QMenu menu;
//    QMenu* submenu;
//    QString name;
//    unsigned int i, n = cont_widgets_.size();

//    for (QString view_class : view_class_list_)
//    {
//        submenu = menu.addMenu(view_class);

//        QAction* new_window_action =
//            submenu->addAction("New Window", this, SLOT(addViewNewWindowSlot()));
//        new_window_action->setData(view_class);

//        for (i = 0; i < n; ++i)
//        {
//            name = cont_widgets_[i]->name();
//            QAction* action = submenu->addAction(name, this, SLOT(addViewSlot()));
//            QStringList list;
//            list.append(view_class);
//            list.append(QString::number(i));
//            action->setData(list);
//        }
//    }

    //  std::map<std::string, Configuration> &templates =
    //  ViewManager::getInstance().getConfiguration()
    //          .getConfigurationTemplates ();

    //  if (templates.size() > 0)
    //  {
    //      QMenu *templatesubmenu = menu.addMenu ("Templates");

    //      std::map<std::string, Configuration>::const_iterator it;
    //      for (it = templates.begin(); it != templates.end(); it++)
    //      {
    //          submenu = templatesubmenu->addMenu( it->first.c_str());

    //          QAction* newaction = submenu->addAction( "New Window", this,
    //          SLOT(addTemplateNewWindowSlot()) ); newaction->setData( QVariant(
    //          tr(it->first.c_str()) ) );

    //          for( i=0; i<n; ++i )
    //          {
    //            name = cont_widgets_[ i ]->name();
    //            QAction* action = submenu->addAction( name, this, SLOT(addTemplateSlot()) );

    //            traced_assert(add_template_actions_.find (action) == add_template_actions_.end());
    //            add_template_actions_ [action] = std::pair <std::string, int> (it->first.c_str(),
    //            i);
    //          }
    //      }
    //  }

//    menu.exec(QCursor::pos());
//}

//void ViewManagerWidget::addViewNewWindowSlot()
//{
//    QAction* action = dynamic_cast<QAction*>(QObject::sender());
//    traced_assert(action);
//    QString class_name = action->data().toString();
//    ViewContainerWidget* container_widget = view_manager_.addNewContainerWidget();
//    container_widget->viewContainer().addView(class_name.toStdString());

//    update();
//}

//void ViewManagerWidget::addViewSlot()
//{
//    QAction* action = dynamic_cast<QAction*>(QObject::sender());
//    traced_assert(action);
//    QStringList list = action->data().toStringList();
//    traced_assert(list.size() == 2);
//    QString class_name = list.at(0);
//    QString number_str = list.at(1);
//    bool ok;
//    unsigned int containter_id = number_str.toUInt(&ok);
//    traced_assert(ok);

//    loginf << "class " << class_name.toStdString();

//    if (containter_id < 0 || containter_id >= cont_widgets_.size())
//        throw(std::runtime_error("ViewManagerWidget: addViewSlot: container out of bounds"));
//    cont_widgets_[containter_id]->addView(class_name.toStdString());
//}

void ViewManagerWidget::update()
{
    loginf;

//    cont_widgets_.clear();

//    QLayoutItem* child;
//    while ((child = cont_layout_->takeAt(0)) != nullptr)
//    {
//        cont_layout_->removeItem(child);
//    }

//    std::map<std::string, ViewContainer*> containers = view_manager_.getContainers();

//    // loginf  << "update size containers " << containers.size();

//    std::map<std::string, ViewContainer*>::iterator it;
//    for (it = containers.begin(); it != containers.end(); it++)
//    {
//        cont_widgets_.push_back(it->second->configWidget());
//        cont_layout_->addWidget(it->second->configWidget());
//    }
}

// void ViewManagerWidget::addTemplateSlot ()
//{
//    QAction *action = (QAction*) sender();

//    traced_assert(add_template_actions_.find (action) != add_template_actions_.end());
//    std::pair <std::string, int> data = add_template_actions_ [action];

//    loginf << "start" << data.first << " in window " <<
//    data.second; int containter_id = data.second;

//    if( containter_id < 0 || containter_id >= cont_widgets_.size() )
//      throw( std::runtime_error( "ViewManagerWidget: addTemplateSlot: container out of bounds" )
//      );

//    cont_widgets_[ containter_id ]->addTemplateView (data.first);

//}

// void ViewManagerWidget::addTemplateNewWindowSlot ()
//{
//    QAction *action = (QAction*) sender();
//    QVariant variant = action->data();
//    loginf << "start" << variant.toString().toStdString();
//    ViewManager::getInstance().addContainerWithTemplateView(variant.toString().toStdString());
//}
