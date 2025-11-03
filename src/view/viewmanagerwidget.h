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

#pragma once

#include <QFrame>
#include <QStringList>
#include <map>

class ViewManager;
class ViewContainerConfigWidget;
class QVBoxLayout;
class QPushButton;
class QAction;

class ViewManagerWidget : public QFrame
{
    Q_OBJECT

  private slots:
    void databaseBusy();
    void databaseIdle();

    //void addViewMenuSlot();
    //void addViewSlot();
    //void addViewNewWindowSlot();
    //  void addTemplateSlot ();
    //  void addTemplateNewWindowSlot ();

  public:
    ViewManagerWidget(ViewManager& view_manager);
    virtual ~ViewManagerWidget();

    void update();

  private:
    ViewManager& view_manager_;
    QVBoxLayout* layout_;
    QVBoxLayout* cont_layout_;

    QPushButton* add_button_;

//    QStringList view_class_list_;
    std::map<QAction*, std::pair<std::string, int> > add_template_actions_;
};
