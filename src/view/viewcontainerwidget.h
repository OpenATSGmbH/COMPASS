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

#include <QWidget>

#include "configurable.h"

class QTabWidget;
class ViewManager;
class ViewContainer;

/**
 * Serves as container for different views. May be embedded in a parent widget, or in new window,
 * which is decided by constructor.
 *
 * Includes DBConfigTab, therefore DB HAS to be loaded before a constructor is called.
 */
class ViewContainerWidget : public QWidget, public Configurable
{
  public:
    ViewContainerWidget(const std::string& class_id, const std::string& instance_id,
                        ViewManager* parent);
    virtual ~ViewContainerWidget();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    const std::string& name() { return name_; }
    ViewContainer& viewContainer() const;

    void updateFeatures();

  protected:
    ViewManager& view_manager_;
    std::string name_;

    unsigned int pos_x_;
    unsigned int pos_y_;
    unsigned int width_;
    unsigned int height_;
    unsigned int min_width_;
    unsigned int min_height_;

    QTabWidget* tab_widget_{nullptr};
    ViewContainer* view_container_{nullptr};

    void updateWindowTitle();

    void closeEvent(QCloseEvent* event);
    virtual void moveEvent(QMoveEvent* event);
    virtual void resizeEvent(QResizeEvent* event);

    virtual void checkSubConfigurables();
};
