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

#include "viewcontainerwidget.h"

#include "compass.h"
#include "config.h"
#include "files.h"
#include "logger.h"
#include "stringconv.h"
#include "viewcontainer.h"
#include "viewmanager.h"
#include "ui_test_common.h"

#include <QMoveEvent>
#include <QResizeEvent>
#include <QTabWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QMessageBox>

using namespace Utils;

ViewContainerWidget::ViewContainerWidget(const std::string& class_id,
                                         const std::string& instance_id, 
                                         ViewManager* view_manager)
    : QWidget(nullptr),
      Configurable(class_id, instance_id, view_manager),
      view_manager_(*view_manager)
{
    logdbg << "instance " << instanceId();

    registerParameter("pos_x", &pos_x_, 0u);
    registerParameter("pos_y", &pos_y_, 0u);
    registerParameter("width", &width_, 1200u);
    registerParameter("height", &height_, 900u);
    registerParameter("min_width", &min_width_, 1200u);
    registerParameter("min_height", &min_height_, 900u);

    name_ = "Window" + std::to_string(String::getAppendedInt(instanceId()));

    //set a nice object name by which we can differentiate multiple windows in qt's object hierarchy
    UI_TEST_OBJ_NAME(this, QString::fromStdString(name_))

    QIcon ats_icon(Files::getIconFilepath("ats.png").c_str());
    setWindowIcon(ats_icon);  // for the glory of the empire

    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    // layout->setSpacing(0);
    // layout->setMargin(0);

    tab_widget_ = new QTabWidget();
    traced_assert(tab_widget_);
    layout->addWidget(tab_widget_);

    // setLayout (layout);
    setMinimumSize(QSize(min_width_, min_height_));
    setGeometry(pos_x_, pos_y_, width_, height_);

    setAttribute(Qt::WA_DeleteOnClose, true);

    createSubConfigurables();

    if (COMPASS::instance().disableAddRemoveViews())
        setWindowFlags(Qt::Window | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);

    show();

    logdbg << "end";
}

ViewContainerWidget::~ViewContainerWidget()
{
    loginf;

    traced_assert(view_container_);
    delete view_container_;
    view_container_ = nullptr;

    loginf << "done";
}

void ViewContainerWidget::generateSubConfigurable(const std::string& class_id,
                                                  const std::string& instance_id)
{
    if (class_id.compare("ViewContainer") == 0)
    {
        traced_assert(tab_widget_);
        traced_assert(!view_container_);
        view_container_ = new ViewContainer(class_id, instance_id, this, &view_manager_,
                                            tab_widget_, String::getAppendedInt(instanceId()));
        traced_assert(view_container_);
    }
    else
        throw std::runtime_error("ViewContainerWidget: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

void ViewContainerWidget::checkSubConfigurables()
{
    if (!view_container_)
    {
        generateSubConfigurable("ViewContainer", instanceId() + "ViewContainer0");
        traced_assert(view_container_);
    }
}

ViewContainer& ViewContainerWidget::viewContainer() const { return *view_container_; }

void ViewContainerWidget::closeEvent(QCloseEvent* event)
{
    loginf << "instance " << instanceId();

    view_manager_.removeContainerWidget(instanceId());
    QWidget::closeEvent(event);
}

void ViewContainerWidget::moveEvent(QMoveEvent* event)
{
    logdbg << instanceId();
    pos_x_ = event->pos().x();
    pos_y_ = event->pos().y();
}

void ViewContainerWidget::resizeEvent(QResizeEvent* event)
{
    logdbg << instanceId();
    width_ = event->size().width();
    height_ = event->size().height();
}

void ViewContainerWidget::updateWindowTitle()
{
    std::string title = COMPASS::instance().versionString() + " " + name_;
    QWidget::setWindowTitle(title.c_str());
}

void ViewContainerWidget::updateFeatures()
{
    updateWindowTitle();
}
