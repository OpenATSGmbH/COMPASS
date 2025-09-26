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

#include "viewconfigwidget.h"
#include "viewwidget.h"
#include "viewpresetwidget.h"
#include "viewmanager.h"
#include "compass.h"

#include <iostream>

#include <QTabWidget>
#include <QVBoxLayout>

/********************************************************************************************
 * ViewConfigWidget
 ********************************************************************************************/

/**
*/
ViewConfigWidget::ViewConfigWidget(ViewWidget* view_widget, QWidget* parent, Qt::WindowFlags f) 
:   QWidget     (parent, f)
,   view_widget_(view_widget)
{
    traced_assert(view_widget_);

    setObjectName("configwidget");

    setMinimumWidth(MinWidth);
}

/**
 * React on loading start.
*/
void ViewConfigWidget::loadingStarted()
{
    setDisabled(true);
}

/**
 * React on loading end.
*/
void ViewConfigWidget::loadingDone()
{
    setDisabled(false);
}

/**
 * React on redraw start.
*/
void ViewConfigWidget::redrawStarted()
{
    setDisabled(true);
}

/**
 * React on redraw end.
*/
void ViewConfigWidget::redrawDone()
{
    setDisabled(false);
}

/**
 * Reacts on switching the application mode.
 */
void ViewConfigWidget::appModeSwitch(AppMode app_mode) 
{
    //per default do nothing
}

/**
 * Reacts on config changes.
*/
void ViewConfigWidget::configChanged()
{
    //per default do nothing
}

/**
 * Reacts on changes in the display (e.g. if display information should be visualized in the config widget).
 * (Note: Only called if both config and data widget exist)
 */
void ViewConfigWidget::onDisplayChange()
{
    //invoke derived
    onDisplayChange_impl();
}

/**
 * Generates json view information.
 */
nlohmann::json ViewConfigWidget::viewInfoJSON() const
{
    nlohmann::json info;

    //@TODO: add general information?

    //add view-specific information
    viewInfoJSON_impl(info);

    return info;
}

/********************************************************************************************
 * TabStyleViewConfigWidget
 ********************************************************************************************/

/**
*/
TabStyleViewConfigWidget::TabStyleViewConfigWidget(ViewWidget* view_widget, QWidget* parent, Qt::WindowFlags f) 
:   ViewConfigWidget(view_widget, parent, f)
{
    //create main layout
    main_layout_ = new QVBoxLayout;
    main_layout_->setMargin(1);
    main_layout_->setContentsMargins(1, 1, 1, 1);
    setLayout(main_layout_);

    // //create view preset widget
    // if (COMPASS::instance().viewManager().viewPresetsEnabled())
    // {
    //     auto preset_widget = new ViewPresetWidget(getWidget()->getView(), this);
        
    //     main_layout_->addWidget(preset_widget);
    // }

    //create tab widget
    tab_widget_ = new QTabWidget(this);
    //tab_widget_->setStyleSheet("QTabBar::tab { height: "+ QString::number(TabHeight) + "px; }");
    main_layout_->addWidget(tab_widget_);
}
