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

#include "viewcomponent.h"
#include "appmode.h"
#include "json_fwd.hpp"

#include <QWidget>

class ViewWidget;
class View;

class QTabWidget;
class QVBoxLayout;

/**
 * Base class for view configuration widgets, which are held in the configuration area of the ViewWidget.
 * Derive and reimplement as needed.
 */
class ViewConfigWidget : public QWidget, public ViewComponent
{
public:
    ViewConfigWidget(ViewWidget* view_widget, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~ViewConfigWidget() = default;

    void onDisplayChange();

    virtual void loadingStarted();
    virtual void loadingDone();
    virtual void redrawStarted();
    virtual void redrawDone();
    virtual void appModeSwitch(AppMode app_mode);
    virtual void configChanged();
    virtual void onInit() {} //reacts on view init (both data and config widget created)

    nlohmann::json viewInfoJSON() const override final;

    static const int MinWidth  = 400;
    
protected:
    virtual void onDisplayChange_impl() {} 
    virtual void viewInfoJSON_impl(nlohmann::json& info) const {}

    ViewWidget* getWidget() { return view_widget_; }

private:
    ViewWidget* view_widget_ = nullptr;
};

/**
 * This config widget already contains a tab widget arranged in a vertical layout,
 * which can be retrieved and reused in derived classes.
 */
class TabStyleViewConfigWidget : public ViewConfigWidget
{
public:
    TabStyleViewConfigWidget(ViewWidget* view_widget, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~TabStyleViewConfigWidget() = default;

    static const int TabHeight = 42;

protected:
    QTabWidget* getTabWidget() { return tab_widget_; }
    QVBoxLayout* getMainLayout() { return main_layout_; }

private:
    QTabWidget*  tab_widget_  = nullptr;
    QVBoxLayout* main_layout_ = nullptr;
};
