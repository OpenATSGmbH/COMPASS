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

#include "gridviewwidget.h"
#include "gridviewconfigwidget.h"
#include "gridviewdatawidget.h"
#include "gridview.h"
#include "viewtoolwidget.h"
#include "viewtoolswitcher.h"
#include "files.h"

#include <QHBoxLayout>
#include <QSettings>
#include <QSplitter>
#include <QTabWidget>

/**
 */
GridViewWidget::GridViewWidget(const std::string& class_id, 
                               const std::string& instance_id,
                               Configurable* config_parent, 
                               GridView* view,
                               QWidget* parent)
:   VariableViewWidget(class_id, instance_id, config_parent, view, parent)
{
    auto data_widget = new GridViewDataWidget(this);
    setDataWidget(data_widget);

    auto config_widget = new GridViewConfigWidget(this);
    setConfigWidget(config_widget);

    typedef GridViewDataTool Tool;

    auto activeIfDataShownCB = [ data_widget ] (QAction* a)
    {
        a->setEnabled(data_widget->isDrawn());
    };

    auto activeIfVariableDataShownCB = [ data_widget, view ] (QAction* a)
    {
        a->setEnabled(data_widget->isDrawn() && view->showsVariables());
    };

    UNUSED_VARIABLE(activeIfVariableDataShownCB);

    getViewToolSwitcher()->addTool(Tool::GV_NAVIGATE_TOOL, "Navigate", {}, QIcon(), Qt::OpenHandCursor);
    //getViewToolSwitcher()->addTool(Tool::GV_SELECT_TOOL, "Select", Qt::Key_S, getIcon("select_action.png"), Qt::CrossCursor);
    getViewToolSwitcher()->addTool(Tool::GV_ZOOM_RECT_TOOL, "Zoom to Rectangle", Qt::Key_R, getIcon("zoom_select_action.png"), Qt::CrossCursor);

    getViewToolSwitcher()->setDefaultTool(Tool::GV_NAVIGATE_TOOL);
    
    //getViewToolWidget()->addTool(Tool::GV_SELECT_TOOL, activeIfVariableDataShownCB);
    getViewToolWidget()->addTool(Tool::GV_ZOOM_RECT_TOOL, activeIfDataShownCB);

    getViewToolWidget()->addSpacer();

    //getViewToolWidget()->addActionCallback("Invert Selection", [=] () { data_widget->invertSelectionSlot(); }, activeIfVariableDataShownCB, getIcon("select_invert.png"));
    //getViewToolWidget()->addActionCallback("Delete Selection", [=] () { data_widget->clearSelectionSlot(); }, activeIfVariableDataShownCB, getIcon("select_delete.png"));

    getViewToolWidget()->addSpacer();

    getViewToolWidget()->addActionCallback("Zoom to Home", [=] () { data_widget->resetZoomSlot(); }, {}, getIcon("zoom_home.png"), Qt::Key_Space);
}

/**
 */
GridViewWidget::~GridViewWidget() = default;

/**
 */
GridView* GridViewWidget::getView() 
{ 
    auto view = dynamic_cast<GridView*>(ViewWidget::getView());
    traced_assert(view);
    return view;
}

/**
 */
GridViewDataWidget* GridViewWidget::getViewDataWidget()
{
    auto w = dynamic_cast<GridViewDataWidget*>(ViewWidget::getViewDataWidget());
    traced_assert(w);
    return w;
}

/**
 */
const GridViewDataWidget* GridViewWidget::getViewDataWidget() const
{
    auto w = dynamic_cast<const GridViewDataWidget*>(ViewWidget::getViewDataWidget());
    traced_assert(w);
    return w;
}

/**
 */
GridViewConfigWidget* GridViewWidget::getViewConfigWidget()
{
    auto w = dynamic_cast<GridViewConfigWidget*>(ViewWidget::getViewConfigWidget());
    traced_assert(w);
    return w;
}

/**
 */
const GridViewConfigWidget* GridViewWidget::getViewConfigWidget() const
{
    auto w = dynamic_cast<const GridViewConfigWidget*>(ViewWidget::getViewConfigWidget());
    traced_assert(w);
    return w;
}
