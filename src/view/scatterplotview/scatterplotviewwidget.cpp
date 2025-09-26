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

#include "scatterplotviewwidget.h"
#include "scatterplotviewconfigwidget.h"
#include "scatterplotviewdatawidget.h"
#include "scatterplotview.h"
#include "viewtoolwidget.h"
#include "viewtoolswitcher.h"
#include "files.h"

#include <QHBoxLayout>
#include <QSettings>
#include <QSplitter>
#include <QTabWidget>

/**
 */
ScatterPlotViewWidget::ScatterPlotViewWidget(const std::string& class_id, 
                                             const std::string& instance_id,
                                             Configurable* config_parent, 
                                             ScatterPlotView* view,
                                             QWidget* parent)
:   VariableViewWidget(class_id, instance_id, config_parent, view, parent)
{
    auto data_widget = new ScatterPlotViewDataWidget(this);
    setDataWidget(data_widget);

    auto config_widget = new ScatterPlotViewConfigWidget(this);
    setConfigWidget(config_widget);

    typedef ScatterPlotViewDataTool Tool;

    auto activeIfDataShownCB = [ data_widget ] (QAction* a)
    {
        a->setEnabled(data_widget->isDrawn());
    };

    auto activeIfVariableDataShownCB = [ data_widget, view ] (QAction* a)
    {
        a->setEnabled(data_widget->isDrawn() && view->showsVariables());
    };

    getViewToolSwitcher()->addTool(Tool::SP_NAVIGATE_TOOL, "Navigate", {}, QIcon(), Qt::OpenHandCursor);
    getViewToolSwitcher()->addTool(Tool::SP_SELECT_TOOL, "Select", Qt::Key_S, getIcon("select_action.png"), Qt::CrossCursor);
    getViewToolSwitcher()->addTool(Tool::SP_ZOOM_RECT_TOOL, "Zoom to Rectangle", Qt::Key_R, getIcon("zoom_select_action.png"), Qt::CrossCursor);

    getViewToolSwitcher()->setDefaultTool(Tool::SP_NAVIGATE_TOOL);
    
    getViewToolWidget()->addTool(Tool::SP_SELECT_TOOL, activeIfVariableDataShownCB);
    getViewToolWidget()->addTool(Tool::SP_ZOOM_RECT_TOOL, activeIfDataShownCB);

    getViewToolWidget()->addSpacer();

    getViewToolWidget()->addActionCallback("Invert Selection", [=] () { data_widget->invertSelectionSlot(); }, activeIfVariableDataShownCB, getIcon("select_invert.png"));
    getViewToolWidget()->addActionCallback("Delete Selection", [=] () { data_widget->clearSelectionSlot(); }, activeIfVariableDataShownCB, getIcon("select_delete.png"));

    getViewToolWidget()->addSpacer();

    getViewToolWidget()->addActionCallback("Zoom to Home", [=] () { data_widget->resetZoomSlot(); }, {}, getIcon("zoom_home.png"), Qt::Key_Space);
}

/**
 */
ScatterPlotViewWidget::~ScatterPlotViewWidget() = default;

/**
 */
ScatterPlotView* ScatterPlotViewWidget::getView() 
{ 
    auto view = dynamic_cast<ScatterPlotView*>(ViewWidget::getView());
    traced_assert(view);
    return view;
}

/**
 */
ScatterPlotViewDataWidget* ScatterPlotViewWidget::getViewDataWidget()
{
    auto w = dynamic_cast<ScatterPlotViewDataWidget*>(ViewWidget::getViewDataWidget());
    traced_assert(w);
    return w;
}

/**
 */
const ScatterPlotViewDataWidget* ScatterPlotViewWidget::getViewDataWidget() const
{
    auto w = dynamic_cast<const ScatterPlotViewDataWidget*>(ViewWidget::getViewDataWidget());
    traced_assert(w);
    return w;
}

/**
 */
ScatterPlotViewConfigWidget* ScatterPlotViewWidget::getViewConfigWidget()
{
    auto w = dynamic_cast<ScatterPlotViewConfigWidget*>(ViewWidget::getViewConfigWidget());
    traced_assert(w);
    return w;
}

/**
 */
const ScatterPlotViewConfigWidget* ScatterPlotViewWidget::getViewConfigWidget() const
{
    auto w = dynamic_cast<const ScatterPlotViewConfigWidget*>(ViewWidget::getViewConfigWidget());
    traced_assert(w);
    return w;
}
