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

#include "histogramviewwidget.h"
#include "histogramviewconfigwidget.h"
#include "histogramviewdatawidget.h"
//#include "files.h"
#include "viewtoolswitcher.h"
#include "viewtoolwidget.h"

#include <QHBoxLayout>
#include <QSettings>
#include <QSplitter>
#include <QTabWidget>

#include "histogramview.h"

/**
 */
HistogramViewWidget::HistogramViewWidget(const std::string& class_id, const std::string& instance_id,
                                     Configurable* config_parent, HistogramView* view,
                                     QWidget* parent)
:   VariableViewWidget(class_id, instance_id, config_parent, view, parent)
{
    auto data_widget = new HistogramViewDataWidget(this);
    setDataWidget(data_widget);

    auto config_widget = new HistogramViewConfigWidget(this);
    setConfigWidget(config_widget);

    typedef HistogramViewDataTool Tool;

    auto activeIfVariableDataShownCB = [ data_widget, view ] (QAction* a)
    {
        a->setEnabled(data_widget->isDrawn() && view->showsVariables());
    };

    getViewToolSwitcher()->addTool(Tool::HG_DEFAULT_TOOL, "", {}, QIcon(), Qt::ArrowCursor);
    getViewToolSwitcher()->addTool(Tool::HG_SELECT_TOOL, "Select", Qt::Key_S, getIcon("select_action.png"), Qt::CrossCursor);
    getViewToolSwitcher()->addTool(Tool::HG_ZOOM_TOOL, "Zoom", Qt::Key_Z, getIcon("zoom_select_action.png"), Qt::CrossCursor);
    getViewToolSwitcher()->setDefaultTool(Tool::HG_DEFAULT_TOOL);

    //we could add the default action if we wanted
    getViewToolWidget()->addTool(Tool::HG_SELECT_TOOL, activeIfVariableDataShownCB);
    getViewToolWidget()->addTool(Tool::HG_ZOOM_TOOL, activeIfVariableDataShownCB);

    getViewToolWidget()->addSpacer();

    getViewToolWidget()->addActionCallback("Invert Selection", [=] () { data_widget->invertSelectionSlot(); }, activeIfVariableDataShownCB, getIcon("select_invert.png"));
    getViewToolWidget()->addActionCallback("Delete Selection", [=] () { data_widget->clearSelectionSlot(); }, activeIfVariableDataShownCB, getIcon("select_delete.png"));

    getViewToolWidget()->addSpacer();

    getViewToolWidget()->addActionCallback("Zoom to Home", [=] () { data_widget->resetZoomSlot(); }, {}, getIcon("zoom_home.png"), Qt::Key_Space);
}

/**
 */
HistogramViewWidget::~HistogramViewWidget() = default;

/**
 */
HistogramViewDataWidget* HistogramViewWidget::getViewDataWidget()
{
    auto w = dynamic_cast<HistogramViewDataWidget*>(ViewWidget::getViewDataWidget());
    traced_assert(w);
    return w;
}

/**
 */
const HistogramViewDataWidget* HistogramViewWidget::getViewDataWidget() const
{
    auto w =  dynamic_cast<const HistogramViewDataWidget*>(ViewWidget::getViewDataWidget());
    traced_assert(w);
    return w;
}

/**
 */
HistogramViewConfigWidget* HistogramViewWidget::getViewConfigWidget()
{
    auto w =  dynamic_cast<HistogramViewConfigWidget*>(ViewWidget::getViewConfigWidget());
    traced_assert(w);
    return w;
}

/**
 */
const HistogramViewConfigWidget* HistogramViewWidget::getViewConfigWidget() const
{
    auto w =  dynamic_cast<const HistogramViewConfigWidget*>(ViewWidget::getViewConfigWidget());
    traced_assert(w);
    return w;
}

/**
 */
HistogramView* HistogramViewWidget::getView() 
{ 
    auto view = dynamic_cast<HistogramView*>(ViewWidget::getView()); 
    traced_assert(view);
    return view;
}
