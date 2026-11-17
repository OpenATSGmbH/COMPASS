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
#include "scatterplotviewrenderwidget.h"
#include "scatterplotviewmodel.h"
#include "viewdatawidgetadapter.h"
#include "scatterplotview.h"
#include "viewtoolwidget.h"
#include "viewtoolswitcher.h"
#include "files.h"

#include <QAction>
#include <QApplication>
#include <QHBoxLayout>
#include <QSettings>
#include <QSplitter>
#include <QTabWidget>

/**
 */
ScatterPlotViewWidget::ScatterPlotViewWidget(ScatterPlotView* view, QWidget* parent)
:   VariableViewWidget(view, parent)
{
    render_widget_ = new ScatterPlotViewRenderWidget;

    auto render_widget = render_widget_;

    auto update_display = [ render_widget, view ] ()
    {
        render_widget->setDisplayData(view->model().displayData(), view->renderOptions());
    };

    auto data_widget = new ViewDataWidgetAdapter(this, &view->model(), render_widget, update_display);
    setDataWidget(data_widget);

    auto config_widget = new ScatterPlotViewConfigWidget(this);
    setConfigWidget(config_widget);

    registerTools(view);
}

/**
 * The selection tools need the model, the view registers them. The zoom tools are tools of the render
 * widget, registered here at their place in the toolbar. The render widget has registered the navigate tool.
 */
void ScatterPlotViewWidget::registerTools(ScatterPlotView* view)
{
    auto model         = &view->model();
    auto render_widget = render_widget_;

    auto activeIfVariableDataShownCB = [ render_widget, view ] (QAction* a)
    {
        a->setEnabled(render_widget->isDrawn() && view->showsVariables());
    };

    ViewTool select;
    select.id       = SP_SELECT_TOOL;
    select.name     = "Select";
    select.key      = Qt::Key_S;
    select.icon     = getIcon("select_action.png");
    select.cursor   = Qt::CrossCursor;
    select.kind     = ViewTool::Kind::Mode;
    select.update   = activeIfVariableDataShownCB;
    select.callback = [ render_widget, model ] (bool on)
    {
        if (!on)
            return;

        render_widget->requestRectangle([ model ] (const QRectF& rect)
        {
            //with the control key the selection from before stays
            const bool add_to_selection = QApplication::keyboardModifiers() & Qt::ControlModifier;

            model->selectRectangle(rect, add_to_selection);
        });
    };

    ViewTool invert_selection;
    invert_selection.id       = SP_INVERT_SELECTION_TOOL;
    invert_selection.name     = "Invert Selection";
    invert_selection.icon     = getIcon("select_invert.png");
    invert_selection.kind     = ViewTool::Kind::Trigger;
    invert_selection.update   = activeIfVariableDataShownCB;
    invert_selection.callback = [ model ] (bool) { model->invertSelection(); };

    ViewTool delete_selection;
    delete_selection.id       = SP_DELETE_SELECTION_TOOL;
    delete_selection.name     = "Delete Selection";
    delete_selection.icon     = getIcon("select_delete.png");
    delete_selection.kind     = ViewTool::Kind::Trigger;
    delete_selection.update   = activeIfVariableDataShownCB;
    delete_selection.callback = [ model ] (bool) { model->clearSelection(); };

    render_widget->registerTool(select);
    render_widget->registerZoomRectangleTool();

    getViewToolWidget()->addSpacer();

    render_widget->registerTool(invert_selection);
    render_widget->registerTool(delete_selection);

    getViewToolWidget()->addSpacer();

    render_widget->registerZoomHomeTool();
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
