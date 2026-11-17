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
#include "histogramviewrenderwidget.h"
#include "histogramviewmodel.h"
#include "viewdatawidgetadapter.h"
//#include "files.h"
#include "viewtoolwidget.h"

#include <QAction>
#include <QHBoxLayout>
#include <QSettings>
#include <QSplitter>
#include <QTabWidget>

#include "histogramview.h"

/**
 */
HistogramViewWidget::HistogramViewWidget(HistogramView* view, QWidget* parent)
:   VariableViewWidget(view, parent)
{
    auto model = &view->model();

    render_widget_ = new HistogramViewRenderWidget;

    auto render_widget = render_widget_;

    auto update_display = [ render_widget, view ] ()
    {
        render_widget->setDisplayData(view->model().displayData(), view->renderOptions());
    };

    auto data_widget = new ViewDataWidgetAdapter(this, model, render_widget, update_display);

    //the chart is listed only together with a result of the buffers
    data_widget->setViewInfoFunc([ render_widget, model ] (nlohmann::json& info)
    {
        model->viewInfoJSON(info);

        if (model->hasValidResult())
            render_widget->viewInfoJSON(info);
    });

    setDataWidget(data_widget);

    auto config_widget = new HistogramViewConfigWidget(this);
    setConfigWidget(config_widget);

    registerTools(view);
}

/**
 * The tools that need the model. The render widget has registered the navigate tool.
 */
void HistogramViewWidget::registerTools(HistogramView* view)
{
    auto model         = &view->model();
    auto render_widget = render_widget_;

    auto activeIfVariableDataShownCB = [ render_widget, view ] (QAction* a)
    {
        a->setEnabled(render_widget->isDrawn() && view->showsVariables());
    };

    ViewTool select;
    select.id       = HG_SELECT_TOOL;
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

        render_widget->requestRectangle([ render_widget, model ] (const QRectF& rect)
        {
            if (auto bins = render_widget->binRange(rect))
                model->selectBins(bins->first, bins->second);
        });
    };

    ViewTool zoom;
    zoom.id       = HG_ZOOM_TOOL;
    zoom.name     = "Zoom";
    zoom.key      = Qt::Key_Z;
    zoom.icon     = getIcon("zoom_select_action.png");
    zoom.cursor   = Qt::CrossCursor;
    zoom.kind     = ViewTool::Kind::Mode;
    zoom.update   = activeIfVariableDataShownCB;
    zoom.callback = [ render_widget, model ] (bool on)
    {
        if (!on)
            return;

        render_widget->requestRectangle([ render_widget, model ] (const QRectF& rect)
        {
            if (auto bins = render_widget->binRange(rect))
                model->zoomToBins(bins->first, bins->second);
        });
    };

    ViewTool invert_selection;
    invert_selection.id       = HG_INVERT_SELECTION_TOOL;
    invert_selection.name     = "Invert Selection";
    invert_selection.icon     = getIcon("select_invert.png");
    invert_selection.kind     = ViewTool::Kind::Trigger;
    invert_selection.update   = activeIfVariableDataShownCB;
    invert_selection.callback = [ model ] (bool) { model->invertSelection(); };

    ViewTool delete_selection;
    delete_selection.id       = HG_DELETE_SELECTION_TOOL;
    delete_selection.name     = "Delete Selection";
    delete_selection.icon     = getIcon("select_delete.png");
    delete_selection.kind     = ViewTool::Kind::Trigger;
    delete_selection.update   = activeIfVariableDataShownCB;
    delete_selection.callback = [ model ] (bool) { model->clearSelection(); };

    ViewTool zoom_home;
    zoom_home.id       = HG_ZOOM_HOME_TOOL;
    zoom_home.name     = "Zoom to Home";
    zoom_home.key      = Qt::Key_Space;
    zoom_home.icon     = getIcon("zoom_home.png");
    zoom_home.kind     = ViewTool::Kind::Trigger;
    zoom_home.callback = [ model ] (bool) { model->resetZoom(); };

    render_widget->registerTool(select);
    render_widget->registerTool(zoom);

    getViewToolWidget()->addSpacer();

    render_widget->registerTool(invert_selection);
    render_widget->registerTool(delete_selection);

    getViewToolWidget()->addSpacer();

    render_widget->registerTool(zoom_home);
}

/**
 */
HistogramViewWidget::~HistogramViewWidget() = default;

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
