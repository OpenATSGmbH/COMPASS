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

#include "tableviewwidget.h"
#include "tableview.h"
#include "tableviewconfigwidget.h"
#include "tableviewmodel.h"
#include "tableviewrenderwidget.h"

#include <QHBoxLayout>
#include <QSettings>
#include <QSplitter>
#include <QTabWidget>

/*
 */
TableViewWidget::TableViewWidget(TableView* view, QWidget* parent)
:   ViewWidget(view, parent)
{
    auto model = &view->model();

    render_widget_ = new TableViewRenderWidget(*view, *model);

    auto render_widget = render_widget_;

    //the render widget builds the rows itself, no display before the first load
    setRenderWidget(model, render_widget, [ render_widget ] () { render_widget->redraw(); }, false);

    connect(model, &TableViewModel::clearDataSignal, render_widget, &TableViewRenderWidget::clearDataSlot);
    connect(model, &TableViewModel::loadingDoneSignal, render_widget, &TableViewRenderWidget::loadingDoneSlot);
    connect(model, &TableViewModel::recomputeSignal, render_widget, &TableViewRenderWidget::recomputeSlot);
    connect(model, &TableViewModel::layerStateChangedSignal, render_widget, &TableViewRenderWidget::layerStateChangedSlot);

    connect(render_widget, &TableViewRenderWidget::displayChangedSignal, this, [ this ] () { renderDisplayChanged(); });

    auto config_widget = new TableViewConfigWidget(this);
    setConfigWidget(config_widget);
}

/*
 */
TableViewWidget::~TableViewWidget() = default;

/**
 */
TableViewConfigWidget* TableViewWidget::getViewConfigWidget()
{
    auto w =  dynamic_cast<TableViewConfigWidget*>(ViewWidget::getViewConfigWidget());
    traced_assert(w);
    return w;
}

/**
 */
const TableViewConfigWidget* TableViewWidget::getViewConfigWidget() const
{
    auto w =  dynamic_cast<const TableViewConfigWidget*>(ViewWidget::getViewConfigWidget());
    traced_assert(w);
    return w;
}

/**
 */
TableView* TableViewWidget::getView() 
{ 
    auto view = dynamic_cast<TableView*>(ViewWidget::getView());
    traced_assert(view);
    return view;
}
