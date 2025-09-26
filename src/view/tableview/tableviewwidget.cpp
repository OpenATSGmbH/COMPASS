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
#include "tableviewdatawidget.h"

#include <QHBoxLayout>
#include <QSettings>
#include <QSplitter>
#include <QTabWidget>

/*
 */
TableViewWidget::TableViewWidget(const std::string& class_id, 
                                     const std::string& instance_id,
                                     Configurable* config_parent, 
                                     TableView* view,
                                     QWidget* parent)
:   ViewWidget(class_id, instance_id, config_parent, view, parent)
{
    auto data_widget = new TableViewDataWidget(this);
    setDataWidget(data_widget);

    auto config_widget = new TableViewConfigWidget(this);
    setConfigWidget(config_widget);
}

/*
 */
TableViewWidget::~TableViewWidget() = default;

/**
 */
TableViewDataWidget* TableViewWidget::getViewDataWidget()
{
    auto w = dynamic_cast<TableViewDataWidget*>(ViewWidget::getViewDataWidget());
    traced_assert(w);
    return w;
}

/**
 */
const TableViewDataWidget* TableViewWidget::getViewDataWidget() const
{
    auto w =  dynamic_cast<const TableViewDataWidget*>(ViewWidget::getViewDataWidget());
    traced_assert(w);
    return w;
}

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
