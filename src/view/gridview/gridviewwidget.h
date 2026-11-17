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

#include "variableviewwidget.h"

#include <QPointer>

class GridView;
class GridViewConfigWidget;
class GridViewRenderWidget;
class GridBackgroundRootItem;
class GridBackgroundSectorsItem;
class LayerTreeModel;

/**
 * Widget of the Grid View. Holds the render widget and the background items of the layer panel, whose
 * checked states are settings of the view.
*/
class GridViewWidget : public VariableViewWidget
{
public:
    GridViewWidget(GridView* view, QWidget* parent = nullptr);
    virtual ~GridViewWidget();

    GridViewRenderWidget* renderWidget() { return render_widget_; }
    GridViewConfigWidget* getViewConfigWidget();
    const GridViewConfigWidget* getViewConfigWidget() const;

    GridView* getView();

    /// the background settings or the grid opacity of the view changed (also on a reconfigure / preset)
    void backgroundSettingsChanged();

private:
    void registerTools();

    /// adds the background subtree and the grid opacity menu to the layer panel
    void layerPanelAttached();
    void syncBackgroundItems();

    GridBackgroundRootItem* backgroundRootItem() const;
    GridBackgroundSectorsItem* sectorsRootItem() const;

    GridViewRenderWidget* render_widget_ = nullptr; // owned by the data widget (adapter)

    QPointer<LayerTreeModel>   background_model_;          // model holding the items below
    GridBackgroundRootItem*    background_root_ = nullptr;
    GridBackgroundSectorsItem* sectors_root_    = nullptr; // top level Sectors item
};
