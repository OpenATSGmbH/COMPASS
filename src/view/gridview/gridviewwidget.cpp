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
#include "gridviewrenderwidget.h"
#include "gridviewmodel.h"
#include "gridview.h"
#include "gridbackgrounditems.h"
#include "viewdatawidgetadapter.h"
#include "viewlayertreemodel.h"
#include "dbcontentlayer.h"
#include "viewmanager.h"
#include "db_context_manager.h"
#include "viewtoolwidget.h"
#include "viewtoolswitcher.h"
#include "files.h"

#include "json.hpp"

#include <QAction>
#include <QMenu>

/**
 */
GridViewWidget::GridViewWidget(GridView* view, QWidget* parent)
:   VariableViewWidget(view, parent)
{
    render_widget_ = new GridViewRenderWidget;

    auto render_widget = render_widget_;
    auto model         = &view->model();

    auto update_display = [ render_widget, view ] ()
    {
        render_widget->setDisplayData(view->model().displayData(), view->renderOptions());
    };

    auto data_widget = new ViewDataWidgetAdapter(this, model, render_widget, update_display);

    //the zoom state needs the data bounds of the model and the axis ranges of the render widget
    data_widget->setViewInfoFunc([ model, render_widget ] (nlohmann::json& info)
    {
        model->viewInfoJSON(info);
        render_widget->viewInfoJSON(info);

        const auto data_bounds = model->dataBoundsXY();
        const auto axis_ranges = render_widget->axisRanges();

        info[ "axis_zoom_active" ] = data_bounds.has_value() && axis_ranges.has_value() ?
                                     GridViewRenderWidget::zoomActive(data_bounds.value(), axis_ranges.value()) : false;
    });

    setDataWidget(data_widget);

    //the config widget attaches the layer panel
    connect(model, &ViewModel::layerPanelAttachedSignal, this, [ this ] () { layerPanelAttached(); });

    //e.g. the map item shows whether the map is available
    connect(&render_widget_->background(), &GridBackgroundRenderer::updateSignal, this, [ this ] ()
    {
        syncBackgroundItems();
    });

    connect(&render_widget_->background(), &GridBackgroundRenderer::availableChangedSignal, this, [ this ] ()
    {
        if (auto root = backgroundRootItem())
            root->notifySubtreeChanged();
        if (auto sectors = sectorsRootItem())
            sectors->notifySubtreeChanged();
    });

    //the sector layers below the sectors item
    auto& ctx_man = view->viewManager().dbContextManager();

    auto sectorsChanged = [ this ] ()
    {
        if (auto sectors = sectorsRootItem())
            sectors->update();
    };

    connect(&ctx_man, &context::DBContextManager::sectorsChangedSignal, this, sectorsChanged);
    connect(&ctx_man, &context::DBContextManager::activeContextChangedSignal, this, sectorsChanged);

    auto config_widget = new GridViewConfigWidget(this);
    setConfigWidget(config_widget);

    registerTools();
}

/**
 * The zoom tools are tools of the render widget, registered here at their place in the toolbar. The
 * render widget has registered the navigate tool. The selection tools are not offered.
 */
void GridViewWidget::registerTools()
{
    render_widget_->registerZoomRectangleTool();

    getViewToolWidget()->addSpacer();
    getViewToolWidget()->addSpacer();

    render_widget_->registerZoomHomeTool();
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

/**
 */
void GridViewWidget::backgroundSettingsChanged()
{
    auto view = getView();

    //the display before the first load does not count as drawn, see ViewDataWidgetAdapter
    const bool drawn = render_widget_->isDrawn();

    render_widget_->setDisplayData(view->model().displayData(), view->renderOptions());

    if (!drawn)
        render_widget_->resetDrawState();

    //e.g. a preset applied
    syncBackgroundItems();
}

/**
 */
void GridViewWidget::syncBackgroundItems()
{
    if (auto root = backgroundRootItem())
        root->syncFromSettings();
    if (auto sectors = sectorsRootItem())
        sectors->syncFromSettings();
}

/**
*/
GridBackgroundRootItem* GridViewWidget::backgroundRootItem() const
{
    //the model belongs to the layer panel, which may be gone
    if (!background_model_)
        return nullptr;

    return background_root_;
}

/**
*/
GridBackgroundSectorsItem* GridViewWidget::sectorsRootItem() const
{
    //the model belongs to the layer panel, which may be gone
    if (!background_model_)
        return nullptr;

    return sectors_root_;
}

/**
*/
void GridViewWidget::layerPanelAttached()
{
    auto view  = getView();
    auto model = view->model().layerPanelModel();
    if (!model)
        return;

    //grid opacity in the context menu of the data item, in steps as the map opacity
    if (auto* data_root = model->dbContentRootItem())
    {
        data_root->setMenuExtension([ view ] (QMenu& menu)
        {
            menu.addSection("Grid");

            QMenu* opacity_menu = menu.addMenu("Opacity");

            for (unsigned int i = 0; i <= 100; i += 10)
            {
                QAction* act = opacity_menu->addAction(QString::number(i) + "%");
                act->setCheckable(true);
                act->setChecked(view->settings().grid_opacity == i);

                QObject::connect(act, &QAction::triggered, view, [ view, i ] () { view->setGridOpacity(i); });
            }
        });
    }

    const auto& renderer = render_widget_->background();

    //sectors as a top level item (as in the Geographic View), above the Background Map
    auto sectors = std::make_unique<GridBackgroundSectorsItem>(*view, renderer);

    sectors_root_ = static_cast<GridBackgroundSectorsItem*>(model->addRootItem(std::move(sectors)));
    sectors_root_->update();

    auto root = std::make_unique<GridBackgroundRootItem>(*view, renderer);

    background_root_  = static_cast<GridBackgroundRootItem*>(model->addRootItem(std::move(root)));
    background_model_ = model;
}
