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

#include "viewdatawidgetadapter.h"
#include "viewmodel.h"
#include "viewrenderwidget.h"
#include "viewwidget.h"
#include "view.h"
#include "buffer.h"
#include "dbcontent/variable/variable.h"
#include "logger.h"
#include "traced_assert.h"

#include <QApplication>
#include <QScopedValueRollback>
#include <QVBoxLayout>

/**
 */
ViewDataWidgetAdapter::ViewDataWidgetAdapter(ViewWidget* view_widget,
                                             ViewModel* model,
                                             ViewRenderWidget* render_widget,
                                             const UpdateDisplayFunc& update_display,
                                             QWidget* parent)
:   ViewDataWidget (view_widget, parent)
,   view_          (view_widget->getView())
,   model_         (model         )
,   render_widget_ (render_widget )
,   update_display_(update_display)
{
    traced_assert(view_);
    traced_assert(model_);
    traced_assert(render_widget_);
    traced_assert(update_display_);

    auto layout = new QVBoxLayout;
    layout->setMargin(0);
    layout->addWidget(render_widget_);

    setLayout(layout);

    connect(model_, &ViewModel::displayDataChangedSignal, this, &ViewDataWidgetAdapter::displayDataChanged);
    connect(model_, &ViewModel::dataLoadedSignal, this, &ViewDataWidget::dataLoaded);
    connect(model_, &ViewModel::processingStartedSignal, this, &ViewDataWidget::processingStartedSignal);
    connect(model_, &ViewModel::processingFinishedSignal, this, &ViewDataWidget::processingFinishedSignal);

    // the display before the first load does not count as drawn
    update_display_();
    render_widget_->resetDrawState();
}

/**
 */
ViewDataWidgetAdapter::~ViewDataWidgetAdapter() = default;

/**
 */
ViewToolSwitcher* ViewDataWidgetAdapter::renderToolSwitcher()
{
    return &render_widget_->toolSwitcher();
}

/**
 * The render widget gets the display data of the model and the render options of the view.
 */
void ViewDataWidgetAdapter::updateDisplay()
{
    update_display_();

    setDrawState(render_widget_->hasContent() ? ViewDrawState::DrawnContent :
                 render_widget_->isDrawn()    ? ViewDrawState::Drawn        :
                                                ViewDrawState::NotDrawn);
}

/**
 * The model has new display data, e.g. from the commit of an asynchronous recompute.
 */
void ViewDataWidgetAdapter::displayDataChanged()
{
    updateDisplay();

    if (!redrawing_)
        emit displayChanged();
}

/**
 */
void ViewDataWidgetAdapter::loadingStarted()
{
    loginf;

    //clear and update display
    model_->loadingStarted();
    render_widget_->clear();

    redrawData(false, false);
}

/**
 */
void ViewDataWidgetAdapter::loadingDone()
{
    loginf;

    model_->loadingDone();
}

/**
 */
void ViewDataWidgetAdapter::updateFromSource(const DBContentDataSet& source,
                                             const std::vector<std::string>& names, bool reset, bool last)
{
    model_->updateFromSource(source, names, reset, last);
}

/**
 */
void ViewDataWidgetAdapter::clearData()
{
    model_->clearData();
    render_widget_->clear();

    setDrawState(ViewDrawState::NotDrawn);
}

/**
 * As ViewDataWidget::redrawData(). A recompute runs in the model, which reports its new display data
 * with a signal, at once or after an asynchronous computation.
 */
ViewDataWidget::ViewDrawState ViewDataWidgetAdapter::redrawData(bool recompute, bool notify)
{
    loginf << "recompute " << recompute << " notify " << notify;

    //the model runs one recompute when its outstanding work has finished
    if (recompute && model_->hasPendingProcessing())
    {
        model_->recompute();
        return drawState();
    }

    if (notify)
    {
        emit redrawStarted();
        QApplication::processEvents(); //process any ui reactions on this signal before ui is blocked by redraw
    }

    {
        QScopedValueRollback<bool> redrawing(redrawing_, true);

        if (recompute)
            model_->recompute();
        else
            updateDisplay();
    }

    if (notify)
    {
        emit redrawDone();
    }

    //signal display changed to whom it may concern
    emit displayChanged();

    return drawState();
}

/**
 */
void ViewDataWidgetAdapter::liveReload()
{
    //@TODO: implement live reload for variable based views
}

/**
 */
void ViewDataWidgetAdapter::beginDataReload()
{
    model_->beginDataReload();
}

/**
 */
bool ViewDataWidgetAdapter::hasData() const
{
    return model_->hasData();
}

/**
 */
bool ViewDataWidgetAdapter::hasAnnotations_impl() const
{
    return model_->hasAnnotation();
}

/**
 */
bool ViewDataWidgetAdapter::hasPendingProcessing() const
{
    return model_->hasPendingProcessing();
}

/**
 */
void ViewDataWidgetAdapter::databaseClosed()
{
    model_->databaseClosed();
}

/**
 */
ViewLayerTreeModel* ViewDataWidgetAdapter::layerTreeModel()
{
    return model_->layerPanelModel();
}

/**
 */
void ViewDataWidgetAdapter::attachLayerPanel(ViewLayerTreeModel* layer_model)
{
    model_->attachLayerPanel(layer_model);
}

/**
 * Checks if the required read set of the view is available in the loaded data.
 */
bool ViewDataWidgetAdapter::isVariableSetLoaded() const
{
    const auto& view_data = model_->viewData();
    if (view_data.empty())
        return false;

    for (const auto& dbcontent_data : view_data)
    {
        auto var_set = view_->getSet(dbcontent_data.first);
        for (auto var : var_set.getSet())
        {
            if (!dbcontent_data.second->hasAnyPropertyNamed(var->name()))
                return false;
        }
    }

    return true;
}

/**
 */
void ViewDataWidgetAdapter::invertSelectionSlot()
{
    model_->invertSelection();
}

/**
 */
void ViewDataWidgetAdapter::clearSelectionSlot()
{
    model_->clearSelection();
}

/**
 */
const ViewDataWidget::BufferData& ViewDataWidgetAdapter::viewInfoBuffers() const
{
    return model_->viewData();
}

/**
 */
void ViewDataWidgetAdapter::viewInfoJSON_impl(nlohmann::json& info) const
{
    if (view_info_)
    {
        view_info_(info);
        return;
    }

    model_->viewInfoJSON(info);
    render_widget_->viewInfoJSON(info);
}
