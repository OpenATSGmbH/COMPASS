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

#include "viewdatawidget.h"

#include <functional>

class View;
class ViewModel;
class ViewRenderWidget;

/**
 * Data widget of a view that is split into a view model and a render widget.
 * Holds the render widget and forwards the functions of ViewDataWidget to the model and the render widget,
 * so View, ViewWidget and ViewLoadStateWidget need no second code path.
 *
 * The render widget does not know the model. The adapter gives it the display data of the model and the
 * render options of the view, on each change of the display data and on each redraw. That call is view
 * specific (setDisplayData(data, options)), so the view widget passes it in as update_display.
 */
class ViewDataWidgetAdapter : public ViewDataWidget
{
    Q_OBJECT
public:
    typedef std::function<void()>                UpdateDisplayFunc;
    typedef std::function<void(nlohmann::json&)> ViewInfoFunc;

    /// model: NOT owned, must outlive the adapter. render_widget: owned by the adapter.
    ViewDataWidgetAdapter(ViewWidget* view_widget,
                          ViewModel* model,
                          ViewRenderWidget* render_widget,
                          const UpdateDisplayFunc& update_display,
                          QWidget* parent = nullptr);
    virtual ~ViewDataWidgetAdapter();

    /// replaces the default view info (of the model, then of the render widget)
    void setViewInfoFunc(const ViewInfoFunc& func) { view_info_ = func; }

    ViewToolSwitcher* renderToolSwitcher() override;

    void loadingStarted() override;
    void loadingDone() override;
    void updateFromSource(const DBContentDataSet& source,
                          const std::vector<std::string>& names, bool reset, bool last) override;
    void clearData() override;
    ViewDrawState redrawData(bool recompute, bool notify = false) override;
    void liveReload() override;
    void beginDataReload() override;

    bool hasData() const override;
    bool hasPendingProcessing() const override;

    void databaseClosed() override;

    ViewLayerTreeModel* layerTreeModel() override;
    void attachLayerPanel(ViewLayerTreeModel* layer_model) override;

    bool isVariableSetLoaded() const override;

public slots:
    void invertSelectionSlot() override;
    void clearSelectionSlot() override;

protected:
    bool hasAnnotations_impl() const override;
    void viewInfoJSON_impl(nlohmann::json& info) const override;
    const BufferData& viewInfoBuffers() const override;

    // hooks of the base class flows, which the adapter replaces completely
    void toolChanged_impl(int) override {}
    void loadingStarted_impl() override {}
    void updateFromSource_impl(const DBContentDataSet&, const std::vector<std::string>&, bool, bool) override {}
    void clearData_impl() override {}
    void clearIntermediateRedrawData_impl() override {}
    ViewDrawState redrawData_impl(bool) override { return drawState(); }
    void liveReload_impl() override {}

private:
    void updateDisplay();
    void displayDataChanged();

    View*             view_          = nullptr;
    ViewModel*        model_         = nullptr;
    ViewRenderWidget* render_widget_ = nullptr;

    UpdateDisplayFunc update_display_;
    ViewInfoFunc      view_info_;

    bool redrawing_ = false; // inside redrawData(), which emits displayChanged() itself
};
