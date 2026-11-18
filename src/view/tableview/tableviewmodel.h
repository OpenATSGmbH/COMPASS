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

#include "viewmodel.h"

/**
 * Model of the Table View. It holds only what the base holds: buffers, layer aggregates and snapshots,
 * asynchronous processor.
 *
 * Exceptions of the Table View (readme_view_render_widgets.md, section 7.6): the render widget holds a
 * pointer to the model, reads the buffers and builds the table rows. There is no display data.
 * The model reports its flows with signals, the view widget connects them to the render widget.
 */
class TableViewModel : public ViewModel
{
    Q_OBJECT

signals:
    /// the buffers were cleared
    void clearDataSignal();
    /// the loaded data is complete
    void loadingDoneSignal();
    /// the table rows are computed again
    void recomputeSignal();
    /// the layer snapshots or their visibility changed
    void layerStateChangedSignal();

public:
    using ViewModel::LayerAggregates;

    TableViewModel(DBContentManager& dbcont_man, context::DBContextManager& ctx_man, AppState& app_state);
    virtual ~TableViewModel();

    /// the Table View shows no annotations
    bool setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata,
                       std::string* error = nullptr) override;
    void clearAnnotation() override {}
    bool hasAnnotation() const override { return false; }

    // used by the render widget

    using ViewModel::asyncProcessor;
    using ViewModel::shutdownAsyncProcessor;
    using ViewModel::layerSnapshots;

    /// layer snapshots from the (cached) layer aggregation of the loaded buffers
    void rebuildLayerTree() override final;
    /// keeps a layer aggregation computed elsewhere (e.g. on a worker), the layer tree follows it
    void applyLayerAggregates(const LayerAggregates& agg);

    /// true when the current panel state hides any layer
    bool anyLayerHidden() const;

protected:
    void updateFromSource_impl(bool reset, bool last) override {}
    void clearData_impl() override;
    void recompute_impl() override;
    void loadingStarted_impl() override;
    void loadingDone_impl() override;

    /// the allowed layers => table filter + recompute
    void layerVisibilityChanged() override final;

private:
    /// layer snapshots + data subtree from layer aggregates
    void applyLayerTree(const LayerAggregates& agg);
};
