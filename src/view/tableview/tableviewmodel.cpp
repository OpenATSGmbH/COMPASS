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

#include "tableviewmodel.h"
#include "viewlayersnapshot.h"
#include "logger.h"

/**
 */
TableViewModel::TableViewModel(DBContentManager& dbcont_man, context::DBContextManager& ctx_man, AppState& app_state)
:   ViewModel(dbcont_man, ctx_man, app_state)
{
}

/**
 */
TableViewModel::~TableViewModel() = default;

/**
 */
bool TableViewModel::setAnnotation(const nlohmann::json& /*feature*/, const PlotMetadata& /*metadata*/,
                                   std::string* error)
{
    if (error)
        *error = "the Table View shows no annotations";

    return false;
}

/**
 */
void TableViewModel::clearData_impl()
{
    emit clearDataSignal();

    invalidateLayerAggregates();
}

/**
 */
void TableViewModel::loadingStarted_impl()
{
    //new data incoming - the cached layer aggregation goes stale; the async prepare
    //commit refills it
    invalidateLayerAggregates();
}

/**
 * The render widget prepares the rows, asynchronously offline.
 */
void TableViewModel::loadingDone_impl()
{
    emit loadingDoneSignal();
}

/**
 */
void TableViewModel::recompute_impl()
{
    resetCounts();

    emit recomputeSignal();
}

/**
 * Single fire at the end of any visibility change (group toggles cascade through multiple
 * leaves but emit hiddenChangedSignal once). A color mode change only rebuilds the layer
 * tree (default): AllBufferTableModel::setLayerColors emits dataChanged so the icon column
 * refreshes without a full redraw.
 */
void TableViewModel::layerVisibilityChanged()
{
    emit layerStateChangedSignal();

    recompute();
}

/**
 * Records per (ds_type, ds_name, line, dbcontent) layer from the (cached) layer aggregation;
 * on the asynchronous load path the scan runs on a worker instead, see applyLayerAggregates().
 */
void TableViewModel::rebuildLayerTree()
{
    if (!hasLayerPanel())
        return;

    applyLayerTree(layerAggregates());
}

/**
 */
void TableViewModel::applyLayerAggregates(const LayerAggregates& agg)
{
    //keep the worker-computed aggregation: color mode changes restyle the panel from
    //it without another full row scan (see rebuildLayerTree)
    setLayerAggregates(agg);

    applyLayerTree(layerAggregates());
}

/**
 * The tree part of rebuildLayerTree, from precomputed layer aggregates.
 */
void TableViewModel::applyLayerTree(const LayerAggregates& agg)
{
    if (!hasLayerPanel())
        return;

    updateLayerTree(snapshotsFromAggregates(agg));

    // Push initial allowed set to the table so the first redraw filters correctly.
    emit layerStateChangedSignal();
}

/**
 */
bool TableViewModel::anyLayerHidden() const
{
    for (const auto& layer : layerSnapshots())
        if (!layer->visible())
            return true;

    return false;
}
