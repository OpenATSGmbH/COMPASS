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

#include "histogramviewmodel.h"
#include "histogramgenerator.h"
#include "histogramgeneratorbuffer.h"
#include "appstate.h"
#include "buffer.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/variable/reportvariable.h"
#include "viewlayersnapshot.h"
#include "viewasyncprocessor.h"
#include "viewvariable.h"
#include "plotmetadata.h"
#include "property_templates.h"
#include "viewpointgenerator.h"
#include "logger.h"
#include "global.h"
#include "traced_assert.h"

#include "json.hpp"

#include <set>

using namespace std;

namespace
{
    /// Group key derived from a full layer id ("<ds_type>:<ds_name>:L<n>:
    /// <dbcontent>") for the given color mode - this is what the chart
    /// actually stacks on (one bar set per group). Matches the semantics of
    /// AppState::colorMode():
    ///   0 DSType, 1 DBContent, 2 Data Source, 3 Data Source + Line.
    std::string groupKeyFor(const std::string& layer_id, unsigned int color_mode)
    {
        std::vector<std::string> parts;
        size_t start = 0;
        for (size_t i = 0; i <= layer_id.size(); ++i)
        {
            if (i == layer_id.size() || layer_id[i] == ':')
            {
                parts.push_back(layer_id.substr(start, i - start));
                start = i + 1;
            }
        }
        if (parts.size() != 4)
            return layer_id;

        switch (color_mode)
        {
            case 0: return parts[0];                                        // DSType
            case 1: return parts[3];                                        // DBContent
            case 2: return parts[0] + ":" + parts[1];                       // Data Source
            case 3: return parts[0] + ":" + parts[1] + ":" + parts[2];      // Data Source + Line
            default: return layer_id;
        }
    }
}

/**
 */
HistogramViewModel::HistogramViewModel(DBContentManager& dbcont_man,
                                       context::DBContextManager& ctx_man,
                                       AppState& app_state)
:   VariableViewModel(dbcont_man, ctx_man, app_state)
{
}

/**
 */
HistogramViewModel::~HistogramViewModel()
{
    //the generator workers capture this - join them before the members die
    shutdownAsyncProcessor();
}

/**
 */
void HistogramViewModel::setVariable(ViewVariable& variable)
{
    setVariables({ &variable });

    //the empty chart before the first load shows the axis label
    display_data_ = displayDataFromGenerator();
}

/**
 */
std::shared_ptr<const HistogramDisplayData> HistogramViewModel::displayDataFromAnnotation(const nlohmann::json& feature,
                                                                                         const PlotMetadata& metadata,
                                                                                         std::string* error)
{
    try
    {
        if (!feature.is_object())
            throw std::runtime_error("histogram annotation feature is not an object");

        if (!feature.contains(ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram))
            throw std::runtime_error("histogram annotation feature missing '"
                + ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram + "' field");

        auto data = std::make_shared<HistogramDisplayData>();

        if (!data->histogram.fromJSON(feature[ ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram ]))
            throw std::runtime_error("could not read histogram from annotation");

        data->title        = metadata.title_;
        data->x_axis_label = metadata.xAxisLabel();
        data->valid        = data->histogram.hasData();

        return data;
    }
    catch (const std::exception& e)
    {
        if (error)
            *error = e.what();
    }

    return {};
}

/**
 */
bool HistogramViewModel::setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata, std::string* error)
{
    loginf;

    annotation_data_  = displayDataFromAnnotation(feature, metadata, error);
    annotation_valid_ = annotation_data_ != nullptr;

    if (!annotation_valid_)
    {
        //the empty chart of a malformed annotation keeps its title and axis label
        auto data = std::make_shared<HistogramDisplayData>();
        data->title        = metadata.title_;
        data->x_axis_label = metadata.xAxisLabel();

        annotation_data_ = data;
    }

    setAnnotationActive();

    return annotation_valid_;
}

/**
 */
bool HistogramViewModel::updateFromAnnotation()
{
    display_data_ = annotation_data_;

    return annotation_valid_;
}

/**
 */
void HistogramViewModel::publishDisplayData()
{
    display_data_ = computed_data_;
}

/**
 * The histogram is rebuilt without the hidden layers (the hidden set itself is kept by
 * the layer tree model).
 */
void HistogramViewModel::layerVisibilityChanged()
{
    recompute();
}

/**
 * resolveSeriesColor outputs differ => snapshots (leaf icon + panel group colors) AND the
 * display data, so the stacked bar colors match. The histogram values are unchanged -
 * recompute() does recompute, but that is cheap compared to a buffer reload.
 */
void HistogramViewModel::colorModeChanged()
{
    rebuildLayerTree();
    recompute();
}

/**
 */
void HistogramViewModel::resetHistogram()
{
    histogram_generator_.reset();
    computed_data_.reset();
}

/**
 */
void HistogramViewModel::updateDataEvent(bool requires_reset)
{
    //current generator makes no sense any more
    resetHistogram();

    //new data -> old zoom bin indices are meaningless
    saved_zoom_range_.reset();

    //new data -> per-row layer ids and the layer aggregation are stale
    row_layer_ids_valid_ = false;
    invalidateLayerAggregates();

    // Offline the per-row scans - the layer aggregation for the panel and the row
    // layer ids the generator consumes - run on a worker; the commit installs them and
    // rebuilds the panel, and the base defers the load-done recompute until then (the
    // recompute then only runs the generator with the ids already computed).
    // Live stays synchronous: the feed mutates the same buffers every tick.
    bool async = appState().appMode() != AppMode::LiveRunning
              && !viewData().empty()
              && hasLayerPanel();

    if (async)
    {
        auto scan_input = std::make_shared<view_layer_scan::ScanInput>(
            view_layer_scan::makeScanInput(viewData(), dbContentManager(), dbContextManager()));

        auto agg     = std::make_shared<LayerAggregates>();
        auto row_ids = std::make_shared<view_layer_scan::RowLayerIndex>();

        asyncProcessor().launch("histogram view layer data",
            [ scan_input, agg, row_ids ] ()
            {
                *agg     = view_layer_scan::aggregateLayers(*scan_input);
                *row_ids = view_layer_scan::computeRowLayerIndex(*scan_input);
            },
            [ this, agg, row_ids ] ()
            {
                row_layer_index_     = std::move(*row_ids);
                row_layer_ids_valid_ = true;

                setLayerAggregates(std::move(*agg));

                applyLayerTree(layerAggregates());
            });

        return;
    }

    // Buffers have changed; repopulate the layer panel (empty if no data).
    rebuildLayerTree();
}

/**
 */
void HistogramViewModel::resetVariableData()
{
    resetHistogram();

    row_layer_ids_valid_ = false;
    invalidateLayerAggregates();
}

/**
 */
void HistogramViewModel::resetIntermediateVariableData()
{
    resetHistogram();
}

/**
 */
void HistogramViewModel::resetDisplayData()
{
    display_data_.reset();
}

/**
 * Asynchronous recompute (offline), used by the load-done path and by interactive
 * recomputes: the generator update runs on a worker, the display data commits
 * on the main thread once done. On load it is launched after the layer data job of
 * updateDataEvent completed (same serialized processor; its commit installed
 * row_layer_index_ and the layer panel this reads).
 */
bool HistogramViewModel::launchAsyncRecompute()
{
    //main thread right before the generator update, asynchronous or not:
    //the worker must not read the layer tree model
    snapshotHiddenLayerIds();

    //live mutates the buffers per tick - synchronous recompute
    if (appState().appMode() == AppMode::LiveRunning)
        return false;

    if (viewData().empty())
        return false;

    asyncProcessor().launch("histogram view generator update",
        [ this ] ()
        {
            clearIntermediateData();
            updateFromVariables();
        },
        [ this ] ()
        {
            publishDisplayData();

            emit displayDataChangedSignal();
        });

    return true;
}

/**
 * All dbcontents are handled inside the histogram generator.
 */
void HistogramViewModel::updateFromVariables()
{
    loginf;

    traced_assert(numVariables() == 1);
    traced_assert(variable(0).hasVariable());

    auto& var = variable(0);

    if (viewData().empty())
    {
        computed_data_ = displayDataFromGenerator();
        return;
    }

    // a Report Variable is a column of a Report Table, joined onto its host data contents only
    dbContent::Variable*       data_var   = var.variablePtr();
    dbContent::MetaVariable*   meta_var   = var.metaVariablePtr();
    dbContent::ReportVariable* report_var = var.reportVariablePtr();

    traced_assert(meta_var || data_var || report_var);

    auto data_type = meta_var   ? meta_var->dataType()   :
                     report_var ? report_var->dataType() : data_var->dataType();

    #define UpdateFunc(PDType, DType, Suffix) \
        histogram_generator_.reset(new HistogramGeneratorBufferT<DType>(&mutableViewData(), data_var, meta_var, report_var));

    #define NotFoundFunc                                                                                                          \
        const std::string msg = "HistogramViewModel: updateFromVariables: impossible for property type " + Property::asString(data_type); \
        logerr << msg;                                                                                                            \
        throw std::runtime_error(msg);

    #define UnsupportedFunc(PDType, DType, Suffix) traced_assert(true);

    SwitchPropertyDataTypeNumeric(data_type, UpdateFunc, UnsupportedFunc, UnsupportedFunc, NotFoundFunc)

    traced_assert(histogram_generator_);

    // Build per-buffer row-layer ids once per data change (covers every row; already
    // computed on a worker for asynchronous loads). The row filter uses the full layer
    // id (panel visibility is per-layer). The layer lookup fed to the generator
    // returns the color-mode *group key* - that way the generator produces one bucket
    // per visible group (e.g. 5 in DSType mode) rather than per full-layer, and no
    // merge step is needed downstream.
    if (!row_layer_ids_valid_)
    {
        computeRowLayerIds();
        row_layer_ids_valid_ = true;
    }

    HistogramGeneratorBuffer* buf_gen =
        dynamic_cast<HistogramGeneratorBuffer*>(histogram_generator_.get());
    traced_assert(buf_gen);

    const unsigned int color_mode = appState().colorMode();

    // Everything the generator needs per row is precomputed per POOL ENTRY here - the
    // color-mode group key and the hidden flag. A row then costs one array lookup;
    // previously each row built its layer id, split it in groupKeyFor and probed two
    // string keyed maps, over millions of rows.
    // snapshot taken on the main thread (launchAsyncRecompute()), this may run on a worker
    const std::set<std::string>& hidden_layer_ids = hiddenLayerIds();

    const auto& pool = row_layer_index_.pool;

    std::vector<std::string> group_keys(pool.size());
    std::vector<char>        hidden_by_group(pool.size(), 0);

    for (size_t g = 1; g < pool.size(); ++g)
    {
        group_keys[ g ]      = groupKeyFor(pool[ g ], color_mode);
        hidden_by_group[ g ] = hidden_layer_ids.count(pool[ g ]) ? 1 : 0;
    }

    buf_gen->setRowGroups(std::move(group_keys),
        [this](const std::string& dbc, unsigned int i) -> std::uint16_t
        {
            return row_layer_index_.indexOf(dbc, i);
        });

    if (!hidden_layer_ids.empty())
    {
        buf_gen->setRowFilter(
            [this, hidden_by_group](const std::string& dbc, unsigned int i) -> bool
            {
                auto idx = row_layer_index_.indexOf(dbc, i);
                if (idx == view_layer_scan::RowLayerIndex::UnmappedIndex)
                    return true;   // unmappable rows still contribute to scan nulls
                return idx >= hidden_by_group.size() || !hidden_by_group[ idx ];
            });
    }

    histogram_generator_->update();
    //histogram_generator_->print();

    // Re-apply any zoom that was active before this regeneration, so a
    // selection-triggered recompute doesn't reset the user's zoom.
    // Cleared on data reload and on explicit resetZoom, so this only
    // fires for same-data recomputes.
    if (saved_zoom_range_.has_value())
        histogram_generator_->zoom(saved_zoom_range_->first,
                                   saved_zoom_range_->second);

    //variable missing from buffer?
    if (buf_gen->dataNotInBuffer())
        setVariableState(0, VariableState::MissingFromBuffer);

    computed_data_ = displayDataFromGenerator();

    //add to standard counts
    addNullCount(buf_gen->getResults().buffer_null_count);
    addNanCount (buf_gen->getResults().buffer_nan_count );

    loginf << "done";
}

/**
 * Creates the display data from the current generator's results. Without a result the
 * display data holds the axis label only.
 */
std::shared_ptr<const HistogramDisplayData> HistogramViewModel::displayDataFromGenerator() const
{
    auto data = std::make_shared<HistogramDisplayData>();

    data->x_axis_label = variable(0).description();

    if (!histogram_generator_ || !variablesOk() || !histogram_generator_->hasValidResult())
        return data;

    // Color map keyed by the generator's group key (DSType / DBContent /
    // Data Source / Data Source + Line - determined by color mode). All
    // full-layer payloads that collapse to the same group share the same
    // color by construction, so "first wins" is safe.
    const unsigned int color_mode = appState().colorMode();

    std::map<std::string, QColor> group_colors;
    for (const auto& layer : layerSnapshots())
        group_colors.try_emplace(groupKeyFor(layer->key(), color_mode), layer->color());

    histogram_generator_->getResults().toRaw(data->histogram, group_colors, ViewModel::ColorSelected);

    data->valid = data->histogram.hasData();

    return data;
}

/**
 */
void HistogramViewModel::selectBins(unsigned int first, unsigned int last)
{
    loginf << "first " << first << " last " << last;

    //an annotation has no buffer data to select
    if (!histogram_generator_)
        return;

    histogram_generator_->select(first, last);

    //note: triggers a view update, otherwise a manual update would be needed
    emit selectionChangedSignal();
}

/**
 */
void HistogramViewModel::zoomToBins(unsigned int first, unsigned int last)
{
    if (!histogram_generator_)
        return;

    //zoom to bin range and refill with data
    histogram_generator_->zoom(first, last);

    //remember the zoom so a subsequent recompute (e.g. selection-driven)
    //can re-apply it after the generator is rebuilt
    saved_zoom_range_ = std::make_pair(first, last);

    //update the display data
    computed_data_ = displayDataFromGenerator();
    publishDisplayData();

    emit displayDataChangedSignal();
}

/**
 */
void HistogramViewModel::resetZoom()
{
    loginf;

    //reset the preserved zoom so the upcoming recompute uses the full range
    saved_zoom_range_.reset();

    if (histogram_generator_ && histogram_generator_->subRangeActive())
        recompute();
}

/**
 */
bool HistogramViewModel::hasValidResult() const
{
    //a recompute worker replaces the generator, the main thread must not read it meanwhile
    return !hasPendingAsyncWork() && histogram_generator_ && histogram_generator_->hasValidResult();
}

/**
 */
ViewInfos HistogramViewModel::viewInfo() const
{
    ViewInfos vinfos;

    vinfos.addSection("Histogram Range");

    if (!hasValidResult())
    {
        vinfos.addInfo("info_range_min", "Minimum:"            , "-");
        vinfos.addInfo("info_range_max", "Maximum:"            , "-");
        vinfos.addInfo("info_oor_count", "Out of range values:", "-");
    }
    else
    {
        const auto  range   = histogram_generator_->currentRangeAsLabels();
        const auto& results = histogram_generator_->getResults();

        vinfos.addInfo("info_range_min", "Minimum:", range.first.empty()  ? "Not available" : range.first , range.first.empty() );
        vinfos.addInfo("info_range_max", "Maximum:", range.second.empty() ? "Not available" : range.second, range.second.empty());
        vinfos.addInfo("info_oor_count", "Out of range values:", std::to_string(results.not_inserted_count), false);

        if (histogram_generator_->subRangeActive())
            vinfos.addInfo("info_zoom_active", "", "Zoom active", true);
    }

    return vinfos;
}

/**
 */
void HistogramViewModel::viewInfoJSON(nlohmann::json& info) const
{
    bool valid = hasValidResult();

    info[ "result_valid" ] = valid;

    if (!valid)
        return;

    auto range       = histogram_generator_->currentRangeAsLabels();
    bool zoom_active = histogram_generator_->subRangeActive();

    const auto& results     = histogram_generator_->getResults();
    const auto& dbc_results = results.content_results;

    auto obtainRanges = [ & ] ()
    {
        if (dbc_results.size() == 0)
            return std::vector<std::string>();

        const auto& bins = dbc_results.begin()->second.bins;
        if (bins.size() == 0)
            return std::vector<std::string>();

        std::vector<std::string> ranges;
        for (const auto& bin : bins)
            ranges.push_back(bin.labels.label_min);
        ranges.push_back(bins.rbegin()->labels.label_max);

        return ranges;
    };
    UNUSED_VARIABLE(obtainRanges);

    info[ "result_range_min"      ] = range.first;
    info[ "result_range_max"      ] = range.second;
    info[ "result_zoom_active"    ] = zoom_active;
    info[ "result_num_bins"       ] = histogram_generator_->currentBins();
    info[ "result_oor_count"      ] = results.not_inserted_count;
    info[ "result_null_count"     ] = results.null_count;
    info[ "result_null_sel_count" ] = results.null_selected_count;
    info[ "result_sel_count"      ] = results.selected_count;
    info[ "result_valid_count"    ] = results.valid_count;
    //info[ "result_counts"         ] = results.valid_counts;
    //info[ "result_sel_counts"     ] = results.selected_counts;
    info[ "result_max_count"      ] = results.max_count;
    info[ "result_discrete"       ] = dbc_results.size() > 0 ? dbc_results.begin()->second.bins_are_categories : false;
    //info[ "result_ranges"         ] = obtainRanges();
}

/**
 * Notice: the cached aggregation is reused while the data is unchanged (e.g. a color mode
 * change), on the asynchronous load path the scan runs on a worker, see updateDataEvent.
 */
void HistogramViewModel::rebuildLayerTree()
{
    if (!hasLayerPanel())
        return;

    applyLayerTree(layerAggregates());
}

/**
 * The tree part of rebuildLayerTree, from precomputed layer aggregates.
 */
void HistogramViewModel::applyLayerTree(const LayerAggregates& agg)
{
    if (!hasLayerPanel())
        return;

    updateLayerTree(snapshotsFromAggregates(agg));
}

/**
 */
void HistogramViewModel::computeRowLayerIds()
{
    // shared scan helper (worker version runs in updateDataEvent's task)
    row_layer_index_ = view_layer_scan::computeRowLayerIndex(
        view_layer_scan::makeScanInput(viewData(), dbContentManager(), dbContextManager()));
}
