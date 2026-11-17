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

#include "variableviewmodel.h"
#include "histogramdisplaydata.h"
#include "viewlayerscan.h"
#include "viewinfo.h"

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <utility>

class HistogramGenerator;

/**
 * Model of the Histogram View. Builds the HistogramDisplayData, from the buffers (histogram generator)
 * or from an annotation.
 */
class HistogramViewModel : public VariableViewModel
{
public:
    HistogramViewModel(DBContentManager& dbcont_man, context::DBContextManager& ctx_man, AppState& app_state);
    virtual ~HistogramViewModel();

    /// the variable of the view (data to bin, axis label), NOT owned
    void setVariable(ViewVariable& variable);

    /// Annotation path, no model object and no manager necessary. Empty pointer: malformed annotation,
    /// error then holds the reason.
    static std::shared_ptr<const HistogramDisplayData> displayDataFromAnnotation(const nlohmann::json& feature,
                                                                                const PlotMetadata& metadata,
                                                                                std::string* error = nullptr);

    bool setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata,
                       std::string* error = nullptr) override;

    /// from the buffers or from the annotation, an empty pointer after a clear
    std::shared_ptr<const HistogramDisplayData> displayData() const { return display_data_; }

    /// Bin indices as HistogramViewRenderWidget::binRange() gives them.
    void zoomToBins(unsigned int first, unsigned int last);
    void resetZoom();
    void selectBins(unsigned int first, unsigned int last);

    ViewInfos viewInfo() const;

    /// the histogram of the buffers is computed
    bool hasValidResult() const;
    void viewInfoJSON(nlohmann::json& info) const override;

protected:
    // load-time asynchronous recompute: the generator update runs on a worker (it
    // competes with the other views' workers and can exceed the ping budget on the
    // main thread otherwise), the display data commits on the main thread
    bool launchAsyncRecompute() override final;

    void updateDataEvent(bool requires_reset) override final;
    void resetVariableData() override final;
    void resetIntermediateVariableData() override final;
    void resetDisplayData() override final;
    void updateFromVariables() override final;
    bool updateFromAnnotation() override final;
    void publishDisplayData() override final;

    /// layer snapshots from the (cached) layer aggregation of the loaded buffers
    void rebuildLayerTree() override final;
    /// hidden layers are left out => recompute
    void layerVisibilityChanged() override final;
    /// other layer colors => snapshots and display data
    void colorModeChanged() override final;

private:
    void resetHistogram();
    /// display data from the current generator's results
    std::shared_ptr<const HistogramDisplayData> displayDataFromGenerator() const;

    /// layer snapshots + data subtree from layer aggregates (the scan may run on a worker)
    void applyLayerTree(const LayerAggregates& agg);

    /// Populate row_layer_index_ from the current buffers. Rows without ds_id/line_id are unmapped.
    void computeRowLayerIds();

    std::unique_ptr<HistogramGenerator> histogram_generator_;

    std::shared_ptr<const HistogramDisplayData> computed_data_;   // result of the last generator update
    std::shared_ptr<const HistogramDisplayData> display_data_;    // published, main thread only
    std::shared_ptr<const HistogramDisplayData> annotation_data_; // of the annotation set last
    bool                                        annotation_valid_ = false;

    /// Current zoom bin range [first, second], valid only while zoom is
    /// active. Captured on zoomToBins and re-applied after a generator
    /// regeneration (e.g. a selection-triggered recompute) so the
    /// user's zoom survives. Cleared on data reload and on resetZoom.
    boost::optional<std::pair<unsigned int, unsigned int>> saved_zoom_range_;

    /// Per-row layer id as a pool index, computed once per data change (on a worker for
    /// asynchronous loads, see updateDataEvent). Two bytes per row instead of a string -
    /// with millions of rows the string form costs one heap allocation each, and made
    /// every generator row do string work. Referenced via closures passed as the
    /// HistogramGeneratorBuffer row filter + group lookup, so this must outlive the
    /// generator.
    view_layer_scan::RowLayerIndex row_layer_index_;

    /// row_layer_index_ matches the current data (invalidated on data change/clear,
    /// NOT per recompute - the ids only depend on the loaded buffers; the base's layer
    /// aggregation cache follows the same validity)
    bool row_layer_ids_valid_ {false};
};
