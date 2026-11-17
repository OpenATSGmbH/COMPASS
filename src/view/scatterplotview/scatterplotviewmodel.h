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

#include "variableviewstashmodel.h"
#include "scatterplotdisplaydata.h"

#include <memory>
#include <string>

/**
 * Model of the Scatterplot View. Builds the ScatterPlotDisplayData, from the buffers (stash)
 * or from an annotation.
 */
class ScatterPlotViewModel : public VariableViewStashModel
{
public:
    ScatterPlotViewModel(DBContentManager& dbcont_man, context::DBContextManager& ctx_man, AppState& app_state);
    virtual ~ScatterPlotViewModel();

    /// the variables of the two axes (data, axis labels, tick label format), NOT owned
    void setVariables(ViewVariable& variable_x, ViewVariable& variable_y);

    /// Annotation path, no model object and no manager necessary. Empty pointer: malformed annotation,
    /// error then holds the reason.
    static std::shared_ptr<const ScatterPlotDisplayData> displayDataFromAnnotation(const nlohmann::json& feature,
                                                                                  const PlotMetadata& metadata,
                                                                                  std::string* error = nullptr);

    bool setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata,
                       std::string* error = nullptr) override;

    /// from the buffers or from the annotation, an empty pointer after a clear
    std::shared_ptr<const ScatterPlotDisplayData> displayData() const { return display_data_; }

    /// Selects the loaded values inside the rectangle (chart coordinates). add_to_selection keeps
    /// the selection from before.
    void selectRectangle(const QRectF& chart_rect, bool add_to_selection);

    /// adds the bounds of the display data to the info of the stash
    void viewInfoJSON(nlohmann::json& info) const override;

protected:
    void resetDisplayData() override final;
    bool updateFromAnnotation() override final;
    void publishDisplayData() override final;

    void processStash(const VariableViewStash<double>& stash) override final;
    void resetStashDependentData() override final;

    /// Selected points are pooled into the "Selected" overlay series and shown
    /// under that layer panel id, not under their own group id.
    std::string selectionLayerId() const override final;

    /// layer snapshots from the series of the display data
    void rebuildLayerTree() override final;
    /// the display data gets the checked state of the layers
    void layerVisibilityChanged() override final;
    /// nothing, the own color mode recompute rebuilds the layer tree
    void colorModeChanged() override final;

private:
    typedef std::shared_ptr<ScatterPlotDisplayData> DataPtr;

    static DataPtr dataFromAnnotation(const nlohmann::json& feature,
                                      const PlotMetadata& metadata,
                                      std::string* error);

    /// display data without series, with the axis labels and the axis info of the variables
    DataPtr dataFromVariables() const;
    /// identity of the two axis variables
    std::string variableDomainId() const;

    /// The data becomes the display data: the layer tree follows its series, and the series get the
    /// checked state of the layers. content_ok: the content can be shown (variables ok, valid annotation).
    void publish(const DataPtr& data, bool content_ok);
    void updateLayerTreeFrom(const ScatterPlotDisplayData& data);
    /// true if a series has another visible flag than its layer
    bool layerVisibilityDiffers(const ScatterPlotDisplayData& data) const;
    void applyLayerVisibility(ScatterPlotDisplayData& data) const;
    /// new display data if the checked state of a layer differs from its series
    void republishLayerVisibility();

    DataPtr computed_data_;                                      // result of the last stash processing
    std::shared_ptr<const ScatterPlotDisplayData> display_data_; // published, main thread only

    std::shared_ptr<const ScatterPlotDisplayData> annotation_data_; // of the annotation set last
    bool                                          annotation_valid_ = false;

    bool content_ok_ = false; // of the display data, see publish()
};
