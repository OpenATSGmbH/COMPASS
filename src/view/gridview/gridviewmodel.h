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
#include "griddisplaydata.h"
#include "grid2d_defs.h"

#include <QRectF>

#include <functional>
#include <memory>
#include <string>
#include <vector>

class Grid2D;

/**
 * Model of the Grid View. Builds the GridDisplayData, from the buffers (stash) or from an annotation.
 * Holds the outlines of the sectors of the data context and follows their changes.
 */
class GridViewModel : public VariableViewStashModel
{
public:
    /// Inputs of the buffer path that come from the settings of the view.
    struct Settings
    {
        grid2d::ValueType         value_type       = grid2d::ValueType::ValueTypeCountValid;
        unsigned int              grid_resolution  = 50;
        PropertyDataType          legend_data_type = PropertyDataType::DOUBLE;
        dbContent::Representation legend_repr      = dbContent::Representation::STANDARD;
        bool                      lonlat_axes      = false; // x and y are longitude and latitude
    };
    typedef std::function<Settings()> SettingsFunc;
    /// the text labels of the shown view point, see textLabels()
    typedef std::function<std::vector<GridTextLabel>()> LabelsFunc;
    /// the points and lines of the shown view point, see overlays()
    typedef std::function<std::vector<GridOverlay>()> OverlaysFunc;

    GridViewModel(DBContentManager& dbcont_man, context::DBContextManager& ctx_man, AppState& app_state);
    virtual ~GridViewModel();

    /// read on the main thread before each computation of the display data from the buffers
    void setSettingsFunc(const SettingsFunc& func) { settings_func_ = func; }
    /// called when a geographic grid annotation is set
    void setLabelsFunc(const LabelsFunc& func) { labels_func_ = func; }
    /// called when a geographic grid annotation is set
    void setOverlaysFunc(const OverlaysFunc& func) { overlays_func_ = func; }

    /// the variables of x, y and the distributed value, NOT owned
    void setVariables(ViewVariable& variable_x, ViewVariable& variable_y, ViewVariable& variable_z);

    /// the outlines of all sectors of the data context
    static std::vector<GridSectorLine> sectorLines(const context::DBContextManager& ctx_man);

    /// the text features of the view point, the labels of each geographic grid of it
    static std::vector<GridTextLabel> textLabels(const nlohmann::json& view_point);
    /// the points, lines and line strings of the view point, in the order of its annotations
    static std::vector<GridOverlay> overlays(const nlohmann::json& view_point);
    /// the background set of the view point (ViewPointGenFeatureGrid::ViewPointFieldNameBackground),
    /// no value if it has none or it is not readable
    static boost::optional<GridBackgroundSettings> backgroundFromViewPoint(const nlohmann::json& view_point);

    /// Annotation path, no model object and no manager necessary. sectors: see sectorLines(). The sectors,
    /// labels, overlays and background apply to a geographic grid only.
    /// Empty pointer: malformed annotation, error then holds the reason.
    static std::shared_ptr<const GridDisplayData> displayDataFromAnnotation(
        const nlohmann::json& feature,
        const PlotMetadata& metadata,
        const std::vector<GridSectorLine>& sectors = {},
        const std::vector<GridTextLabel>& labels = {},
        const std::vector<GridOverlay>& overlays = {},
        const boost::optional<GridBackgroundSettings>& background = boost::none,
        std::string* error = nullptr);

    /// A geographic map without grid: the overlays and labels on the background, over the area they cover.
    /// Not valid (the empty chart) without a position.
    static std::shared_ptr<const GridDisplayData> mapDisplayData(
        const std::vector<GridSectorLine>& sectors,
        const std::vector<GridTextLabel>& labels,
        const std::vector<GridOverlay>& overlays,
        const boost::optional<GridBackgroundSettings>& background = boost::none);

    bool setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata,
                       std::string* error = nullptr) override;

    /// from the buffers or from the annotation
    std::shared_ptr<const GridDisplayData> displayData() const { return display_data_; }

    /// the display data has a grid with cells
    bool hasValidGrid() const;

    /// A database was opened or closed: a display without grid follows the position extent of the database.
    void databaseChanged();

    /// x / y range of the loaded values, no value without loaded values
    boost::optional<QRectF> dataBoundsXY() const;

    void viewInfoJSON(nlohmann::json& info) const override;

    static const double RangeMinDefault;

protected:
    void resetDisplayData() override final;
    bool updateFromAnnotation() override final;
    void publishDisplayData() override final;

    void prepareStashProcessing() override final;
    void processStash(const VariableViewStash<double>& stash) override final;
    void resetStashDependentData() override final;

    /// the grid is built again from the stash, only the checked layers contribute
    void layerVisibilityChanged() override final;

private:
    typedef std::shared_ptr<GridDisplayData> DataPtr;

    static DataPtr dataFromAnnotation(const nlohmann::json& feature,
                                      const PlotMetadata& metadata,
                                      std::string* error);

    /// display data without grid, with the axis labels and the axis info of the variables
    DataPtr dataFromVariables() const;
    std::string variableDomainId() const;
    QRectF databasePositionExtent() const;

    void readSettings();

    /// Builds the grid from the stash, only the groups that are not hidden in the layer panel
    /// (snapshot hiddenLayerIds()). Touches neither the stash nor the layer tree, so it can run on
    /// a layer toggle. Can run on a worker.
    void buildGridFromStash();

    boost::optional<QRectF> getPlanarVariableBounds(int var_x, int var_y, bool fix_small_ranges) const;
    boost::optional<std::pair<double, double>> getVariableBounds(int var, bool fix_small_ranges) const;

    /// the data with the current sector lines becomes the display data
    void publish(const DataPtr& data);
    /// the grid of the stash as display data
    void publishGrid();
    void sectorsChanged();

    SettingsFunc settings_func_;
    Settings     settings_; // read on the main thread, see readSettings()
    LabelsFunc   labels_func_;
    OverlaysFunc overlays_func_;

    std::unique_ptr<Grid2D>       grid_;       // of the stash, for the view info
    boost::optional<Grid2DLayer>  grid_layer_; // of the stash

    // built by processStash() (may run on a worker), current after publishDisplayData()
    // (the layer tree leaves reference the layer snapshots)
    boost::optional<LayerSnapshots> pending_layers_;

    std::shared_ptr<const GridDisplayData> display_data_;    // published, main thread only
    std::shared_ptr<const GridDisplayData> annotation_data_; // of the annotation set last

    std::vector<GridSectorLine> sectors_;
};
