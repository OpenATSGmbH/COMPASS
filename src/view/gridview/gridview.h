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

#include "variableview.h"
#include "representation.h"
#include "property.h"
#include "grid2d_defs.h"
#include "colormap_defs.h"
#include "gridbackground_defs.h"

#include <boost/optional.hpp>

#include <set>

class GridViewWidget;
class GridViewDataSource;
class GridViewDataWidget;
class ViewContainer;

/**
*/
class GridView : public VariableView
{
public:
    struct Settings
    {
        Settings();

        int          value_type;

        unsigned int grid_resolution;
        unsigned int max_grid_resolution;

        std::string  render_color_value_min;
        std::string  render_color_value_max;
        int          render_color_scale;
        unsigned int render_color_num_steps;
        bool         render_color_log_scale;

        unsigned int grid_opacity; // percent

        // background map (longitude / latitude grids), see gridbackground_defs.h
        bool           background_show;
        std::string    background_color;
        bool           background_map_show;
        std::string    background_map_file;
        unsigned int   background_map_opacity; // percent
        bool           background_borders_show;
        bool           background_sectors_show;
        std::string    background_sectors_hidden_layers; // JSON array of sector layer names
        std::string    background_sectors_hidden;        // JSON array of sector ids
    };

    enum class Variable
    {
        DataVarX = 0,
        DataVarY,
        DataVarZ
    };

    GridView(nlohmann::json& config, ViewContainer* parent);
    virtual ~GridView() override;

    void setValueType(grid2d::ValueType type, bool notify_changes);
    void setGridResolution(unsigned int n, bool notify_changes);
    void setPixelsPerCell(unsigned int n, bool notify_changes);
    void setColorScale(colorscale::ColorScale scale, bool notify_changes);
    void setColorSteps(unsigned int n, bool notify_changes);
    void setColorLogScale(bool log_scale, bool notify_changes);
    void setMinValue(const std::string& value_str, bool notify_changes);
    void setMaxValue(const std::string& value_str, bool notify_changes);

    void setGridOpacity(unsigned int percent);

    // background map, the setters update the display (no recompute)
    void setBackgroundShow(bool show);
    void setBackgroundColor(const std::string& color);
    void setBackgroundMapShow(bool show);
    void setBackgroundMapFile(const std::string& file_name);
    void setBackgroundMapOpacity(unsigned int percent);
    void setBackgroundBordersShow(bool show);
    void setBackgroundSectorsShow(bool show);
    void setBackgroundSectorLayerShow(const std::string& layer_name, bool show);
    bool backgroundSectorLayerShown(const std::string& layer_name) const;
    /// a renamed sector layer keeps its checked state
    void renameBackgroundSectorLayer(const std::string& old_name, const std::string& new_name);
    void setBackgroundSectorShow(unsigned int sector_id, bool show);
    bool backgroundSectorShown(unsigned int sector_id) const;
    std::set<std::string>  backgroundHiddenSectorLayers() const;
    std::set<unsigned int> backgroundHiddenSectors() const;

    /// true if the grid is computed over longitude (x) and latitude (y) of the variables
    bool hasLonLatAxes();

    boost::optional<double> getMinValue() const;
    boost::optional<double> getMaxValue() const;

    PropertyDataType currentDataType() const; 
    PropertyDataType currentLegendDataType() const; 

    /// Representation the legend labels should use. A count or a statistic is
    /// no longer the variable itself, so it carries no representation.
    dbContent::Representation currentLegendRepresentation();

    virtual void accept(LatexVisitor& v) override;

    virtual bool canShowAnnotations() const override final { return true; }
    virtual std::set<std::string> acceptedAnnotationFeatureTypes() const override;

    const GridViewDataWidget* getDataWidget() const;

    const Settings& settings() const { return settings_; }

    static const int DecimalsDefault;

protected:
    friend class LatexVisitor;

    virtual void checkSubConfigurables() override;
    virtual void updateSelection_impl() override;

    virtual void preVariableChangedEvent(int idx, const std::string& dbcont, const std::string& name) override;
    virtual void postVariableChangedEvent(int idx) override;

    virtual bool init_impl() override;
    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;
    virtual void onConfigurationChanged_impl(const std::vector<std::string>& changed_params) override;

    void backgroundChanged();

    virtual dbContent::VariableSet getBaseSet(const std::string& dbcontent_name) override;

    void updateSettingsFromVariable();
    void updateSettings(const std::string& dbcont, const std::string& name);

    GridViewDataWidget* getDataWidget();

    GridViewWidget* widget_ = nullptr;

    Settings settings_;

    PropertyDataType prop_data_type_before_change_;
};
