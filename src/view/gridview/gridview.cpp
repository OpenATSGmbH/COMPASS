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

#include "gridview.h"
#include "viewcontainer.h"
#include "gridviewdatawidget.h"
#include "gridviewwidget.h"
#include "viewvariable.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "property_templates.h"

#include "viewpointgenerator.h"

#include "dbcontent.h"
#include "dbcontentmanager.h"
#include "viewmanager.h"

#include "json.hpp"

#include <set>

#include "logger.h"
#include "latexvisitor.h"

#include <QApplication>

/**
*/
GridView::Settings::Settings() 
:   value_type            (grid2d::ValueType::ValueTypeCountValid)
,   grid_resolution       (50       )
,   max_grid_resolution   (2000       )
,   render_color_value_min(""       )
,   render_color_value_max(""       )
,   render_color_scale    ((int)colorscale::ColorScale::Green2Red)
,   render_color_num_steps(10       )
,   render_color_log_scale(false    )
,   grid_opacity          (100      )
,   background_show                 (true                    )
,   background_color                ("#FFFFFF"               )
,   background_map_show             (false                   ) // offline PCs never wait for tiles by default
,   background_map_file             ("openstreetmap_2d.earth")
,   background_map_opacity          (100                     )
,   background_borders_show         (true                    )
,   background_sectors_show         (true                    )
,   background_sectors_hidden_layers("[]"                    )
,   background_sectors_hidden       ("[]"                    )
{
}

const int GridView::DecimalsDefault = 3;

/**
*/
GridView::GridView(nlohmann::json& config, ViewContainer* parent)
:   VariableView(config, parent)
{
    registerParameter("value_type", &settings_.value_type, Settings().value_type);
    registerParameter("grid_resolution", &settings_.grid_resolution, Settings().grid_resolution);
    registerParameter("max_grid_resolution", &settings_.max_grid_resolution, Settings().max_grid_resolution);
    registerParameter("render_color_scale", &settings_.render_color_scale, Settings().render_color_scale);
    registerParameter("render_color_num_steps", &settings_.render_color_num_steps, Settings().render_color_num_steps);
    registerParameter("render_color_log_scale", &settings_.render_color_log_scale, Settings().render_color_log_scale);
    registerParameter("render_color_value_min", &settings_.render_color_value_min, Settings().render_color_value_min);
    registerParameter("render_color_value_max", &settings_.render_color_value_max, Settings().render_color_value_max);
    registerParameter("grid_opacity", &settings_.grid_opacity, Settings().grid_opacity);

    registerParameter("background_show", &settings_.background_show, Settings().background_show);
    registerParameter("background_color", &settings_.background_color, Settings().background_color);
    registerParameter("background_map_show", &settings_.background_map_show, Settings().background_map_show);
    registerParameter("background_map_file", &settings_.background_map_file, Settings().background_map_file);
    registerParameter("background_map_opacity", &settings_.background_map_opacity, Settings().background_map_opacity);
    registerParameter("background_borders_show", &settings_.background_borders_show, Settings().background_borders_show);
    registerParameter("background_sectors_show", &settings_.background_sectors_show, Settings().background_sectors_show);
    registerParameter("background_sectors_hidden_layers", &settings_.background_sectors_hidden_layers, Settings().background_sectors_hidden_layers);
    registerParameter("background_sectors_hidden", &settings_.background_sectors_hidden, Settings().background_sectors_hidden);

    const std::vector<PropertyDataType> valid_types_xy = { PropertyDataType::BOOL,
                                                           PropertyDataType::CHAR,
                                                           PropertyDataType::UCHAR,
                                                           PropertyDataType::INT,
                                                           PropertyDataType::UINT,
                                                           PropertyDataType::LONGINT,
                                                           PropertyDataType::ULONGINT,
                                                           PropertyDataType::FLOAT,
                                                           PropertyDataType::DOUBLE,
                                                           PropertyDataType::TIMESTAMP };
    const std::vector<PropertyDataType> valid_types_z =  { PropertyDataType::BOOL,
                                                           PropertyDataType::CHAR,
                                                           PropertyDataType::UCHAR,
                                                           PropertyDataType::INT,
                                                           PropertyDataType::UINT,
                                                           PropertyDataType::LONGINT,
                                                           PropertyDataType::ULONGINT,
                                                           PropertyDataType::FLOAT,
                                                           PropertyDataType::DOUBLE };

    addVariable("data_var_x", "X"          , "x", META_OBJECT_NAME, dbcontent_vars::meta_var_longitude_.name(), true, true, false, valid_types_xy);
    addVariable("data_var_y", "Y"          , "y", META_OBJECT_NAME, dbcontent_vars::meta_var_latitude_.name() , true, true, false, valid_types_xy);
    addVariable("data_var_z", "Distributed", "z", ""              , ""                                   , true, true, true , valid_types_z );

    updateSettingsFromVariable();

    // create sub done in init
}

/**
*/
GridView::~GridView()
{
    logdbg;

    if (widget_)
    {
        delete widget_;
        widget_ = nullptr;
    }
}

/**
*/
bool GridView::init_impl()
{
    createSubConfigurables();

    return true;
}

/**
*/
void GridView::checkSubConfigurables()
{
    if (!widget_)
    {
        widget_ = new GridViewWidget(this, central_widget_);
        setWidget(widget_);
    }
}

/**
*/
GridViewDataWidget* GridView::getDataWidget()
{
    traced_assert(widget_);
    return widget_->getViewDataWidget();
}

/**
*/
const GridViewDataWidget* GridView::getDataWidget() const
{
    traced_assert(widget_);
    return widget_->getViewDataWidget();
}

/**
*/
dbContent::VariableSet GridView::getBaseSet(const std::string& dbcontent_name)
{
    return dbContent::VariableSet();
}

/**
*/
void GridView::accept(LatexVisitor& v)
{
    v.visit(this);
}

/**
*/
void GridView::updateSelection_impl()
{
    loginf;
    traced_assert(widget_);

    widget_->getViewDataWidget()->redrawData(true);
}

/**
 */
void GridView::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableView::viewInfoJSON_impl(info);

    //@TODO?
}

/**
 */
void GridView::setValueType(grid2d::ValueType type, bool notify_changes)
{
    if (settings_.value_type == type)
        return;

    auto data_type_before_change = currentDataType();

    setParameter(settings_.value_type, (int)type);

    //data type changed => reset min max values (otherwise they stay valid and are kept)
    if (data_type_before_change != currentDataType())
    {
        setParameter(settings_.render_color_value_min, std::string(""));
        setParameter(settings_.render_color_value_max, std::string(""));
    }

    if (notify_changes)
        notifyRefreshNeeded();
}

/**
*/
void GridView::setGridResolution(unsigned int n, bool notify_changes)
{
    if (settings_.grid_resolution == n)
        return;

    setParameter(settings_.grid_resolution, n);

    if (notify_changes)
        notifyRefreshNeeded();
}

/**
*/
void GridView::setColorScale(colorscale::ColorScale scale, bool notify_changes)
{
    if (settings_.render_color_scale == (int)scale)
        return;

    setParameter(settings_.render_color_scale, (int)scale);

    if (notify_changes)
        notifyRefreshNeeded();
}

/**
*/
void GridView::setColorSteps(unsigned int n, bool notify_changes)
{
    if (settings_.render_color_num_steps == n)
        return;

    setParameter(settings_.render_color_num_steps, n);

    if (notify_changes)
        notifyRefreshNeeded();
}

/**
*/
void GridView::setColorLogScale(bool log_scale, bool notify_changes)
{
    if (settings_.render_color_log_scale == log_scale)
        return;

    setParameter(settings_.render_color_log_scale, log_scale);

    if (notify_changes)
        notifyRefreshNeeded();
}

/**
 */
void GridView::setMinValue(const std::string& value_str, bool notify_changes)
{
    if (settings_.render_color_value_min == value_str)
        return;

    setParameter(settings_.render_color_value_min, value_str);
    
    if (notify_changes)
        notifyRefreshNeeded();
}

/**
 */
void GridView::setMaxValue(const std::string& value_str, bool notify_changes)
{
    if (settings_.render_color_value_max == value_str)
        return;

    setParameter(settings_.render_color_value_max, value_str);
    
    if (notify_changes)
        notifyRefreshNeeded();
}

/**
 */
void GridView::setGridOpacity(unsigned int percent)
{
    percent = std::min(100u, percent);

    if (settings_.grid_opacity == percent)
        return;

    setParameter(settings_.grid_opacity, percent);
    backgroundChanged();
}

/**
 */
void GridView::setBackgroundShow(bool show)
{
    if (settings_.background_show == show)
        return;

    setParameter(settings_.background_show, show);
    backgroundChanged();
}

/**
 */
void GridView::setBackgroundColor(const std::string& color)
{
    if (settings_.background_color == color)
        return;

    setParameter(settings_.background_color, color);
    backgroundChanged();
}

/**
 */
void GridView::setBackgroundMapShow(bool show)
{
    if (settings_.background_map_show == show)
        return;

    setParameter(settings_.background_map_show, show);
    backgroundChanged();
}

/**
 */
void GridView::setBackgroundMapFile(const std::string& file_name)
{
    if (settings_.background_map_file == file_name)
        return;

    setParameter(settings_.background_map_file, file_name);
    backgroundChanged();
}

/**
 */
void GridView::setBackgroundMapOpacity(unsigned int percent)
{
    percent = std::min(100u, percent);

    if (settings_.background_map_opacity == percent)
        return;

    setParameter(settings_.background_map_opacity, percent);
    backgroundChanged();
}

/**
 */
void GridView::setBackgroundBordersShow(bool show)
{
    if (settings_.background_borders_show == show)
        return;

    setParameter(settings_.background_borders_show, show);
    backgroundChanged();
}

/**
 */
void GridView::setBackgroundSectorsShow(bool show)
{
    if (settings_.background_sectors_show == show)
        return;

    setParameter(settings_.background_sectors_show, show);
    backgroundChanged();
}

namespace
{
    std::set<std::string> parseLayerNames(const std::string& json_str)
    {
        std::set<std::string> names;

        auto j = nlohmann::json::parse(json_str, nullptr, false);

        if (j.is_array())
            for (const auto& entry : j)
                if (entry.is_string())
                    names.insert(entry.get<std::string>());

        return names;
    }
}

/**
 */
void GridView::setBackgroundSectorLayerShow(const std::string& layer_name, bool show)
{
    auto hidden = parseLayerNames(settings_.background_sectors_hidden_layers);

    if ((hidden.count(layer_name) == 0) == show)
        return;

    if (show)
        hidden.erase(layer_name);
    else
        hidden.insert(layer_name);

    setParameter(settings_.background_sectors_hidden_layers, nlohmann::json(hidden).dump());
    backgroundChanged();
}

/**
 */
bool GridView::backgroundSectorLayerShown(const std::string& layer_name) const
{
    return parseLayerNames(settings_.background_sectors_hidden_layers).count(layer_name) == 0;
}

/**
 */
void GridView::renameBackgroundSectorLayer(const std::string& old_name, const std::string& new_name)
{
    auto hidden = parseLayerNames(settings_.background_sectors_hidden_layers);

    if (!hidden.count(old_name))
        return;

    hidden.erase(old_name);
    hidden.insert(new_name);

    setParameter(settings_.background_sectors_hidden_layers, nlohmann::json(hidden).dump());
}

/**
 */
std::set<std::string> GridView::backgroundHiddenSectorLayers() const
{
    return parseLayerNames(settings_.background_sectors_hidden_layers);
}

/**
 */
std::set<unsigned int> GridView::backgroundHiddenSectors() const
{
    std::set<unsigned int> ids;

    auto j = nlohmann::json::parse(settings_.background_sectors_hidden, nullptr, false);

    if (j.is_array())
        for (const auto& entry : j)
            if (entry.is_number_unsigned())
                ids.insert(entry.get<unsigned int>());

    return ids;
}

/**
 */
void GridView::setBackgroundSectorShow(unsigned int sector_id, bool show)
{
    auto hidden = backgroundHiddenSectors();

    if ((hidden.count(sector_id) == 0) == show)
        return;

    if (show)
        hidden.erase(sector_id);
    else
        hidden.insert(sector_id);

    setParameter(settings_.background_sectors_hidden, nlohmann::json(hidden).dump());
    backgroundChanged();
}

/**
 */
bool GridView::backgroundSectorShown(unsigned int sector_id) const
{
    return backgroundHiddenSectors().count(sector_id) == 0;
}

/**
 * Notice: the setters only change the display, no recompute.
 */
void GridView::backgroundChanged()
{
    if (widget_)
        getDataWidget()->backgroundSettingsChanged();
}

/**
 */
void GridView::onConfigurationChanged_impl(const std::vector<std::string>& changed_params)
{
    (void)changed_params;

    backgroundChanged();
}

/**
 * A variable is longitude / latitude if it is the meta variable, or the DBContent variable the meta
 * variable maps to.
 */
bool GridView::hasLonLatAxes()
{
    auto isVar = [ & ] (int idx, const std::string& meta_name)
    {
        const auto& view_var = variable(idx);

        if (view_var.isEmpty())
            return false;

        if (view_var.isMetaVariable())
            return view_var.variableDBContent() == META_OBJECT_NAME && view_var.variableName() == meta_name;

        if (!view_var.hasVariable())
            return false;

        auto& meta_var = viewManager().dbContentManager().metaVariable(meta_name);
        const auto& dbcont = view_var.variableDBContent();

        return meta_var.existsIn(dbcont) && meta_var.getFor(dbcont).name() == view_var.variableName();
    };

    return isVar(0, dbcontent_vars::meta_var_longitude_.name()) &&
           isVar(1, dbcontent_vars::meta_var_latitude_.name());
}

/**
 */
void GridView::preVariableChangedEvent(int idx, const std::string& dbcont, const std::string& name)
{
    if (idx == 2)
    {
        //remember distributed var data type before var change
        prop_data_type_before_change_ = currentDataType();
    }
}

/**
 */
void GridView::postVariableChangedEvent(int idx)
{
    //x / y => background available or not, a kept zoom / position no longer fits
    if ((idx == 0 || idx == 1) && widget_)
    {
        getDataWidget()->clearUserView();
        backgroundChanged();
    }

    if (idx == 2)
    {
        //check if distributed var data type changed
        if (prop_data_type_before_change_ != currentDataType())
        {
            //change => reset min max values
            setParameter(settings_.render_color_value_min, std::string(""));
            setParameter(settings_.render_color_value_max, std::string(""));
        }

        updateSettingsFromVariable();
    }
}

/**
 */
void GridView::updateSettingsFromVariable()
{
    const auto& var = variable(2);

    updateSettings(var.settings().data_var_dbcont, var.settings().data_var_name);
}

/**
 */
void GridView::updateSettings(const std::string& dbcont, const std::string& name)
{
    bool is_empty = dbcont.empty() && name.empty();

    if (is_empty)
    {
        loginf << "settings distributed variable to empty";

        //set special settings for empty variable
        setValueType(grid2d::ValueType::ValueTypeCountValid, false);
        setMinValue("", false);
        setMaxValue("", false);
    }
}

/**
 */
boost::optional<double> GridView::getMinValue() const
{
    if (settings_.render_color_value_min.empty())
        return {};

    auto dtype = currentDataType();

    return property_templates::string2Double(dtype, settings_.render_color_value_min);
}

/**
 */
boost::optional<double> GridView::getMaxValue() const
{
    if (settings_.render_color_value_max.empty())
        return {};

    auto dtype = currentDataType();

    return property_templates::string2Double(dtype, settings_.render_color_value_max);
}

/**
 */
PropertyDataType GridView::currentDataType() const
{
    //if annotation is shown, we always use double
    //@TODO: this should be configurable
    if (showsAnnotation())
        return PropertyDataType::DOUBLE;

    auto data_type = variable(2).dataType();

    //counts are active => always override data type
    if (!data_type.has_value() ||
        settings_.value_type == (int)grid2d::ValueType::ValueTypeCountValid ||
        settings_.value_type == (int)grid2d::ValueType::ValueTypeCountNan   ||
        settings_.value_type == (int)grid2d::ValueType::ValueTypeCountValid)
    {
        return PropertyDataType::UINT;
    }

    //in all other cases the data type of the distributed variable should be the right one
    return data_type.value();
}

/**
*/
PropertyDataType GridView::currentLegendDataType() const
{
    //if annotation is shown, we always use double
    //@TODO: this should be configurable
    if (showsAnnotation())
        return PropertyDataType::DOUBLE;

    auto data_type = variable(2).dataType();

    //counts are active => always override data type
    if (!data_type.has_value() ||
        settings_.value_type == (int)grid2d::ValueType::ValueTypeCountValid ||
        settings_.value_type == (int)grid2d::ValueType::ValueTypeCountNan ||
        settings_.value_type == (int)grid2d::ValueType::ValueTypeCountValid)
    {
        return PropertyDataType::UINT;
    }
    else if (settings_.value_type == (int)grid2d::ValueType::ValueTypeMean ||
             settings_.value_type == (int)grid2d::ValueType::ValueTypeStddev ||
             settings_.value_type == (int)grid2d::ValueType::ValueTypeVar)
    {
        return PropertyDataType::DOUBLE;
    }

    //in all other cases the data type of the distributed variable should be the right one
    return data_type.value();
}

/**
*/
dbContent::Representation GridView::currentLegendRepresentation()
{
    //an annotation brings values, not variables
    if (showsAnnotation())
        return dbContent::Representation::STANDARD;

    //a count or a statistic of a variable is not that variable any more, so a
    //Mode 3/A standard deviation must not read as an octal code
    if (settings_.value_type != (int)grid2d::ValueTypeMin &&
        settings_.value_type != (int)grid2d::ValueTypeMax)
        return dbContent::Representation::STANDARD;

    auto& view_var = variable(2);

    if (view_var.isMetaVariable() && view_var.metaVariablePtr())
    {
        //representation() asserts without sub variables
        if (view_var.metaVariablePtr()->hasVariables())
            return view_var.metaVariablePtr()->representation();
    }
    else if (view_var.variablePtr())
    {
        return view_var.variablePtr()->representation();
    }

    return dbContent::Representation::STANDARD;
}

/**
 */
std::set<std::string> GridView::acceptedAnnotationFeatureTypes() const
{
    std::set<std::string> types;
    types.insert(ViewPointGenFeatureGrid::FeatureName);

    return types;
}
