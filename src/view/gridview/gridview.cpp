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
#include "gridviewdatawidget.h"
#include "gridviewwidget.h"
#include "viewvariable.h"
#include "property_templates.h"

#include "viewpointgenerator.h"

#include "dbcontent.h"

#include "compass.h"
#include "logger.h"
#include "latexvisitor.h"

#include <QApplication>

/**
*/
GridView::Settings::Settings() 
:   value_type            (grid2d::ValueType::ValueTypeCountValid)
,   grid_resolution       (50       )
,   render_color_value_min(""       )
,   render_color_value_max(""       )
,   render_color_scale    ((int)colorscale::ColorScale::Green2Red)
,   render_color_num_steps(10       )
{
}

const int GridView::DecimalsDefault = 3;

/**
*/
GridView::GridView(const std::string& class_id, 
                   const std::string& instance_id,
                   ViewContainer* w, 
                   ViewManager& view_manager)
:   VariableView(class_id, instance_id, w, view_manager)
{
    registerParameter("value_type", &settings_.value_type, Settings().value_type);
    registerParameter("grid_resolution", &settings_.grid_resolution, Settings().grid_resolution);
    registerParameter("render_color_scale", &settings_.render_color_scale, Settings().render_color_scale);
    registerParameter("render_color_num_steps", &settings_.render_color_num_steps, Settings().render_color_num_steps);
    registerParameter("render_color_value_min", &settings_.render_color_value_min, Settings().render_color_value_min);
    registerParameter("render_color_value_max", &settings_.render_color_value_max, Settings().render_color_value_max);

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

    addVariable("data_var_x", "X"          , "x", META_OBJECT_NAME, DBContent::meta_var_longitude_.name(), true, true, false, valid_types_xy);
    addVariable("data_var_y", "Y"          , "y", META_OBJECT_NAME, DBContent::meta_var_latitude_.name() , true, true, false, valid_types_xy);
    addVariable("data_var_z", "Distributed", "z", ""              , ""                                   , true, true, true , valid_types_z );

    updateSettingsFromVariable();

    // create sub done in init
}

/**
*/
GridView::~GridView()
{
    loginf;

    if (widget_)
    {
        delete widget_;
        widget_ = nullptr;
    }

    loginf << "done";
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
void GridView::generateSubConfigurable(const std::string& class_id,
                                       const std::string& instance_id)
{
    logdbg << "class_id " << class_id << " instance_id "
           << instance_id;
    
    if (class_id == "GridViewWidget")
    {
        widget_ = new GridViewWidget(class_id, instance_id, this, this, central_widget_);
        setWidget(widget_);
    }
    else
    {
        throw std::runtime_error("GridView: generateSubConfigurable: unknown class_id " + class_id);
    }
}

/**
*/
void GridView::checkSubConfigurables()
{
    if (!widget_)
        generateSubConfigurable("GridViewWidget", "GridViewWidget0");
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
void GridView::updateSelection()
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

    setParameter(settings_.value_type, (int)type);
    setParameter(settings_.render_color_value_min, std::string(""));
    setParameter(settings_.render_color_value_max, std::string(""));

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
void GridView::postVariableChangedEvent(int idx)
{
    if (idx == 2)
        updateSettingsFromVariable();
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
std::set<std::string> GridView::acceptedAnnotationFeatureTypes() const
{
    std::set<std::string> types;
    types.insert(ViewPointGenFeatureGrid::FeatureName);

    return types;
}
