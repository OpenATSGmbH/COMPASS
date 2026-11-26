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

#include "scatterplotviewmodel.h"
#include "appstate.h"
#include "color_provider.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "viewlayerscan.h"
#include "viewlayersnapshot.h"
#include "viewvariable.h"
#include "plotmetadata.h"
#include "viewpointgenerator.h"
#include "stringconv.h"
#include "timeconv.h"
#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

#include <algorithm>

using namespace std;

namespace
{
    /**
     * Qt shows a date time axis in local time, the timestamps of the series are corrected for that.
     */
    void correctSeriesDateTime(ScatterPlotDisplayData& data)
    {
        if (!data.x_axis_is_datetime && !data.y_axis_is_datetime)
            return;

        for (auto& s : data.series.dataSeries())
        {
            for (auto& pos : s.second.scatter_series.points)
            {
                if (data.x_axis_is_datetime) pos[ 0 ] = Utils::Time::correctLongQtUTC((std::int64_t)pos[ 0 ]);
                if (data.y_axis_is_datetime) pos[ 1 ] = Utils::Time::correctLongQtUTC((std::int64_t)pos[ 1 ]);
            }
        }
    }
}

/**
*/
ScatterPlotViewModel::ScatterPlotViewModel(DBContentManager& dbcont_man,
                                           context::DBContextManager& ctx_man,
                                           AppState& app_state)
:   VariableViewStashModel(dbcont_man, ctx_man, app_state, 2, true)
{
    // color mode recompute also without a layer panel (its commit rebuilds the layer tree),
    // see colorModeChanged()
    connect(&app_state, &AppState::colorModeChangedSignal,
            this, [ this ] (unsigned int /*mode*/) { recompute(); });
}

/**
*/
ScatterPlotViewModel::~ScatterPlotViewModel()
{
    //the stash workers call processStash() - join them before the members die
    shutdownAsyncProcessor();
}

/**
*/
void ScatterPlotViewModel::setVariables(ViewVariable& variable_x, ViewVariable& variable_y)
{
    VariableViewModel::setVariables({ &variable_x, &variable_y });

    //the empty chart before the first load shows the axis labels
    display_data_ = dataFromVariables();
}

/**
 * Identity of an axis variable is its content and name, empty while no variable is set.
 */
std::string ScatterPlotViewModel::variableDomainId() const
{
    std::string id = "variables";

    for (int axis_id = 0; axis_id < 2; ++axis_id)
    {
        const auto& view_var = variable(axis_id);

        id += ":";

        if (view_var.hasVariable())
            id += view_var.variableDBContent() + "|" + view_var.variableName();
    }

    return id;
}

/**
*/
ScatterPlotViewModel::DataPtr ScatterPlotViewModel::dataFromVariables() const
{
    auto data = std::make_shared<ScatterPlotDisplayData>();

    data->x_axis_label = variable(0).description();
    data->y_axis_label = variable(1).description();

    data->x_axis_is_datetime = variableIsDateTime(0);
    data->y_axis_is_datetime = variableIsDateTime(1);

    for (int axis_id = 0; axis_id < 2; ++axis_id)
    {
        auto& view_var = variable(axis_id);

        auto dtype = view_var.dataType();
        data->axis_data_type[ axis_id ] = dtype.has_value() ? dtype.value() : PropertyDataType::DOUBLE;

        auto repr = dbContent::Representation::STANDARD;

        if (view_var.isMetaVariable() && view_var.metaVariablePtr())
        {
            //representation() asserts without sub variables
            if (view_var.metaVariablePtr()->hasVariables())
                repr = view_var.metaVariablePtr()->representation();
        }
        else if (view_var.variablePtr())
        {
            repr = view_var.variablePtr()->representation();
        }

        data->axis_repr[ axis_id ] = repr;

        loginf << "axis " << axis_id
               << " data type " << Property::asString(data->axis_data_type[ axis_id ])
               << " representation " << dbContent::Variable::representationToString(repr);
    }

    data->domain_id = variableDomainId();

    return data;
}

/**
*/
std::string ScatterPlotViewModel::selectionLayerId() const
{
    return ScatterPlotDisplayData::SelectedSeriesName;
}

/**
*/
void ScatterPlotViewModel::resetStashDependentData()
{
    // no layer tree rebuild here: runs on the recompute worker too, the tree holds
    // snapshots and is rebuilt on the main thread (publish)
    computed_data_.reset();
}

/**
*/
void ScatterPlotViewModel::resetDisplayData()
{
    display_data_.reset();
}

/**
*/
void ScatterPlotViewModel::processStash(const VariableViewStash<double>& stash)
{
    loginf << "start"
           << " valid: " << stash.valid_count_ << " "
           << " selected: " << stash.selected_count_ << " "
           << " nan: " << stash.nan_value_count_;

    auto data = dataFromVariables();

    //generate dataseries from stash
    ScatterSeries selected_series;
    selected_series.points.reserve(stash.selected_count_);

    size_t num_null_values = 0;

    for (const auto& dbc_stash : stash.groupedStashes())
    {
        ScatterSeries dbc_series;
        dbc_series.points.reserve(dbc_stash.second.unsel_count);

        const auto& x_values = dbc_stash.second.variable_stashes[ 0 ].values;
        const auto& y_values = dbc_stash.second.variable_stashes[ 1 ].values;

        size_t n = x_values.size();

        size_t dbc_null_count = 0;

        for (size_t i = 0; i < n; ++i)
        {
            if (dbc_stash.second.nan_values[ i ])
            {
                //nan = null, each value of the triplet must not be null
                ++num_null_values;
                ++dbc_null_count;
                continue;
            }
            else if (dbc_stash.second.selected_values[ i ])
            {
                selected_series.points.emplace_back(x_values[ i ], y_values[ i ]);
            }
            else
            {
                dbc_series.points.emplace_back(x_values[ i ], y_values[ i ]);
            }
        }

        if (!dbc_series.points.empty())
        {
            std::string name = dbc_stash.first;

            // group key format: "<ds_type>:<ds_name>:L<n>:<dbcontent_name>", malformed => only the name
            view_layer_scan::LayerId layer_id;
            if (auto parsed = view_layer_scan::parseLayerId(name))
                layer_id = *parsed;
            else
                layer_id.ds_name = name;

            QColor line_color = context::resolveSeriesColor(
                layer_id.ds_type, layer_id.ds_name, layer_id.line_index, layer_id.dbcontent, dbContextManager(),
                [ ] (const std::string& key) { return QColor::fromRgb(Utils::String::hash(key)); });

            data->series.addDataSeries(dbc_series, name, line_color, ViewModel::MarkerSizePx, dbc_null_count);
        }
    }

    //@TODO
    addNullCount(num_null_values);

    //add selected dataset as the last one (important for render order)
    if (!selected_series.points.empty())
    {
        data->series.addDataSeries(selected_series, ScatterPlotDisplayData::SelectedSeriesName,
                                   ViewModel::ColorSelected, ViewModel::MarkerSizeSelectedPx);
    }

    correctSeriesDateTime(*data);

    //the layer panel rebuild runs in publishDisplayData (main thread) - this
    //function may run on a worker thread

    data->bounds = data->series.getDataBounds();

    loginf << "done, generated " << data->series.numDataSeries() << " series";

    computed_data_ = data;
}

/**
 * Main thread, after processStash().
 */
void ScatterPlotViewModel::publishDisplayData()
{
    auto data = computed_data_ ? computed_data_ : dataFromVariables();

    //the published data does not change any more
    computed_data_.reset();

    publish(data, variablesOk());
}

/**
*/
ScatterPlotViewModel::DataPtr ScatterPlotViewModel::dataFromAnnotation(const nlohmann::json& feature,
                                                                       const PlotMetadata& metadata,
                                                                       std::string* error)
{
    const auto& series_field = ViewPointGenFeatureScatterSeries::FeatureHistogramFieldNameScatterSeries;

    try
    {
        if (!feature.is_object())
            throw std::runtime_error("scatter series annotation feature is not an object");

        if (!feature.contains(series_field))
            throw std::runtime_error("scatter series annotation feature missing '" + series_field + "' field");

        auto data = std::make_shared<ScatterPlotDisplayData>();

        if (!data->series.fromJSON(feature[ series_field ]))
            throw std::runtime_error("could not read scatter series from annotation");

        data->title        = metadata.title_;
        data->x_axis_label = metadata.xAxisLabel();
        data->y_axis_label = metadata.yAxisLabel();

        data->x_axis_is_datetime = data->series.commonDataTypeX() == ScatterSeries::DataTypeTimestamp;
        data->y_axis_is_datetime = data->series.commonDataTypeY() == ScatterSeries::DataTypeTimestamp;

        //an annotation brings values, not variables, so only the timestamp flag is known
        data->axis_data_type[ 0 ] = data->x_axis_is_datetime ? PropertyDataType::TIMESTAMP : PropertyDataType::DOUBLE;
        data->axis_data_type[ 1 ] = data->y_axis_is_datetime ? PropertyDataType::TIMESTAMP : PropertyDataType::DOUBLE;

        correctSeriesDateTime(*data);

        data->bounds    = data->series.getDataBounds();
        data->domain_id = "annotation";
        data->valid     = data->series.numPoints(true) > 0;

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
std::shared_ptr<const ScatterPlotDisplayData> ScatterPlotViewModel::displayDataFromAnnotation(const nlohmann::json& feature,
                                                                                             const PlotMetadata& metadata,
                                                                                             std::string* error)
{
    return dataFromAnnotation(feature, metadata, error);
}

/**
*/
bool ScatterPlotViewModel::setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata, std::string* error)
{
    loginf;

    auto data = dataFromAnnotation(feature, metadata, error);

    annotation_valid_ = data != nullptr;

    if (!annotation_valid_)
    {
        //the empty chart of a malformed annotation keeps its title and axis labels
        data = std::make_shared<ScatterPlotDisplayData>();
        data->title        = metadata.title_;
        data->x_axis_label = metadata.xAxisLabel();
        data->y_axis_label = metadata.yAxisLabel();
    }

    //another annotation of the view is another value domain
    data->domain_id = "annotation:" + std::to_string(annotationGroupIdx()) + ":" + std::to_string(annotationIdx());

    annotation_data_ = data;

    setAnnotationActive();

    return annotation_valid_;
}

/**
*/
bool ScatterPlotViewModel::updateFromAnnotation()
{
    //the published data gets the checked state of the layers, the annotation data stays as it was read
    publish(std::make_shared<ScatterPlotDisplayData>(*annotation_data_), annotation_valid_);

    return annotation_valid_;
}

/**
*/
void ScatterPlotViewModel::publish(const DataPtr& data, bool content_ok)
{
    traced_assert(data);

    updateLayerTreeFrom(*data);

    // the update above set the flags of the new snapshots
    applyLayerVisibility(*data);

    data->valid = content_ok && data->series.numPoints(true) > 0;

    content_ok_   = content_ok;
    display_data_ = data;
}

/**
 * Layer snapshots from the series, keyed "<ds_type>:<ds_name>:L<n>:<dbcontent>". The pooled
 * "Selected" overlay series does not fit that scheme - it becomes a top leaf, always visible
 * regardless of how the data subtree expands. Main thread.
 */
void ScatterPlotViewModel::updateLayerTreeFrom(const ScatterPlotDisplayData& data)
{
    LayerSnapshots layers;
    layers.reserve(data.series.numDataSeries());

    for (const auto& [ key, series ] : data.series.dataSeries())
    {
        const bool selected = key == ScatterPlotDisplayData::SelectedSeriesName;

        view_layer_scan::LayerId id;
        if (selected)
            id.dbcontent = ScatterPlotDisplayData::SelectedSeriesName;
        else
            id = view_layer_scan::layerIdOrUnknown(key);

        auto layer = std::make_unique<ViewLayerSnapshot>(key,
                                                         id,
                                                         (unsigned int)series.scatter_series.points.size(),
                                                         (unsigned int)series.null_count,
                                                         series.color,
                                                         series.visible);
        layer->setTopLeaf(selected);

        layers.push_back(std::move(layer));
    }

    // annotation series are no loaded data => final even while the data is incomplete
    updateLayerTree(std::move(layers), annotationActive());
}

/**
*/
bool ScatterPlotViewModel::layerVisibilityDiffers(const ScatterPlotDisplayData& data) const
{
    const auto& data_series = data.series.dataSeries();

    for (const auto& layer : layerSnapshots())
    {
        auto it = data_series.find(layer->key());
        if (it != data_series.end() && it->second.visible != layer->visible())
            return true;
    }

    return false;
}

/**
*/
void ScatterPlotViewModel::applyLayerVisibility(ScatterPlotDisplayData& data) const
{
    auto& data_series = data.series.dataSeries();

    for (const auto& layer : layerSnapshots())
    {
        auto it = data_series.find(layer->key());
        if (it != data_series.end())
            it->second.visible = layer->visible();
    }
}

/**
 * The display data is not changed in place, the render widget holds it. So a new checked state
 * of a layer gives a copy with the new visible flags.
 */
void ScatterPlotViewModel::republishLayerVisibility()
{
    if (!display_data_ || !layerVisibilityDiffers(*display_data_))
        return;

    auto data = std::make_shared<ScatterPlotDisplayData>(*display_data_);

    applyLayerVisibility(*data);

    data->valid = content_ok_ && data->series.numPoints(true) > 0;

    display_data_ = data;

    emit displayDataChangedSignal();
}

/**
*/
void ScatterPlotViewModel::rebuildLayerTree()
{
    if (!display_data_)
    {
        updateLayerTree({}, annotationActive());
        return;
    }

    updateLayerTreeFrom(*display_data_);

    // the layers of a new panel can have another checked state than the series
    republishLayerVisibility();
}

/**
 */
void ScatterPlotViewModel::layerVisibilityChanged()
{
    // a recompute worker builds new display data => its commit takes the checked state of the layers
    if (hasPendingAsyncWork())
        return;

    republishLayerVisibility();
}

/**
 * The own color mode recompute (see constructor) builds the series again, its commit rebuilds the
 * layer tree.
 */
void ScatterPlotViewModel::colorModeChanged()
{
}

/**
*/
void ScatterPlotViewModel::selectRectangle(const QRectF& chart_rect, bool add_to_selection)
{
    loginf;

    const QRectF r = chart_rect.normalized();

    //!datetime in the rect needs correction back to utc when selecting data!
    bool correct_datetime_utc = true;
    selectData(r.left(), r.right(), r.top(), r.bottom(), 0, 1, add_to_selection, correct_datetime_utc);
}

/**
 */
void ScatterPlotViewModel::viewInfoJSON(nlohmann::json& info) const
{
    VariableViewStashModel::viewInfoJSON(info);

    const boost::optional<QRectF> bounds = display_data_ ? display_data_->bounds : boost::optional<QRectF>();

    const bool bounds_valid = bounds.has_value();

    info[ "data_bounds_valid" ] = bounds_valid;
    info[ "data_bounds_xmin"  ] = bounds_valid ? bounds->left()   : 0.0;
    info[ "data_bounds_ymin"  ] = bounds_valid ? bounds->top()    : 0.0;
    info[ "data_bounds_xmax"  ] = bounds_valid ? bounds->right()  : 0.0;
    info[ "data_bounds_ymax"  ] = bounds_valid ? bounds->bottom() : 0.0;
}
