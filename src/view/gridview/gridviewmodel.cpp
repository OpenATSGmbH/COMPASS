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

#include "gridviewmodel.h"
#include "grid2d.h"
#include "viewlayersnapshot.h"
#include "viewlayerscan.h"
#include "viewvariable.h"
#include "plotmetadata.h"
#include "viewpointgenerator.h"

#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontentmanager.h"
#include "dbcontentdataengine.h"
#include "db_context_manager.h"
#include "sector.h"
#include "sectorlayer.h"

#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <set>

const double GridViewModel::RangeMinDefault = 1e-12;

/**
*/
GridViewModel::GridViewModel(DBContentManager& dbcont_man,
                             context::DBContextManager& ctx_man,
                             AppState& app_state)
:   VariableViewStashModel(dbcont_man, ctx_man, app_state, 3, true)
{
    sectors_ = sectorLines(ctx_man);

    connect(&ctx_man, &context::DBContextManager::sectorsChangedSignal, this, &GridViewModel::sectorsChanged);
    connect(&ctx_man, &context::DBContextManager::activeContextChangedSignal, this, &GridViewModel::sectorsChanged);

    //a renamed layer is a changed name in the lines
    connect(&ctx_man, &context::DBContextManager::sectorLayerRenamedSignal, this,
            [ this ] (const std::string&, const std::string&) { sectorsChanged(); });
}

/**
*/
GridViewModel::~GridViewModel()
{
    //the stash workers call processStash() - join them before the members die
    shutdownAsyncProcessor();
}

/**
*/
void GridViewModel::setVariables(ViewVariable& variable_x, ViewVariable& variable_y, ViewVariable& variable_z)
{
    VariableViewModel::setVariables({ &variable_x, &variable_y, &variable_z });

    //the empty chart before the first load shows the axis labels
    resetDisplayData();
}

/**
*/
std::vector<GridSectorLine> GridViewModel::sectorLines(const context::DBContextManager& ctx_man)
{
    std::vector<GridSectorLine> lines;

    if (!ctx_man.hasActiveContext() || !ctx_man.sectorsLoaded())
        return lines;

    for (const auto& layer : ctx_man.sectorLayers())
    {
        for (const auto& sector : layer->sectors())
        {
            const auto& points = sector->points();
            if (points.size() < 2)
                continue;

            QPolygonF poly;
            poly.reserve((int)points.size() + 1);

            for (const auto& p : points)
                poly.append(QPointF(p.second, p.first)); // first = latitude, second = longitude

            if (poly.first() != poly.last())
                poly.append(poly.first());

            QColor color(QString::fromStdString(sector->colorStr()));

            GridSectorLine line;
            line.line        = gridbackground::makeLine(std::move(poly));
            line.color       = color.isValid() ? color : QColor(Qt::black);
            line.layer_name  = layer->name();
            line.sector_name = sector->name();
            line.sector_id   = sector->id();

            lines.push_back(std::move(line));
        }
    }

    return lines;
}

/**
 * Malformed text features are skipped with a warning, the Geographic View reports them.
 */
std::vector<GridTextLabel> GridViewModel::textLabels(const nlohmann::json& view_point)
{
    std::vector<GridTextLabel> labels;

    const auto features = ViewPointGenVP::scanForFeatures(view_point, { ViewPointGenFeatureText::FeatureName });

    for (const auto& f : features)
    {
        const auto& feature = f.feature_json;

        try
        {
            GridTextLabel label;
            label.text = feature.at(ViewPointGenFeatureText::FeatureTextFieldNameText).get<std::string>();

            //latitude first, as the Geographic View reads it
            const auto& pos = feature.at(ViewPointGenFeatureText::FeatureTextFieldNamePos);
            if (!pos.is_array() || pos.size() < 2)
                throw std::runtime_error("position is no coordinate");

            label.position = QPointF(pos.at(1).get<double>(), pos.at(0).get<double>());

            auto props_it = feature.find(ViewPointGenFeature::FeatureFieldNameProps);
            if (props_it != feature.end() && props_it->is_object())
            {
                const auto& props = *props_it;

                if (props.contains(ViewPointGenFeature::FeatureFieldNamePropColor))
                {
                    QColor color(QString::fromStdString(props.at(ViewPointGenFeature::FeatureFieldNamePropColor).get<std::string>()));
                    if (color.isValid())
                        label.color = color;
                }

                if (props.contains(ViewPointGenFeatureText::FeatureTextFieldNameFontSize))
                    label.font_size = props.at(ViewPointGenFeatureText::FeatureTextFieldNameFontSize).get<double>();

                if (props.contains(ViewPointGenFeatureText::FeatureTextFieldNameDir))
                {
                    const std::string dir = props.at(ViewPointGenFeatureText::FeatureTextFieldNameDir).get<std::string>();

                    if (dir == ViewPointGenFeatureText::TextDirNameRightDown)
                        label.direction = GridTextLabel::Direction::RightDown;
                    else if (dir == ViewPointGenFeatureText::TextDirNameLeftUp)
                        label.direction = GridTextLabel::Direction::LeftUp;
                    else if (dir == ViewPointGenFeatureText::TextDirNameLeftDown)
                        label.direction = GridTextLabel::Direction::LeftDown;
                }
            }

            labels.push_back(label);
        }
        catch (const std::exception& e)
        {
            logwrn << "text feature '" << f.name << "' skipped: " << e.what();
        }
    }

    return labels;
}

/**
 */
boost::optional<GridBackgroundSettings> GridViewModel::backgroundFromViewPoint(const nlohmann::json& view_point)
{
    if (!view_point.is_object())
        return {};

    auto it = view_point.find(ViewPointGenFeatureGrid::ViewPointFieldNameBackground);
    if (it == view_point.end())
        return {};

    GridBackgroundSettings settings;
    if (!settings.fromJSON(*it))
    {
        logwrn << "grid background of the view point not readable, ignored";
        return {};
    }

    return settings;
}

/**
*/
void GridViewModel::sectorsChanged()
{
    sectors_ = sectorLines(dbContextManager());

    loginf << "sectors " << sectors_.size();

    if (!display_data_)
        return;

    publish(std::make_shared<GridDisplayData>(*display_data_));

    emit displayDataChangedSignal();
}

/**
 * Main thread.
*/
void GridViewModel::readSettings()
{
    settings_ = settings_func_ ? settings_func_() : Settings();
}

/**
 * Identity of an axis variable is its content and name, empty while no variable is set.
 * The distributed variable is no axis, a new one keeps the zoom.
 */
std::string GridViewModel::variableDomainId() const
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
 * Minimum / maximum position of the database (data engine), empty if not known.
 */
QRectF GridViewModel::databasePositionExtent() const
{
    const auto& engine = dbContentManager().dataEngine();

    if (!engine.hasMinMaxPosition())
        return {};

    const auto lat = engine.minMaxLatitude();
    const auto lon = engine.minMaxLongitude();

    QRectF r(lon.first, lat.first, lon.second - lon.first, lat.second - lat.first);

    //a single position => a small area around it
    const double min_size = 0.01;

    if (r.width() < min_size || r.height() < min_size)
        r = QRectF(r.center().x() - min_size / 2.0, r.center().y() - min_size / 2.0, min_size, min_size)
            .united(r);

    return r;
}

/**
 * Main thread, uses the settings read last.
*/
GridViewModel::DataPtr GridViewModel::dataFromVariables() const
{
    auto data = std::make_shared<GridDisplayData>();

    data->x_axis_label = variable(0).description();
    data->y_axis_label = variable(1).description();

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
    }

    data->legend_data_type = settings_.legend_data_type;
    data->legend_repr      = settings_.legend_repr;

    //longitude / latitude => the background shows the position extent of the database
    data->is_geographic = settings_.lonlat_axes;

    if (settings_.lonlat_axes)
        data->database_extent = databasePositionExtent();

    data->domain_id = variableDomainId();

    return data;
}

/**
*/
void GridViewModel::resetDisplayData()
{
    readSettings();

    auto data = dataFromVariables();

    //the empty chart of a shown annotation keeps its title and axis labels, the recompute builds it
    publish(data);
}

/**
*/
void GridViewModel::resetStashDependentData()
{
    grid_.reset();
    grid_layer_.reset();

    // NOTE: the layer snapshots are intentionally not cleared here (may run on a
    // worker). The layer tree's DBContentLeafItems hold raw pointers into them -
    // only the main thread commit replaces them, after rebinding the tree to the
    // pending ones processStash() built.
}

/**
 * Snapshot of the layer panel's hidden groups and of the settings for buildGridFromStash(), which may
 * run on a worker.
 */
void GridViewModel::prepareStashProcessing()
{
    snapshotHiddenLayerIds();
    readSettings();
}

/**
*/
void GridViewModel::processStash(const VariableViewStash<double>& stash)
{
    loginf;

    // Build one snapshot per group (total count + #NULL), unconditionally -
    // the panel should show the full dataset breakdown regardless of which
    // layers are currently visible.
    size_t num_null_values = 0;
    LayerSnapshots layers;
    layers.reserve(stash.groupedStashes().size());

    for (const auto& dbc_values : stash.groupedStashes())
    {
        const std::string& group_key = dbc_values.first;
        const auto&        gds       = dbc_values.second;

        num_null_values += gds.nan_count;

        // group key "<ds_type>:<ds_name>:L<n>:<dbcontent>" as produced when
        // group_per_datasource is on, malformed keys below an "<unknown>" branch
        layers.push_back(std::make_unique<ViewLayerSnapshot>(group_key,
                                                             view_layer_scan::layerIdOrUnknown(group_key),
                                                             (unsigned int)gds.size(),
                                                             (unsigned int)gds.nan_count));
    }

    addNullCount(num_null_values);

    // Pending until publishDisplayData rebinds the tree on the main thread
    // (this function may run on a worker thread): the tree keeps pointing at
    // the current snapshots, which only the commit replaces. A failed or
    // discarded run just leaves its pending layers to be replaced by the next one.
    pending_layers_ = std::move(layers);

    // Aggregate visible groups into the Grid2D.
    buildGridFromStash();
}

/**
*/
void GridViewModel::buildGridFromStash()
{
    grid_.reset();
    grid_layer_.reset();

    const auto& data_ranges = getStash().dataRanges();

    bool has_data = (hasData() &&
                     getStash().hasData() &&
                     variablesOk() &&
                     data_ranges[ 0 ].has_value() &&
                     data_ranges[ 1 ].has_value() &&
                     data_ranges[ 2 ].has_value());
    if (!has_data)
    {
        loginf << "no grid data, has data " << hasData() << " stash data " << getStash().hasData()
               << " variables ok " << variablesOk()
               << " ranges " << data_ranges[ 0 ].has_value() << data_ranges[ 1 ].has_value() << data_ranges[ 2 ].has_value()
               << " groups " << getStash().groupedStashes().size();
        return;
    }

    auto bounds = getPlanarVariableBounds(0, 1, true);
    if (!bounds.has_value() || bounds->isEmpty())
    {
        loginf << "bounds empty, skipping";
        return;
    }
    traced_assert(bounds->isValid());

    auto z_bounds = getVariableBounds(2, false);
    traced_assert(z_bounds.has_value());

    const auto value_type = settings_.value_type;

    // snapshot taken on the main thread, this may run on a worker
    const std::set<std::string>& hidden_series = hiddenLayerIds();

    //extent of the rows the grid will hold: a finite position and, except for the count of
    //missing values, a value of the distributed variable. For a Report Variable most rows have
    //no value, and the rows without one can lie far from the rows with one, the stash ranges
    //then spend the cells on empty area
    {
        const bool need_value = value_type != grid2d::ValueType::ValueTypeCountNan;

        size_t num_rows = 0;
        double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;

        for (const auto& dbc_values : getStash().groupedStashes())
        {
            if (hidden_series.count(dbc_values.first))
                continue;

            const auto& gds      = dbc_values.second;
            const auto& x_values = gds.variable_stashes[ 0 ].values;
            const auto& y_values = gds.variable_stashes[ 1 ].values;
            const auto& z_values = gds.variable_stashes[ 2 ].values;

            for (size_t i = 0; i < z_values.size(); ++i)
            {
                if (gds.isNan(0, i) || gds.isNan(1, i))
                    continue;

                if (need_value && (gds.isNan(2, i) || !std::isfinite(z_values[ i ])))
                    continue;

                const double x = x_values[ i ];
                const double y = y_values[ i ];

                if (!std::isfinite(x) || !std::isfinite(y))
                    continue;

                if (num_rows == 0)
                {
                    xmin = xmax = x;
                    ymin = ymax = y;
                }
                else
                {
                    xmin = std::min(xmin, x);
                    xmax = std::max(xmax, x);
                    ymin = std::min(ymin, y);
                    ymax = std::max(ymax, y);
                }

                ++num_rows;
            }
        }

        if (num_rows > 0)
        {
            //broaden a degenerate extent as getVariableBounds() does
            auto broaden = [] (double& v0, double& v1)
            {
                if (v1 - v0 < RangeMinDefault)
                {
                    const double mid = (v0 + v1) / 2.0;
                    v0 = mid - RangeMinDefault / 2.0;
                    v1 = mid + RangeMinDefault / 2.0;
                }
            };
            broaden(xmin, xmax);
            broaden(ymin, ymax);

            bounds = QRectF(xmin, ymin, xmax - xmin, ymax - ymin);

            loginf << "extent from " << num_rows << " rows with values x " << xmin << " to " << xmax
                   << " y " << ymin << " to " << ymax;
        }
    }

    grid2d::GridResolution res = grid2d::GridResolution().setCellCount(
        settings_.grid_resolution, settings_.grid_resolution);

    grid_.reset(new Grid2D);

    std::string err;
    bool ok = grid_->create(bounds.value(), res, "wgs84", true, &err);

    if (!ok)
        logerr << "creation of grid failed: " << err;

    traced_assert(ok);

    loginf << "created grid of " << grid_->numCellsX() << "x" << grid_->numCellsY();

    for (const auto& dbc_values : getStash().groupedStashes())
    {
        const std::string& group_key = dbc_values.first;
        const auto&        gds       = dbc_values.second;

        // Skip groups the user has hidden in the layer panel.
        if (hidden_series.count(group_key))
            continue;

        const auto& x_values = gds.variable_stashes[ 0 ].values;
        const auto& y_values = gds.variable_stashes[ 1 ].values;
        const auto& z_values = gds.variable_stashes[ 2 ].values;

        traced_assert(x_values.size() == y_values.size() &&
               y_values.size() == z_values.size());

        loginf << "group " << group_key
               << " #x " << x_values.size();

        for (size_t i = 0; i < z_values.size(); ++i)
        {
            if (!gds.isNan(0, i) && !gds.isNan(1, i))
                grid_->addValue(x_values[ i ], y_values[ i ], z_values[ i ]);
        }
    }

    loginf << "added " << grid_->numAdded()
           << " oor "  << grid_->numOutOfRange()
           << " inf "  << grid_->numInf();

    Grid2DLayers layers;
    grid_->addToLayers(layers, grid2d::valueTypeToString(value_type), value_type);

    grid_layer_ = layers.layer(0);

    auto range = grid_layer_->range();
    if (range.has_value())
    {
        loginf << "grid range min " << range->first << " max " << range->second;
        traced_assert(range->first <= range->second);
    }

    loginf << "done, generated " << layers.numLayers() << " layers";
}

/**
 * Main thread, after processStash(). Hidden groups changed since the snapshot (toggled during the
 * recompute, or new leaves below an unchecked group) => the grid is built again here.
 */
void GridViewModel::publishDisplayData()
{
    if (pending_layers_)
    {
        updateLayerTree(std::move(*pending_layers_));
        pending_layers_.reset();
    }

    if (hiddenLayerIdsChanged())
    {
        snapshotHiddenLayerIds();
        buildGridFromStash();
    }

    publishGrid();
}

/**
*/
void GridViewModel::publishGrid()
{
    auto data = dataFromVariables();

    data->layer = grid_layer_;
    data->valid = grid_layer_.has_value() && variablesOk();

    publish(data);
}

/**
*/
void GridViewModel::publish(const DataPtr& data)
{
    traced_assert(data);

    data->sectors = sectors_;

    display_data_ = data;
}

/**
 * Notice: not for the hidden states the tree update itself restores, those are picked up by
 * publishDisplayData().
 */
void GridViewModel::layerVisibilityChanged()
{
    // Annotation mode: hiddenChangedSignal here originates from the
    // annotations subtree's radio activation (AnnotationLeafItem). The
    // activation already triggered a full redraw from the annotation.
    // Building the grid from the stash now would wipe that grid.
    if (annotationActive())
        return;

    // e.g. a background item (no data layer ids) => no re-aggregation
    if (!hiddenLayerIdsChanged())
        return;

    // a recompute worker fills the stash and the grid => its commit picks up the toggle
    if (hasPendingAsyncWork())
        return;

    prepareStashProcessing();

    // A layer toggle only re-aggregates and re-renders, no full recompute. The stash and the
    // layer tree are untouched, the grid is built again from the visible groups.
    buildGridFromStash();
    publishGrid();

    emit displayDataChangedSignal();
}

/**
*/
GridViewModel::DataPtr GridViewModel::dataFromAnnotation(const nlohmann::json& feature,
                                                         const PlotMetadata& metadata,
                                                         std::string* error)
{
    const auto& grid_field     = ViewPointGenFeatureGrid::FeatureGridFieldNameGrid;
    const auto& settings_field = ViewPointGenFeatureGrid::FeatureGridFieldNameRenderSettings;

    try
    {
        if (!feature.is_object() || !feature.contains(grid_field))
            throw std::runtime_error("missing '" + grid_field + "' field");

        Grid2DLayer layer;
        if (!layer.fromJSON(feature[ grid_field ]))
            throw std::runtime_error("could not read grid layer");

        if (layer.data.cols() < 1 || layer.data.rows() < 1)
            throw std::runtime_error("grid layer empty");

        auto data = std::make_shared<GridDisplayData>();

        data->title        = metadata.title_;
        data->x_axis_label = metadata.xAxisLabel();
        data->y_axis_label = metadata.yAxisLabel();

        //an annotation brings values, not variables
        data->legend_data_type = PropertyDataType::DOUBLE;
        data->legend_repr      = dbContent::Representation::STANDARD;

        //grids over other axes (e.g. altitude) have no spatial reference system
        data->is_geographic = layer.hasGeographicReference();

        if (feature.contains(settings_field))
        {
            Grid2DRenderSettings rs;
            if (rs.fromJSON(feature[ settings_field ]))
                data->render_settings = rs;
            else
                logwrn << "render_settings present but unparseable; ignored";
        }

        auto range = layer.range();
        if (range.has_value())
            loginf << "grid range min " << range->first << " max " << range->second;

        data->layer     = std::move(layer);
        data->domain_id = "annotation";
        data->valid     = true;

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
std::shared_ptr<const GridDisplayData> GridViewModel::displayDataFromAnnotation(
    const nlohmann::json& feature,
    const PlotMetadata& metadata,
    const std::vector<GridSectorLine>& sectors,
    const std::vector<GridTextLabel>& labels,
    const boost::optional<GridBackgroundSettings>& background,
    std::string* error)
{
    auto data = dataFromAnnotation(feature, metadata, error);

    if (data && data->is_geographic)
    {
        data->sectors    = sectors;
        data->labels     = labels;
        data->background = background;
    }

    return data;
}

/**
*/
bool GridViewModel::setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata, std::string* error)
{
    loginf;

    auto data = dataFromAnnotation(feature, metadata, error);

    const bool valid = data != nullptr;

    //the text features of the view point are read only for a grid that shows them
    if (valid && data->is_geographic && labels_func_)
        data->labels = labels_func_();

    if (!valid)
    {
        //the empty chart of a malformed annotation keeps its title and axis labels
        data = std::make_shared<GridDisplayData>();
        data->title        = metadata.title_;
        data->x_axis_label = metadata.xAxisLabel();
        data->y_axis_label = metadata.yAxisLabel();
    }

    //another annotation of the view is another value domain
    data->domain_id = "annotation:" + std::to_string(annotationGroupIdx()) + ":" + std::to_string(annotationIdx());

    annotation_data_ = data;

    setAnnotationActive();

    return valid;
}

/**
*/
bool GridViewModel::updateFromAnnotation()
{
    traced_assert(annotation_data_);

    publish(std::make_shared<GridDisplayData>(*annotation_data_));

    return annotation_data_->valid;
}

/**
*/
bool GridViewModel::hasValidGrid() const
{
    if (!display_data_ || !display_data_->layer.has_value())
        return false;

    const auto& d = display_data_->layer->data;

    return (d.cols() > 0 && d.rows() > 0);
}

/**
 * The empty chart was built before the database was open => without axis ranges, where zoom and pan
 * do nothing. Built again with the position extent of the database.
 */
void GridViewModel::databaseChanged()
{
    if (hasValidGrid() || annotationActive())
        return;

    readSettings();

    auto data = dataFromVariables();
    data->valid = false;

    publish(data);

    emit displayDataChangedSignal();
}

/**
 */
boost::optional<QRectF> GridViewModel::getPlanarVariableBounds(int var_x, int var_y, bool fix_small_ranges) const
{
    auto bounds_x = getVariableBounds(var_x, fix_small_ranges);
    auto bounds_y = getVariableBounds(var_y, fix_small_ranges);
    if (!bounds_x.has_value() || !bounds_y.has_value())
        return {};

    double xmin = bounds_x.value().first;
    double xmax = bounds_x.value().second;
    double ymin = bounds_y.value().first;
    double ymax = bounds_y.value().second;

    return QRectF(xmin, ymin, xmax - xmin, ymax - ymin);
}

/**
 */
boost::optional<std::pair<double, double>> GridViewModel::getVariableBounds(int var, bool fix_small_ranges) const
{
    boost::optional<std::pair<double, double>> b = getStash().dataRanges().at(var);
    if (!b.has_value())
        return b;

    traced_assert(std::isfinite(b->first));
    traced_assert(std::isfinite(b->second));
    traced_assert(b->second >= b->first);

    //fix small ranges?
    if (fix_small_ranges && b->second - b->first < RangeMinDefault)
    {
        //broaden range
        const double eps = RangeMinDefault / 2;
        const double mid = (b->first + b->second) / 2;

        b->first  = mid - eps;
        b->second = mid + eps;

        traced_assert(std::isfinite(b->first));
        traced_assert(std::isfinite(b->second));
        traced_assert(b->second >= b->first);
    }

    return b;
}

/**
 */
boost::optional<QRectF> GridViewModel::dataBoundsXY() const
{
    auto xy_bounds = getPlanarVariableBounds(0, 1, false);
    auto z_bounds  = getVariableBounds(2, false);

    if (!xy_bounds.has_value() || !xy_bounds->isValid() || !z_bounds.has_value())
        return {};

    return xy_bounds;
}

/**
 */
void GridViewModel::viewInfoJSON(nlohmann::json& info) const
{
    VariableViewStashModel::viewInfoJSON(info);

    auto xy_bounds    = getPlanarVariableBounds(0, 1, false);
    auto z_bounds     = getVariableBounds(2, false);
    bool bounds_valid = xy_bounds.has_value() && xy_bounds->isValid() && z_bounds.has_value();

    info[ "data_bounds_valid" ] = bounds_valid;
    info[ "data_bounds_xmin"  ] = bounds_valid ? xy_bounds->left()        : 0.0;
    info[ "data_bounds_ymin"  ] = bounds_valid ? xy_bounds->top()         : 0.0;
    info[ "data_bounds_zmin"  ] = bounds_valid ?  z_bounds.value().first  : 0.0;
    info[ "data_bounds_xmax"  ] = bounds_valid ? xy_bounds->right()       : 0.0;
    info[ "data_bounds_ymax"  ] = bounds_valid ? xy_bounds->bottom()      : 0.0;
    info[ "data_bounds_zmax"  ] = bounds_valid ?  z_bounds.value().second : 0.0;

    const auto settings = settings_func_ ? settings_func_() : Settings();

    // the grid built from the stash, the value type of the view applied per cell
    auto range = grid_ && grid_layer_ ? grid_layer_->range() : boost::optional<std::pair<double, double>>();

    const bool has_layer = display_data_ && display_data_->layer.has_value();

    info[ "grid_valid"       ] = grid_ != nullptr;
    info[ "grid_num_added"   ] = grid_ ? grid_->numAdded()      : (size_t)0;
    info[ "grid_num_oor"     ] = grid_ ? grid_->numOutOfRange() : (size_t)0;
    info[ "grid_num_inf"     ] = grid_ ? grid_->numInf()        : (size_t)0;
    info[ "grid_num_layers"  ] = has_layer ? (size_t)1 : (size_t)0;
    info[ "grid_value_type"  ] = grid2d::valueTypeToString(settings.value_type);
    info[ "grid_range_valid" ] = range.has_value();
    info[ "grid_value_min"   ] = range.has_value() ? range->first  : 0.0;
    info[ "grid_value_max"   ] = range.has_value() ? range->second : 0.0;
}
