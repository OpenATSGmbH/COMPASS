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

#include "gridviewdatawidget.h"
#include "gridviewwidget.h"
#include "gridview.h"
#include "viewabledataconfig.h"
#include "gridviewchart.h"
#include "grid2d.h"
#include "grid2dlayer.h"
#include "grid2drendersettings.h"
#include "grid2dlayerrenderer.h"
#include "viewlayersnapshot.h"

#include "viewvariable.h"
#include "viewpointgenerator.h"
#include "colormapwidget.h"
#include "property_templates.h"

#include "buffer.h"

#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"

#include "dbcontentlayer.h"
#include "layertreemodel.h"
#include "viewlayertreemodel.h"
#include "annotationsrootitem.h"

#include "logger.h"

#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>

#include <QChart>
#include <QGraphicsLayout>

#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLegend>
#include <QtCharts/QValueAxis>
#include <QtCharts/QCategoryAxis>

#include <algorithm>
#include <cmath>
#include <set>

/**
*/
GridViewDataWidget::GridViewDataWidget(GridViewWidget* view_widget,
                                       QWidget* parent,
                                       Qt::WindowFlags f)
:   VariableViewStashDataWidget(view_widget, view_widget->getView(), true, parent, f)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    main_layout_ = new QHBoxLayout();
    main_layout_->setMargin(0);

    setLayout(main_layout_);

    legend_ = new ColorLegendWidget(this);
    legend_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    legend_->showSelectionColor(true);
    legend_->showNullColor(false);

    main_layout_->addWidget(legend_);

    x_axis_name_ = view_->variable(0).description();
    y_axis_name_ = view_->variable(1).description();

    updateAxisInfoFromVariables();

    updateGridChart();
}

/**
*/
GridViewDataWidget::~GridViewDataWidget()
{
    logdbg;
}

/**
*/
void GridViewDataWidget::resetGrid()
{
    grid_.reset();
}

/**
*/
void GridViewDataWidget::resetGridChart()
{
    colormap_.reset();

    legend_->setColorMap(ColorMap());
    legend_->setVisible(false);
    legend_->showSelectionColor(false);

    // deleteLater, not a synchronous reset, see releaseWidgetLater()
    releaseWidgetLater(grid_chart_.release(), layout());

    grid_rendering_ = QImage();
    grid_roi_       = QRectF();

    custom_range_invalid_ = false;
}

/**
*/
void GridViewDataWidget::resetGridLayers()
{
    grid_layers_.clear();

    grid_value_min_.reset();
    grid_value_max_.reset();

    x_axis_name_ = "";
    y_axis_name_ = "";
    title_       = "";
}

/**
 * Snapshot of the layer panel's hidden groups for buildGridFromStash(), which may run
 * on a worker.
 */
void GridViewDataWidget::prepareStashProcessing()
{
    snapshotHiddenLayerIds();
}

/**
 * Qt-side commit after processStash (main thread): the pending layers become current
 * and the layer tree is rebound to them, the old ones are released only afterwards.
 * Hidden groups changed since the snapshot (toggled during the recompute, or new leaves
 * below an unchecked group) => the grid is rebuilt here.
 */
void GridViewDataWidget::commitStashDisplayData()
{
    if (pending_layers_)
    {
        updateLayerTree(std::move(*pending_layers_));
        pending_layers_.reset();
    }

    if (!view_->showsAnnotation() && hiddenLayerIdsChanged())
    {
        snapshotHiddenLayerIds();
        buildGridFromStash();
    }
}

/**
*/
void GridViewDataWidget::resetVariableDisplay() 
{
    resetGridChart();
}

/**
*/
void GridViewDataWidget::resetStashDependentData()
{
    resetGrid();
    resetGridLayers();
    annotation_render_settings_.reset();

    // NOTE: the layer snapshots are intentionally not cleared here (may run on a
    // worker). The layer tree's DBContentLeafItems hold raw pointers into them -
    // only the main thread commit replaces them, after rebinding the tree to the
    // pending ones processStash() built.
}

/**
*/
ViewDataWidget::DrawState GridViewDataWidget::updateVariableDisplay() 
{
    return updateGridChart();
}

/**
*/
void GridViewDataWidget::processStash(const VariableViewStash<double>& stash)
{
    loginf;

    x_axis_name_ = view_->variable(0).description();
    y_axis_name_ = view_->variable(1).description();

    updateAxisInfoFromVariables();
    title_       = "";

    // Build one snapshot per group (total count + #NULL), unconditionally -
    // the panel should show the full dataset breakdown regardless of which
    // layers are currently visible.
    size_t num_null_values = 0;
    LayerSnapshots layers;
    layers.reserve(getStash().groupedStashes().size());

    for (const auto& dbc_values : getStash().groupedStashes())
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

    // Pending until commitStashDisplayData rebinds the tree on the main thread
    // (this function may run on a worker thread): the tree keeps pointing at
    // the current snapshots, which only the commit replaces. A failed or
    // discarded run just leaves its pending layers to be replaced by the next one.
    pending_layers_ = std::move(layers);

    // Aggregate visible groups into the Grid2D.
    buildGridFromStash();
}

/**
*/
void GridViewDataWidget::buildGridFromStash()
{
    resetGrid();
    resetGridLayers();

    // resetGridLayers clears the axis/title strings; restore them so the
    // render path keeps the data widget's view of them consistent.
    x_axis_name_ = view_->variable(0).description();
    y_axis_name_ = view_->variable(1).description();

    updateAxisInfoFromVariables();

    const auto& data_ranges = getStash().dataRanges();

    bool has_data = (hasData() &&
                     getStash().hasData() &&
                     variablesOk() &&
                     data_ranges[ 0 ].has_value() &&
                     data_ranges[ 1 ].has_value() &&
                     data_ranges[ 2 ].has_value());
    if (!has_data)
        return;

    auto bounds = getXYVariableBounds(true);
    if (!bounds.has_value() || bounds->isEmpty())
    {
        loginf << "bounds empty, skipping";
        return;
    }
    traced_assert(bounds->isValid());

    auto z_bounds = getZVariableBounds(false);
    traced_assert(z_bounds.has_value());

    const auto& settings   = view_->settings();
    const auto  value_type = (grid2d::ValueType)settings.value_type;

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
        settings.grid_resolution, settings.grid_resolution);

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

    auto layer_name = grid2d::valueTypeToString((grid2d::ValueType)settings.value_type);
    grid_->addToLayers(grid_layers_, layer_name, (grid2d::ValueType)settings.value_type);

    auto range = grid_layers_.layer(0).range();
    grid_value_min_.reset();
    grid_value_max_.reset();
    if (range.has_value())
    {
        loginf << "grid range min " << range->first << " max " << range->second;
        grid_value_min_ = range->first;
        grid_value_max_ = range->second;
        traced_assert(grid_value_min_.value() <= grid_value_max_.value());
    }

    loginf << "done, generated " << grid_layers_.numLayers() << " layers";
}

/**
 * Notice: not for the hidden states the tree update itself restores (see ViewDataWidget),
 * those are picked up by commitStashDisplayData().
 */
void GridViewDataWidget::layerVisibilityChanged()
{
    // Annotation mode: hiddenChangedSignal here originates from the
    // annotations subtree's radio activation (AnnotationLeafItem). The
    // activation already triggered a full redraw via setCurrentAnnotation ->
    // onShowAnnotationChanged -> redrawData(true), which repopulated
    // grid_layers_ from the new annotation. Running buildGridFromStash() now
    // would wipe that grid by trying to rebuild from the (possibly empty)
    // variable stash, leaving the chart blank.
    if (view_->showsAnnotation())
        return;

    // a recompute worker fills the stash and the grid => its commit picks up the toggle
    if (hasPendingAsyncWork_impl())
        return;

    prepareStashProcessing();

    // Match the scatter plot pattern: a layer toggle only re-aggregates and
    // re-renders, no full recompute redraw. The stash and layer tree are
    // untouched here; we only rebuild the Grid2D from the currently visible
    // groups and redraw the chart.
    buildGridFromStash();
    updateGridChart();
}

/**
*/
bool GridViewDataWidget::updateFromAnnotations()
{
    loginf;

    if (!view_->hasCurrentAnnotation())
        return false;

    const auto& anno = view_->currentAnnotation();

    title_       = anno.metadata.title_;
    x_axis_name_ = anno.metadata.xAxisLabel();
    y_axis_name_ = anno.metadata.yAxisLabel();

    const auto& feature = anno.feature_json;

    if (!feature.is_object() || !feature.contains(ViewPointGenFeatureGrid::FeatureGridFieldNameGrid))
    {
        reportAnnotationError("missing '" + std::string(ViewPointGenFeatureGrid::FeatureGridFieldNameGrid) + "' field");
        return false;
    }

    std::unique_ptr<Grid2DLayer> layer(new Grid2DLayer);
    if (!layer->fromJSON(feature[ ViewPointGenFeatureGrid::FeatureGridFieldNameGrid ]))
    {
        reportAnnotationError("could not read grid layer");
        return false;
    }

    if (layer->data.cols() < 1 ||
        layer->data.rows() < 1)
    {
        reportAnnotationError("grid layer empty");
        return false;
    }
    
    grid_layers_.addLayer(std::move(layer));

    auto range = grid_layers_.layer(0).range();

    grid_value_min_.reset();
    grid_value_max_.reset();

    if (range.has_value())
    {
        loginf << "grid range min " << range->first << " max " << range->second;

        grid_value_min_ = range->first;
        grid_value_max_ = range->second;

        traced_assert(grid_value_min_.value() <= grid_value_max_.value());
    }

    annotation_render_settings_.reset();
    if (feature.contains(ViewPointGenFeatureGrid::FeatureGridFieldNameRenderSettings))
    {
        Grid2DRenderSettings rs;
        if (rs.fromJSON(feature[ ViewPointGenFeatureGrid::FeatureGridFieldNameRenderSettings ]))
            annotation_render_settings_ = rs;
        else
            logwrn << "render_settings present but unparseable; ignored";
    }

    loginf << "done, generated " << grid_layers_.numLayers() << " layers";

    return true;
}

/**
*/
bool GridViewDataWidget::hasValidGrid() const
{
    if (grid_layers_.numLayers() == 0)
        return false;

    const auto& d = grid_layers_.layers().front()->data;

    return (d.cols() > 0 && d.rows() > 0);
}

/**
*/
void GridViewDataWidget::mouseMoveEvent(QMouseEvent* event)
{
    //setCursor(current_cursor_);
    //osgEarth::QtGui::ViewerWidget::mouseMoveEvent(event);

    QWidget::mouseMoveEvent(event);
}

/**
*/
void GridViewDataWidget::toolChanged_impl(int mode)
{
    selected_tool_ = (GridViewDataTool)mode;

    if (grid_chart_)
        grid_chart_->onToolChanged();
}

/**
*/
GridViewDataTool GridViewDataWidget::selectedTool() const
{
    return selected_tool_;
}

/**
*/
QPixmap GridViewDataWidget::renderPixmap()
{
    traced_assert(grid_chart_);
    return grid_chart_->grab();
}

/**
 */
void GridViewDataWidget::updateAxisInfoFromVariables()
{
    for (int axis_id = 0; axis_id < 2; ++axis_id)
    {
        auto& view_var = view_->variable(axis_id);

        auto dtype = view_var.dataType();
        axis_data_type_[ axis_id ] = dtype.has_value() ? dtype.value() : PropertyDataType::DOUBLE;

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

        axis_repr_[ axis_id ] = repr;
    }
}

/**
 */
void GridViewDataWidget::updateAxisTicks(QtCharts::QCategoryAxis* axis, int axis_id)
{
    traced_assert(axis);
    traced_assert(axis_id >= 0 && axis_id < 2);

    const auto dtype = axis_data_type_[ axis_id ];
    const auto repr  = axis_repr_[ axis_id ];

    //drop the ticks of the previous range, categories are keyed by their label
    const auto old_labels = axis->categoriesLabels();
    for (const auto& l : old_labels)
        axis->remove(l);

    const double vmin = axis->min();
    const double vmax = axis->max();

    auto ticks       = axis_ticks::generate(vmin, vmax, dtype, repr);
    auto tick_labels = axis_ticks::labels(ticks, dtype, repr);

    traced_assert(ticks.values.size() == tick_labels.size());

    //a category must end above the start value, so keep the start just below
    const double span  = std::max(std::fabs(vmax - vmin), 1e-9);
    const double start = vmin - span * 1e-9;

    axis->setStartValue(start);

    std::set<QString> used;

    for (size_t i = 0; i < ticks.values.size(); ++i)
    {
        if (ticks.values[ i ] <= start)
            continue;

        QString label = QString::fromStdString(tick_labels[ i ]);

        //Qt keys categories by label and drops a duplicate without a word
        if (used.count(label))
            continue;

        used.insert(label);

        axis->append(label, ticks.values[ i ]);
    }
}

/**
*/
boost::optional<QRectF> GridViewDataWidget::getXYVariableBounds(bool fix_small_ranges) const
{
    return getPlanarVariableBounds(0, 1, false, fix_small_ranges);
}

/**
*/
boost::optional<std::pair<double, double>> GridViewDataWidget::getZVariableBounds(bool fix_small_ranges) const
{
    return getVariableBounds(2, false, fix_small_ranges);
}

/**
*/
void GridViewDataWidget::rectangleSelectedSlot (QPointF p1, QPointF p2)
{
    loginf;

    if (grid_chart_ && grid_chart_->chart())
    {
        if (selected_tool_ == GV_ZOOM_RECT_TOOL)
        {
            loginf << "zoom";

            //TODO: prevent from going nuts when zero rect is passed!
            grid_chart_->zoom(p1, p2);
        }
        else if (selected_tool_ == GV_SELECT_TOOL)
        {
            loginf << "select";

            selectData(std::min(p1.x(), p2.x()), std::max(p1.x(), p2.x()), std::min(p1.y(), p2.y()), std::max(p1.y(), p2.y()), 0, 1);
        }
        else
        {
            throw std::runtime_error("GridViewDataWidget: rectangleSelectedSlot: unknown tool " + std::to_string((unsigned int)selected_tool_));
        }
    }

    endTool();
}


/**
*/
void GridViewDataWidget::resetZoomSlot()
{
    loginf;

    if (grid_chart_)
        grid_chart_->resetZoom();
}


/**
*/
ViewDataWidget::DrawState GridViewDataWidget::updateGridChart()
{
    resetGridChart();
    updateRendering();

    QtCharts::QChart* chart = new QtCharts::QChart();
    auto draw_state = updateChart(chart);
    
    grid_chart_.reset(new QtCharts::GridViewChart(this, chart));
    grid_chart_->setObjectName("chart_view");
    grid_chart_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect (grid_chart_.get(), &QtCharts::GridViewChart::rectangleSelectedSignal,
             this, &GridViewDataWidget::rectangleSelectedSlot, Qt::ConnectionType::QueuedConnection);

    main_layout_->insertWidget(0, grid_chart_.get());

    if (colormap_.has_value())
    {
        auto dtype = view_->currentLegendDataType();
        auto repr  = view_->currentLegendRepresentation();

        //the legend labels read like an axis: the representation decides the
        //form, the value range decides how many decimals carry information
        axis_ticks::LabelStyle style;
        style.decimals = GridView::DecimalsDefault;

        const auto& range = colormap_->valueRange();
        if (range.has_value() && range->second > range->first)
            style.decimals = axis_ticks::generate(range->first,
                                                  range->second,
                                                  dtype,
                                                  repr,
                                                  (int)std::max((size_t)2, colormap_->numColors())).style.decimals;

        //update legend widget
        auto decoratorFunc = [ = ] (double v)
        {
            return axis_ticks::label(v, dtype, repr, style);
        };

        legend_->setColorMap(colormap_.value());
        legend_->setDecorator(decoratorFunc);
        legend_->setVisible(true);
    }

    return draw_state;
}

/**
*/
void GridViewDataWidget::updateRendering()
{
    loginf << "rendering";

    custom_range_invalid_ = false;
    colormap_.reset();

    //no valid grid no rendering
    if (!hasValidGrid())
        return;

    const auto& layer    = grid_layers_.layer(0);
    const auto& settings = view_->settings();

    loginf << "input range " 
           << "min = " 
           << (grid_value_min_.has_value() ? std::to_string(grid_value_min_.value()) : "undef") << " "
           << "max = " 
           << (grid_value_max_.has_value() ? std::to_string(grid_value_max_.value()) : "undef");

    Grid2DRenderSettings render_settings;

    //if there is a valid range for the layer values we can do a little extra stuff
    if (grid_value_min_.has_value() && grid_value_max_.has_value())
    {
        //combine data range with user override range
        std::pair<double, double> range(grid_value_min_.value(), grid_value_max_.value());

        //annotation-provided min/max take priority over view UI min/max
        boost::optional<double> ann_min, ann_max;
        if (annotation_render_settings_.has_value())
        {
            ann_min = annotation_render_settings_->min_value;
            ann_max = annotation_render_settings_->max_value;
        }

        auto vmin = ann_min.has_value() ? ann_min : view_->getMinValue();
        auto vmax = ann_max.has_value() ? ann_max : view_->getMaxValue();

        if (vmin.has_value() || vmax.has_value())
        {
            if (vmin.has_value() && vmax.has_value() && vmin.value() <= vmax.value())
            {
                range = std::pair<double, double>(vmin.value(), vmax.value());
            }
            else if (vmin.has_value() && !vmax.has_value())
            {
                range.first = vmin.value();

                if (vmin.value() > grid_value_max_.value())
                    range.second = vmin.value();
            }
            else if (vmax.has_value() && !vmin.has_value())
            {
                range.second = vmax.value();

                if (vmax.value() < grid_value_min_.value())
                    range.first = vmax.value();
            }
            else
            {
                custom_range_invalid_ = true;
            }
        }

        loginf << "combined range min = " << range.first << " max = " << range.second;

        auto dtype = view_->currentLegendDataType();

        //color scale: annotation override -> view setting
        int active_color_scale = settings.render_color_scale;
        if (annotation_render_settings_.has_value()
            && annotation_render_settings_->color_map.valid())
        {
            active_color_scale = static_cast<int>(
                annotation_render_settings_->color_map.colorScale());
        }

        //number of color steps: annotation override -> derived from ui
        size_t num_steps = 0;
        if (annotation_render_settings_.has_value()
            && annotation_render_settings_->color_map.valid())
        {
            num_steps = annotation_render_settings_->color_map.colorSteps();
        }
        else
        {
            num_steps = property_templates::suggestedNumColorSteps(
                dtype, range.first, range.second, settings.render_color_num_steps);
        }

        loginf << "suggested color steps = " << num_steps;

        //create color map
        if (num_steps == 1)
        {
            //single value
            render_settings.color_map.create((colorscale::ColorScale)active_color_scale,
                                            1,
                                            ColorMap::Type::LinearSamples,
                                            range);
        }
        else if (num_steps == 2)
        {
            //binary
            render_settings.color_map.create((colorscale::ColorScale)active_color_scale,
                                            2,
                                            ColorMap::Type::Binary,
                                            range);
        }
        else if (num_steps > 0)
        {
            //multi-value
            render_settings.color_map.create((colorscale::ColorScale)active_color_scale,
                                            num_steps,
                                            ColorMap::Type::LinearSamples,
                                            range);
        }

        //logarithmic scale: annotation override -> view setting
        if (annotation_render_settings_.has_value()
            && annotation_render_settings_->color_map.valid())
        {
            render_settings.color_map.setLogScale(annotation_render_settings_->color_map.logScale(),
                                                  annotation_render_settings_->color_map.logFloor());
        }
        else
        {
            render_settings.color_map.setLogScale(settings.render_color_log_scale);
        }

        colormap_ = render_settings.color_map;
    }

    //render grid
    auto rendering = Grid2DLayerRenderer::render(layer, render_settings);

    grid_rendering_ = rendering.first;
    grid_roi_       = rendering.second.getROI(grid_rendering_.width(), grid_rendering_.height());
    grid_north_up_  = rendering.second.is_north_up;
    ref_            = rendering.second;
}

/**
*/
ViewDataWidget::DrawState GridViewDataWidget::updateChart(QtCharts::QChart* chart)
{
    bool has_valid_grid_data = grid_layers_.numLayers() > 0 &&
                               !grid_rendering_.isNull() && 
                               !grid_roi_.isEmpty();
    
    
    bool has_data = has_valid_grid_data && (variablesOk() || view_->showsAnnotation());

    auto draw_state = ViewDataWidget::DrawState::NotDrawn;

    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setBackgroundRoundness(0);
    chart->setDropShadowEnabled(false);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->setTitle(QString::fromStdString(title_));

    auto createAxes = [ & ] ()
    {
        //createDefaultAxes puts the value range on the axes, which the grid
        //image is positioned against. Replacing an axis afterwards would lose
        //that range, so the ranges are carried over by hand.
        chart->createDefaultAxes();

        traced_assert(chart->axes(Qt::Horizontal).size() == 1);
        traced_assert(chart->axes(Qt::Vertical).size() == 1);

        auto replaceByCategoryAxis = [ & ] (int axis_id, Qt::Alignment alignment)
        {
            const auto orientation = (alignment == Qt::AlignBottom) ? Qt::Horizontal : Qt::Vertical;

            auto axis_old = dynamic_cast<QtCharts::QValueAxis*>(chart->axes(orientation).at(0));
            if (!axis_old)
                return;

            //a plain value axis is right for a plain floating point value
            if (!axis_ticks::needsCustomLabels(axis_data_type_[ axis_id ], axis_repr_[ axis_id ]))
                return;

            const double vmin = axis_old->min();
            const double vmax = axis_old->max();

            //QCategoryAxis derives from QValueAxis, so the value space the grid
            //image is drawn against stays exactly the same
            auto axis = new QtCharts::QCategoryAxis;
            axis->setLabelsPosition(QtCharts::QCategoryAxis::AxisLabelsPositionOnValue);

            chart->removeAxis(axis_old);
            chart->addAxis(axis, alignment);

            for (auto series : chart->series())
                series->attachAxis(axis);

            axis->setRange(vmin, vmax);

            connect(axis, &QtCharts::QCategoryAxis::rangeChanged, this,
                    [ this, chart, axis, axis_id ] (qreal, qreal)
                    {
                        updateAxisTicks(axis, axis_id);

                        //the labels changed, so the room they need changed too
                        ::ChartView::reserveHorizontalLabelMargins(chart);
                    });

            updateAxisTicks(axis, axis_id);
        };

        replaceByCategoryAxis(0, Qt::AlignBottom);
        replaceByCategoryAxis(1, Qt::AlignLeft);

        //config x axis
        loginf << "title x ' "
               << view_->variable(0).description() << "'";
        chart->axes(Qt::Horizontal).at(0)->setTitleText(x_axis_name_.c_str());

        //config y axis
        loginf << "title y ' "
               << view_->variable(1).description() << "'";
        chart->axes(Qt::Vertical).at(0)->setTitleText(y_axis_name_.c_str());

        ::ChartView::reserveHorizontalLabelMargins(chart);
    };

    if (has_data)
    {
        chart->legend()->setVisible(false);

        QtCharts::QScatterSeries* series = new QtCharts::QScatterSeries;
        series->setMarkerSize(0.1);
        series->setColor(Qt::white);

        series->append(grid_roi_.topLeft());
        series->append(grid_roi_.topRight());
        series->append(grid_roi_.bottomRight());
        series->append(grid_roi_.bottomLeft());

        chart->addSeries(series);

        createAxes();

        draw_state = ViewDataWidget::DrawState::DrawnContent;
    }
    else
    {
        //no data -> generate default empty layout
        chart->legend()->setVisible(false);

        QtCharts::QScatterSeries* series = new QtCharts::QScatterSeries;
        chart->addSeries(series);

        createAxes();

        chart->axes(Qt::Horizontal).at(0)->setLabelsVisible(false);
        chart->axes(Qt::Horizontal).at(0)->setGridLineVisible(false);
        chart->axes(Qt::Horizontal).at(0)->setMinorGridLineVisible(false);

        chart->axes(Qt::Vertical).at(0)->setLabelsVisible(false);
        chart->axes(Qt::Vertical).at(0)->setGridLineVisible(false);
        chart->axes(Qt::Vertical).at(0)->setMinorGridLineVisible(false);

        draw_state = ViewDataWidget::DrawState::Drawn;
    }

    chart->update();

    return draw_state;
}

/**
 */
void GridViewDataWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableViewStashDataWidget::viewInfoJSON_impl(info);

    info[ "num_valid"   ] = getStash().valid_count_;
    info[ "num_selected"] = getStash().selected_count_;
    info[ "num_nan"     ] = getStash().nan_value_count_;

    auto xy_bounds    = getXYVariableBounds(false);
    auto z_bounds     = getZVariableBounds(false);
    bool bounds_valid = xy_bounds.has_value() && xy_bounds->isValid() && z_bounds.has_value();

    info[ "data_bounds_valid" ] = bounds_valid;
    info[ "data_bounds_xmin"  ] = bounds_valid ? xy_bounds->left()        : 0.0;
    info[ "data_bounds_ymin"  ] = bounds_valid ? xy_bounds->top()         : 0.0;
    info[ "data_bounds_zmin"  ] = bounds_valid ?  z_bounds.value().first  : 0.0;
    info[ "data_bounds_xmax"  ] = bounds_valid ? xy_bounds->right()       : 0.0;
    info[ "data_bounds_ymax"  ] = bounds_valid ? xy_bounds->bottom()      : 0.0;
    info[ "data_bounds_zmax"  ] = bounds_valid ?  z_bounds.value().second : 0.0;

    // the grid built from the stash, the value type of the view applied per cell
    bool grid_range_valid = grid_ && grid_value_min_.has_value() && grid_value_max_.has_value();

    info[ "grid_valid"       ] = grid_ != nullptr;
    info[ "grid_num_added"   ] = grid_ ? grid_->numAdded()      : (size_t)0;
    info[ "grid_num_oor"     ] = grid_ ? grid_->numOutOfRange() : (size_t)0;
    info[ "grid_num_inf"     ] = grid_ ? grid_->numInf()        : (size_t)0;
    info[ "grid_num_layers"  ] = grid_layers_.numLayers();
    info[ "grid_value_type"  ] = grid2d::valueTypeToString((grid2d::ValueType)view_->settings().value_type);
    info[ "grid_range_valid" ] = grid_range_valid;
    info[ "grid_value_min"   ] = grid_range_valid ? grid_value_min_.value() : 0.0;
    info[ "grid_value_max"   ] = grid_range_valid ? grid_value_max_.value() : 0.0;

    auto zoomActive = [ & ] (const QRectF& bounds_data, const QRectF& bounds_axis)
    {
        if (!bounds_data.isValid() || !bounds_axis.isValid())
            return false;

        return (bounds_axis.left()   > bounds_data.left()  ||
                bounds_axis.right()  < bounds_data.right() ||
                bounds_axis.top()    > bounds_data.top()   ||
                bounds_axis.bottom() < bounds_data.bottom());
    };

    bool has_axes = grid_chart_ &&
                    grid_chart_->chart() &&
                   !grid_chart_->chart()->axes(Qt::Horizontal).empty() &&
                   !grid_chart_->chart()->axes(Qt::Vertical).empty();

    QRectF axis_bounds;

    if (has_axes)
    {
        auto axis_x = grid_chart_->chart()->axes(Qt::Horizontal).first();
        auto axis_y = grid_chart_->chart()->axes(Qt::Vertical).first();

        auto axis_value_x = dynamic_cast<QtCharts::QValueAxis*>(axis_x);
        auto axis_value_y = dynamic_cast<QtCharts::QValueAxis*>(axis_y);

        if (axis_value_x && axis_value_y)
            axis_bounds = QRectF(axis_value_x->min(),
                                 axis_value_y->min(),
                                 axis_value_x->max() - axis_value_x->min(),
                                 axis_value_y->max() - axis_value_y->min());

        //Qt keeps the tick text of a plain value axis to itself, only a category
        //axis can report what it actually shows
        auto axisType = [ ] (QtCharts::QAbstractAxis* axis)
        {
            if (dynamic_cast<QtCharts::QCategoryAxis*>(axis))
                return std::string("category");

            return std::string("value");
        };

        auto axisTickLabels = [ ] (QtCharts::QAbstractAxis* axis)
        {
            nlohmann::json labels = nlohmann::json::array();

            if (auto axis_cat = dynamic_cast<QtCharts::QCategoryAxis*>(axis))
                for (const auto& l : axis_cat->categoriesLabels())
                    labels.push_back(l.toStdString());

            return labels;
        };

        nlohmann::json chart_info;

        chart_info[ "x_axis_label"       ] = axis_x->titleText().toStdString();
        chart_info[ "y_axis_label"       ] = axis_y->titleText().toStdString();
        chart_info[ "x_axis_type"        ] = axisType(axis_x);
        chart_info[ "y_axis_type"        ] = axisType(axis_y);
        chart_info[ "x_axis_tick_labels" ] = axisTickLabels(axis_x);
        chart_info[ "y_axis_tick_labels" ] = axisTickLabels(axis_y);

        info[ "chart" ] = chart_info;
    }

    info[ "axis_bounds_valid" ] = has_axes && axis_bounds.isValid();
    info[ "axis_bounds_xmin"  ] = has_axes ? axis_bounds.left()   : 0.0;
    info[ "axis_bounds_ymin"  ] = has_axes ? axis_bounds.top()    : 0.0;
    info[ "axis_bounds_xmax"  ] = has_axes ? axis_bounds.right()  : 0.0;
    info[ "axis_bounds_ymax"  ] = has_axes ? axis_bounds.bottom() : 0.0;

    info[ "axis_zoom_active"  ] = bounds_valid && has_axes ? zoomActive(xy_bounds.value(), axis_bounds) : false;

    //the legend as the user reads it next to the grid
    nlohmann::json legend_entries = nlohmann::json::array();

    if (legend_)
        for (const auto& entry : legend_->currentLegend().entries())
            legend_entries.push_back(entry.second);

    info[ "legend_entries" ] = legend_entries;
}

/**
*/
boost::optional<std::pair<QImage, RasterReference>> GridViewDataWidget::currentGeoImage() const
{
    return std::make_pair(grid_rendering_, ref_);
}

/**
*/
const ColorLegend& GridViewDataWidget::currentLegend() const
{
    traced_assert(legend_);
    return legend_->currentLegend();
}

