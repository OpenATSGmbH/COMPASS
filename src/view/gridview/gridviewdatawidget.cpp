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
#include "gridviewchart.h"
#include "grid2d.h"
#include "grid2dlayer.h"
#include "grid2drendersettings.h"
#include "grid2dlayerrenderer.h"

#include "viewvariable.h"
#include "viewpointgenerator.h"
#include "colormapwidget.h"
#include "property_templates.h"

#include "buffer.h"

#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"

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

#include <algorithm>

/**
*/
GridViewDataWidget::GridViewDataWidget(GridViewWidget* view_widget,
                                       QWidget* parent,
                                       Qt::WindowFlags f)
:   VariableViewStashDataWidget(view_widget, view_widget->getView(), false, parent, f)
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

    grid_chart_.reset();

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
*/
bool GridViewDataWidget::postLoadTrigger()
{
    //no redraw triggered
    return false;
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

    const auto& data_ranges = getStash().dataRanges();

    x_axis_name_ = view_->variable(0).description();
    y_axis_name_ = view_->variable(1).description();
    title_       = "";

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
        loginf << "bounds empty, skipping...";
        return;
    }
    traced_assert(bounds->isValid());

    auto z_bounds = getZVariableBounds(false);
    traced_assert(z_bounds.has_value());

    const auto& settings = view_->settings();

    grid2d::GridResolution res = grid2d::GridResolution().setCellCount(settings.grid_resolution, settings.grid_resolution);

    grid_.reset(new Grid2D);

    std::string err;
    bool ok = grid_->create(bounds.value(), res, "wgs84", true, &err);

    if (!ok)
        logerr << "creation of grid failed: " << err;

    traced_assert(ok);

    loginf << "created grid of " << grid_->numCellsX() << "x" << grid_->numCellsY();

    size_t num_null_values = 0;

    for (const auto& dbc_values : getStash().groupedStashes())
    {
        const auto& x_values = dbc_values.second.variable_stashes[ 0 ].values;
        const auto& y_values = dbc_values.second.variable_stashes[ 1 ].values;
        const auto& z_values = dbc_values.second.variable_stashes[ 2 ].values;

        traced_assert(x_values.size() == y_values.size() &&
               y_values.size() == z_values.size());

        loginf << "dbcontent " << dbc_values.first
               << " #x " << x_values.size()
               << " #y " << y_values.size()
               << " #z " << z_values.size();

        for (size_t i = 0; i < z_values.size(); ++i)
        {
            if (dbc_values.second.nan_values[ i ])
                ++num_null_values;

            if (!dbc_values.second.isNan(0, i) && !dbc_values.second.isNan(1, i))
                grid_->addValue(x_values[ i ], y_values[ i ], z_values[ i ]);
        }
    }

    addNullCount(num_null_values);

    loginf << "start"
           << " added " << grid_->numAdded() 
           << " oor "   << grid_->numOutOfRange() 
           << " inf "   << grid_->numInf();

    loginf << "getting layer";

    auto layer_name = grid2d::valueTypeToString((grid2d::ValueType)settings.value_type);

    loginf << "value type = " << layer_name;

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
        return false;

    std::unique_ptr<Grid2DLayer> layer(new Grid2DLayer);
    if (!layer->fromJSON(feature[ ViewPointGenFeatureGrid::FeatureGridFieldNameGrid ]))
    {
        logerr << "could not read grid layer";
        return false;
    }

    if (layer->data.cols() < 1 || 
        layer->data.rows() < 1)
    {
        logerr << "grid layer empty";
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
void GridViewDataWidget::invertSelectionSlot()
{
    loginf;

    for (auto& buf_it : viewData())
    {
        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

        for (unsigned int cnt=0; cnt < buf_it.second->size(); ++cnt)
        {
            if (selected_vec.isNull(cnt))
                selected_vec.set(cnt, true);
            else
                selected_vec.set(cnt, !selected_vec.get(cnt));
        }
    }

    emit view_->selectionChangedSignal();
}

/**
*/
void GridViewDataWidget::clearSelectionSlot()
{
    loginf;

    for (auto& buf_it : viewData())
    {
        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

        for (unsigned int cnt=0; cnt < buf_it.second->size(); ++cnt)
            selected_vec.set(cnt, false);
    }

    emit view_->selectionChangedSignal();
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

        //update legend widget
        auto decoratorFunc = [ = ] (double v)
        {
            return property_templates::double2String(dtype, v, GridView::DecimalsDefault);
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

        auto vmin = view_->getMinValue();
        auto vmax = view_->getMaxValue();
        
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

        //derive suggested number of color steps from ui value
        size_t num_steps = property_templates::suggestedNumColorSteps(dtype, 
                                                                      range.first, 
                                                                      range.second, 
                                                                      settings.render_color_num_steps);

        loginf << "suggested color steps = " << num_steps;

        //create color map
        if (num_steps == 1)
        {
            //single value
            render_settings.color_map.create((colorscale::ColorScale)settings.render_color_scale,
                                            1,
                                            ColorMap::Type::LinearSamples,
                                            range);
        }
        else if (num_steps == 2)
        {
            //binary
            render_settings.color_map.create((colorscale::ColorScale)settings.render_color_scale,
                                            2,
                                            ColorMap::Type::Binary,
                                            range);
        }
        else if (num_steps > 0)
        {
            //multi-value
            render_settings.color_map.create((colorscale::ColorScale)settings.render_color_scale,
                                            num_steps,
                                            ColorMap::Type::LinearSamples,
                                            range);
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
        chart->createDefaultAxes();

        //config x axis
        loginf << "title x ' "
               << view_->variable(0).description() << "'";
        traced_assert(chart->axes(Qt::Horizontal).size() == 1);
        chart->axes(Qt::Horizontal).at(0)->setTitleText(x_axis_name_.c_str());

        //config y axis
        loginf << "title y ' "
               << view_->variable(1).description() << "'";
        traced_assert(chart->axes(Qt::Vertical).size() == 1);
        chart->axes(Qt::Vertical).at(0)->setTitleText(y_axis_name_.c_str());
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

    // auto zoomActive = [ & ] (const QRectF& bounds_data, const QRectF& bounds_axis)
    // {
    //     if (!bounds_data.isValid() || !bounds_axis.isValid())
    //         return false;

    //     return (bounds_axis.left()   > bounds_data.left()  ||
    //             bounds_axis.right()  < bounds_data.right() ||
    //             bounds_axis.top()    > bounds_data.top()   ||
    //             bounds_axis.bottom() < bounds_data.bottom());
    // };

    //@TODO
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
