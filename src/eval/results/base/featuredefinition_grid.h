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

#include "eval/results/base/featuredefinition.h"
#include "eval/results/base/featuredefinition_t.h"

#include "sectorlayer.h"

#include "view/gridview/grid2d.h"
#include "view/gridview/grid2dlayer.h"
#include "view/gridview/grid2drendersettings.h"
#include "view/gridview/grid2dlayerrenderer.h"

#include "view/points/viewpointgenerator.h"

#include "evaluationcalculator.h"

#include <string>

namespace EvaluationRequirementResult
{

/**
 * How to add detail positions to a grid.
 */
enum class GridAddDetailMode
{
    AddEvtPosition = 0,     // add event position 
    AddEvtRefPosition,      // add event ref position
    AddPositionsAsPolyLine  // add all event positions as polygonal line
};

/**
 * Grid feature definition base class.
*/
template<typename T>
class FeatureDefinitionGridBase : public FeatureDefinition
{
public:
    FeatureDefinitionGridBase(const EvaluationCalculator& calculator,
                              const std::string& description_type,
                              const std::string& feature_description,
                              bool generate_geoimage,
                              const boost::optional<unsigned int>& grid_num_cells_x = boost::optional<unsigned int>(),
                              const boost::optional<unsigned int>& grid_num_cells_y = boost::optional<unsigned int>())
    :   FeatureDefinition    (calculator, description_type, feature_description, "", "")
    ,   grid_num_cells_x_    (grid_num_cells_x)
    ,   grid_num_cells_y_    (grid_num_cells_y)
    ,   generate_geoimage_   (generate_geoimage)
    {
        //default converter
        converter_ = [ & ] (const T& v) { return static_cast<double>(v); };
    }
    virtual ~FeatureDefinitionGridBase() = default;

    /**
    */
    bool isValid() const override final
    {
        //check if all data series are valid
        for (const auto& ds : data_series_)
            if (!ds.isValid())
                return false;

        return true;
    }

    /**
    */
    std::unique_ptr<ViewPointGenFeature> createFeature_impl(const Base* result) const override final
    {
        traced_assert(isValid());
        traced_assert(converter_);

        loginf << "creating grid...";

        //create suitably sized grid
        QRectF roi = gridBounds(result->sectorLayer(), {});

        //loginf << "roi: " << roi.x() << "," << roi.y() << "," << roi.width() << "," << roi.height();

        unsigned int grid_num_cells_x = grid_num_cells_x_.has_value() ? grid_num_cells_x_.value() : calculator().settings().grid_num_cells_x;
        unsigned int grid_num_cells_y = grid_num_cells_y_.has_value() ? grid_num_cells_y_.value() : calculator().settings().grid_num_cells_y;

        auto resolution = grid2d::GridResolution().setCellCount(grid_num_cells_x,
                                                                grid_num_cells_y);
        Grid2D grid;
        bool grid_ok = grid.create(roi, resolution, "wgs84", true);

        //!shall not fail! (otherwise sector bounds might be strange)
        traced_assert(grid_ok);

        loginf << "filling grid...";

        //generate grid layers
        Grid2DLayers layers;
        std::map<std::string, Grid2DRenderSettings> render_settings_map;

        for (const auto& ds : data_series_)
        {
            //reset grid for new data layer
            grid.reset();

            bool add_as_polys = ds.add_detail_mode == GridAddDetailMode::AddPositionsAsPolyLine;

            //obtain positions + values
            std::vector<std::pair<size_t, size_t>> detail_ranges;
            auto values = ds.getValuesPlusPosition(result, 
                                                   ds.positionMode(),
                                                   add_as_polys ? &detail_ranges : nullptr);
            if (values.empty())
                continue;

            if (add_as_polys)
            {
                //add as per detail polygons
                for (const auto& r : detail_ranges)
                {
                    //skip single positions
                    if (r.second < 2)
                        continue;

                    auto pos_getter = [ & ] (double& x, double& y, size_t idx) 
                    { 
                        x =  values[ r.first + idx ].longitude;
                        y =  values[ r.first + idx ].latitude;
                    };

                    double v = converter_(values[ r.first ].value);

                    grid.addPoly(pos_getter, r.second, v);
                }
            }
            else
            {
                //just add as single values
                for (const auto& pos : values)
                {
                    double v = converter_(pos.value);
                    grid.addValue(pos.longitude, pos.latitude, v);
                }
            }

            //@TODO: check oor cases
            //assert(grid.numOutOfRange() == 0);

            //get render settings and override some values
            Grid2DRenderSettings render_settings = ds.render_settings;

            //store layer
            grid.addToLayers(layers, ds.series_name, ds.value_type);

            //store render settings
            render_settings_map[ ds.series_name ] = render_settings;
        }

        loginf << "creating features...";

        if (layers.numLayers() < 1)
        {
            loginf << "no layers created, skipping...";
            return {};
        }

        const auto& layer = *layers.layers().begin();

        std::unique_ptr<ViewPointGenFeature> feature;

        if (generate_geoimage_)
        {
            //get render settings
            traced_assert(render_settings_map.count(layer->name));
            const auto& rs = render_settings_map.at(layer->name);

            //render layer
            auto render_result = Grid2DLayerRenderer::render(*layer, rs);

            //loginf << "rendered image size: " << render_result.first.width() << "x" << render_result.first.height();
            //render_result.first.save(QString::fromStdString("/home/mcphatty/layer_" + l.first + ".png"));

            //create geo image annotation
            std::unique_ptr<ViewPointGenFeatureGeoImage> geo_image;
            geo_image.reset(new ViewPointGenFeatureGeoImage(render_result.first, render_result.second));

            feature = std::move(geo_image);
        }
        else
        {
            std::unique_ptr<ViewPointGenFeatureGrid> grid_feat;
            grid_feat.reset(new ViewPointGenFeatureGrid(*layer));

            feature = std::move(grid_feat);
        }

        return feature;
    }

protected:
    /**
    */
    void addDataSeriesInternal(const ValueSource<T>& value_source,
                               grid2d::ValueType value_type,
                               GridAddDetailMode add_detail_mode,
                               const Grid2DRenderSettings& rsettings)
    {
        //@TODO: if we had an annotation feature which could hold multiple grid layers we could remove this assert...
        traced_assert(data_series_.empty());

        GridDataSeries ds;

        //@TODO: make real data series possible in the grid annotation, then we can pass the name from the outside
        ds.series_name         = "DataSeries" + std::to_string(data_series_.size()); 
        ds.series_value_source = value_source;

        ds.value_type      = value_type;
        ds.add_detail_mode = add_detail_mode;

        ds.render_settings = rsettings;
        ds.render_settings.show_selected = false;

        data_series_.push_back(ds);
    }

    std::function<double(const T&)> converter_;

private:
    /**
    */
    struct GridDataSeries : public FeatureDefinitionDataSeries<T>
    {
        bool isValid() const override
        {
            return FeatureDefinitionDataSeries<T>::isValid();
        }

        /**
        */
        DetailValuePositionMode positionMode() const
        {
            //convert to position mode
            if (add_detail_mode == GridAddDetailMode::AddEvtPosition)
                return DetailValuePositionMode::EventPosition;
            else if (add_detail_mode == GridAddDetailMode::AddEvtRefPosition)
                return DetailValuePositionMode::EventRefPosition;
            else if (add_detail_mode == GridAddDetailMode::AddPositionsAsPolyLine)
                return DetailValuePositionMode::AllPositions;
            
            return DetailValuePositionMode::EventPosition;
        }

        grid2d::ValueType    value_type      = grid2d::ValueType::ValueTypeMean;
        GridAddDetailMode    add_detail_mode = GridAddDetailMode::AddEvtPosition;
        Grid2DRenderSettings render_settings;
    };

    /**
    */
    QRectF gridBounds(const SectorLayer& sector_layer, const boost::optional<double>& border_factor) const
    {
        auto lat_range = sector_layer.getMinMaxLatitude();
        auto lon_range = sector_layer.getMinMaxLongitude();

        QRectF roi(lon_range.first, lat_range.first, lon_range.second - lon_range.first, lat_range.second - lat_range.first);

        return grid2d::GridResolution::addBorder(roi, border_factor, -180.0, 180.0, -90.0, 90.0);
    };

    std::vector<GridDataSeries> data_series_;

    boost::optional<unsigned int> grid_num_cells_x_;
    boost::optional<unsigned int> grid_num_cells_y_;

    bool generate_geoimage_ = true;
};

/**
 * Grid feature definition.
*/
template<typename T>
class FeatureDefinitionGrid : public FeatureDefinitionGridBase<T>
{
public:
    FeatureDefinitionGrid(const EvaluationManager& eval_manager,
                          const std::string& feature_description,
                          bool generate_geoimage,
                          const boost::optional<unsigned int>& grid_num_cells_x = boost::optional<unsigned int>(),
                          const boost::optional<unsigned int>& grid_num_cells_y = boost::optional<unsigned int>())
    :   FeatureDefinitionGridBase<T>(eval_manager, "grid", feature_description, generate_geoimage, grid_num_cells_x, grid_num_cells_y)
    {
    }
    virtual ~FeatureDefinitionGrid() = default;

    /**
    */
    FeatureDefinitionGrid& addDataSeries(const ValueSource<T>& value_source,
                                         grid2d::ValueType value_type,
                                         GridAddDetailMode add_detail_mode,
                                         const boost::optional<double>& render_value_min = boost::optional<double>(),
                                         const boost::optional<double>& render_value_max = boost::optional<double>(),
                                         ColorMap::ColorScale render_color_scale = FeatureDefinition::ColorScaleDefault,
                                         unsigned int render_color_steps = FeatureDefinition::NumColorStepsDefault)
    {
        Grid2DRenderSettings rsettings;
        rsettings.min_value = render_value_min;
        rsettings.max_value = render_value_max;
        rsettings.color_map.create(render_color_scale, render_color_steps);

        FeatureDefinitionGridBase<T>::addDataSeriesInternal(value_source,
                                                            value_type,
                                                            add_detail_mode,
                                                            rsettings);
        return *this;
    }
};

/**
 * Binary grid definition.
*/
class FeatureDefinitionBinaryGrid : public FeatureDefinitionGridBase<bool>
{
public:
    FeatureDefinitionBinaryGrid(const EvaluationCalculator& calculator,
                                const std::string& feature_description,
                                const boost::optional<unsigned int>& grid_num_cells_x = boost::optional<unsigned int>(),
                                const boost::optional<unsigned int>& grid_num_cells_y = boost::optional<unsigned int>())
    :   FeatureDefinitionGridBase<bool>(calculator, "binary_grid", feature_description, true, grid_num_cells_x, grid_num_cells_y)
    {
        //converts bool -> double
        converter_ = [ & ] (const bool& v) { return v ? 1.0 : 0.0; };
    }
    virtual ~FeatureDefinitionBinaryGrid() = default;

    /**
    */
    FeatureDefinitionBinaryGrid& addDataSeries(const ValueSource<bool>& value_source,
                                               GridAddDetailMode add_detail_mode,
                                               bool invert_values,
                                               const QColor& render_false_color = Qt::red,
                                               const QColor& render_true_color = Qt::green)
    {
        Grid2DRenderSettings rsettings;
        rsettings.min_value = 0.0;
        rsettings.max_value = 1.0;
        rsettings.color_map.create(render_false_color, render_true_color, 2, ColorMap::Type::Binary);

        auto invertFunc = [ invert_values ] (const bool& v)
        {
            return invert_values ? !v : v;
        };

        ValueSource<bool> vsource = value_source;
        vsource.value_tr_func = invertFunc;

        addDataSeriesInternal(vsource,
                              grid2d::ValueType::ValueTypeMin,
                              add_detail_mode,
                              rsettings);
        return *this;
    }
};

}
