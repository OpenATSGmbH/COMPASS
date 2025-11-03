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

#include "rasterreference.h"
#include "colormap.h"

#include <map>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/optional.hpp>

class ColorMap;

/**
 * A grid layer obtained from a Grid2D.
*/
struct Grid2DLayer
{
    bool hasData() const;
    bool hasFlags() const;

    boost::optional<std::pair<double, double>> range() const;

    nlohmann::json toJSON(bool binary = false) const;
    bool fromJSON(const nlohmann::json& obj);

    static const std::string TagName;
    static const std::string TagNumCellsX;
    static const std::string TagNumCellsY;
    static const std::string TagData;
    static const std::string TagDataRaw;
    static const std::string TagFlags;
    static const std::string TagRef;

    std::string     name;  // layer name
    Eigen::MatrixXd data;  // grid data
    Eigen::MatrixXi flags; // data flags
    RasterReference ref;   // geo-reference
};

/**
*/
class Grid2DLayers
{
public:
    typedef std::unique_ptr<Grid2DLayer>               LayerPtr;
    typedef std::vector<LayerPtr>                      Layers;
    typedef std::map<std::string, std::vector<size_t>> LayerMap;

    Grid2DLayers();
    virtual ~Grid2DLayers();

    void clear();
    void addLayer(const std::string& name, 
                  const RasterReference& ref,
                  const Eigen::MatrixXd& data,
                  const Eigen::MatrixXi& flags);
    void addLayer(LayerPtr&& layer);

    size_t numLayers() const;
    const Layers& layers() const;
    const Grid2DLayer& layer(size_t idx) const;
    const Grid2DLayer& layer(const std::string& name) const;
    std::vector<const Grid2DLayer*> layers(const std::string& name) const;
    bool hasLayer(const std::string& name) const;

    nlohmann::json toJSON(bool binary = false) const;
    bool fromJSON(const nlohmann::json& obj);

    static const std::string TagLayers;

private:
    Layers   layers_;
    LayerMap layer_map_;
};
