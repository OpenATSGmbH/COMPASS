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

#include "grid2dlayer.h"
#include "grid2d.h"

#include "logger.h"

#include <iostream>

#include <QString>

/*********************************************************************************
 * Grid2DLayer
 *********************************************************************************/

const std::string Grid2DLayer::TagName      = "name";
const std::string Grid2DLayer::TagNumCellsX = "num_cells_x";
const std::string Grid2DLayer::TagNumCellsY = "num_cells_y";
const std::string Grid2DLayer::TagData      = "data";
const std::string Grid2DLayer::TagDataRaw   = "data_raw";
const std::string Grid2DLayer::TagFlags     = "flags";
const std::string Grid2DLayer::TagRef       = "reference";

/**
*/
bool Grid2DLayer::hasData() const
{
    return (data.cols() > 0 && data.rows() > 0);
}

/**
*/
bool Grid2DLayer::hasFlags() const
{
    return (hasData() && flags.cols() == data.cols() && flags.rows() == data.rows());
}

/**
*/
boost::optional<std::pair<double, double>> Grid2DLayer::range() const
{
    double vmin = std::numeric_limits<double>::max();
    double vmax = std::numeric_limits<double>::lowest();

    const double* d = data.data();

    bool has_range = false;

    size_t n = (size_t)data.size();

    for (size_t i = 0; i < n; ++i)
    {
        if (d[ i ] == Grid2D::InvalidValue)
            continue;

        if (d[ i ] < vmin) vmin = d[ i ];
        if (d[ i ] > vmax) vmax = d[ i ];

        has_range = true;
    }

    if (!has_range)
        return {};

    return std::make_pair(vmin, vmax);
}

/**
 * WGS84 by name ("wgs84", "EPSG:4326"), case and surrounding blanks ignored.
*/
bool Grid2DLayer::isGeographicSRS(const std::string& srs)
{
    const auto s = QString::fromStdString(srs).trimmed().toLower();

    return s == "wgs84" || s == "epsg:4326";
}

/**
 * Returns the geographic region of the cells with a value. Uses the same cell to
 * pixel mapping as Grid2DLayerRenderer::render() (row flip if north is up).
*/
QRectF Grid2DLayer::geoROIOfValidCells() const
{
    if (!hasData())
        return QRectF();

    const int nrows = (int)data.rows();
    const int ncols = (int)data.cols();

    int x_min = ncols, x_max = -1;
    int y_min = nrows, y_max = -1;

    for (int r = 0; r < nrows; ++r)
    {
        for (int c = 0; c < ncols; ++c)
        {
            if (data(r, c) == Grid2D::InvalidValue)
                continue;

            //image row of the cell
            const int y = ref.is_north_up ? nrows - 1 - r : r;

            if (c < x_min) x_min = c;
            if (c > x_max) x_max = c;
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }
    }

    if (x_max < x_min || y_max < y_min)
        return QRectF();

    const double lon_min = ref.img_origin_x +  x_min      * ref.img_pixel_size_x;
    const double lon_max = ref.img_origin_x + (x_max + 1) * ref.img_pixel_size_x;

    double lat_min, lat_max;
    if (ref.is_north_up)
    {
        lat_max = ref.img_origin_y -  y_min      * ref.img_pixel_size_y;
        lat_min = ref.img_origin_y - (y_max + 1) * ref.img_pixel_size_y;
    }
    else
    {
        lat_min = ref.img_origin_y +  y_min      * ref.img_pixel_size_y;
        lat_max = ref.img_origin_y + (y_max + 1) * ref.img_pixel_size_y;
    }

    return QRectF(lat_min, lon_min, lat_max - lat_min, lon_max - lon_min);
}

/**
*/
nlohmann::json Grid2DLayer::toJSON() const
{
    nlohmann::json obj;

    unsigned int nx = (unsigned int)data.cols();
    unsigned int ny = (unsigned int)data.rows();

    obj[ TagName      ] = name;
    obj[ TagRef       ] = ref.toJSON();
    obj[ TagNumCellsX ] = nx;
    obj[ TagNumCellsY ] = ny;

    //write data as json array (text only, the database compresses the stored json)
    nlohmann::json jdata = nlohmann::json::array();

    for (unsigned int y = 0; y < ny; ++y)
    {
        nlohmann::json jrow = nlohmann::json::array();
        for (unsigned int x = 0; x < nx; ++x)
            jrow.push_back(data(y, x));

        jdata.push_back(jrow);
    }

    obj[ TagData ] = jdata;

    return obj;
}

/**
*/
bool Grid2DLayer::fromJSON(const nlohmann::json& obj)
{
    *this = {};

    if (!obj.is_object() ||
        !obj.contains(TagName) || 
        !obj.contains(TagNumCellsX) ||
        !obj.contains(TagNumCellsY) ||
        (!obj.contains(TagData) && !obj.contains(TagDataRaw)) ||
        !obj.contains(TagRef))
        return false;

    name = obj[ TagName ];

    if (!ref.fromJSON(obj[ TagRef ]))
        return false;

    unsigned int nx = obj[ TagNumCellsX ];
    unsigned int ny = obj[ TagNumCellsY ];

    data.resize(ny, nx);
    data.setZero();

    bool read_raw = obj.contains(TagDataRaw);

    if (read_raw)
    {
        //read data binary (only written by older versions, kept for stored results)
        const auto& jdata = obj[ TagDataRaw ];

        if (!jdata.is_string())
            return false;

        //read binary
        std::string str = jdata;

        QByteArray ba_base64(str.data(), str.size());
        QByteArray ba = QByteArray::fromBase64(ba_base64);

        const double* data_raw = (const double*)(ba.data());

        memcpy(data.data(), data_raw, data.size() * sizeof(double));
    }
    else
    {
        //read data from json array
        const auto& jdata = obj[ TagData ];
        if (!jdata.is_array() || jdata.size() != ny)
            return false;

        for (unsigned int y = 0; y < ny; ++y)
        {
            const auto& jrow = jdata[ y ];
            if (!jrow.is_array() || jrow.size() != nx)
                return false;

            for (unsigned int x = 0; x < nx; ++x)
                data(y, x) = jrow[ x ];
        }
    }

    return true;
}

/*********************************************************************************
 * Grid2DLayers
 *********************************************************************************/

const std::string Grid2DLayers::TagLayers = "layers";

/**
*/
Grid2DLayers::Grid2DLayers() = default;

/**
*/
Grid2DLayers::~Grid2DLayers() = default;

/**
*/
void Grid2DLayers::clear()
{
    layers_.clear();
    layer_map_.clear();
}

/**
*/
void Grid2DLayers::addLayer(const std::string& name, 
                            const RasterReference& ref,
                            const Eigen::MatrixXd& data,
                            const Eigen::MatrixXi& flags)
{
    size_t idx = layers_.size();

    LayerPtr l(new Grid2DLayer);
    l->name  = name;
    l->ref   = ref;
    l->data  = data;
    l->flags = flags;

    layer_map_[ name ].push_back(idx);
    layers_.push_back(std::move(l));
}

/**
*/
void Grid2DLayers::addLayer(LayerPtr&& layer)
{
    traced_assert(layer);

    size_t idx = layers_.size();

    layer_map_[ layer->name ].push_back(idx);
    layers_.push_back(std::move(layer));
}

/**
*/
size_t Grid2DLayers::numLayers() const
{
    return layers_.size();
}

/**
*/
const Grid2DLayers::Layers& Grid2DLayers::layers() const
{
    return layers_;
}

/**
*/
const Grid2DLayer& Grid2DLayers::layer(size_t idx) const
{
    const auto& l = layers_.at(idx);
    traced_assert(l);

    return *l;
}

/**
*/
const Grid2DLayer& Grid2DLayers::layer(const std::string& name) const
{
    traced_assert(layer_map_.count(name));
    
    const auto& indices = layer_map_.at(name);
    traced_assert(!indices.empty());

    const auto& l = layers_.at(indices[ 0 ]);
    traced_assert(l);

    return *l;
}

/**
*/
std::vector<const Grid2DLayer*> Grid2DLayers::layers(const std::string& name) const
{
    traced_assert(layer_map_.count(name));
    
    const auto& indices = layer_map_.at(name);

    std::vector<const Grid2DLayer*> lyrs;
    lyrs.reserve(indices.size());

    for (int idx : indices)
        lyrs.push_back(layers_.at(idx).get());

    return lyrs;
}

/**
*/
bool Grid2DLayers::hasLayer(const std::string& name) const
{
    return (layer_map_.count(name) > 0 && layer_map_.at(name).size() > 0);
}

/**
*/
nlohmann::json Grid2DLayers::toJSON() const
{
    nlohmann::json obj;

    nlohmann::json jlayers = nlohmann::json::array();

    for (const auto& l : layers_)
        jlayers.push_back(l->toJSON());

    obj[ TagLayers ] = jlayers;

    return obj;
}

/**
*/
bool Grid2DLayers::fromJSON(const nlohmann::json& obj)
{
    clear();

    if (!obj.is_object() || !obj.contains(TagLayers))
        return false;

    const auto& jlayers = obj[ TagLayers ];
    if (!jlayers.is_array())
        return false;

    for (const auto& jl : jlayers)
    {
        LayerPtr l(new Grid2DLayer);
        if (l->fromJSON(jl))
            return false;

        addLayer(std::move(l));
    }

    return true;
}
