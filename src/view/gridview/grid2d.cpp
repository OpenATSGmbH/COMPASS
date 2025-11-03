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

#include "grid2d.h"
#include "grid2dlayer.h"

#include "logger.h"

#include <iostream>

const double Grid2D::InvalidValue = std::numeric_limits<double>::max();

/**
*/
Grid2D::Grid2D()
{
    vtype_indices_.resize(grid2d::NumValueTypes);
    vtype_indices_[ grid2d::ValueTypeCountValid ] = IndexCountValid;
    vtype_indices_[ grid2d::ValueTypeCountNan   ] = IndexCountNan;
    vtype_indices_[ grid2d::ValueTypeMin        ] = IndexMin;
    vtype_indices_[ grid2d::ValueTypeMax        ] = IndexMax;
    vtype_indices_[ grid2d::ValueTypeMean       ] = IndexMean;
    vtype_indices_[ grid2d::ValueTypeVar        ] = IndexVar;
    vtype_indices_[ grid2d::ValueTypeStddev     ] = IndexStddev;
}

/**
*/
Grid2D::~Grid2D() = default;

/**
*/
bool Grid2D::valid() const
{
    return (n_cells_ > 0);
}

/**
*/
bool Grid2D::create(const QRectF& roi,
                    const grid2d::GridResolution& resolution,
                    const std::string& srs,
                    bool srs_is_north_up,
                    std::string* err)
{
    if (err) *err = "";

    clear();

    if (!roi.isValid())
    {
        if (err) *err = "roi is invalid";
        return false;
    }

    if (roi.isEmpty())
    {
        if (err) *err = "roi is empty";
        return false;
    }
    
    if (!resolution.valid())
    {
        if (err) 
        {
            std::stringstream ss;
            ss << "resolution invalid:" << " nx "     << resolution.num_cells_x 
                                        << " ny "     << resolution.num_cells_y 
                                        << " sx "     << resolution.cell_size_x 
                                        << " sy "     << resolution.cell_size_y
                                        << " border " << resolution.border
                                        << " type "   << (int)resolution.type; 
            *err = ss.str();
        }
        return false;
    }

    //compute needed resolution values and adjusted roi
    QRectF roi_adj = resolution.resolution(n_cells_x_,
                                           n_cells_y_,
                                           cell_size_x_,
                                           cell_size_y_,
                                           roi);

    //loginf << "x0: " << roi.x() << "\n"
    //       << "y0: " << roi.y() << "\n"
    //       << "w0: " << roi.width() << "\n"
    //       << "h0: " << roi.height() << "\n"
    //       << "x: " << roi_adj.x() << "\n"
    //       << "y: " << roi_adj.y() << "\n"
    //       << "w: " << roi_adj.width() << "\n"
    //       << "h: " << roi_adj.height() << "\n"
    //       << "nx: " << n_cells_x_ << "\n"
    //       << "ny: " << n_cells_y_ << "\n"
    //       << "sx: " << cell_size_x_ << "\n"
    //       << "sy: " << cell_size_y_ << "\n";

    if (roi_adj.isEmpty())
    {
        if (err)
        {
            std::stringstream ss;
            ss << "adjusted roi empty: x " << roi_adj.x() << " y " << roi_adj.y() << " w " << roi_adj.width() << " h " << roi_adj.height();
            *err = ss.str();
        }
        return false;
    }

    cell_size_x_inv_ = 1.0 / cell_size_x_;
    cell_size_y_inv_ = 1.0 / cell_size_y_;
    n_cells_         = n_cells_x_ * n_cells_y_;
    x0_              = roi_adj.left();
    y0_              = roi_adj.top();
    x1_              = roi_adj.right();
    y1_              = roi_adj.bottom();

    layers_.resize(NumLayers, Eigen::MatrixXd(n_cells_y_, n_cells_x_));

    flags_.resize(n_cells_y_, n_cells_x_);
    flags_.setConstant(0);

    ref_.set(srs, roi_adj, n_cells_x_, n_cells_y_, srs_is_north_up);

    reset();

    return true;
}

/**
*/
void Grid2D::clear()
{
    layers_.resize(0);
    flags_ = {};

    n_cells_x_ = 0;
    n_cells_y_ = 0;
    n_cells_   = 0;

    num_added_ = 0;
    num_oor_   = 0;
    num_inf_   = 0;
}

/**
*/
void Grid2D::reset()
{
    layers_[ IndexCountValid ].setConstant(0);
    layers_[ IndexCountNan   ].setConstant(0);
    layers_[ IndexMean       ].setConstant(0);
    layers_[ IndexMean2      ].setConstant(0);
    layers_[ IndexMin        ].setConstant(std::numeric_limits<double>::max());
    layers_[ IndexMax        ].setConstant(std::numeric_limits<double>::lowest());

    flags_.setConstant(0);
 
    num_added_ = 0;
    num_oor_   = 0;
    num_inf_   = 0;
}

/**
*/
void Grid2D::setFlags(size_t x, size_t y, unsigned char flags, bool ok)
{
    if (ok)
        flags_(y, x) |= flags;
    else
        flags_(y, x) &= ~flags;
}

/**
*/
void Grid2D::setFlags(const QRectF& roi, unsigned char flags, bool ok)
{
    std::vector<std::pair<size_t, size_t>> idxs;
    indices(idxs, roi);

    for (auto idx : idxs)
        setFlags(idx.first, idx.second, flags, ok);
}

/**
*/
void Grid2D::resetFlags()
{
    flags_.setConstant(0);
}

/**
*/
void Grid2D::select(size_t x, size_t y, bool ok)
{
    setFlags(x, y, grid2d::CellFlags::CellSelected, ok);
}

/**
*/
void Grid2D::select(const QRectF& roi, bool ok)
{
    setFlags(roi, grid2d::CellFlags::CellSelected, ok);
}

/**
*/
QRectF Grid2D::gridBounds() const
{
    if (!valid())
        return QRectF();

    return QRectF(x0_, y0_, x1_ - x0_, y1_ - y0_);
}

/**
 * Returns a portion of the grid which completely covers the given crop rect.
*/
void Grid2D::cropGrid(QRectF& roi, 
                      QRect& region, 
                      const QRectF& crop_rect,
                      const QRectF& grid_roi,
                      int grid_cells_x,
                      int grid_cells_y,
                      bool grid_north_up,
                      bool region_in_image_space,
                      int pixels_per_cell)
{
    roi    = QRectF();
    region = QRect();

    if (grid_roi.isEmpty() || grid_cells_x < 1 || grid_cells_y < 1)
        return;

    double gx0 = grid_roi.left();
    double gy0 = grid_roi.top();
    double gx1 = grid_roi.right();
    double gy1 = grid_roi.bottom();

    if (gx1 < crop_rect.x() ||
        gy1 < crop_rect.y() ||
        gx0 > crop_rect.x() + crop_rect.width() ||
        gy0 > crop_rect.y() + crop_rect.height())
        return;

    double x0 = gx0;
    double y0 = gy0;
    double x1 = gx1;
    double y1 = gy1;

    int img_x0 = 0;
    int img_y0 = 0;
    int img_x1 = grid_cells_x;
    int img_y1 = grid_cells_y;

    int h = grid_cells_y;

    double cell_size_x = (x1 - x0) / grid_cells_x;
    double cell_size_y = (y1 - y0) / grid_cells_y;

    //scale values to multiple pixels per cell
    if (region_in_image_space && pixels_per_cell > 0)
    {
        img_x1 *= pixels_per_cell;
        img_y1 *= pixels_per_cell;

        h *= pixels_per_cell;

        cell_size_x /= pixels_per_cell;
        cell_size_y /= pixels_per_cell;
    }

    double cell_size_x_inv = 1.0 / cell_size_x;
    double cell_size_y_inv = 1.0 / cell_size_y;

    //apply crop rect
    if (x0 < crop_rect.x())
    {
        int nx = std::floor((crop_rect.x() - x0) * cell_size_x_inv);
        if (nx >= 2)
        {
            x0     += nx * cell_size_x;
            img_x0 += nx;
        }
    }
    if (y0 < crop_rect.y())
    {
        int ny = std::floor((crop_rect.y() - y0) * cell_size_y_inv);
        if (ny >= 2)
        {
            y0     += ny * cell_size_y;
            img_y0 += ny;
        }
    }
    if (x1 > crop_rect.right())
    {
        int nx = std::floor((x1 - crop_rect.right()) * cell_size_x_inv);
        if (nx >= 2)
        {
            x1     -= nx * cell_size_x;
            img_x1 -= nx;
        }
    }
    if (y1 > crop_rect.bottom())
    {
        int ny = std::floor((y1 - crop_rect.bottom()) * cell_size_y_inv);
        if (ny >= 2)
        {
            y1     -= ny * cell_size_y;
            img_y1 -= ny;
        }
    }

    if (x0 > x1 || y0 > y1)
        return;

    roi    = QRectF(x0, y0, x1 - x0, y1 - y0);
    region = QRect(img_x0, img_y0, img_x1 - img_x0, img_y1 - img_y0);

    //invert region depending on coord system
    if (region_in_image_space && grid_north_up)
    {
        region = QRect(region.x(),
                       h - img_y1,
                       region.width(),
                       region.height());
    }
}

/**
*/
Grid2D::IndexError Grid2D::index(size_t& idx_x, size_t& idx_y, double x, double y, bool clamp) const
{
    if (!std::isfinite(x) || !std::isfinite(y))
        return IndexError::InfIndex;
    
    //out of range?
    if (!clamp && (x < x0_ || x > x1_ || y < y0_ || y > y1_))
        return IndexError::OOR;

    //clamp to grid bounds?
    if (clamp && x < x0_)
        x = x0_;
    if (clamp && y < y0_)
        y = y0_;
    if (clamp && x > x1_)
        x = x1_;
    if (clamp && y > y1_)
        y = y1_;
    
    idx_x = std::min(n_cells_x_ - 1, static_cast<size_t>((x - x0_) * cell_size_x_inv_));
    idx_y = std::min(n_cells_y_ - 1, static_cast<size_t>((y - y0_) * cell_size_y_inv_));
    
    return IndexError::NoError;
}

/**
*/
void Grid2D::indices(std::vector<std::pair<size_t, size_t>>& indices, const QRectF& roi) const
{
    indices.clear();

    auto grid_bounds = gridBounds();
    if (!grid_bounds.contains(roi) &&
        !roi.contains(grid_bounds) &&
        !roi.intersects(grid_bounds))
        return;

    size_t x00, y00,
           x10, y10,
           x11, y11,
           x01, y01;

    auto err0 = index(x00, y00, roi.left() , roi.top()   , true);
    auto err1 = index(x10, y10, roi.right(), roi.top()   , true);
    auto err2 = index(x11, y11, roi.right(), roi.bottom(), true);
    auto err3 = index(x01, y01, roi.left() , roi.bottom(), true);

    traced_assert(err0 == IndexError::NoError &&
           err1 == IndexError::NoError &&
           err2 == IndexError::NoError &&
           err3 == IndexError::NoError);

    size_t x0 = std::min(x00, std::min(x10, std::min(x11, x01)));
    size_t y0 = std::min(y00, std::min(y10, std::min(y11, y01)));
    size_t x1 = std::max(x00, std::max(x10, std::max(x11, x01)));
    size_t y1 = std::max(y00, std::max(y10, std::max(y11, y01)));

    traced_assert(x0 <= x1);
    traced_assert(y0 <= y1);

    indices.reserve((x1 - x0 + 1) * (y1 - y0 + 1));

    for (size_t y = y0; y <= y1; ++y)
        for (size_t x = x0; x <= x1; ++x)
            indices.emplace_back(x, y);
}

/**
*/
Grid2D::IndexError Grid2D::checkAdd(size_t& idx_x, size_t& idx_y, double x, double y, double v)
{
    traced_assert(valid());

    auto err = index(idx_x, idx_y, x, y);

    if (err == IndexError::OOR)
    {
        //logerr << "position oor: " << x << "," << y;
        ++num_oor_;
    }
    else if (err == IndexError::InfIndex)
        ++num_inf_;

    if (err != IndexError::NoError)
        return err;

    if (!std::isfinite(v))
    {
        ++num_inf_;
        return IndexError::InfValue;
    }

    return IndexError::NoError;
}

/**
*/
bool Grid2D::addValue(double x, double y, double v)
{
    size_t idx_x, idx_y;
    auto err = checkAdd(idx_x, idx_y, x, y, v);

    if (err == IndexError::InfValue)
    {
        //value is inf => log in cell
        auto& count_inf = layers_[ IndexCountNan ](idx_y, idx_x);
        count_inf += 1;
    }
    
    if (err != IndexError::NoError)
        return false;

    auto& vmin  = layers_[ IndexMin        ](idx_y, idx_x);
    auto& vmax  = layers_[ IndexMax        ](idx_y, idx_x);
    auto& count = layers_[ IndexCountValid ](idx_y, idx_x);
    auto& mean  = layers_[ IndexMean       ](idx_y, idx_x);
    auto& mean2 = layers_[ IndexMean2      ](idx_y, idx_x);

    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;

    //welford's online algorithm
    //https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
    count += 1;
    double delta = v - mean;
    mean += delta / count;
    double delta2 = v - mean;
    mean2 += delta * delta2;
    
    ++num_added_;

    return true;
}

/**
*/
size_t Grid2D::addLineInternal(double x0, 
                               double y0, 
                               double x1, 
                               double y1, 
                               double v, 
                               std::set<std::pair<size_t, size_t>>& visited,
                               int subsampling)
{
    if (!std::isfinite(x0) ||
        !std::isfinite(y0) ||
        !std::isfinite(x1) ||
        !std::isfinite(y1))
        return 0;

    int cells_x = std::max(1, (int)std::ceil(std::fabs(x1 - x0) * cell_size_x_inv_));
    int cells_y = std::max(1, (int)std::ceil(std::fabs(y1 - y0) * cell_size_y_inv_));
    int samples = std::max(cells_x, cells_y) * 2 * subsampling;

    size_t idx_x, idx_y;
    size_t added = 0;
    for (int i = 0; i <= samples; ++i)
    {
        double t1 = (double)i / (double)samples;
        double t0 = (1.0 - t1);
        double  x = t0 * x0 + t1 * x1;
        double  y = t0 * y0 + t1 * y1;

        if (index(idx_x, idx_y, x, y) != Grid2D::IndexError::NoError)
            continue;

        if (visited.find(std::make_pair(idx_x, idx_y)) != visited.end())
            continue;

        if (!addValue(x, y, v))
            continue;

        visited.insert(std::make_pair(idx_x, idx_y));
        ++added;
    }

    return added;
}

/**
*/
size_t Grid2D::addLine(double x0, 
                       double y0, 
                       double x1, 
                       double y1, 
                       double v, 
                       int subsampling)
{
    std::set<std::pair<size_t, size_t>> visited;
    return addLineInternal(x0, y0, x1, y1, v, visited, subsampling);
}

/**
*/
size_t Grid2D::addPoly(const std::vector<Eigen::Vector2d>& positions,
                       double v, 
                       int subsampling)
{
    std::set<std::pair<size_t, size_t>> visited;
    size_t added = 0;
    for (size_t i = 1; i < positions.size(); ++i)
    {
        added += addLineInternal(positions[ i - 1 ][ 0 ],
                                 positions[ i - 1 ][ 1 ],
                                 positions[ i     ][ 0 ],
                                 positions[ i     ][ 1 ],
                                 v,
                                 visited,
                                 subsampling);
    }
    
    return added;
}

/**
*/
size_t Grid2D::addPoly(const std::function<void(double&, double&, size_t)>& pos_getter,
                       size_t n,
                       double v, 
                       int subsampling)
{
    traced_assert(pos_getter);

    std::set<std::pair<size_t, size_t>> visited;
    double x0, y0, x1, y1;
    size_t added = 0;
    for (size_t i = 1; i < n; ++i)
    {
        pos_getter(x0, y0, i - 1);
        pos_getter(x1, y1, i    );
        added += addLineInternal(x0, y0, x1, y1, v, visited, subsampling);
    }

    return added;
}

/**
*/
// bool Grid2D::setValue(double x, double y, double v)
// {
//     size_t idx_x, idx_y;
//     if (checkAdd(idx_x, idx_y, x, y, v) != IndexError::NoError)
//         return false;

//     layers_[ IndexValue      ](idx_y, idx_x) = v;
//     layers_[ IndexCountValid ](idx_y, idx_x) = 1;

//     return true;
// }

/**
*/
bool Grid2D::setCount(double x, double y, size_t count)
{
    size_t idx_x, idx_y;
    if (checkAdd(idx_x, idx_y, x, y, 0.0) != IndexError::NoError)
        return false;

    layers_[ IndexCountValid ](idx_y, idx_x) = (double)count;

    return true;
}

/**
*/
Eigen::MatrixXd Grid2D::getValues(grid2d::ValueType vtype) const
{
    ValueIndex vindex = vtype_indices_[ vtype ];

    Eigen::MatrixXd l = layers_[ vindex ];
    double* d = l.data();

    const auto&   count_valid  = layers_[ IndexCountValid ];
    //const auto&   count_nan    = layers_[ IndexCountNan   ];
    const double* dcount_valid = count_valid.data();
    //const double* dcount_nan   = count_nan.data();

    size_t n = (size_t)l.size();

    if (vtype == grid2d::ValueType::ValueTypeCountNan)
    {
        for (size_t i = 0; i < n; ++i)
            if (d[ i ] == 0)
                d[ i ] = InvalidValue;
    }
    else
    {
        for (size_t i = 0; i < n; ++i)
            if(dcount_valid[ i ] == 0)
                d[ i ] = InvalidValue;
        
        if (vtype == grid2d::ValueType::ValueTypeVar)
        {
            for (size_t i = 0; i < n; ++i)
                if (d[ i ] != InvalidValue)
                    d[ i ] /= dcount_valid[ i ];
        }
        else if (vtype == grid2d::ValueType::ValueTypeStddev)
        {
            for (size_t i = 0; i < n; ++i)
                if (d[ i ] != InvalidValue)
                    d[ i ] = std::sqrt(d[ i ] / dcount_valid[ i ]);
        }
    }

    return l;
}

/**
*/
const RasterReference& Grid2D::getReference() const
{
    return ref_;
}

/**
*/
const Eigen::MatrixXi& Grid2D::getFlags() const
{
    return flags_;
}

/**
*/
std::unique_ptr<Grid2DLayer> Grid2D::createLayer(const std::string& layer_name,
                                                 grid2d::ValueType vtype) const
{
    traced_assert(!layer_name.empty());

    std::unique_ptr<Grid2DLayer> layer(new Grid2DLayer);
    layer->name  = layer_name;
    layer->data  = getValues(vtype);
    layer->flags = getFlags();
    layer->ref   = getReference();

    return layer;
}

/**
*/
void Grid2D::addToLayers(Grid2DLayers& layers, 
                         const std::string& layer_name,
                         grid2d::ValueType vtype) const
{
    layers.addLayer(createLayer(layer_name, vtype));
}
