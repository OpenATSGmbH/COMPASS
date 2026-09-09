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

#include "targetreport3dgrid.h"

#include "grid2d.h"
#include "grid2d_defs.h"
#include "grid2dlayer.h"

#include <QRectF>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace
{
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kDegPerRad = 180.0 / M_PI;
}

TargetReport3DGrid::TargetReport3DGrid(double cell_size_m, double cell_size_ft, double ref_lat_deg)
    : cell_size_m_(cell_size_m), cell_size_ft_(cell_size_ft), ref_lat_deg_(ref_lat_deg)
{
    dlat_deg_ = (cell_size_m / kEarthRadiusM) * kDegPerRad;
    const double cos_lat = std::cos(ref_lat_deg * M_PI / 180.0);
    const double safe_cos = (std::abs(cos_lat) > 1e-6) ? cos_lat : 1.0;
    dlon_deg_ = ((cell_size_m / kEarthRadiusM) / safe_cos) * kDegPerRad;
}

TargetReport3DGrid::Key
TargetReport3DGrid::keyFor(double lat_deg, double lon_deg, double baro_alt_ft) const
{
    Key k;
    k.lat_bin = static_cast<std::int32_t>(std::floor(lat_deg / dlat_deg_));
    k.lon_bin = static_cast<std::int32_t>(std::floor(lon_deg / dlon_deg_));
    k.alt_bin = static_cast<std::int32_t>(std::floor(baro_alt_ft / cell_size_ft_));
    return k;
}

TargetReport3DGrid::Cell&
TargetReport3DGrid::cellAt(double lat_deg, double lon_deg, double baro_alt_ft)
{
    return cells_[keyFor(lat_deg, lon_deg, baro_alt_ft)];
}

void TargetReport3DGrid::addEUI(double lat_deg, double lon_deg, double baro_alt_ft)
{
    cellAt(lat_deg, lon_deg, baro_alt_ft).num_eui += 1;
}

void TargetReport3DGrid::addMUI(double lat_deg, double lon_deg, double baro_alt_ft)
{
    cellAt(lat_deg, lon_deg, baro_alt_ft).num_mui += 1;
}

void TargetReport3DGrid::addExtra(double lat_deg, double lon_deg, double baro_alt_ft,
                                  std::uint64_t num)
{
    cellAt(lat_deg, lon_deg, baro_alt_ft).num_extra += num;
}

void TargetReport3DGrid::addAccuracySample(double lat_deg, double lon_deg, double baro_alt_ft,
                                           double distance_m, double reported_stddev_m)
{
    auto& c = cellAt(lat_deg, lon_deg, baro_alt_ft);
    ++c.num_samples;

    if (std::isfinite(distance_m))
    {
        c.sum_distance_m += distance_m;
        ++c.num_distance;
    }

    if (std::isfinite(reported_stddev_m) && reported_stddev_m > 0.0)
    {
        c.sum_stddev_m += reported_stddev_m;
        ++c.num_stddev;

        if (std::isfinite(distance_m) && reported_stddev_m > 1e-6)
        {
            c.sum_ratio += distance_m / reported_stddev_m;
            ++c.num_ratio;
        }
    }
}

void TargetReport3DGrid::addOffsetSample(double lat_deg, double lon_deg, double baro_alt_ft,
                                         double radial_m, double tangential_m)
{
    if (!std::isfinite(radial_m) || !std::isfinite(tangential_m))
        return;

    auto& c = cellAt(lat_deg, lon_deg, baro_alt_ft);
    c.sum_radial_m     += radial_m;
    c.sum_tangential_m += tangential_m;
    ++c.num_offset;
}

std::uint64_t TargetReport3DGrid::pack2(std::int32_t a, std::int32_t b)
{
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32)
         | static_cast<std::uint64_t>(static_cast<std::uint32_t>(b));
}

std::pair<std::int32_t, std::int32_t> TargetReport3DGrid::unpack2(std::uint64_t key)
{
    auto lo = static_cast<std::int32_t>(key & 0xFFFFFFFFULL);
    auto hi = static_cast<std::int32_t>((key >> 32) & 0xFFFFFFFFULL);
    return { hi, lo };
}

std::uint64_t TargetReport3DGrid::horizontalKey(double lat_deg, double lon_deg) const
{
    Key k = keyFor(lat_deg, lon_deg, 0.0);
    return pack2(k.lat_bin, k.lon_bin);
}

std::pair<double, double> TargetReport3DGrid::horizontalCellCenter(std::uint64_t key) const
{
    std::int32_t lat_bin, lon_bin;
    std::tie(lat_bin, lon_bin) = unpack2(key);
    return { (static_cast<double>(lat_bin) + 0.5) * dlat_deg_,
             (static_cast<double>(lon_bin) + 0.5) * dlon_deg_ };
}

namespace
{
inline void mergeInto(TargetReport3DGrid::Cell& dst, const TargetReport3DGrid::Cell& src)
{
    dst.num_eui          += src.num_eui;
    dst.num_mui          += src.num_mui;
    dst.num_extra        += src.num_extra;
    dst.num_samples      += src.num_samples;
    dst.num_distance     += src.num_distance;
    dst.sum_distance_m   += src.sum_distance_m;
    dst.num_stddev       += src.num_stddev;
    dst.sum_stddev_m     += src.sum_stddev_m;
    dst.num_ratio        += src.num_ratio;
    dst.sum_ratio        += src.sum_ratio;
    dst.num_offset       += src.num_offset;
    dst.sum_radial_m     += src.sum_radial_m;
    dst.sum_tangential_m += src.sum_tangential_m;
}
}

std::unordered_map<std::uint64_t, TargetReport3DGrid::Cell>
TargetReport3DGrid::projectHorizontal() const
{
    std::unordered_map<std::uint64_t, Cell> out;
    for (const auto& kv : cells_)
        mergeInto(out[pack2(kv.first.lat_bin, kv.first.lon_bin)], kv.second);
    return out;
}

std::unordered_map<std::uint64_t, TargetReport3DGrid::Cell>
TargetReport3DGrid::projectAltLon() const
{
    std::unordered_map<std::uint64_t, Cell> out;
    for (const auto& kv : cells_)
        mergeInto(out[pack2(kv.first.alt_bin, kv.first.lon_bin)], kv.second);
    return out;
}

std::unordered_map<std::uint64_t, TargetReport3DGrid::Cell>
TargetReport3DGrid::projectAltLat() const
{
    std::unordered_map<std::uint64_t, Cell> out;
    for (const auto& kv : cells_)
        mergeInto(out[pack2(kv.first.alt_bin, kv.first.lat_bin)], kv.second);
    return out;
}

TargetReport3DGrid::ProjectionResult::ProjectionResult() = default;
TargetReport3DGrid::ProjectionResult::~ProjectionResult() = default;
TargetReport3DGrid::ProjectionResult::ProjectionResult(ProjectionResult&&) noexcept = default;
TargetReport3DGrid::ProjectionResult&
TargetReport3DGrid::ProjectionResult::operator=(ProjectionResult&&) noexcept = default;

namespace
{
double percentileSorted(const std::vector<double>& sorted, double p)
{
    if (sorted.empty()) return 0.0;
    if (p <= 0.0) return sorted.front();
    if (p >= 1.0) return sorted.back();
    double idx = p * (sorted.size() - 1);
    auto   lo  = static_cast<std::size_t>(std::floor(idx));
    auto   hi  = static_cast<std::size_t>(std::ceil(idx));
    double w   = idx - static_cast<double>(lo);
    return sorted[lo] * (1.0 - w) + sorted[hi] * w;
}
}

TargetReport3DGrid::ProjectionResult
TargetReport3DGrid::projectionLayer(Projection projection,
                                    const CellScalar& cell_value,
                                    const CellWeight& cell_samples,
                                    const std::string& layer_name) const
{
    ProjectionResult out;

    // 1. Project 3D cells down to 2D, merging per the chosen axis.
    std::unordered_map<std::uint64_t, Cell> projected;
    switch (projection)
    {
        case Projection::Horizontal: projected = projectHorizontal(); break;
        case Projection::AltLon:     projected = projectAltLon();     break;
        case Projection::AltLat:     projected = projectAltLat();     break;
    }

    if (projected.empty())
        return out;

    // 2. Apply the cell-value functor; collect (a_bin, b_bin, value, samples).
    //    Bin layout per projection (matches projectXxx() pack2 order):
    //    - Horizontal: pack2(lat_bin, lon_bin)  -> (a, b) = (lat, lon)
    //    - AltLon:     pack2(alt_bin, lon_bin)  -> (a, b) = (alt, lon)
    //    - AltLat:     pack2(alt_bin, lat_bin)  -> (a, b) = (alt, lat)
    //    For all three the "x" of the resulting 2D layer is `b` (lon or lat)
    //    and the "y" is `a` (lat or alt).
    std::vector<LayerSample> samples;
    samples.reserve(projected.size());

    std::uint64_t total_samples = 0;
    std::size_t   spc_min = std::numeric_limits<std::size_t>::max();
    std::size_t   spc_max = 0;

    for (const auto& kv : projected)
    {
        std::int32_t a, b;
        std::tie(a, b) = unpack2(kv.first);

        std::uint64_t s = cell_samples ? cell_samples(kv.second) : 0;
        total_samples += s;
        spc_min = std::min(spc_min, static_cast<std::size_t>(s));
        spc_max = std::max(spc_max, static_cast<std::size_t>(s));

        auto v_opt = cell_value(kv.second);
        if (!v_opt.has_value() || !std::isfinite(*v_opt))
            continue;

        samples.push_back({b, a, *v_opt, s});
    }

    return buildLayer(projection, samples, total_samples, projected.size(),
                      spc_min, spc_max, layer_name);
}

TargetReport3DGrid::ProjectionResult
TargetReport3DGrid::horizontalLayer(
    const std::unordered_map<std::uint64_t, double>& value_by_key,
    const std::unordered_map<std::uint64_t, std::uint64_t>& samples_by_key,
    const std::string& layer_name) const
{
    std::vector<LayerSample> samples;
    samples.reserve(value_by_key.size());

    std::uint64_t total_samples = 0;
    std::size_t   spc_min = std::numeric_limits<std::size_t>::max();
    std::size_t   spc_max = 0;

    for (const auto& kv : value_by_key)
    {
        std::int32_t lat_bin, lon_bin;
        std::tie(lat_bin, lon_bin) = unpack2(kv.first);

        std::uint64_t s = 0;
        auto s_it = samples_by_key.find(kv.first);
        if (s_it != samples_by_key.end())
            s = s_it->second;
        total_samples += s;
        spc_min = std::min(spc_min, static_cast<std::size_t>(s));
        spc_max = std::max(spc_max, static_cast<std::size_t>(s));

        if (!std::isfinite(kv.second))
            continue;

        samples.push_back({lon_bin, lat_bin, kv.second, s});
    }

    return buildLayer(Projection::Horizontal, samples, total_samples, value_by_key.size(),
                      spc_min, spc_max, layer_name);
}

TargetReport3DGrid::ProjectionResult
TargetReport3DGrid::buildLayer(Projection projection,
                               const std::vector<LayerSample>& samples,
                               std::uint64_t total_samples,
                               std::size_t num_cells,
                               std::size_t spc_min,
                               std::size_t spc_max,
                               const std::string& layer_name) const
{
    ProjectionResult out;

    if (samples.empty())
        return out;

    // 1. Bin ranges over the valued samples.
    std::int32_t x_min_bin = samples.front().bin_x, x_max_bin = samples.front().bin_x;
    std::int32_t y_min_bin = samples.front().bin_y, y_max_bin = samples.front().bin_y;
    for (const auto& s : samples)
    {
        x_min_bin = std::min(x_min_bin, s.bin_x);
        x_max_bin = std::max(x_max_bin, s.bin_x);
        y_min_bin = std::min(y_min_bin, s.bin_y);
        y_max_bin = std::max(y_max_bin, s.bin_y);
    }

    // 2. Determine continuous bounds and per-axis cell sizes.
    double x_size = 0.0, y_size = 0.0;
    std::string srs = "wgs84";
    // `srs_is_north_up` is misnamed in the raster pipeline: it controls the y
    // flip in Grid2DLayerRenderer, which lays out pixel-row 0 at the painter
    // top. The chart draws image-top at the larger-y end of the y axis, so
    // setting this to true (= flip) is what produces "larger y at the top",
    // which is what we want for both latitude and altitude.
    const bool srs_is_north_up = true;

    switch (projection)
    {
        case Projection::Horizontal:
            x_size = dlon_deg_;            y_size = dlat_deg_;
            out.x_axis_label = "Longitude (deg)";
            out.y_axis_label = "Latitude (deg)";
            break;
        case Projection::AltLon:
            x_size = dlon_deg_;            y_size = cell_size_ft_;
            out.x_axis_label = "Longitude (deg)";
            out.y_axis_label = "Altitude (ft)";
            srs = "";
            break;
        case Projection::AltLat:
            x_size = dlat_deg_;            y_size = cell_size_ft_;
            out.x_axis_label = "Latitude (deg)";
            out.y_axis_label = "Altitude (ft)";
            srs = "";
            break;
    }

    out.x_min = static_cast<double>(x_min_bin)     * x_size;
    out.x_max = static_cast<double>(x_max_bin + 1) * x_size;
    out.y_min = static_cast<double>(y_min_bin)     * y_size;
    out.y_max = static_cast<double>(y_max_bin + 1) * y_size;

    const std::size_t nx = static_cast<std::size_t>(x_max_bin - x_min_bin + 1);
    const std::size_t ny = static_cast<std::size_t>(y_max_bin - y_min_bin + 1);

    // 3. Build a Grid2D, deposit one value per occupied cell at its center.
    Grid2D grid;
    QRectF roi(out.x_min, out.y_min, out.x_max - out.x_min, out.y_max - out.y_min);
    if (!grid.create(roi,
                     grid2d::GridResolution().setCellCount(nx, ny),
                     srs,
                     srs_is_north_up))
        return out;

    std::vector<double> values;
    values.reserve(samples.size());
    for (const auto& s : samples)
    {
        double cx = (static_cast<double>(s.bin_x) + 0.5) * x_size;
        double cy = (static_cast<double>(s.bin_y) + 0.5) * y_size;
        grid.addValue(cx, cy, s.value);
        values.push_back(s.value);
    }

    out.layer = grid.createLayer(layer_name, grid2d::ValueType::ValueTypeMean);
    if (!out.layer)
        return out;

    // 4. Stats over per-cell scalar values.
    out.cells_with_value = values.size();
    std::sort(values.begin(), values.end());
    out.v_min    = values.front();
    out.v_max    = values.back();
    out.v_mean   = std::accumulate(values.begin(), values.end(), 0.0)
                   / static_cast<double>(values.size());
    out.v_median = percentileSorted(values, 0.5);
    out.v_p95    = percentileSorted(values, 0.95);

    out.total_samples = total_samples;
    out.spc_min       = (spc_min == std::numeric_limits<std::size_t>::max()) ? 0 : spc_min;
    out.spc_max       = spc_max;
    out.spc_mean      = num_cells == 0
                            ? 0.0
                            : static_cast<double>(total_samples) / static_cast<double>(num_cells);

    out.valid = true;
    return out;
}
