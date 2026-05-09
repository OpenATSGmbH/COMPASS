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

#include <cmath>

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

void TargetReport3DGrid::addAccuracySample(double lat_deg, double lon_deg, double baro_alt_ft,
                                           double distance_m, double reported_stddev_m)
{
    auto& c = cellAt(lat_deg, lon_deg, baro_alt_ft);
    c.distances_m.push_back(distance_m);
    c.reported_stddevs_m.push_back(reported_stddev_m);
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

namespace
{
inline void mergeInto(TargetReport3DGrid::Cell& dst, const TargetReport3DGrid::Cell& src)
{
    dst.num_eui += src.num_eui;
    dst.num_mui += src.num_mui;
    dst.distances_m.insert(dst.distances_m.end(),
                           src.distances_m.begin(), src.distances_m.end());
    dst.reported_stddevs_m.insert(dst.reported_stddevs_m.end(),
                                  src.reported_stddevs_m.begin(),
                                  src.reported_stddevs_m.end());
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
