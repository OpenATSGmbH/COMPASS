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

#include <array>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

/**
 * @brief Sparse 3D grid (latitude / longitude / barometric flight level) for
 *        target-report-based MLAT inspectors.
 *
 * Cells are addressed by integer bins; horizontal bin size is given in meters
 * (converted to degrees at the configured reference latitude) and vertical bin
 * size in feet. The grid is sparse: cells that receive no data are not stored.
 *
 * Per cell the grid keeps:
 *  - PD counters: num_eui (expected slots), num_mui (missed slots) for Feature 2.
 *  - Position-offset samples: distance to RefTraj, plus the reported per-report
 *    std-dev (Cartesian magnitude). Used by Feature 3 to derive mean / median /
 *    P95 of distance, mean of reported std-dev, and the distance / std-dev ratio.
 *
 * The grid produces three 2D projections (horizontal, alt/lon, alt/lat) for
 * downstream rendering. Aggregation across the dropped axis is sum-of-counters
 * for PD and concatenation-of-samples for the accuracy series, so derived
 * statistics on the projected grid match what would be produced if the same
 * samples had been bucketed into the projected cell directly.
 */
class TargetReport3DGrid
{
public:
    struct Cell
    {
        std::uint64_t       num_eui = 0;
        std::uint64_t       num_mui = 0;
        std::vector<double> distances_m;        // Feature 3: |MLAT - RefTraj|
        std::vector<double> reported_stddevs_m; // Feature 3: sqrt(sx^2 + sy^2)
    };

    /**
     * @param cell_size_m   horizontal cell side length in meters.
     * @param cell_size_ft  vertical cell side length in feet.
     * @param ref_lat_deg   reference latitude used to convert meters to degrees
     *                      longitude. Pass the centre of the area of interest.
     */
    TargetReport3DGrid(double cell_size_m, double cell_size_ft, double ref_lat_deg);

    void addEUI(double lat_deg, double lon_deg, double baro_alt_ft);
    void addMUI(double lat_deg, double lon_deg, double baro_alt_ft);
    void addAccuracySample(double lat_deg, double lon_deg, double baro_alt_ft,
                           double distance_m, double reported_stddev_m);

    std::size_t numCells() const { return cells_.size(); }

    double cellSizeMeters() const { return cell_size_m_; }
    double cellSizeFeet()   const { return cell_size_ft_; }

    /// Horizontal projection: aggregate over alt bin.  key = (lat_bin, lon_bin).
    std::unordered_map<std::uint64_t, Cell> projectHorizontal() const;
    /// Altitude / longitude projection: aggregate over lat bin. key = (alt_bin, lon_bin).
    std::unordered_map<std::uint64_t, Cell> projectAltLon() const;
    /// Altitude / latitude projection: aggregate over lon bin. key = (alt_bin, lat_bin).
    std::unordered_map<std::uint64_t, Cell> projectAltLat() const;

    /// Pack two 32-bit signed bin indices into a single 64-bit key (used as
    /// the result map's key in the projection helpers).
    static std::uint64_t pack2(std::int32_t a, std::int32_t b);
    static std::pair<std::int32_t, std::int32_t> unpack2(std::uint64_t key);

private:
    struct Key
    {
        std::int32_t lat_bin = 0;
        std::int32_t lon_bin = 0;
        std::int32_t alt_bin = 0;
        bool operator==(const Key& o) const
        { return lat_bin == o.lat_bin && lon_bin == o.lon_bin && alt_bin == o.alt_bin; }
    };

    struct KeyHash
    {
        std::size_t operator()(const Key& k) const noexcept
        {
            std::uint64_t h = static_cast<std::uint32_t>(k.lat_bin);
            h = (h * 1315423911u) ^ static_cast<std::uint32_t>(k.lon_bin);
            h = (h * 1315423911u) ^ static_cast<std::uint32_t>(k.alt_bin);
            return static_cast<std::size_t>(h);
        }
    };

    Key keyFor(double lat_deg, double lon_deg, double baro_alt_ft) const;
    Cell& cellAt(double lat_deg, double lon_deg, double baro_alt_ft);

    double cell_size_m_;
    double cell_size_ft_;
    double ref_lat_deg_;
    double dlat_deg_;
    double dlon_deg_;

    std::unordered_map<Key, Cell, KeyHash> cells_;
};
