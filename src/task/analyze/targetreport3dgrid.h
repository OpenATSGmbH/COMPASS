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
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct Grid2DLayer;

/**
 * @brief Sparse 3D grid (latitude / longitude / barometric flight level) for
 *        target-report-based MLAT inspectors.
 *
 * Cells are addressed by integer bins; horizontal bin size is given in meters
 * (converted to degrees at the configured reference latitude) and vertical bin
 * size in feet. The grid is sparse: cells that receive no data are not stored.
 *
 * Per cell the grid keeps running sums only - never per-sample arrays - so that
 * the per-cell footprint is O(1) regardless of how many target reports fall
 * into the cell. This is the central reason the grid scales to multi-hour
 * 1 Hz CAT020 inputs without blowing memory: per-sample storage would push
 * peak memory into tens of GB on heavy datasets, especially because each of
 * the three 2D projections temporarily duplicates per-cell content.
 *
 *  - PD counters: num_eui (expected slots), num_mui (missed slots) for Feature 2.
 *  - Accuracy running sums for Feature 3:
 *      sum_distance_m / num_distance     -> mean horizontal offset
 *      sum_stddev_m   / num_stddev       -> mean reported std-dev
 *      sum_ratio      / num_ratio        -> mean (offset / reported std-dev)
 *
 * The three 2D projections (horizontal, alt/lon, alt/lat) merge cells along
 * the dropped axis by summing these counters, so the projected per-cell mean
 * is the sample-weighted mean across the merged cells.
 *
 * Note: median / P95 of distance can no longer be derived from a cell. The
 * task-wide median / P95 in the inspector summary is computed from a flat
 * sample list maintained at the inspector level, not from cells.
 */
class TargetReport3DGrid
{
public:
    struct Cell
    {
        std::uint64_t num_eui = 0;
        std::uint64_t num_mui = 0;

        std::uint64_t num_samples    = 0;  // total accuracy samples added (any kind)

        std::uint64_t num_distance   = 0;
        double        sum_distance_m = 0.0;

        std::uint64_t num_stddev     = 0;
        double        sum_stddev_m   = 0.0;

        std::uint64_t num_ratio      = 0;
        double        sum_ratio      = 0.0;
    };

    /**
     * @param cell_size_m   horizontal cell side length in meters.
     * @param cell_size_ft  vertical cell side length in feet.
     * @param ref_lat_deg   reference latitude used to convert meters to degrees
     *                      longitude. Pass the center of the area of interest.
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

    /// One of three projections of the 3D grid down to a 2D layer.
    /// - Horizontal: x = lon (deg),   y = lat (deg);   geo-renderable.
    /// - AltLon:    x = lon (deg),   y = baro alt (ft); not geo.
    /// - AltLat:    x = lat (deg),   y = baro alt (ft); not geo.
    enum class Projection
    {
        Horizontal = 0,
        AltLon,
        AltLat
    };

    /// Result of `projectionLayer(...)`. The `layer` carries the per-cell scalar
    /// values plus a `RasterReference` matching the layer's continuous bounds.
    /// For Horizontal the consumer passes it to `Grid2DLayerRenderer::render`
    /// and wraps as `ViewPointGenFeatureGeoImage`; for AltLon / AltLat it is
    /// passed directly to `ViewPointGenFeatureGrid` with axis labels carried
    /// by `PlotMetadata`.
    struct ProjectionResult
    {
        bool                          valid = false;

        std::unique_ptr<Grid2DLayer>  layer;

        std::string x_axis_label;
        std::string y_axis_label;

        double x_min = 0.0, x_max = 0.0;
        double y_min = 0.0, y_max = 0.0;

        // Stats over per-cell scalar values (cells where the scalar functor
        // returned a value).
        std::size_t cells_with_value = 0;
        double      v_min   = 0.0;
        double      v_mean  = 0.0;
        double      v_max   = 0.0;
        double      v_median= 0.0;
        double      v_p95   = 0.0;

        // Stats over per-cell sample counts (the CellWeight functor).
        std::uint64_t total_samples = 0;
        std::size_t   spc_min       = 0;
        std::size_t   spc_max       = 0;
        double        spc_mean      = 0.0;

        ProjectionResult();
        ~ProjectionResult();
        ProjectionResult(ProjectionResult&&) noexcept;
        ProjectionResult& operator=(ProjectionResult&&) noexcept;
        ProjectionResult(const ProjectionResult&) = delete;
        ProjectionResult& operator=(const ProjectionResult&) = delete;
    };

    using CellScalar = std::function<std::optional<double>(const Cell&)>;
    using CellWeight = std::function<std::uint64_t(const Cell&)>;

    ProjectionResult projectionLayer(Projection projection,
                                     const CellScalar& cell_value,
                                     const CellWeight& cell_samples,
                                     const std::string& layer_name = "value") const;

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
