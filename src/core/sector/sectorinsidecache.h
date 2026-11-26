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

#include <boost/optional.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

class Sector;

/**
 * Prepared sector inside tests per sector layer and delta.
 *
 * Shared by the Sectors filter (live mode, in memory) and the DuckDB function sector_inside()
 * (offline loads, in the WHERE clause). Thread safe, DuckDB calls the function from several
 * threads at once.
 *
 * The cache keeps its own copies of the sector geometry. A Sector holds the prepared shape for
 * one delta only, so the context's Sector objects stay untouched and several deltas can be
 * prepared side by side. A layer is set from the context's sectors, a prepared layer is built
 * on first use per delta and reused until the layer geometry changes.
 */
class SectorInsideCache
{
public:
    /// name of the database function fed by this cache,
    /// sector_inside(layer VARCHAR, latitude DOUBLE, longitude DOUBLE, altitude DOUBLE, delta_deg DOUBLE),
    /// altitude is the barometric altitude in feet, NULL if not available
    static constexpr const char* DBFunctionName = "sector_inside";

    /**
     * Geometry of one sector, enough to rebuild the inside test.
     */
    struct SectorGeometry
    {
        unsigned int id        = 0;
        std::string  name;
        bool         exclusion = false;
        std::vector<std::pair<double,double>> points; //latitude, longitude
        boost::optional<double> min_altitude; //barometric, feet
        boost::optional<double> max_altitude; //barometric, feet

        bool operator==(const SectorGeometry& other) const;
        bool operator!=(const SectorGeometry& other) const { return !(*this == other); }
    };

    /**
     * Bounds of the non-exclusion sectors of a layer in degrees.
     */
    struct LayerBounds
    {
        double lat_min = 0.0;
        double lat_max = 0.0;
        double lon_min = 0.0;
        double lon_max = 0.0;
    };

    /**
     * The sectors of one layer, prepared for one delta. Read only after construction, usable
     * from several threads.
     * delta_deg > 0 inflates the normal sectors, delta_deg < 0 deflates them, 0 is plain
     * containment. The exclusion sectors are prepared with the opposite delta, an inflation of
     * the layer deflates its exclusion sectors.
     */
    class PreparedLayer
    {
    public:
        PreparedLayer(const std::string& layer_name,
                      const std::vector<SectorGeometry>& sectors,
                      double delta_deg);
        ~PreparedLayer();

        /**
         * Inside the layer: inside a normal sector and not inside an exclusion sector. A sector
         * contains a position if it is inside its 2D shape and, if an altitude is given, the
         * altitude is inside the altitude limits of the sector. Without altitude the limits
         * are not checked.
         */
        bool isInside(double latitude, double longitude,
                      bool has_altitude = false, double altitude = 0.0) const;

        const std::string& layerName() const { return layer_name_; }
        double deltaDeg() const { return delta_deg_; }
        size_t numSectors() const { return sectors_.size(); }
        size_t numExclusionSectors() const { return exclusion_sectors_.size(); }

    private:
        /**
         * One prepared sector with its altitude limits.
         */
        struct Entry
        {
            std::shared_ptr<Sector> sector;
            double                  delta_deg = 0.0;
            boost::optional<double> min_altitude;
            boost::optional<double> max_altitude;

            bool contains(double latitude, double longitude, bool has_altitude, double altitude) const;
        };

        std::string        layer_name_;
        double             delta_deg_ = 0.0;
        std::vector<Entry> sectors_;
        std::vector<Entry> exclusion_sectors_;
    };

    SectorInsideCache();
    virtual ~SectorInsideCache();

    /**
     * Sets the geometry of a layer. The prepared shapes of the layer are dropped if the
     * geometry changed. Returns true if it changed.
     */
    bool setLayer(const std::string& layer_name, const std::vector<std::shared_ptr<Sector>>& sectors);
    bool setLayer(const std::string& layer_name, std::vector<SectorGeometry> sectors);

    void removeLayer(const std::string& layer_name);
    void clear();

    bool hasLayer(const std::string& layer_name) const;
    std::vector<std::string> layerNames() const;

    /// bounds of the non-exclusion sectors of a layer, none for an unknown layer or a layer
    /// without non-exclusion sectors
    boost::optional<LayerBounds> bounds(const std::string& layer_name) const;

    /**
     * Prepared layer for a delta, built on first use. Null for an unknown layer.
     */
    std::shared_ptr<const PreparedLayer> prepared(const std::string& layer_name, double delta_deg);

    static SectorGeometry geometryOf(const Sector& sector);

private:
    using PreparedKey = std::pair<std::string, double>;

    mutable std::mutex mutex_;

    std::map<std::string, std::vector<SectorGeometry>>           layers_;
    std::map<PreparedKey, std::shared_ptr<const PreparedLayer>>  prepared_;
};
