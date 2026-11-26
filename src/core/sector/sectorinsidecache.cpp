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

#include "sectorinsidecache.h"
#include "sector.h"
#include "logger.h"

#include <QColor>

#include <algorithm>

/***********************************************************************************
 * SectorInsideCache::SectorGeometry
 ***********************************************************************************/

bool SectorInsideCache::SectorGeometry::operator==(const SectorGeometry& other) const
{
    return id           == other.id &&
           name         == other.name &&
           exclusion    == other.exclusion &&
           points       == other.points &&
           min_altitude == other.min_altitude &&
           max_altitude == other.max_altitude;
}

/***********************************************************************************
 * SectorInsideCache::PreparedLayer
 ***********************************************************************************/

SectorInsideCache::PreparedLayer::PreparedLayer(const std::string& layer_name,
                                                const std::vector<SectorGeometry>& sectors,
                                                double delta_deg)
    : layer_name_(layer_name), delta_deg_(delta_deg)
{
    for (const auto& geometry : sectors)
    {
        if (geometry.points.size() < 3)
        {
            logwrn << "sector '" << geometry.name << "' of layer '" << layer_name
                   << "' has " << geometry.points.size() << " points, skipped";
            continue;
        }

        //an exclusion sector removes positions, it shrinks where the normal sectors grow
        Entry entry;
        entry.delta_deg    = geometry.exclusion ? -delta_deg : delta_deg;
        entry.min_altitude = geometry.min_altitude;
        entry.max_altitude = geometry.max_altitude;

        entry.sector = std::make_shared<Sector>(geometry.id, geometry.name, layer_name,
                                                false, geometry.exclusion, QColor(), geometry.points);
        entry.sector->createFastInsideTest(entry.delta_deg);

        if (geometry.exclusion)
            exclusion_sectors_.push_back(entry);
        else
            sectors_.push_back(entry);
    }

    loginf << "prepared layer '" << layer_name << "' delta_deg " << delta_deg
           << " sectors " << sectors_.size() << " exclusion sectors " << exclusion_sectors_.size();
}

SectorInsideCache::PreparedLayer::~PreparedLayer() = default;

bool SectorInsideCache::PreparedLayer::Entry::contains(double latitude, double longitude,
                                                       bool has_altitude, double altitude) const
{
    if (has_altitude)
    {
        if (min_altitude.has_value() && altitude < min_altitude.value())
            return false;

        if (max_altitude.has_value() && altitude > max_altitude.value())
            return false;
    }

    return sector->isInside(latitude, longitude, delta_deg);
}

bool SectorInsideCache::PreparedLayer::isInside(double latitude, double longitude,
                                                bool has_altitude, double altitude) const
{
    bool inside = false;

    for (const auto& entry : sectors_)
    {
        if (entry.contains(latitude, longitude, has_altitude, altitude))
        {
            inside = true;
            break;
        }
    }

    if (!inside)
        return false;

    for (const auto& entry : exclusion_sectors_)
    {
        if (entry.contains(latitude, longitude, has_altitude, altitude))
            return false;
    }

    return true;
}

/***********************************************************************************
 * SectorInsideCache
 ***********************************************************************************/

SectorInsideCache::SectorInsideCache() = default;

SectorInsideCache::~SectorInsideCache() = default;

SectorInsideCache::SectorGeometry SectorInsideCache::geometryOf(const Sector& sector)
{
    SectorGeometry geometry;

    geometry.id        = sector.id();
    geometry.name      = sector.name();
    geometry.exclusion = sector.isExclusionSector();
    geometry.points    = sector.points();

    if (sector.hasMinimumAltitude())
        geometry.min_altitude = sector.minimumAltitude();

    if (sector.hasMaximumAltitude())
        geometry.max_altitude = sector.maximumAltitude();

    return geometry;
}

bool SectorInsideCache::setLayer(const std::string& layer_name,
                                 const std::vector<std::shared_ptr<Sector>>& sectors)
{
    std::vector<SectorGeometry> geometries;
    geometries.reserve(sectors.size());

    for (const auto& sector : sectors)
    {
        if (sector)
            geometries.push_back(geometryOf(*sector));
    }

    return setLayer(layer_name, std::move(geometries));
}

bool SectorInsideCache::setLayer(const std::string& layer_name, std::vector<SectorGeometry> sectors)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = layers_.find(layer_name);

    if (it != layers_.end() && it->second == sectors)
        return false;

    layers_[ layer_name ] = std::move(sectors);

    //drop the prepared shapes of this layer
    for (auto pit = prepared_.begin(); pit != prepared_.end();)
    {
        if (pit->first.first == layer_name)
            pit = prepared_.erase(pit);
        else
            ++pit;
    }

    return true;
}

void SectorInsideCache::removeLayer(const std::string& layer_name)
{
    std::lock_guard<std::mutex> lock(mutex_);

    layers_.erase(layer_name);

    for (auto pit = prepared_.begin(); pit != prepared_.end();)
    {
        if (pit->first.first == layer_name)
            pit = prepared_.erase(pit);
        else
            ++pit;
    }
}

void SectorInsideCache::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);

    layers_.clear();
    prepared_.clear();
}

bool SectorInsideCache::hasLayer(const std::string& layer_name) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return layers_.count(layer_name) > 0;
}

std::vector<std::string> SectorInsideCache::layerNames() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> names;
    names.reserve(layers_.size());

    for (const auto& it : layers_)
        names.push_back(it.first);

    return names;
}

boost::optional<SectorInsideCache::LayerBounds> SectorInsideCache::bounds(const std::string& layer_name) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = layers_.find(layer_name);
    if (it == layers_.end())
        return {};

    LayerBounds bounds;
    bool first = true;

    for (const auto& geometry : it->second)
    {
        if (geometry.exclusion)
            continue;

        for (const auto& point : geometry.points)
        {
            if (first)
            {
                bounds.lat_min = bounds.lat_max = point.first;
                bounds.lon_min = bounds.lon_max = point.second;
                first = false;
            }
            else
            {
                bounds.lat_min = std::min(bounds.lat_min, point.first);
                bounds.lat_max = std::max(bounds.lat_max, point.first);
                bounds.lon_min = std::min(bounds.lon_min, point.second);
                bounds.lon_max = std::max(bounds.lon_max, point.second);
            }
        }
    }

    if (first)
        return {};

    return bounds;
}

std::shared_ptr<const SectorInsideCache::PreparedLayer> SectorInsideCache::prepared(const std::string& layer_name,
                                                                                    double delta_deg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    PreparedKey key(layer_name, delta_deg);

    auto pit = prepared_.find(key);
    if (pit != prepared_.end())
        return pit->second;

    auto lit = layers_.find(layer_name);
    if (lit == layers_.end())
        return nullptr;

    //built under the lock, so a second thread waits for the shape instead of building its own
    auto prepared = std::make_shared<const PreparedLayer>(layer_name, lit->second, delta_deg);

    prepared_[ key ] = prepared;

    return prepared;
}
