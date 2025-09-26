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

#include <string>
#include <vector>
#include <memory>

class Sector;

namespace dbContent {
class TargetPosition;
}

/**
*/
class SectorLayer
{
public:
    SectorLayer(const std::string& name);
    virtual ~SectorLayer() = default;

    std::string name() const;

    bool hasSector (const std::string& name);
    std::shared_ptr<Sector> sector (const std::string& name);

    void addSector (std::shared_ptr<Sector> sector);
    void removeSector (std::shared_ptr<Sector> sector);
    void clearSectors();

    unsigned int size () const { return sectors_.size(); };

    std::vector<std::shared_ptr<Sector>>& sectors() { return sectors_; }
    const std::vector<std::shared_ptr<Sector>>& sectors() const { return sectors_; }

    std::pair<double, double> getMinMaxLatitude() const;
    std::pair<double, double> getMinMaxLongitude() const;

    virtual bool isInside(const dbContent::TargetPosition& pos,
                          bool has_ground_bit, 
                          bool ground_bit_set) const;
    bool isInside(double latitude, double longitude, double delta_deg) const;
    // ignores exclude sectors

    bool hasExclusionSector() const;

protected:
    void checkExclusion();

    const std::string name_;

    std::vector<std::shared_ptr<Sector>> sectors_;

    unsigned int num_exclusion_sectors_ = 0;
};
