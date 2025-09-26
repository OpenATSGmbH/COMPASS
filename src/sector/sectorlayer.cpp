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

#include "sectorlayer.h"
#include "sector.h"
#include "dbcontent/target/targetposition.h"
#include "logger.h"

#include "traced_assert.h"

using namespace std;

SectorLayer::SectorLayer(const std::string& name)
  : name_(name)
{
}

std::string SectorLayer::name() const
{
    return name_;
}

bool SectorLayer::hasExclusionSector() const
{
    return (num_exclusion_sectors_ > 0);
}

bool SectorLayer::hasSector (const std::string& name)
{
    auto iter = std::find_if(sectors_.begin(), sectors_.end(),
                             [&name](const shared_ptr<Sector>& x) { return x->name() == name;});

    return iter != sectors_.end();
}

std::shared_ptr<Sector> SectorLayer::sector(const std::string& name)
{
    traced_assert(hasSector(name));

    auto iter = std::find_if(sectors_.begin(), sectors_.end(),
                             [&name](const shared_ptr<Sector>& x) { return x->name() == name;});
    traced_assert(iter != sectors_.end());

    return *iter;
}

void SectorLayer::addSector (std::shared_ptr<Sector> sector)
{
    traced_assert(!hasSector(sector->name()));
    traced_assert(sector->layerName() == name_);
    sectors_.push_back(sector);

    if (sector->isExclusionSector())
        ++num_exclusion_sectors_;
}

void SectorLayer::removeSector (std::shared_ptr<Sector> sector)
{
    traced_assert(hasSector(sector->name()));

    string name = sector->name();
    auto iter = std::find_if(sectors_.begin(), sectors_.end(),
                             [&name](const shared_ptr<Sector>& x) { return x->name() == name;});
    traced_assert(iter != sectors_.end());

    bool is_exclusion = sector->isExclusionSector();

    sectors_.erase(iter);
    traced_assert(!hasSector(sector->name()));

    if (is_exclusion)
    {
        traced_assert(num_exclusion_sectors_ > 0);
        --num_exclusion_sectors_;
    }
}

void SectorLayer::clearSectors()
{
    sectors_               = {};
    num_exclusion_sectors_ = 0;
}

bool SectorLayer::isInside(const dbContent::TargetPosition& pos,
                           bool has_ground_bit, 
                           bool ground_bit_set)  const
{
    bool is_inside         = false;
    bool is_inside_exclude = false;

    // check if inside normal ones
    for (auto& sec_it : sectors_)
    {
        if (sec_it->isExclusionSector())
            continue;

        if (sec_it->isInside(pos, has_ground_bit, ground_bit_set))
        {
            logdbg << "'" << name_ << "' true, has alt " << pos.has_altitude_ << " alt " << pos.altitude_ ;

            is_inside = true;
            break;
        }
    }

    if (!is_inside) // not inside normal sector
        return false;

    // is inside normal sector

    if (!hasExclusionSector()) // nothin more to check
        return true;

    // check if inside exclude ones
    for (auto& sec_it : sectors_)
    {
        if (!sec_it->isExclusionSector())
            continue;

        if (sec_it->isInside(pos, has_ground_bit, ground_bit_set))
        {
            logdbg << "'" << name_ << "' (exclusion): true, has alt " << pos.has_altitude_ << " alt " << pos.altitude_;

            is_inside_exclude = true;
            break;
        }
    }

    return !is_inside_exclude; // true if in no exclude, false if in include
}

bool SectorLayer::isInside(double latitude, double longitude, double delta_deg) const
{
    bool is_inside         = false;

    // check if inside normal ones
    for (auto& sec_it : sectors_)
    {
        if (sec_it->isExclusionSector())
            continue;

        if (sec_it->isInside(latitude, longitude, delta_deg))
        {
            is_inside = true;
            break;
        }
    }

    logdbg2 << "'" << name_ << "' is_inside " << is_inside; 
    return is_inside; // not inside normal sector
}

std::pair<double, double> SectorLayer::getMinMaxLatitude() const
{
    double min{0}, max{0};
    double tmp_min{0}, tmp_max{0};
    bool first = true;

    traced_assert(sectors_.size());

    for (auto& sec_it : sectors_)
    {
        if (first)
        {
            tie (min, max) = sec_it->getMinMaxLatitude();
            first = false;
        }
        else
        {
            tie (tmp_min, tmp_max) = sec_it->getMinMaxLatitude();
            min = std::min(min, tmp_min);
            max = std::max(max, tmp_max);
        }
    }

    return {min, max};
}

std::pair<double, double> SectorLayer::getMinMaxLongitude() const
{
    double min{0}, max{0};
    double tmp_min{0}, tmp_max{0};
    bool first = true;

    traced_assert(sectors_.size());

    for (auto& sec_it : sectors_)
    {
        if (first)
        {
            tie (min, max) = sec_it->getMinMaxLongitude();
            first = false;
        }
        else
        {
            tie (tmp_min, tmp_max) = sec_it->getMinMaxLongitude();
            min = std::min(min, tmp_min);
            max = std::max(max, tmp_max);
        }
    }

    return {min, max};
}
