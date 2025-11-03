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

#include "airspace.h"
#include "sectorlayer.h"
#include "sector.h"
#include "dbcontent/target/targetposition.h"
#include "json.hpp"
#include "logger.h"
#include "traced_assert.h"

#include <fstream>

const std::string AirSpace::LowerHeightLayerName = "LowerHeightFilter";
const QColor      AirSpace::DefaultSectorColor   = QColor(255, 255, 0);

/**
 */
AirSpace::AirSpace() = default;

/**
 */
AirSpace::~AirSpace() = default;

/**
 */
void AirSpace::clear()
{
    layers_.clear();
}

/**
 */
std::shared_ptr<SectorLayer> AirSpace::lowerHeightFilterLayer() const
{
    for (const auto& l : layers_)
        if (l->name() == LowerHeightLayerName)
            return l;

    return nullptr;
}

namespace 
{
    struct SectorData
    {
        std::string name;
        int         own_volume;
        int         used_for_checking;
        double      own_height_min;
        double      own_height_max;

        std::vector<std::pair<double, double>> points;
    };

    boost::optional<SectorData> readSector(nlohmann::json& j_sector, 
                                           unsigned int& sec_id)
    {
        if (!j_sector.contains("name") ||
            !j_sector.contains("own_height_min") ||
            !j_sector.contains("own_height_max") ||
            !j_sector.contains("own_volume") ||
            !j_sector.contains("used_for_checking"))
        {
            logerr << "ill-defined sector, json '" << j_sector.dump(4);
            return {};
        }

        std::string name = j_sector["name"];

        int    own_volume        = std::atoi(j_sector["own_volume"       ].get<std::string>().c_str());
        int    used_for_checking = std::atoi(j_sector["used_for_checking"].get<std::string>().c_str());
        double own_height_min    = std::atof(j_sector["own_height_min"   ].get<std::string>().c_str());
        double own_height_max    = std::atof(j_sector["own_height_max"   ].get<std::string>().c_str());

        std::map<int, std::pair<double, double>> ordered_points;

        if (j_sector.contains("geographic_points"))
        {
            nlohmann::json& geographic_points = j_sector["geographic_points"];
            if (!geographic_points.is_array())
            {
                logerr << "geographic_points is not an array";
                return {};
            }

            for (auto& j_gp : geographic_points.get<nlohmann::json::array_t>())
            {
                if (!j_gp.contains("index") ||
                    !j_gp.contains("latitude") ||
                    !j_gp.contains("longitude"))
                {
                    logerr << "ill-defined geographic point, json '" << j_gp.dump(4);
                    return {};
                }

                int    id  = std::atoi(j_gp["index"    ].get<std::string>().c_str());
                double lat = std::atof(j_gp["latitude" ].get<std::string>().c_str());
                double lon = std::atof(j_gp["longitude"].get<std::string>().c_str());

                if (!ordered_points.insert(std::make_pair(id, std::make_pair(lat, lon))).second)
                {
                    logerr << "duplicate geographic point ID " << id;
                    return {};
                }
            }
        }

        SectorData data;
     
        data.name              = name;
        data.own_volume        = own_volume;
        data.used_for_checking = used_for_checking;
        data.own_height_min    = own_height_min;
        data.own_height_max    = own_height_max;

        data.points.reserve(ordered_points.size());

        for (const auto& elem : ordered_points)
            data.points.push_back(elem.second);

        return data;
    }

    std::shared_ptr<SectorLayer> readSectorRecursive(nlohmann::json& j_sector, 
                                                     unsigned int& sec_id,
                                                     std::shared_ptr<SectorLayer> parent)
    {
        //try read sector
        auto sector_data = readSector(j_sector, sec_id);
        if (!sector_data.has_value())
            return nullptr;

        bool has_nested_sectors = j_sector.contains("air_space_sectors");

        //!no double nestings!
        if (parent && has_nested_sectors)
        {
            logerr << "double nesting of sectors detected in layer '" + parent->name() + "'";
            return nullptr;
        }

        std::shared_ptr<SectorLayer> ret;

        if (parent)
        {
            //we are a nested sector => add to parent layer
            auto sec = std::make_shared<Sector>(sec_id++, 
                                                sector_data->name,
                                                parent->name(),
                                                false, //do not serialize yet, as we are not part of the evaluation
                                                false, 
                                                AirSpace::DefaultSectorColor, 
                                                sector_data->points);
            sec->setMinimumAltitude(sector_data->own_height_min);

            parent->addSector(sec);

            ret = parent;
        }
        else
        {
            //we do not have a parent layer => generate a new one
            auto layer = std::make_shared<SectorLayer>(sector_data->name);
            
            if (has_nested_sectors)
            {
                //we obtain nested sectors => treat sector as layer
                nlohmann::json& air_space_sectors = j_sector["air_space_sectors"];
                if (!air_space_sectors.is_array())
                {
                    logerr << "air_space_sectors is not an array";
                    return nullptr;
                }

                for (auto& j_sector2 : air_space_sectors.get<nlohmann::json::array_t>())
                    if (readSectorRecursive(j_sector2, sec_id, layer) == nullptr)
                        return nullptr;
            }
            else 
            {
                //no nested sectors => we are our own layer
                auto sec = std::make_shared<Sector>(sec_id++, 
                                            sector_data->name,
                                            sector_data->name,
                                            false, //do not serialize yet, as we are not part of the evaluation
                                            false, 
                                            AirSpace::DefaultSectorColor, 
                                            sector_data->points);
                sec->setMinimumAltitude(sector_data->own_height_min);
                sec->setMaximumAltitude(sector_data->own_height_max);

                layer->addSector(sec);
            }

            ret = layer;
        }
        
        return ret;
    }
}

/**
 */
bool AirSpace::readJSON(const std::string& fn,
                        unsigned int base_id)
{
    std::ifstream input_file(fn, std::ifstream::in);

    clear();

    try
    {
        nlohmann::json j = nlohmann::json::parse(input_file);

        if (!j.contains("air_space_sector_definition"))
        {
            logerr << "file does not contain air_space_sector_definition";
            return false;
        }

        nlohmann::json& air_space_sector_definition = j["air_space_sector_definition"];

        if (!air_space_sector_definition.contains("air_space_sectors"))
        {
            logerr << "file does not contain air_space_sectors";
            return false;
        }

        nlohmann::json& air_space_sectors = air_space_sector_definition["air_space_sectors"];
        if (!air_space_sectors.is_array())
        {
            logerr << "air_space_sectors is not an array";
            return false;
        }

        std::vector<std::shared_ptr<SectorLayer>> layers;

        unsigned int sec_id = base_id;

        for (auto& j_sector : air_space_sectors.get<nlohmann::json::array_t>())
        {
            auto layer = readSectorRecursive(j_sector, base_id, nullptr);
            if (!layer)
                return false;

            layers.push_back(layer);
        }

        if (sec_id == base_id)
        {
            logerr << "no sectors found";
            return false;
        }

        layers_.insert(layers_.begin(), layers.begin(), layers.end());
    }
    catch (nlohmann::json::exception& e)
    {
        logerr << "could not load file '" << fn << "': " << e.what();

        clear();

        return false;
    }

    return true;
}

/**
 */
const std::vector<std::shared_ptr<SectorLayer>>& AirSpace::layers() const
{
    return layers_;
}

/**
*/
AirSpace::AboveCheckResult AirSpace::isAbove(const SectorLayer* layer,
                                             const dbContent::TargetPosition& pos,
                                             bool has_ground_bit,
                                             bool ground_bit_set)
{
    traced_assert(layer);

    //check bounds of layer
    auto lat_min_max = layer->getMinMaxLatitude();
    auto lon_min_max = layer->getMinMaxLongitude();

    if (pos.latitude_  < lat_min_max.first  ||
        pos.latitude_  > lat_min_max.second ||
        pos.longitude_ < lon_min_max.first  ||
        pos.longitude_ > lon_min_max.second)
        return AboveCheckResult::OutOfAirspace;

    bool was_inside_xy = false;

    //check individual sectors
    for (auto sector : layer->sectors())
    {
        //@TODO: should there alwys only be a correctly set minimum?
        //assert( sector->hasMinimumAltitude());
        //assert(!sector->hasMaximumAltitude());

        //not inside of sector area?
        if (!sector->isInside(pos, has_ground_bit, ground_bit_set, Sector::InsideCheckType::XY))
            continue;

        was_inside_xy = true;
        
        //if we are inside a sector and above min altitude => above
        if (sector->isInside(pos, has_ground_bit, ground_bit_set, Sector::InsideCheckType::ZMinOnly))
            return AboveCheckResult::Above;
    }

    //inside but not above any min height => below
    if (was_inside_xy)
        return AboveCheckResult::Below;

    //not inside any sector?
    return AboveCheckResult::OutOfAirspace;
}
