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

#include "unitmanager.h"
#include "dimension.h"
#include "logger.h"

#include <math.h>


UnitManager::UnitManager() : Configurable("UnitManager", "UnitManager0", 0, "units.json")
{
    createSubConfigurables();
}

UnitManager::~UnitManager()
{
    for (auto it : dimensions_)
        delete it.second;
    dimensions_.clear();
}

void UnitManager::generateSubConfigurable(const std::string& class_id,
                                          const std::string& instance_id)
{
    if (class_id == "Dimension")
    {
        Dimension* dimension = new Dimension(class_id, instance_id, this);
        traced_assert(dimensions_.find(dimension->instanceId()) == dimensions_.end());
        dimensions_.insert(std::pair<std::string, Dimension*>(dimension->instanceId(), dimension));
    }
    else
        throw std::runtime_error("UnitManager: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

void UnitManager::checkSubConfigurables()
{
    if (dimensions_.count("Angle") == 0)
    {
        generateSubConfigurableFromConfig("Dimension", "Angle");

        dimensions_.at("Angle")->addUnit("Degree", 1.0, "");
        dimensions_.at("Angle")->addUnit("Radian", M_PI / 180.0, "");
    }

    if (dimensions_.count("Length") == 0)
    {
        generateSubConfigurableFromConfig("Dimension", "Length");

        dimensions_.at("Length")->addUnit("Meter", 1.0, "");
        dimensions_.at("Length")->addUnit("Kilometer", 1.0 / 1000.0, "");
        dimensions_.at("Length")->addUnit("Mile", 1.0 / 1609.344, "");
        dimensions_.at("Length")->addUnit("Nautical Mile", 1.0 / 1852.0, "");
    }

    if (dimensions_.count("Height") == 0)
    {
        generateSubConfigurableFromConfig("Dimension", "Height");

        dimensions_.at("Height")->addUnit("Feet", 1.0, "");
        dimensions_.at("Height")->addUnit("Flight Level", 1.0 / 100.0, "");
        dimensions_.at("Height")->addUnit("Meter", 0.3048, "");
    }

    if (dimensions_.count("Time") == 0)
    {
        generateSubConfigurableFromConfig("Dimension", "Time");

        dimensions_.at("Time")->addUnit("Second", 1.0, "");
        dimensions_.at("Time")->addUnit("Minute", 1.0 / 60.0, "");
        dimensions_.at("Time")->addUnit("Hour", 1.0 / 3600.0, "");
        dimensions_.at("Time")->addUnit("MilliSeconds", 1000.0, "");
        dimensions_.at("Time")->addUnit("V7Time", 128.0, "");
        dimensions_.at("Time")->addUnit("V6Time", 4096.0, "");
    }

    if (dimensions_.count("Speed") == 0)
    {
        generateSubConfigurableFromConfig("Dimension", "Speed");

        dimensions_.at("Speed")->addUnit("Knots", 1.0, "");
        dimensions_.at("Speed")->addUnit("Meter/Second", 0.514444, "");
        dimensions_.at("Speed")->addUnit("NM/Second", 1.0 / 3600.0, "");
        dimensions_.at("Speed")->addUnit("Mach", 0.00149984, "");
    }
}
