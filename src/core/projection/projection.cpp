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

#include "projection.h"
#include "projectioncoordinatesystembase.h"
#include "db_context_manager.h"
#include "logger.h"
#include "projectionmanager.h"
#include "compass.h"

Projection::Projection(nlohmann::json& config, ProjectionManager* parent)
    : Configurable(config, parent), proj_manager_(*parent)
{
    registerParameter("name", &name_, std::string());

    traced_assert(name_.size());

    // createSubConfigurables called in subclasses
}

Projection::~Projection() {}

void Projection::getGroundRange(
    unsigned int id, double slant_range_m, bool has_altitude, double altitude_m,
    double& ground_range_m, double& adjusted_altitude_m, bool debug)
{
    coordinateSystem(id).getGroundRange(slant_range_m, has_altitude, altitude_m,
                                        ground_range_m, adjusted_altitude_m, debug);

    return;
}

void Projection::addAllCoordinateSystems()
{
    logdbg << "adding";

    if (!coordinate_systems_added_)
    {
        boost::mutex::scoped_lock locker(coordinate_systems_mutex_);

        if (coordinate_systems_added_)
            return;

        auto& ctx_man = proj_manager_.compass().dbContextManager();

        if (!ctx_man.hasActiveContext())
            return; // will be called again when context becomes available

        for (const auto& [ds_id, ds] : ctx_man.activeContext().dataSources())
        {
            if (!hasCoordinateSystem(ds.id()))
            {
                if (!ds.hasPosition())
                    continue;

                logdbg << "adding " << ds.name();
                addCoordinateSystem(ds.id(), ds.latitude(), ds.longitude(), ds.altitude());
            }
        }

        coordinate_systems_added_ = true;
    }
}

std::string Projection::name() const { return name_; }

void Projection::name(const std::string& name) { name_ = name; }

bool Projection::coordinateSystemsAdded()
{
    boost::mutex::scoped_lock locker(coordinate_systems_mutex_);

    return coordinate_systems_added_;
}

bool Projection::hasMissingCoordinateSystem(unsigned int ds_id)
{
    return missing_coordinate_systems_.count(ds_id);
}
void Projection::addMissingCoordinateSystem(unsigned int ds_id)
{
    missing_coordinate_systems_.insert(ds_id);
}

