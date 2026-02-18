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

#include "projection.h"

class ProjectionManager;
class GeoCoordinateSystem;

class GeoProjection : public Projection
{
public:
    GeoProjection(const std::string& class_id, const std::string& instance_id,
                  ProjectionManager& proj_manager);
    virtual ~GeoProjection();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    virtual std::vector<unsigned int> ids() override;

    virtual bool hasCoordinateSystem(unsigned int id) override;
    virtual void addCoordinateSystem(unsigned int id, double latitude_deg, double longitude_deg,
                                     double altitude_m) override;
    virtual ProjectionCoordinateSystemBase& coordinateSystem(unsigned int id) override;
    virtual void clearCoordinateSystems() override;

    virtual bool polarToWGS84(unsigned int id, double azimuth_rad, double slant_range_m,
                              bool has_baro_altitude, double baro_altitude_ft, double& latitude,
                              double& longitude) override;

    virtual bool wgs842PolarHorizontal(unsigned int id, double latitude_deg, double longitude_deg,
                                       double& azimuth_rad, double& ground_range_m) override;

protected:
    std::map<unsigned int, std::unique_ptr<GeoCoordinateSystem>> coordinate_systems_;

    virtual void checkSubConfigurables() override;
};

