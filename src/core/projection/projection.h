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

#include "configurable.h"
#include "radarbiasinfo.h"

#include <boost/thread/mutex.hpp>

class ProjectionManager;
class ProjectionCoordinateSystemBase;

class Projection : public Configurable
{
public:
    Projection(const std::string& class_id, const std::string& instance_id,
               ProjectionManager& proj_manager);
    virtual ~Projection();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    virtual bool hasCoordinateSystem(unsigned int id) = 0;
    virtual void addCoordinateSystem(unsigned int id, double latitude_deg, double longitude_deg,
                                     double altitude_m) = 0;

    virtual std::vector<unsigned int> ids() = 0;
    virtual ProjectionCoordinateSystemBase& coordinateSystem(unsigned int id) = 0;

    virtual void clearCoordinateSystems() = 0;
    virtual bool polarToWGS84(unsigned int id, double azimuth_rad, double slant_range_m,
                              bool has_baro_altitude, double baro_altitude_ft,
                              double& latitude_deg, double& longitude_deg, double& alt_wgs_m, bool debug=false) = 0;

    virtual bool localXYToWGS84(unsigned int id, double x_m, double y_m,
                              double& latitude_deg, double& longitude_deg, double& alt_wgs_m, bool debug=false) = 0;

    virtual bool polarToWGS84(unsigned int id, double azimuth_rad, double slant_range_m,
                              bool has_baro_altitude, double baro_altitude_ft,
                              double& ground_range_m,
                              double& latitude_deg, double& longitude_deg, double& alt_wgs_m, bool debug=false) = 0;

    virtual bool polarToWGS84(unsigned int id, double azimuth_rad, double slant_range_m,
                              bool has_baro_altitude, double baro_altitude_ft, RadarBiasInfo& bias_info,
                              double& latitude_deg, double& longitude_deg, double& alt_wgs_m, bool debug=false) = 0;

    virtual bool wgs842PolarHorizontal(unsigned int id,
                                       double latitude_deg, double longitude_deg, double alt_wgs_m,
                                       double& azimuth_rad, double& slant_range_m, double& ground_range_m,
                                       double& radar_altitude_m, bool debug=false) = 0;

    void getGroundRange(unsigned int id, double slant_range_m,
                          bool has_altitude, double altitude_m,
                          double& ground_range_m, double& adjusted_altitude_m, bool debug=false);

    void addAllCoordinateSystems(); // only adds if not already added

    std::string name() const;
    void name(const std::string& name);

    bool coordinateSystemsAdded();

    bool hasMissingCoordinateSystem(unsigned int ds_id);
    void addMissingCoordinateSystem(unsigned int ds_id);

protected:
    ProjectionManager& proj_manager_;

    std::string name_;

    boost::mutex coordinate_systems_mutex_;
    bool coordinate_systems_added_ {false};

    std::set<unsigned int> missing_coordinate_systems_;

    virtual void checkSubConfigurables();
};


