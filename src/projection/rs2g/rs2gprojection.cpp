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

#include "rs2gprojection.h"

#include "global.h"
#include "logger.h"
#include "projectionmanager.h"
#include "rs2gcoordinatesystem.h"

RS2GProjection::RS2GProjection(const std::string& class_id, const std::string& instance_id,
                               ProjectionManager& proj_manager)
    : Projection(class_id, instance_id, proj_manager)
{
}

RS2GProjection::~RS2GProjection() {}

void RS2GProjection::generateSubConfigurable(const std::string& class_id,
                                             const std::string& instance_id)
{
}

void RS2GProjection::checkSubConfigurables() {}

std::vector<unsigned int> RS2GProjection::ids()
{
    std::vector<unsigned int> ids;

    for (auto& coord_sys : coordinate_systems_)
        ids.push_back(coord_sys.first);

    return ids;
}

bool RS2GProjection::hasCoordinateSystem(unsigned int id) { return coordinate_systems_.count(id); }

void RS2GProjection::addCoordinateSystem(unsigned int id, double latitude_deg, double longitude_deg,
                                         double altitude_m)
{
    if (hasCoordinateSystem(id))
        return;

    coordinate_systems_[id].reset(
        new RS2GCoordinateSystem(id, latitude_deg, longitude_deg, altitude_m));
}

ProjectionCoordinateSystemBase& RS2GProjection::coordinateSystem(unsigned int id)
{
    traced_assert(hasCoordinateSystem(id));

    return *coordinate_systems_.at(id).get();
}


void RS2GProjection::clearCoordinateSystems()
{
    coordinate_systems_.clear();
    coordinate_systems_added_ = false;
}

bool RS2GProjection::polarToWGS84(unsigned int id, double azimuth_rad, double slant_range_m,
                                  bool has_baro_altitude, double baro_altitude_ft,
                                  double& latitude_deg, double& longitude_deg, double& alt_wgs_m, bool debug)
{
    if (!hasCoordinateSystem(id))
        logerr << "no coord system for " << id;

    traced_assert(hasCoordinateSystem(id));

    if (debug)
        loginf << "azimuth_rad " << azimuth_rad << " slant_range_m " << slant_range_m
               << " has_baro " << has_baro_altitude << " baro_altitude_ft " << baro_altitude_ft;

    bool ret {false};

    double ground_range_m, ecef_x, ecef_y, ecef_z;

    ret = coordinate_systems_.at(id)->calculateRadSlt2Geocentric(
        azimuth_rad, slant_range_m, has_baro_altitude, baro_altitude_ft * FT2M,
        ground_range_m, ecef_x, ecef_y, ecef_z, debug);

    if (debug)
        loginf << "ecef_x " << std::fixed << ecef_x << " ecef_y " << ecef_y
               << " ecef_z " << ecef_z;

    if (ret)
    {
        ret = coordinate_systems_.at(id)->geocentric2Geodesic(ecef_x, ecef_y, ecef_z,
                                                              latitude_deg, longitude_deg, alt_wgs_m, debug);


        if (debug)
            loginf << "latitude_deg " << latitude_deg
                   << " longitude_deg " << longitude_deg << " alt_wgs_m " << alt_wgs_m;
        // what to do with altitude?
    }

    return ret;
}


bool RS2GProjection::localXYToWGS84(unsigned int id, double x_m, double y_m,
                                    double& latitude_deg, double& longitude_deg, double& alt_wgs_m, bool debug)
{
    if (!hasCoordinateSystem(id))
        logerr << "no coord system for " << id;

    traced_assert(hasCoordinateSystem(id));

    bool ret {false};

    double ecef_x, ecef_y, ecef_z;

    coordinate_systems_.at(id)->localCart2Geocentric(x_m, y_m, 0, ecef_x, ecef_y, ecef_z);

    if (debug)
        loginf << "ecef_x " << std::fixed << ecef_x << " ecef_y " << ecef_y
               << " ecef_z " << ecef_z;

    ret = coordinate_systems_.at(id)->geocentric2Geodesic(ecef_x, ecef_y, ecef_z,
                                                          latitude_deg, longitude_deg, alt_wgs_m, debug);


    if (debug)
        loginf << "latitude_deg " << latitude_deg
               << " longitude_deg " << longitude_deg << " alt_wgs_m " << alt_wgs_m;
    // what to do with altitude?

    return ret;
}

bool RS2GProjection::polarToWGS84(unsigned int id, double azimuth_rad, double slant_range_m,
                                  bool has_baro_altitude, double baro_altitude_ft,
                                  double& ground_range_m,
                                  double& latitude_deg, double& longitude_deg, double& alt_wgs_m, bool debug)
{
    if (!hasCoordinateSystem(id))
        logerr << "no coord system for " << id;

    traced_assert(hasCoordinateSystem(id));

    if (debug)
        loginf << "azimuth_rad " << azimuth_rad << " slant_range_m " << slant_range_m
               << " has_baro " << has_baro_altitude << " baro_altitude_ft " << baro_altitude_ft;

    bool ret {false};

    double ecef_x, ecef_y, ecef_z;

    ret = coordinate_systems_.at(id)->calculateRadSlt2Geocentric(
        azimuth_rad, slant_range_m, has_baro_altitude, baro_altitude_ft * FT2M,
        ground_range_m, ecef_x, ecef_y, ecef_z, debug);

    if (debug)
        loginf << "ecef_x " << std::fixed << ecef_x << " ecef_y " << ecef_y
               << " ecef_z " << ecef_z;

    if (ret)
    {
        ret = coordinate_systems_.at(id)->geocentric2Geodesic(ecef_x, ecef_y, ecef_z,
                                                              latitude_deg, longitude_deg, alt_wgs_m, debug);


        if (debug)
            loginf << "latitude_deg " << latitude_deg
                   << " longitude_deg " << longitude_deg << " alt_wgs_m " << alt_wgs_m;
        // what to do with altitude?
    }

    return ret;
}

bool RS2GProjection::polarToWGS84(
    unsigned int id, double azimuth_rad, double slant_range_m,
    bool has_baro_altitude, double baro_altitude_ft, RadarBiasInfo& bias_info,
    double& latitude_deg, double& longitude_deg, double& alt_wgs_m, bool debug)
{
    if (!hasCoordinateSystem(id))
        logerr << "no coord system for " << id;

    traced_assert(hasCoordinateSystem(id));

    if (debug)
        loginf << "azimuth_rad " << azimuth_rad << " slant_range_m " << slant_range_m
               << " has_baro " << has_baro_altitude << " baro_altitude_ft " << baro_altitude_ft;

    bool ret {false};

    double ecef_x, ecef_y, ecef_z;

    ret = coordinate_systems_.at(id)->calculateRadSlt2Geocentric(
        azimuth_rad, slant_range_m, has_baro_altitude, baro_altitude_ft * FT2M, bias_info,
        ecef_x, ecef_y, ecef_z, debug);

    if (debug)
        loginf << "ecef_x " << std::fixed << ecef_x << " ecef_y " << ecef_y
               << " ecef_z " << ecef_z;

    if (ret)
    {
        ret = coordinate_systems_.at(id)->geocentric2Geodesic(ecef_x, ecef_y, ecef_z,
                                                              latitude_deg, longitude_deg, alt_wgs_m, debug);


        if (debug)
            loginf << "latitude_deg " << latitude_deg
                   << " longitude_deg " << longitude_deg << " alt_wgs_m " << alt_wgs_m;
        // what to do with altitude?
    }

    return ret;
}

bool RS2GProjection::wgs842PolarHorizontal(unsigned int id,
                                           double latitude_deg, double longitude_deg, double alt_wgs_m,
                                           double& azimuth_rad, double& slant_range_m, double& ground_range_m,
                                           double& radar_altitude_m, bool debug)
{
    if (!hasCoordinateSystem(id))
        logerr << "no coord system for " << id;

    traced_assert(hasCoordinateSystem(id));

    if (debug)
        loginf << "latitude_deg " << latitude_deg
               << " longitude_deg " << longitude_deg << " alt_wgs_m " << alt_wgs_m;

    // geodesic 2 geocentric
    double ecef_x, ecef_y, ecef_z;

    coordinate_systems_.at(id)->geodesic2Geocentric(latitude_deg * DEG2RAD, longitude_deg * DEG2RAD, alt_wgs_m,
                                                    ecef_x, ecef_y, ecef_z, debug);

    if (debug)
        loginf << "ecef_x " << std::fixed << ecef_x << " ecef_y " << ecef_y
               << " ecef_z " << ecef_z;

    double local_x, local_y, local_z;

    coordinate_systems_.at(id)->geocentric2LocalCart(ecef_x, ecef_y, ecef_z,
                                                     local_x, local_y, local_z, debug);

    if (debug)
        loginf << "local_x " << std::fixed << local_x
               << " local_y " << local_y << " local_z " << local_z;

    coordinate_systems_.at(id)->localCart2RadarSlant(local_x, local_y, local_z,
                                                     azimuth_rad, slant_range_m, ground_range_m,
                                                     radar_altitude_m, debug);

    if (debug)
        loginf << "azimuth_rad " << azimuth_rad
               << " slant_range_m " << slant_range_m
               << " ground_range_m " << ground_range_m << " radar_altitude_m " << radar_altitude_m;

    return true;
}
