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

#include "rs2gcoordinatesystem.h"

#include "global.h"
#include "stringconv.h"
#include "logger.h"
#include "radarbiasinfo.h"

using namespace Utils;

RS2GCoordinateSystem::RS2GCoordinateSystem(unsigned int id, double latitude_deg,
                                           double longitude_deg, double altitude_m)
    : ProjectionCoordinateSystemBase(id, latitude_deg, longitude_deg, altitude_m)
{
    logdbg << "id " << id_ << " lat " << latitude_deg_ << " long "
           << longitude_deg_ << " altitude " << altitude_m_;

    double lat_rad = latitude_deg_ * DEG2RAD;
    double long_rad = longitude_deg_ * DEG2RAD;

    rs2gFillMat(lat_rad, long_rad, rs2g_A_);

    rs2g_T_Ai_ = rs2g_A_.transpose();

    // Eigen::Matrix3d A_p0(3, 3), A_q0(3, 3);
    // A_p0.setZero();
    // A_q0.setZero();

    // Eigen::Vector3d b_p0(3), b_q0(3);
    // b_p0.setZero();
    // b_q0.setZero();

    // rs2gFillMat(lat_rad, long_rad, A_p0);
    // // mult(A_q0, trans(A_p0), *A_p0q0_); // A_p0q0 = A_q0 * Transpose(A_p0) in doxygen ...
    // rs2g_A_p0q0_ = A_q0 * A_p0.transpose();
    // //(*it).second.A_p0q0 = A_p0q0_;

    // // #if defined(DEBUG_ARTAS_TRF)
    // //       printf(" - radar matrix final values\n");
    // //       print_all_matrix(*A_p0q0_);
    // // #endif

    // rs2gFillVec(lat_rad, long_rad, altitude_m_, b_p0);

    // // add(scaled(b_q0, -1.0), b_p0, b_p0); // b_p0 = b_p0 - b_q0 in doxygen ...
    // b_p0 = b_p0 - b_q0;

    // // mult(A_q0, b_p0, *b_p0q0);    // b_p0q0 = A_q0 * (b_p0 - b_q0) in doxygen ...
    // rs2g_b_p0q0_ = A_q0 * b_p0;

    // // #if defined(DEBUG_ARTAS_TRF)
    // //       printf(" - radar vector final values\n");
    // //       print_vector(*b_p0q0);
    // // #endif
    // //    }

    // from setradar
    rs2gFillVec(lat_rad, long_rad, altitude_m_, rs2g_bi_);

    h_r_ = altitude_m_;
}

double RS2GCoordinateSystem::azimuth(double x_m, double y_m)
{
    double azimuth_rad = 0.0;

    if (x_m == 0.0 && y_m == 0.0)
        return azimuth_rad;

    //    this is the implementation in ARTAS/COMSOFT
    //    IRS document Appendix A2 v6.5 2002/06/28
    //
    //    if (x != 0.0 || y != 0.0) {
    //       if (y == 0.0) {
    //          if (x > 0.0)
    //             azimuth = M_PI / 2.0;
    //          else
    //             azimuth = 3.0 * M_PI / 2.0;
    //       }
    //       else {
    //          azimuth = atan(x/y);
    //          if (y < 0.0)
    //             azimuth += M_PI;
    //          else {
    //             if (x < 0.0)
    //                azimuth += 2.0 * M_PI;
    //          }
    //       }
    //    }

    // this is an equivalent implementation to the above
    // and was taken from the TRANSLIB library function
    // 'azimuth' in file 'artas_trans.c'
    if (fabs(y_m) < ALMOST_ZERO)
        azimuth_rad = (x_m / fabs(x_m)) * M_PI / 2.0;
    else
        azimuth_rad = atan2(x_m, y_m);

    if (azimuth_rad < 0.0)
        azimuth_rad += 2.0 * M_PI;

    return azimuth_rad;
}

void RS2GCoordinateSystem::radarSlant2LocalCart(double azimuth_rad, double rho_m,
                                                bool has_altitude, double altitude_m, double& ground_range_m,
                                                double& local_x, double& local_y, double& local_z, bool debug)
{
    // logdbg << "in x: " << local[0] << " y: " << local[1]
    //        << " z: " << local[2];

    double elevation_m {0}; // absolute WGS‑84 height

    if (has_altitude)
        elevation_m = altitude_m;
    else
        elevation_m = h_r_;  // the Z value has not been filled so use at least the radar height

    double elev_angle_rad = rs2gElevation(elevation_m, rho_m);

    ground_range_m = rho_m * cos(elev_angle_rad);

    if (debug)
        loginf << "has_altitude " << has_altitude
               << " elevation_m " << String::doubleToStringPrecision(elevation_m, 2)
               << " elev_angle_rad " << String::doubleToStringPrecision(elev_angle_rad, 6)
               << " ground_range_m " << String::doubleToStringPrecision(ground_range_m, 2);

    //    if (rho < 50000)
    //    {
    //        loginf << "radarSlant2LocalCart: in x: " << local[0] << " y: " << local[1] << " z: "
    //        << local[2]; loginf << "start" << rho << " elevation: " <<
    //        elevation << " azimuth: " << azimuth;
    //    }

    local_x = ground_range_m * sin(azimuth_rad);
    local_y = ground_range_m * cos(azimuth_rad);
    local_z = rho_m * sin(elev_angle_rad);

    if (debug)
        loginf << "local_x " << std::fixed << local_x
               << " local_y " << local_y << " local_z " << local_z;
}

void RS2GCoordinateSystem::radarSlant2LocalCart(double azimuth_rad, double rho_m,
                                                bool has_altitude, double altitude_m,
                                                RadarBiasInfo& bias_info,
                                                double& local_x, double& local_y, double& local_z, bool debug)
{
    double elevation_m {0};

    if (has_altitude)
        elevation_m = altitude_m;
    else
        elevation_m = h_r_;  // the Z value has not been filled so use at least the radar height

    double elev_angle_rad = rs2gElevation(elevation_m, rho_m);

    double ground_range_m = rho_m * cos(elev_angle_rad);

    if (bias_info.bias_valid_)
    {
        // Apply corrections: 
        // range_corrected = (range_measured - range_bias) / (1 + range_gain)
        // azimuth_corrected = azimuth_measured - azimuth_bias
        ground_range_m = (ground_range_m - bias_info.range_bias_m_) / (1.0 + bias_info.range_gain_);
        azimuth_rad = azimuth_rad - bias_info.azimuth_bias_deg_ * DEG2RAD;
    }

    if (debug)
        loginf << "has_altitude " << has_altitude
               << " elevation_m " << String::doubleToStringPrecision(elevation_m, 2)
               << " elev_angle_rad " << String::doubleToStringPrecision(elev_angle_rad, 6)
               << " ground_range_m " << String::doubleToStringPrecision(ground_range_m, 2);

    local_x = ground_range_m * sin(azimuth_rad);
    local_y = ground_range_m * cos(azimuth_rad);
    local_z = rho_m * sin(elev_angle_rad);

    if (debug)
        loginf << "local_x " << std::fixed << local_x
               << " local_y " << local_y << " local_z " << local_z;
}

void RS2GCoordinateSystem::localCart2RadarSlant(double local_x, double local_y, double local_z,
                                                double& azimuth_rad, double& slant_range_m, double& ground_range_m,
                                                double& altitude_m, bool debug)
{
    slant_range_m = sqrt(local_x*local_x + local_y*local_y + local_z*local_z);

    azimuth_rad = azimuth(local_x, local_y);

    ground_range_m =  sqrt(local_x*local_x + local_y*local_y); //getGroundRange(slant_range_m, true, h_r_ + local_z, debug);

    altitude_m = h_r_ + local_z;
}

// void RS2GCoordinateSystem::sysCart2SysStereo(Eigen::Vector3d& b, double* x, double* y)
// {
//     double H = sqrt(pow(b[0], 2) + pow(b[1], 2) + pow(b[2] + rs2g_ho_ + rs2g_Rto_, 2)) - rs2g_Rto_;
//     double k = 2 * rs2g_Rto_ / (2 * rs2g_Rto_ + rs2g_ho_ + b[2] + H);

//     *x = k * b[0];
//     *y = k * b[1];
// }

void RS2GCoordinateSystem::localCart2Geocentric(double local_x, double local_y, double local_z,
                                                double& ecef_x, double& ecef_y, double& ecef_z, bool debug)
{
    Eigen::Vector3d local_pos(local_x, local_y, local_z);

    // local cartesian to geocentric
    // mult(T_Ai_, input, Xinput); // Xinput = Transposed(Ai_) * input
    Eigen::Vector3d ecef_pos = rs2g_T_Ai_ * local_pos + rs2g_bi_;

    // add(bi_, Xinput, input); // Xinput = Xinput + bi_
    //ecef_pos += rs2g_bi_;

    ecef_x = ecef_pos[0];
    ecef_y = ecef_pos[1];
    ecef_z = ecef_pos[2];
}

void RS2GCoordinateSystem::geocentric2LocalCart(double ecef_x, double ecef_y, double ecef_z,
                                                double& local_x, double& local_y, double& local_z, bool debug)
{
    Eigen::Vector3d ecef_pos(ecef_x, ecef_y, ecef_z);

    Eigen::Vector3d local_pos = rs2g_A_ * (ecef_pos - rs2g_bi_);

    local_x = local_pos[0];
    local_y = local_pos[1];
    local_z = local_pos[2];
}

void RS2GCoordinateSystem::geodesic2LocalCart(double lat_rad, double lon_rad, double height_m,
                                              double& local_x, double& local_y, double& local_z, bool debug)
{
    double ecef_x, ecef_y, ecef_z;

    geodesic2Geocentric(lat_rad, lon_rad, height_m,
                        ecef_x, ecef_y, ecef_z);

    geocentric2LocalCart(ecef_x, ecef_y, ecef_z,
                         local_x, local_y, local_z);

}

bool RS2GCoordinateSystem::calculateRadSlt2Geocentric(double azimuth_rad, double slant_range_m,
                                                      bool has_altitude, double altitude_m, double& ground_range_m,
                                                      double& ecef_x, double& ecef_y, double& ecef_z, bool debug)
{
    Eigen::Vector3d local_pos(3);

    // the coordinates are in radar slant coordinates

    // radar slant to local cartesian
    double local_x, local_y, local_z;

    radarSlant2LocalCart(azimuth_rad, slant_range_m, has_altitude, altitude_m,
                         ground_range_m, local_x, local_y, local_z, debug);

    if (debug)
        loginf << "local_x " << std::fixed << local_x
               << " local_y " << local_y << " local_z " << local_z;

    // local cartesian to geocentric

    localCart2Geocentric(local_x, local_y, local_z, ecef_x, ecef_y, ecef_z);

    // geocentric to geodesic
    // Geocentric2Geodesic(pos);
    // done later

    return true;
}

bool RS2GCoordinateSystem::calculateRadSlt2Geocentric(double azimuth_rad, double slant_range_m,
                                                      bool has_altitude, double altitude_m,
                                                      RadarBiasInfo& bias_info,
                                                      double& ecef_x, double& ecef_y, double& ecef_z, bool debug)
{
    Eigen::Vector3d local_pos(3);

    // the coordinates are in radar slant coordinates

    // radar slant to local cartesian
    double local_x, local_y, local_z;

    radarSlant2LocalCart(azimuth_rad, slant_range_m, has_altitude, altitude_m, bias_info,
                         local_x, local_y, local_z, debug);

    if (debug)
        loginf << "local_x " << std::fixed << local_x
               << " local_y " << local_y << " local_z " << local_z;

    // local cartesian to geocentric

    localCart2Geocentric(local_x, local_y, local_z, ecef_x, ecef_y, ecef_z);

    // geocentric to geodesic
    // Geocentric2Geodesic(pos);
    // done later

    return true;
}

void RS2GCoordinateSystem::geodesic2Geocentric(double lat_rad, double lon_rad, double height_m,
                                               double& ecef_x, double& ecef_y, double& ecef_z, bool debug)
{
    // L = lat_rad
    // G = lon_rad
    // H = height_m

    double eta_s = EE_A / sqrt(1 - EE_E2 * pow(sin(lat_rad), 2));

    ecef_x = (eta_s + height_m) * cos(lat_rad) * cos(lon_rad);
    ecef_y = (eta_s + height_m) * cos(lat_rad) * sin(lon_rad);
    ecef_z = (eta_s * (1 - EE_E2) + height_m) * sin(lat_rad);

    // auto T = getTVector(lat_rad, lon_rad, height_m);

    // Eigen::Vector3d ecef_pos = T + rs2g_T_Ai_ * Eigen::Vector3d(lat_rad, lon_rad, height_m);

    // ecef_x = ecef_pos[0];
    // ecef_y = ecef_pos[1];
    // ecef_z = ecef_pos[2];

    // double ecef_x2 = (eta_s + height_m) * cos(lat_rad) * cos(lon_rad);
    // double ecef_y2 = (eta_s + height_m) * cos(lat_rad) * sin(lon_rad);
    // double ecef_z2 = (eta_s * (1 - EE_E2) + height_m) * sin(lat_rad);

    // traced_assert(fabs(ecef_x - ecef_x2) < 1E-10);
    // traced_assert(fabs(ecef_y - ecef_y2) < 1E-10);
    // traced_assert(fabs(ecef_z - ecef_z2) < 1E-10);
}

// as in R matrix
void RS2GCoordinateSystem::rs2gFillMat(double lat_rad, double lon_rad, Eigen::Matrix3d& A)
{
    A(0, 0) = -sin(lon_rad);
    A(0, 1) = cos(lon_rad);
    A(0, 2) = 0.0;
    A(1, 0) = -sin(lat_rad) * cos(lon_rad);
    A(1, 1) = -sin(lat_rad) * sin(lon_rad);
    A(1, 2) = cos(lat_rad);
    A(2, 0) = cos(lat_rad) * cos(lon_rad);
    A(2, 1) = cos(lat_rad) * sin(lon_rad);
    A(2, 2) = sin(lat_rad);
}

void RS2GCoordinateSystem::rs2gFillVec(double lat_rad, double lon_rad, double height_m, Eigen::Vector3d& b)
{
    geodesic2Geocentric(lat_rad, lon_rad, height_m, b[0], b[1], b[2]);
}

// Eigen::Vector3d RS2GCoordinateSystem::getTVector(double lat_rad, double lon_rad, double height_m)
// {
//     double eta_s = EE_A / sqrt(1 - EE_E2 * pow(sin(lat_rad), 2));

//     // L_S = lat_rad
//     // G_S = lon_rad
//     // h_s = height_m

//     double sin_lat = sin(lat_rad);

//     return Eigen::Vector3d (0,
//                            EE_E2 * eta_s * sin_lat * cos (lat_rad),
//                            EE_E2 * eta_s * sin_lat * sin_lat - (eta_s + height_m));
// }


bool RS2GCoordinateSystem::geocentric2Geodesic(double ecef_x, double ecef_y, double ecef_z,
                                               double& lat_deg, double& lon_deg, double& height_m, bool debug)
{
    double d_xy = sqrt(pow(ecef_x, 2) + pow(ecef_y, 2));

    double G = atan2(ecef_y, ecef_x);

    double L = atan2(ecef_z, (d_xy * (1 - EE_A * EE_E2 / sqrt(pow(d_xy, 2) + pow(ecef_z, 2)))));

    double eta = EE_A / sqrt(1 - EE_E2 * pow(sin(L), 2));
    double H = d_xy / cos(L) - eta;

    double Li;
    if (L >= 0.0)
        Li = -0.1;
    else
        Li = 0.1;

    while (fabs(L - Li) > PRECISION_GEODESIC)
    {
        Li = L;
        L = atan2(ecef_z * (1 + H / eta), (d_xy * (1 - EE_E2 + H / eta)));

        eta = EE_A / sqrt(1 - EE_E2 * pow(sin(L), 2));
        H = d_xy / cos(L) - eta;
    }

    lat_deg = L * RAD2DEG;
    lon_deg = G * RAD2DEG;
    height_m = H;

    return !std::isnan(lat_deg) && !std::isnan(lon_deg);
}

