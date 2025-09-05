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

#include "targetreportdefs.h"
#include "accuracy.h"

#include <Eigen/Core>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "json.hpp"

namespace reconstruction
{

enum class CoordSystem
{
    Unknown = 0,
    WGS84,
    Cart
};

enum CovMatFlags
{
    CovMatPos = 1 << 0,
    CovMatVel = 1 << 1,
    CovMatAcc = 1 << 2
};

/**
*/
struct Measurement
{
    Eigen::Vector2d position2D(CoordSystem cs) const;

    void position2D(const Eigen::Vector2d& pos, CoordSystem cs);

    double distance(const Measurement& other, CoordSystem cs) const;
    double distanceSqr(const Measurement& other, CoordSystem cs) const;

    double geodeticDistance(const Measurement& other) const;
    double bearing(const Measurement& other) const;
    Utils::Accuracy::GeodeticDistanceInfo geodeticDistanceInfo(const Measurement& other) const;
    double mahalanobisDistanceGeodetic(const Measurement& other) const;
    double mahalanobisDistanceGeodeticSqr(const Measurement& other) const;

    boost::optional<double> mahalanobisDistance(const Measurement& other, 
                                                unsigned char components = CovMatPos,
                                                bool verbose = false) const;
    boost::optional<double> mahalanobisDistanceSqr(const Measurement& other, 
                                                   unsigned char components = CovMatPos,
                                                   bool verbose = false) const;
    boost::optional<double> likelihood(const Measurement& other, 
                                       unsigned char components = CovMatPos,
                                       bool verbose = false) const;
    boost::optional<double> logLikelihood(const Measurement& other, 
                                          unsigned char components = CovMatPos,
                                          bool verbose = false) const;

    bool hasVelocity() const;
    bool hasAcceleration() const;
    bool hasStdDevPosition() const;
    bool hasCovPosition() const;

    dbContent::targetReport::Position position() const;
    dbContent::targetReport::PositionAccuracy positionAccuracy() const;

    bool hasStdDevVelocity() const;
    bool hasStdDevAccel() const;

    std::string asString(const std::string& prefix = "") const;

    Eigen::VectorXd stateVec(unsigned char components = 255) const;
    Eigen::MatrixXd covMat(unsigned char components = 255) const;
    unsigned char covMatFlags() const;
    bool setFromCovMat(const Eigen::MatrixXd& C, unsigned char components = 255);

    std::pair<unsigned long, boost::posix_time::ptime> uniqueID() const;

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    static const std::string FieldSourceID;
    static const std::string FieldTS;
    
    static const std::string FieldInterp;
    static const std::string FieldInterpFirst;
    static const std::string FieldPosAccCorr;

    static const std::string FieldLat;
    static const std::string FieldLon;

    static const std::string FieldX;
    static const std::string FieldY;
    static const std::string FieldZ;

    static const std::string FieldVX;
    static const std::string FieldVY;
    static const std::string FieldVZ;

    static const std::string FieldAX;
    static const std::string FieldAY;
    static const std::string FieldAZ;

    static const std::string FieldXStdDev;
    static const std::string FieldYStdDev;
    static const std::string FieldXYCov;

    static const std::string FieldVXStdDev;
    static const std::string FieldVYStdDev;

    static const std::string FieldAXStdDev;
    static const std::string FieldAYStdDev;

    static const std::string FieldQVar;
    static const std::string FieldQVarInterp;

    boost::optional<unsigned long> source_id;           // source of the measurement

    boost::posix_time::ptime t;                         // timestamp

    bool                     mm_interp         = false; // measurement has been interpolated (e.g. by spline interpolator)
    bool                     mm_interp_first   = false; // measurement is the first one in an interpolated interval
    bool                     pos_acc_corrected = false; // position accuracy has been corrected due to invalid correlation coefficient
    bool                     test_clutter      = false; // measurement is test clutter (for debugging purpose)

    double                   lat;                       // wgs84 latitude 
    double                   lon;                       // wgs84 longitude

    mutable double           x;                         // x position (hack: mutable because of on-demand projection in KalmanEstimator)
    mutable double           y;                         // y position (hack: mutable because of on-demand projection in KalmanEstimator)
    boost::optional<double>  z;                         // optional z position

    boost::optional<double>  vx;                        // speed vector x
    boost::optional<double>  vy;                        // speed vector y
    boost::optional<double>  vz;                        // speed vector z

    boost::optional<double>  ax;                        // accel vector x
    boost::optional<double>  ay;                        // accel vector y
    boost::optional<double>  az;                        // accel vector z

    boost::optional<double>  x_stddev;                  // position stddev x
    boost::optional<double>  y_stddev;                  // position stddev y
    boost::optional<double>  xy_cov;                    // position xy-covariance value

    boost::optional<double>  vx_stddev;                 // velocity stddev x
    boost::optional<double>  vy_stddev;                 // velocity stddev y

    boost::optional<double>  ax_stddev;                 // accel stddev x
    boost::optional<double>  ay_stddev;                 // accel stddev y

    boost::optional<float>   Q_var;                     // optional mm-specific process noise variance
    boost::optional<float>   Q_var_interp;              // optional mm-specific process noise variance for interpolation
};

}  // namespace reconstruction
