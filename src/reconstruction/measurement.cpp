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


#include "measurement.h"

#include "kalman_filter.h"

#include "global.h"
#include "timeconv.h"
#include "logger.h"

#include <osgEarth/GeoMath>

namespace reconstruction
{

const std::string Measurement::FieldSourceID    = "source_id";
const std::string Measurement::FieldTS          = "t";

const std::string Measurement::FieldInterp      = "mm_interp";
const std::string Measurement::FieldInterpFirst = "mm_interp_first";
const std::string Measurement::FieldPosAccCorr  = "pos_acc_corrected";
const std::string Measurement::FieldStopped     = "stopped";

const std::string Measurement::FieldLat         = "lat";
const std::string Measurement::FieldLon         = "lon";

const std::string Measurement::FieldX           = "x";
const std::string Measurement::FieldY           = "y";
const std::string Measurement::FieldZ           = "z";

const std::string Measurement::FieldVX          = "vx";
const std::string Measurement::FieldVY          = "vy";
const std::string Measurement::FieldVZ          = "vz";

const std::string Measurement::FieldAX          = "ax";
const std::string Measurement::FieldAY          = "ay";
const std::string Measurement::FieldAZ          = "az";

const std::string Measurement::FieldXStdDev     = "x_stddev";
const std::string Measurement::FieldYStdDev     = "y_stddev";
const std::string Measurement::FieldXYCov       = "xy_cov";

const std::string Measurement::FieldVXStdDev    = "vx_stddev";
const std::string Measurement::FieldVYStdDev    = "vy_stddev";

const std::string Measurement::FieldAXStdDev    = "ax_stddev";
const std::string Measurement::FieldAYStdDev    = "ay_stddev";

const std::string Measurement::FieldQVar        = "Q_var";
const std::string Measurement::FieldQVarInterp  = "Q_var_interp";

/**
*/
Eigen::Vector2d Measurement::position2D(CoordSystem cs) const
{
    if (cs == CoordSystem::WGS84)
        return Eigen::Vector2d(lat, lon);
    return Eigen::Vector2d(x, y);
}

/**
*/
void Measurement::position2D(const Eigen::Vector2d& pos, CoordSystem cs)
{
    if (cs == CoordSystem::WGS84)
    {
        lat = pos[ 0 ];
        lon = pos[ 1 ];
    }
    x = pos[ 0 ];
    y = pos[ 1 ];
}

/**
*/
double Measurement::distance(const Measurement& other, CoordSystem cs) const
{
    if (cs == CoordSystem::WGS84)
        return (Eigen::Vector2d(lat, lon) - Eigen::Vector2d(other.lat, other.lon)).norm();

    if (z.has_value() && other.z.has_value())
        return (Eigen::Vector3d(x, y, z.value()) - Eigen::Vector3d(other.x, other.y, other.z.value())).norm();

    return (Eigen::Vector2d(x, y) - Eigen::Vector2d(other.x, other.y)).norm();
}

/**
*/
double Measurement::distanceSqr(const Measurement& other, CoordSystem cs) const
{
    if (cs == CoordSystem::WGS84)
        return (Eigen::Vector2d(lat, lon) - Eigen::Vector2d(other.lat, other.lon)).squaredNorm();

    if (z.has_value() && other.z.has_value())
        return (Eigen::Vector3d(x, y, z.value()) - Eigen::Vector3d(other.x, other.y, other.z.value())).squaredNorm();

    return (Eigen::Vector2d(x, y) - Eigen::Vector2d(other.x, other.y)).squaredNorm();
}

/**
*/
Utils::Accuracy::GeoInfo Measurement::geoInfo() const
{
    return Utils::Accuracy::GeoInfo(lat, lon);
}

/**
*/
Utils::Accuracy::GeoAccInfo Measurement::geoAccuracyInfo() const
{
    traced_assert(hasStdDevPosition());
    return Utils::Accuracy::GeoAccInfo(lat, lon, x_stddev.value(), y_stddev.value(), hasCovPosition() ? xy_cov.value() : 0.0);
}

/**
*/
double Measurement::geodeticDistance(const Measurement& other) const
{
    return geoInfo().geodeticDistance(other.geoInfo());
}

/**
*/
double Measurement::bearing(const Measurement& other) const
{
    return geoInfo().bearing(other.geoInfo());
}

/**
*/
Utils::Accuracy::GeodeticDistanceInfo Measurement::geodeticDistanceInfo(const Measurement& other) const
{
    return geoAccuracyInfo().distance(other.geoAccuracyInfo());
}

/**
*/
double Measurement::mahalanobisDistanceGeodetic(const Measurement& other) const
{
    return geodeticDistanceInfo(other).mahalanobisDistance();
}

/**
*/
double Measurement::mahalanobisDistanceGeodeticSqr(const Measurement& other) const
{
    return geodeticDistanceInfo(other).mahalanobisDistanceSqr();
}

/**
*/
boost::optional<double> Measurement::mahalanobisDistance(const Measurement& other,
                                                         unsigned char components,
                                                         bool verbose) const
{
    auto P = covMat(components);
    auto R = other.covMat(components);

    auto x = stateVec(components);
    auto z = other.stateVec(components);

    auto y = z - x;

    if (verbose)
    {
        loginf << "detection state:\n" << z;
        loginf << "detection covmat:\n" << R;
        loginf << "predicted state:\n" << x;
        loginf << "predicted covmat:\n" << P + R;
    }

    return kalman::KalmanFilter::mahalanobis(y, P + R, false);
}

/**
*/
boost::optional<double> Measurement::mahalanobisDistanceSqr(const Measurement& other, 
                                                            unsigned char components,
                                                            bool verbose) const
{
    auto P = covMat(components);
    auto R = other.covMat(components);

    auto x = stateVec(components);
    auto z = other.stateVec(components);

    auto y = z - x;

    if (verbose)
    {
        loginf << "detection state:\n" << z;
        loginf << "detection covmat:\n" << R;
        loginf << "predicted state:\n" << x;
        loginf << "predicted covmat:\n" << P + R;
    }

    return kalman::KalmanFilter::mahalanobisSqr(y, P + R, false);
}

/**
*/
boost::optional<double> Measurement::likelihood(const Measurement& other,
                                                unsigned char components,
                                                bool verbose) const
{
    auto P = covMat(components);
    auto R = other.covMat(components);

    auto x = stateVec(components);
    auto z = other.stateVec(components);

    auto y = z - x;

    if (verbose)
    {
        loginf << "detection state:\n" << z;
        loginf << "detection covmat:\n" << R;
        loginf << "predicted state:\n" << x;
        loginf << "predicted covmat:\n" << P + R;
    }

    return kalman::KalmanFilter::likelihood(y, P + R, true);
}

/**
*/
boost::optional<double> Measurement::logLikelihood(const Measurement& other,
                                                   unsigned char components,
                                                   bool verbose) const
{
    auto P = covMat(components);
    auto R = other.covMat(components);

    auto x = stateVec(components);
    auto z = other.stateVec(components);

    auto y = z - x;

    if (verbose)
    {
        loginf << "detection state:\n" << z;
        loginf << "detection covmat:\n" << R;
        loginf << "predicted state:\n" << x;
        loginf << "predicted covmat:\n" << P + R;
    }

    return kalman::KalmanFilter::logLikelihood(y, P + R, true);
}

/**
*/
bool Measurement::hasVelocity() const
{
    if (!vx.has_value() || !vy.has_value())
        return false;

    if (z.has_value() && !vz.has_value())
        return false;

    return true;
}

/**
*/
bool Measurement::hasAcceleration() const
{
    if (!ax.has_value() || !ay.has_value())
        return false;

    if (z.has_value() && !az.has_value())
        return false;

    return true;
}

/**
*/
bool Measurement::hasStdDevPosition() const
{
    return (x_stddev.has_value() && y_stddev.has_value());
}

/**
 */
bool Measurement::hasCovPosition() const
{
    return xy_cov.has_value();
}

/**
*/
dbContent::targetReport::Position Measurement::position() const
{
    return {lat, lon};
}

/**
*/
dbContent::targetReport::PositionAccuracy Measurement::positionAccuracy() const
{
    traced_assert(hasStdDevPosition());

    if (xy_cov.has_value())
        return dbContent::targetReport::PositionAccuracy(*x_stddev, *y_stddev, *xy_cov);
    else
        return dbContent::targetReport::PositionAccuracy(*x_stddev, *y_stddev, 0);
}

/**
*/
bool Measurement::hasStdDevVelocity() const
{
    return (vx_stddev.has_value() && vy_stddev.has_value());
}

/**
*/
bool Measurement::hasStdDevAccel() const
{
    return (ax_stddev.has_value() && ay_stddev.has_value());
}

/**
*/
std::string Measurement::asString(const std::string& prefix) const
{
    std::stringstream ss;

    std::string alt_str = (z.has_value() ? std::to_string(z.value()) : "-");

    std::string vx_str  = (vx.has_value() ? std::to_string(vx.value()) : "-");
    std::string vy_str  = (vy.has_value() ? std::to_string(vy.value()) : "-");
    std::string vz_str  = (vz.has_value() ? std::to_string(vz.value()) : "-");

    std::string ax_str  = (ax.has_value() ? std::to_string(ax.value()) : "-");
    std::string ay_str  = (ay.has_value() ? std::to_string(ay.value()) : "-");
    std::string az_str  = (az.has_value() ? std::to_string(az.value()) : "-");

    std::string x_stddev_str = (x_stddev.has_value() ? std::to_string(x_stddev.value()) : "-");
    std::string y_stddev_str = (y_stddev.has_value() ? std::to_string(y_stddev.value()) : "-");
    std::string xy_cov_str   = (  xy_cov.has_value() ? std::to_string(xy_cov.value())   : "-");

    std::string vx_stddev_str = (vx_stddev.has_value() ? std::to_string(vx_stddev.value()) : "-");
    std::string vy_stddev_str = (vy_stddev.has_value() ? std::to_string(vy_stddev.value()) : "-");

    std::string ax_stddev_str = (ax_stddev.has_value() ? std::to_string(ax_stddev.value()) : "-");
    std::string ay_stddev_str = (ay_stddev.has_value() ? std::to_string(ay_stddev.value()) : "-");

    std::string Q_var_str        = (Q_var.has_value()        ? std::to_string(Q_var.value())        : "-");
    std::string Q_var_interp_str = (Q_var_interp.has_value() ? std::to_string(Q_var_interp.value()) : "-");

    ss << prefix << "source_id:    " << (source_id.has_value() ? std::to_string(source_id.value()) : "-") << std::endl;
    ss << prefix << "timestamp:    " << Utils::Time::toString(t) << std::endl;
    ss << prefix << "interp:       " << mm_interp << std::endl;
    ss << prefix << "pos acc corr: " << pos_acc_corrected << std::endl;
    ss << prefix << "pos wgs84:    " << lat << ", " << lon << std::endl;
    ss << prefix << "pos cart:     " << x << ", " << y << ", " << alt_str << " (" << x_stddev_str << ", " << y_stddev_str << ", " << xy_cov_str << ")" << std::endl;
    ss << prefix << "velocity:     " << vx_str << ", " << vy_str << " (" << vx_stddev_str << ", " << vy_stddev_str << ")" << std::endl;
    ss << prefix << "accel:        " << ax_str << ", " << ay_str << " (" << ax_stddev_str << ", " << ay_stddev_str << ")" << std::endl;
    ss << prefix << "Q_var:        " << Q_var_str << std::endl;
    ss << prefix << "Q_var_interp: " << Q_var_interp_str;

    return ss.str();
}

/**
*/
Eigen::VectorXd Measurement::stateVec(unsigned char flags) const
{
    bool with_position = (flags & CovMatFlags::CovMatPos);
    bool with_velocity = (flags & CovMatFlags::CovMatVel);
    bool with_accell   = (flags & CovMatFlags::CovMatAcc);

    traced_assert(!with_velocity || hasVelocity()    );
    traced_assert(!with_accell   || hasAcceleration());

    if (with_position)
    {
        if (with_velocity)
        {
            if (with_accell)
            {
                //generate 6 from position, velocity and acceleration
                Eigen::VectorXd X = Eigen::VectorXd::Zero(6);
                X[ 0 ] = x;
                X[ 1 ] = vx.value();
                X[ 2 ] = ax.value();
                X[ 3 ] = y;
                X[ 4 ] = vy.value();
                X[ 5 ] = ay.value();

                return X;
            }
            else
            {
                //generate 4 from position and velocity
                Eigen::VectorXd X = Eigen::VectorXd::Zero(4);
                X[ 0 ] = x;
                X[ 1 ] = vx.value();
                X[ 2 ] = y;
                X[ 3 ] = vy.value();

                return X;
            }
        }
        else
        {
            //generate 2 from position
            Eigen::VectorXd X = Eigen::VectorXd::Zero(2);
            X[ 0 ] =  x;
            X[ 1 ] =  y;

            return X;
        }
    }

    return Eigen::VectorXd();
}

/**
*/
Eigen::MatrixXd Measurement::covMat(unsigned char flags) const
{
    bool with_position = (flags & CovMatFlags::CovMatPos);
    bool with_velocity = (flags & CovMatFlags::CovMatVel);
    bool with_accell   = (flags & CovMatFlags::CovMatAcc);

    traced_assert(!with_position || hasStdDevPosition());
    traced_assert(!with_velocity || hasStdDevVelocity());
    traced_assert(!with_accell   || hasStdDevAccel()   );

    if (with_position)
    {
        if (with_velocity)
        {
            if (with_accell)
            {
                //generate 6x6 from position, velocity and acceleration
                Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6);
                C(0, 0) =  x_stddev.value() *  x_stddev.value();
                C(1, 1) = vx_stddev.value() * vx_stddev.value();
                C(2, 2) = ax_stddev.value() * ax_stddev.value();
                C(3, 3) =  y_stddev.value() *  y_stddev.value();
                C(4, 4) = vy_stddev.value() * vy_stddev.value();
                C(5, 5) = ay_stddev.value() * ay_stddev.value();

                if (hasCovPosition())
                {
                    C(0, 3) = xy_cov.value();
                    C(3, 0) = xy_cov.value();
                }

                return C;
            }
            else
            {
                //generate 4x4 from position and velocity
                Eigen::MatrixXd C = Eigen::MatrixXd::Zero(4, 4);
                C(0, 0) =  x_stddev.value() *  x_stddev.value();
                C(1, 1) = vx_stddev.value() * vx_stddev.value();
                C(2, 2) =  y_stddev.value() *  y_stddev.value();
                C(3, 3) = vy_stddev.value() * vy_stddev.value();

                if (hasCovPosition())
                {
                    C(0, 2) = xy_cov.value();
                    C(2, 0) = xy_cov.value();
                }

                return C;
            }
        }
        else
        {
            //generate 2x2 from position
            Eigen::MatrixXd C = Eigen::MatrixXd::Zero(2, 2);
            C(0, 0) =  x_stddev.value() *  x_stddev.value();
            C(1, 1) =  y_stddev.value() *  y_stddev.value();

            if (hasCovPosition())
            {
                C(0, 1) = xy_cov.value();
                C(1, 0) = xy_cov.value();
            }

            return C;
        }
    }

    return Eigen::MatrixXd();
}

/**
*/
unsigned char Measurement::covMatFlags() const
{
    unsigned char flags = 0;

    if (hasStdDevPosition())
        flags |= CovMatFlags::CovMatPos;
    if (hasStdDevVelocity())
        flags |= CovMatFlags::CovMatVel;
    if (hasStdDevAccel())
        flags |= CovMatFlags::CovMatAcc;

    return flags;
}

namespace covmat
{
    void setMMPosAcc(Measurement& mm, const Eigen::MatrixXd& C, int idx0, int idx1)
    {
        mm.x_stddev = std::sqrt(C(idx0, idx0));
        mm.y_stddev = std::sqrt(C(idx1, idx1));
    }
    void setMMPosCov(Measurement& mm, const Eigen::MatrixXd& C, int idx0, int idx1)
    {
        mm.xy_cov = C(idx0, idx1);
    }
    void setMMVelAcc(Measurement& mm, const Eigen::MatrixXd& C, int idx0, int idx1)
    {
        mm.vx_stddev = std::sqrt(C(idx0, idx0));
        mm.vy_stddev = std::sqrt(C(idx1, idx1));
    }
    void setMMAccAcc(Measurement& mm, const Eigen::MatrixXd& C, int idx0, int idx1)
    {
        mm.ax_stddev = std::sqrt(C(idx0, idx0));
        mm.ay_stddev = std::sqrt(C(idx1, idx1));
    }
}

/**
*/
bool Measurement::setFromCovMat(const Eigen::MatrixXd& C, unsigned char flags)
{
    if (C.cols() == 2 && C.rows() == 2)
    {
        if (flags & CovMatFlags::CovMatPos) 
        {
            covmat::setMMPosAcc(*this, C, 0, 1);
            covmat::setMMPosCov(*this, C, 0, 1);
        }

        return true;
    }
    else if (C.cols() == 4 && C.rows() == 4)
    {
        if (flags & CovMatFlags::CovMatPos) 
        {
            covmat::setMMPosAcc(*this, C, 0, 2);
            covmat::setMMPosCov(*this, C, 0, 2);
        }
        if (flags & CovMatFlags::CovMatVel) covmat::setMMVelAcc(*this, C, 1, 3);

        return true;
    }
    else if (C.cols() == 6 && C.rows() == 6)
    {
        if (flags & CovMatFlags::CovMatPos) 
        {
            covmat::setMMPosAcc(*this, C, 0, 3);
            covmat::setMMPosCov(*this, C, 0, 3);
        }
        if (flags & CovMatFlags::CovMatVel) covmat::setMMVelAcc(*this, C, 1, 4);
        if (flags & CovMatFlags::CovMatAcc) covmat::setMMAccAcc(*this, C, 2, 5);

        return true;
    }

    //weird covmat size
    return false;
}

/**
*/
std::pair<unsigned long, boost::posix_time::ptime> Measurement::uniqueID() const
{
    traced_assert(source_id.has_value());
    return std::pair<unsigned long, boost::posix_time::ptime>(source_id.value(), t);
}

/**
*/
Measurement Measurement::replaceTimestamp(const boost::posix_time::ptime& t) const
{
    auto mm_cpy = *this;
    mm_cpy.t = t;

    return mm_cpy;
}

/**
*/
nlohmann::json Measurement::toJSON() const
{
    nlohmann::json j;

    if (source_id.has_value())
        j[ FieldSourceID ] = source_id.value();

    j[ FieldTS ] = Utils::Time::toString(t);
    j[ FieldInterp ] = mm_interp;
    j[ FieldInterpFirst ] = mm_interp_first;
    j[ FieldPosAccCorr ] = pos_acc_corrected;
    j[ FieldStopped ] = stopped;

    j[ FieldLat ] = lat;
    j[ FieldLon ] = lon;

    j[ FieldX ] = x;
    j[ FieldY ] = y;
    if (z.has_value())
        j[ FieldZ ] = z.value();

    if (vx.has_value())
        j[ FieldVX ] = vx.value();
    if (vy.has_value())
        j[ FieldVY ] = vy.value();
    if (vz.has_value())
        j[ FieldVZ ] = vz.value();

    if (ax.has_value())
        j[ FieldAX ] = ax.value();
    if (ay.has_value())
        j[ FieldAY ] = ay.value();
    if (az.has_value())
        j[ FieldAZ ] = az.value();
    
    if (x_stddev.has_value())
        j[ FieldXStdDev ] = x_stddev.value();
    if (y_stddev.has_value())
        j[ FieldYStdDev ] = y_stddev.value();
    if (xy_cov.has_value())
        j[ FieldXYCov ] = xy_cov.value();

    if (vx_stddev.has_value())
        j[ FieldVXStdDev ] = vx_stddev.value();
    if (vy_stddev.has_value())
        j[ FieldVYStdDev ] = vy_stddev.value();
    
    if (ax_stddev.has_value())
        j[ FieldAXStdDev ] = ax_stddev.value();
    if (ay_stddev.has_value())
        j[ FieldAYStdDev ] = ay_stddev.value();

    if (Q_var.has_value())
        j[ FieldQVar ] = Q_var.value();
    if (Q_var_interp.has_value())
        j[ FieldQVarInterp ] = Q_var_interp.value();

    return j;
}

/**
*/
bool Measurement::fromJSON(const nlohmann::json& j)
{
    if (!j.is_object())
        return false;

    if (!j.contains(FieldTS)         ||
        !j.contains(FieldLat)        ||
        !j.contains(FieldLon)        ||
        !j.contains(FieldX)          ||
        !j.contains(FieldY))
    {
        return false;
    }

    if (j.contains(FieldSourceID))
        source_id = j[ FieldSourceID ].get<unsigned long>();

    t = Utils::Time::fromString(j[ FieldTS ].get<std::string>());

    if (j.contains(FieldInterp))
        mm_interp = j[ FieldInterp ].get<bool>();
    if (j.contains(FieldInterpFirst))
        mm_interp_first = j[ FieldInterpFirst ].get<bool>();
    if (j.contains(FieldPosAccCorr))
        pos_acc_corrected = j[ FieldPosAccCorr ].get<bool>();
    if (j.contains(FieldStopped))
        stopped = j[ FieldStopped ].get<bool>();

    lat = j[ FieldLat ].get<double>();
    lon = j[ FieldLon ].get<double>();

    x = j[ FieldX ].get<double>();
    y = j[ FieldY ].get<double>();
    if (j.contains(FieldZ))
        z = j[ FieldZ ].get<double>();

    if (j.contains(FieldVX))
        vx = j[ FieldVX ].get<double>();
    if (j.contains(FieldVY))
        vy = j[ FieldVY ].get<double>();
    if (j.contains(FieldVZ))
        vz = j[ FieldVZ ].get<double>();

    if (j.contains(FieldAX))
        ax = j[ FieldAX ].get<double>();
    if (j.contains(FieldAY))
        ay = j[ FieldAY ].get<double>();
    if (j.contains(FieldAZ))
        az = j[ FieldAZ ].get<double>();

    if (j.contains(FieldXStdDev))
        x_stddev = j[ FieldXStdDev ].get<double>();
    if (j.contains(FieldYStdDev))
        y_stddev = j[ FieldYStdDev ].get<double>();
    if (j.contains(FieldXYCov))
        xy_cov = j[ FieldXYCov ].get<double>();

    if (j.contains(FieldVXStdDev))
        vx_stddev = j[ FieldVXStdDev ].get<double>();
    if (j.contains(FieldVYStdDev))
        vy_stddev = j[ FieldVYStdDev ].get<double>();

    if (j.contains(FieldAXStdDev))
        ax_stddev = j[ FieldAXStdDev ].get<double>();
    if (j.contains(FieldAYStdDev))
        ay_stddev = j[ FieldAYStdDev ].get<double>();

    if (j.contains(FieldQVar))
        Q_var = j[ FieldQVar ].get<double>();
    if (j.contains(FieldQVarInterp))
        Q_var_interp = j[ FieldQVarInterp ].get<double>();

    return true;
}

}  // namespace reconstruction
