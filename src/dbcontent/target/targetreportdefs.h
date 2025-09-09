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

#include "datasourcebase.h"

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>

#include <map>
#include <memory>


namespace dbContent 
{

class DBContentAccessor;
class TargetReportAccessor;

namespace targetReport
{

/**
 */
struct Position
{
    Position() {}
    Position(double lat, double lon) : latitude_(lat), longitude_(lon) {}

    double latitude_  {0}; // deg
    double longitude_ {0}; // deg
};

/**
 */
class BarometricAltitude
{
  public:

    enum class Source
    {
        Barometric_ModeC = 0,
        Barometric_CAT062_Trusted,
        Barometric_CAT062_Secondary
    };

    BarometricAltitude() {}
    BarometricAltitude(Source source,
                       float alt,
                       const boost::optional<bool>& v,
                       const boost::optional<bool>& g)
        :   source_  (source)
          ,   altitude_(alt   )
          ,   valid_   (v     )
          ,   garbled_ (g     ) {}

    Source                source_   = Source::Barometric_ModeC;
    float                 altitude_ = 0.0f;
    boost::optional<bool> valid_;
    boost::optional<bool> garbled_;

    bool hasReliableValue () const
    {
        return valid_ && *valid_ && garbled_ && !(*garbled_) ;
    }
};

/**
 */
class PositionAccuracy
{
  public:
    PositionAccuracy() = default;
    PositionAccuracy(double x_stddev,
                     double y_stddev,
                     double xy_cov)
        :   x_stddev_(x_stddev),
          y_stddev_(y_stddev),
          xy_cov_  (xy_cov  ) {}

    PositionAccuracy operator* (double scale) const
    { return PositionAccuracy(x_stddev_ * scale, y_stddev_ * scale, xy_cov_ * scale * scale); }

    PositionAccuracy& operator* (double scale)
    {
        x_stddev_ *= scale;
        y_stddev_ *= scale;
        xy_cov_ *= scale*scale;

        return *this;
    }

    double x_stddev_ {0}; // m
    double y_stddev_ {0}; // m
    double xy_cov_   {0}; // m^2

    double avgStdDev() const { return (x_stddev_+ y_stddev_) / 2.0; }
    double minStdDev() const { return std::min(x_stddev_, y_stddev_); }
    double maxStdDev() const { return std::max(x_stddev_, y_stddev_); }


    PositionAccuracy getScaledToMinStdDev (double min_std_dev) const;
    void scaleToMinStdDev(double min_stddev);
    void scaleUsing(double scale_factor);

    std::string asStr() const;

    bool isNormal() const
    { return std::isfinite(x_stddev_) && std::isfinite(y_stddev_) && std::isfinite(xy_cov_); }
};

/**
 */
struct AccuracyTables
{
    static const std::map<int, float> adsb_nucr_nacv_accuracies;
    static const std::map<int, float> adsb_v0_accuracies;
    static const std::map<int, float> adsb_v12_accuracies;
};


/**
 */
struct Velocity
{
    Velocity() {}
    Velocity(double track_angle, double speed) : track_angle_(track_angle), speed_(speed) {}

    double track_angle_ {0}; // true north, deg
    double speed_       {0}; // m/s
};

/**
 */
struct VelocityAccuracy
{
    VelocityAccuracy() = default;
    VelocityAccuracy(double vx_stddev, double vy_stddev) : vx_stddev_(vx_stddev), vy_stddev_(vy_stddev) {}

    double minStdDev() const { return std::min(vx_stddev_, vy_stddev_); }
    double maxStdDev() const { return std::max(vx_stddev_, vy_stddev_); }

    VelocityAccuracy getScaledToMinStdDev (double min_std_dev) const;
    void scaleToMinStdDev(double min_stddev);

    double vx_stddev_ {0}; // m/s
    double vy_stddev_ {0}; // m/s
};

struct AccelerationAccuracy
{
    AccelerationAccuracy() = default;
    AccelerationAccuracy(double ax_stddev, double ay_stddev) : ax_stddev_(ax_stddev), ay_stddev_(ay_stddev) {}

    double minStdDev() const { return std::min(ax_stddev_, ay_stddev_); }
    double maxStdDev() const { return std::max(ax_stddev_, ay_stddev_); }

    double ax_stddev_ {0}; // m/s^2
    double ay_stddev_ {0}; // m/s^2
};

class ModeACode
{
  public:

    ModeACode() {}
    ModeACode(unsigned int code,
              const boost::optional<bool>& v,
              const boost::optional<bool>& g,
              const boost::optional<bool>& smoothed)
        :   code_  (code)
          ,   valid_   (v)
          ,   garbled_ (g)
          ,   smoothed_(smoothed)
    {}

    unsigned int code_;
    boost::optional<bool> valid_;
    boost::optional<bool> garbled_;
    boost::optional<bool> smoothed_;

    bool hasReliableValue () const
    {
        if (valid_ && *valid_ == false)
            return false;

        if (garbled_ && *garbled_ == true)
            return false;

        return true;
    }

    virtual std::string asStr() const;
};

struct BaseInfo
{
    unsigned int buffer_index_ {0};
    unsigned long record_num_ {0};
    unsigned int dbcont_id_ {0};
    unsigned int ds_id_ {0};
    dbContent::DataSourceType ds_type_{DataSourceType::Other};
    unsigned int line_id_ {0};
    boost::posix_time::ptime timestamp_;

    virtual std::string asStr() const;
};


struct ReconstructorInfo : public BaseInfo
{
    bool in_current_slice_ {false};        // true in current, false if from old slice (glue data)
    bool is_calculated_reference_ {false}; // the target report stems from a previous reconstructor run

    boost::optional<unsigned int> acad_;
    boost::optional<std::string> acid_;
    boost::optional<targetReport::ModeACode> mode_a_code_;

    boost::optional<unsigned int> track_number_;
    boost::optional<bool> track_begin_;
    boost::optional<bool> track_end_;

    boost::optional<targetReport::Position> position_;
    boost::optional<targetReport::Position> position_corrected_;
    boost::optional<targetReport::PositionAccuracy> position_accuracy_;

    bool unsused_ds_pos_ {false}; // set if data source should not be used for pos
    bool invalidated_pos_ {false}; // if invalidated by validate function
    bool is_pos_outlier_ {false}; // if set by outlier detection

    boost::optional<targetReport::BarometricAltitude> barometric_altitude_;

    boost::optional<targetReport::Velocity> velocity_;
    boost::optional<targetReport::VelocityAccuracy> velocity_accuracy_;

    boost::optional<double> track_angle_;
    boost::optional<bool> ground_bit_;
    bool data_source_is_ground_only {false};

    // adsb
    boost::optional<unsigned int> mops_;
    boost::optional<unsigned int> ecat_;

    static const double GroundSpeedMin;

    boost::optional<targetReport::Position>& position();
    const boost::optional<targetReport::Position>& position() const;

    virtual std::string asStr() const;

    bool isModeSDetection() const;
    bool isModeACDetection() const;
    bool isPrimaryOnlyDetection() const;
    bool isUnreliablePrimaryOnlyDetection() const;

    bool isOnGround() const; // false if unknown
    bool isMoving() const;   // true if unknown

    bool doNotUsePosition() const;
};

// tmp list

// flag old/current slice

// secondary stuff: acid, acad, mode a, mode c
// position, corrected position, position acc, corrected pos acc
// velocity, velocity acc
// ground bit

// Kalman update (x,P,F,Q, ...)



//class PerDSChain
//{
//public:
//    PerDSChain(std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name,
//             unsigned int ds_id);
//    virtual ~PerDSChain();

//    void addIndex (boost::posix_time::ptime timestamp, unsigned int index);

//    bool hasData() const;
//    unsigned int size () const;
//};

//class FullChain
//{

//};


} // namespace targetReport

} // namespace dbContent

