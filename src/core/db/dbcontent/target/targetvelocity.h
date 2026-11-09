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

#include <boost/optional.hpp>

#include <memory>

namespace dbContent {

class TargetVelocity
{
public:
    TargetVelocity() {}

    double track_angle_ {0}; // true north, deg
    double speed_ {0}; // m/s
};

class TargetVelocityAccuracy
{
public:
    TargetVelocityAccuracy() = default;
    TargetVelocityAccuracy(double vx_stddev, double vy_stddev)
        : vx_stddev_(vx_stddev), vy_stddev_(vy_stddev)
    {}

    double vx_stddev_ {0}; // m/s
    double vy_stddev_ {0}; // m/s
};

class DBContentAccessor;

boost::optional<TargetVelocityAccuracy> getVelocityAccuracy(
        std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name, unsigned int index);

boost::optional<TargetVelocityAccuracy> getVelocityAccuracyADSB(
        std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name, unsigned int index);

boost::optional<TargetVelocityAccuracy> getVelocityAccuracyTracker(
        std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name, unsigned int index);

}
