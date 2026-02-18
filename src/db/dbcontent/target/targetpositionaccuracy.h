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

class TargetPositionAccuracy
{
public:
    TargetPositionAccuracy() = default;
    TargetPositionAccuracy(double x_stddev, double y_stddev, double xy_cov)
        : x_stddev_(x_stddev), y_stddev_(y_stddev), xy_cov_(xy_cov)
    {}

    double x_stddev_ {0}; // m
    double y_stddev_ {0}; // m
    double xy_cov_ {0}; // m^2

    double max() const { return std::max(x_stddev_, y_stddev_); }
};

class DBContentAccessor;
class BufferAccessor;

boost::optional<TargetPositionAccuracy> getPositionAccuracy(
        std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name, unsigned int index);

boost::optional<TargetPositionAccuracy> getRadarPositionAccuracy(
        std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name, unsigned int index);

boost::optional<TargetPositionAccuracy> getADSBPositionAccuracy(
        std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name, unsigned int index);

// cat010, cat020, cat062, reftraj
boost::optional<TargetPositionAccuracy> getXYPositionAccuracy(
        std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name, unsigned int index);

} // namespace dbContent
