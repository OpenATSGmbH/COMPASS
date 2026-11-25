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

#include "targetreportmapping.h"
#include "projection/transformation.h"
#include "number.h"
#include "timeconv.h"
#include "logger.h"
#include "traced_assert.h"

#include <iterator>
#include <tuple>

using namespace boost::posix_time;
using namespace Utils;

namespace dbContent {

namespace TargetReport {

Bracket findBracket(const IndexMap& indexes, const ptime& timestamp)
{
    Bracket ret;

    auto lower = indexes.lower_bound(timestamp);
    auto upper = indexes.upper_bound(timestamp);

    if (lower != indexes.begin())
        ret.before = DataID(*indexes.lower_bound(std::prev(lower)->first));

    if (lower != upper)
        ret.exact = DataID(*lower);

    if (upper != indexes.end())
        ret.after = DataID(*upper);

    return ret;
}

DataMapping findDataMapping(const IndexMap& indexes, const ptime& timestamp)
{
    DataMapping ret;
    ret.timestamp_ = timestamp;

    Bracket bracket = findBracket(indexes, timestamp);

    if (bracket.exact)
    {
        ret.has_ref1_       = true;
        ret.timestamp_ref1_ = timestamp;
        ret.dataid_ref1_    = *bracket.exact;
    }
    else if (bracket.before && bracket.after)
    {
        ret.has_ref1_       = true;
        ret.timestamp_ref1_ = bracket.before->timestamp();
        ret.dataid_ref1_    = *bracket.before;

        ret.has_ref2_       = true;
        ret.timestamp_ref2_ = bracket.after->timestamp();
        ret.dataid_ref2_    = *bracket.after;
    }

    return ret;
}

DataMappingTimes findDataMappingTimes(const IndexMap& indexes, const ptime& timestamp)
{
    DataMappingTimes ret;
    ret.timestamp_ = timestamp;

    Bracket bracket = findBracket(indexes, timestamp);

    const auto& other2 = bracket.exact ? bracket.exact : bracket.after;

    if (!other2 || (!bracket.exact && !bracket.before))
        return ret;

    ret.has_other2_       = true;
    ret.timestamp_other2_ = other2->timestamp();
    ret.dataid_other2_    = *other2;

    if (bracket.before)
    {
        ret.has_other1_       = true;
        ret.timestamp_other1_ = bracket.before->timestamp();
        ret.dataid_other1_    = *bracket.before;
    }

    return ret;
}

bool withinMaxGap(const DataMapping& mapping, const time_duration& d_max)
{
    if (!mapping.has_ref1_)
        return false;

    if (!mapping.has_ref2_)
        return mapping.timestamp_ref1_ == mapping.timestamp_;

    return mapping.timestamp_ - mapping.timestamp_ref1_ <= d_max
           && mapping.timestamp_ref2_ - mapping.timestamp_ <= d_max;
}

boost::optional<Sample> interpolate(const Sample& s1, const Sample& s2,
                                    const ptime& timestamp,
                                    Transformation& trafo)
{
    traced_assert(s1.timestamp < s2.timestamp);
    traced_assert(s1.timestamp <= timestamp && timestamp <= s2.timestamp);

    Sample ret;
    ret.timestamp = timestamp;

    const TargetPosition& pos1 = s1.pos;
    const TargetPosition& pos2 = s2.pos;

    if (pos1.latitude_ == pos2.latitude_ && pos1.longitude_ == pos2.longitude_)
    {
        ret.pos = pos1;
        return ret;
    }

    // float as before the move into this function, keeps the results identical
    float d_t  = Time::partialSeconds(s2.timestamp - s1.timestamp);
    float d_t2 = Time::partialSeconds(timestamp - s1.timestamp);

    bool ok;
    double x_pos, y_pos;

    std::tie(ok, x_pos, y_pos) = trafo.distanceCart(
        pos1.latitude_, pos1.longitude_, pos2.latitude_, pos2.longitude_);

    if (!ok)
    {
        logerr << "error with latitude " << pos2.latitude_ << " longitude " << pos2.longitude_;
        return {};
    }

    double v_x = x_pos / d_t;
    double v_y = y_pos / d_t;

    double latitude, longitude;

    std::tie(ok, latitude, longitude) = trafo.wgsAddCartOffset(
        pos1.latitude_, pos1.longitude_, v_x * d_t2, v_y * d_t2);

    if (!ok)
    {
        logerr << "error with offset from latitude " << pos1.latitude_ << " longitude " << pos1.longitude_;
        return {};
    }

    bool  has_altitude = false;
    float altitude     = 0.0;

    if (pos1.has_altitude_ && !pos2.has_altitude_)
    {
        has_altitude = true;
        altitude     = pos1.altitude_;
    }
    else if (!pos1.has_altitude_ && pos2.has_altitude_)
    {
        has_altitude = true;
        altitude     = pos2.altitude_;
    }
    else if (pos1.has_altitude_ && pos2.has_altitude_)
    {
        float v_alt  = (pos2.altitude_ - pos1.altitude_) / d_t;
        has_altitude = true;
        altitude     = pos1.altitude_ + v_alt * d_t2;
    }

    ret.pos = TargetPosition(latitude, longitude, has_altitude, true, altitude);

    if (s1.speed && s2.speed)
    {
        double acceleration_ms2 = (s2.speed->speed_ - s1.speed->speed_) / d_t;

        TargetVelocity speed;
        speed.speed_       = s1.speed->speed_ + acceleration_ms2 * d_t2;
        speed.track_angle_ = Number::interpolateAngle(s1.speed->track_angle_, s2.speed->track_angle_, d_t2 / d_t);

        ret.speed = speed;
    }

    if (s1.acceleration && s2.acceleration)
        ret.acceleration = *s1.acceleration + (*s2.acceleration - *s1.acceleration) / d_t * d_t2;

    if (s1.rocd && s2.rocd)
        ret.rocd = *s1.rocd + (*s2.rocd - *s1.rocd) / d_t * d_t2;

    return ret;
}

} // namespace TargetReport

} // namespace dbContent
