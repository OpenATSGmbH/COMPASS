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

#include "targetposition.h"
#include "dbcontent/target/targetvelocity.h"

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/optional.hpp>

#include <map>
#include <stdexcept>

class Transformation;

namespace dbContent {

namespace TargetReport {

/**
 */
struct Index
{
    Index() = default;
    Index(unsigned int idx_ext,
          unsigned int idx_int) : idx_external(idx_ext), idx_internal(idx_int) {}

    unsigned int idx_external; //external index (usually index into buffer)
    unsigned int idx_internal; //internal index (index into internal data structures)
};

/// timestamp -> index of the target reports of a chain
typedef std::multimap<boost::posix_time::ptime, Index> IndexMap;

/**
 */
class DataID
{
  public:
    typedef std::pair<const boost::posix_time::ptime, Index> IndexPair;

    DataID() = default;
    DataID(const boost::posix_time::ptime& timestamp) : timestamp_(timestamp), valid_(true) {}
    DataID(const boost::posix_time::ptime& timestamp, const Index& index) : timestamp_(timestamp), index_(index), valid_(true) {}
    DataID(const IndexPair& ipair) : timestamp_(ipair.first), index_(ipair.second), valid_(true) {}
    virtual ~DataID() = default;

    bool valid() const { return valid_; }
    const boost::posix_time::ptime& timestamp() const { return timestamp_; }
    bool hasIndex() const { return index_.has_value(); }

    DataID& addIndex(const Index& index)
    {
        index_ = index;
        return *this;
    }

    const Index& index() const
    {
        if (!hasIndex())
            throw std::runtime_error("DataID: index: No index stored");
        return index_.value();
    }

  private:
    boost::posix_time::ptime timestamp_;
    boost::optional<Index>   index_;
    bool                     valid_ = false;
};


struct DataMappingTimes // mapping to respective tst data
{
    boost::posix_time::ptime timestamp_; // tod of test

    bool has_other1_ {false};
    boost::posix_time::ptime timestamp_other1_;
    DataID dataid_other1_;

    bool has_other2_ {false};
    boost::posix_time::ptime timestamp_other2_;
    DataID dataid_other2_;
};


struct DataMapping // mapping to respective ref data
{
    boost::posix_time::ptime timestamp_; // timestamp of original

    bool has_ref1_ {false};
    boost::posix_time::ptime timestamp_ref1_;
    DataID dataid_ref1_;

    bool has_ref2_ {false};
    boost::posix_time::ptime timestamp_ref2_;
    DataID dataid_ref2_;

    bool has_ref_pos_ {false};
    dbContent::TargetPosition pos_ref_;

    bool has_ref_spd_ {false};
    dbContent::TargetVelocity spd_ref_;

    bool has_ref_acc_ {false}; // acceleration m/s2
    double ref_acc_{0};

    bool has_ref_rocd_ {false}; // rate of climb/descent ft/min
    double ref_rocd_{0};
};

/// The entries of an IndexMap around a timestamp. With several entries at one timestamp,
/// the first of them is used.
struct Bracket
{
    boost::optional<DataID> exact;  // entry at the timestamp
    boost::optional<DataID> before; // entry at the last timestamp before the timestamp
    boost::optional<DataID> after;  // entry at the first timestamp after the timestamp
};

Bracket findBracket(const IndexMap& indexes, const boost::posix_time::ptime& timestamp);

/// First entry with begin <= timestamp < end (the first of several at one timestamp),
/// none if there is none. An entry at end belongs to a range that begins there.
boost::optional<DataID> firstInRange(const IndexMap& indexes,
                                     const boost::posix_time::ptime& begin,
                                     const boost::posix_time::ptime& end);

/// Last entry with begin <= timestamp <= end (the last of several at one timestamp),
/// none if there is none.
boost::optional<DataID> lastInRange(const IndexMap& indexes,
                                    const boost::posix_time::ptime& begin,
                                    const boost::posix_time::ptime& end);

/// Reference entries for a timestamp: the exact entry as ref1 only, else the entries
/// before (ref1) and after (ref2), or none if one of them is missing. Positions,
/// speeds etc. are not set.
DataMapping findDataMapping(const IndexMap& indexes, const boost::posix_time::ptime& timestamp);

/// Entries around a timestamp: other1 is the entry before, other2 the exact entry or else
/// the entry after. Without an exact entry both other1 and other2 are needed, else none is set.
DataMappingTimes findDataMappingTimes(const IndexMap& indexes, const boost::posix_time::ptime& timestamp);

/// True if the mapping is an exact match, or if both reference entries are at most d_max
/// away from the mapping's timestamp.
bool withinMaxGap(const DataMapping& mapping, const boost::posix_time::time_duration& d_max);

/// The values of one target report used for the interpolation.
struct Sample
{
    boost::posix_time::ptime timestamp;
    TargetPosition pos;
    boost::optional<TargetVelocity> speed;  // m/s, track angle deg
    boost::optional<double> acceleration;   // m/s2
    boost::optional<double> rocd;           // ft/min
};

/// Linear interpolation between s1 and s2 at timestamp, s1.timestamp < s2.timestamp and
/// timestamp in [s1.timestamp, s2.timestamp]. The position is interpolated in the local
/// Cartesian plane around s1, the track angle along the shorter turn. A value is only
/// interpolated if both samples have it, the altitude also if only one sample has it (then
/// that one is used). If both horizontal positions are equal, the horizontal position is s1's,
/// the altitude is interpolated, speed, acceleration and rate of climb/descent are not set.
/// Returns none if the transformation fails.
boost::optional<Sample> interpolate(const Sample& s1, const Sample& s2,
                                    const boost::posix_time::ptime& timestamp,
                                    Transformation& trafo);

} // namespace TargetReport

} // namespace dbContent
