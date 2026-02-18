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

#include "dbcontent/target/targetreportdefs.h"
#include "dbcontent/target/targetposition.h"
#include "dbcontent/target/targetpositionaccuracy.h"
#include "dbcontent/target/targetvelocity.h"
#include "projection/transformation.h"

#include "boost/date_time/posix_time/ptime.hpp"
#include <boost/optional.hpp>

#include <vector>
#include <map>
#include <memory>
#include <vector>
#include <set>
#include <string>


namespace dbContent {

class DBContentAccessor;

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

    bool has_ref_rocd_ {false}; // rate of limb/descent ft/min
    double ref_rocd_{0};
};

class Chain
{
public:
    typedef TargetReport::DataID                                  DataID;
    typedef std::multimap<boost::posix_time::ptime, Index>      IndexMap;

    Chain(std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name);
    virtual ~Chain();

    void addIndex (boost::posix_time::ptime timestamp, unsigned int index);

    bool hasData() const;

    void finalize () const;

    const std::string& dbContent() const { return dbcontent_name_; }

    unsigned int size () const;
    unsigned int ignoredSize() const;

    boost::posix_time::ptime timeBegin() const;
    std::string timeBeginStr() const;
    boost::posix_time::ptime timeEnd() const;
    std::string timeEndStr() const;
    boost::posix_time::time_duration timeDuration() const;

    std::set<std::string> acids() const;
    std::string acidsStr() const;

    std::set<unsigned int> acads() const;
    std::string acadsStr() const;

    std::set<unsigned int> modeACodes() const;
    std::string modeACodesStr() const;

    bool hasModeC() const;
    float modeCMin() const;
    std::string modeCMinStr() const;
    float modeCMax() const;
    std::string modeCMaxStr() const;

    bool isPrimaryOnly () const;

    const IndexMap& timestampIndexes() const;

    double latitudeMin() const;
    double latitudeMax() const;
    double longitudeMin() const;
    double longitudeMax() const;

    bool hasPos() const;

    DataID dataID(const boost::posix_time::ptime& timestamp) const;

    std::vector<DataID> dataIDsBetween(const boost::posix_time::ptime& timestamp0,
                                       const boost::posix_time::ptime& timestamp1,
                                       bool include_t0,
                                       bool include_t1) const;

    unsigned int dsID(const DataID& id) const;

    TargetPosition pos(const DataID& id) const;
    boost::optional<TargetPosition> posOpt(const DataID& id) const;

    std::vector<TargetPosition> positionsBetween(const boost::posix_time::ptime& timestamp0,
                                                 const boost::posix_time::ptime& timestamp1,
                                                 bool include_t0,
                                                 bool include_t1) const;

    boost::optional<TargetPositionAccuracy> posAccuracy(const DataID& id) const;

    boost::optional<TargetVelocity> speed(const DataID& id) const;
    // estimate ref baro alt at tod,index TODO should be replaced by real altitude reconstructor

    boost::optional<TargetVelocityAccuracy> speedAccuracy(const DataID& id) const;

    boost::optional<std::string> acid(const DataID& id) const;

    boost::optional<unsigned int> modeA(
            const DataID& id, bool ignore_invalid = true, bool ignore_garbled = true) const; // only if set

    boost::optional<float> modeC(
            const DataID& id, bool ignore_invalid = true, bool ignore_garbled = true) const; // only if set

    boost::optional<unsigned int> acad(const DataID& id) const;

    boost::optional<bool> groundBit(const DataID& id) const; // gbs

    boost::optional<unsigned int> tstTrackNum(const DataID& id) const;

    boost::optional<float> groundSpeed(const DataID& id) const; // m/s

    boost::optional<float> trackAngle(const DataID& id) const; // deg

    boost::optional<double> acceleration(const DataID& id) const; // m/s2
    boost::optional<float> rocd(const DataID& id) const; // ft/min
    boost::optional<unsigned char> momLongAcc(const DataID& id) const;
    boost::optional<unsigned char> momTransAcc(const DataID& id) const;
    boost::optional<unsigned char> momVertRate(const DataID& id) const;

    boost::optional<unsigned char> trackCoasting(const DataID& id) const;

    Index indexFromDataID(const DataID& id) const;
    boost::posix_time::ptime timestampFromDataID(const DataID& id) const;

    DataMapping calculateDataMapping(boost::posix_time::ptime timestamp) const; // test tod
    void addPositionsSpeedsToMapping (DataMapping& mapping) const;

    DataMappingTimes findDataMappingTimes(boost::posix_time::ptime timestamp_ref) const; // ref tod

    void setIgnoredPositions(std::vector<bool> ignored_positions);
    bool ignorePosition(const DataID& id) const;

protected:
    std::shared_ptr<dbContent::DBContentAccessor> accessor_;
    std::string dbcontent_name_;

    std::multimap<boost::posix_time::ptime, Index> timestamp_index_lookup_; // timestamp -> index
    std::vector<unsigned int> indexes_;

    boost::optional<std::vector<bool>> ignored_positions_;

    mutable std::set<std::string> acids_;
    mutable std::set<unsigned int> acads_;
    mutable std::set<unsigned int> mode_a_codes_;

    mutable bool  has_mode_c_ {false};
    mutable float mode_c_min_ {0};
    mutable float mode_c_max_ {0};

    mutable bool   has_pos_      {false};
    mutable double latitude_min_ {0};
    mutable double latitude_max_ {0};

    mutable double longitude_min_ {0};
    mutable double longitude_max_ {0};

    mutable Transformation trafo_;

    void updateACIDs() const;
    void updateACADs() const;
    void updateModeACodes() const;
    void updateModeCMinMax() const;
    void updatePositionMinMax() const;

    std::pair<bool, float> estimateAltitude (
            const boost::posix_time::ptime& timestamp, unsigned int index_internal) const;

};

} // namespace TargetReport

} // namespace dbContent
