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

#include <cstdint>
#include "dbcontent/target/targetreportdefs.h"
#include "dbcontent/target/targetreportmapping.h"
#include "targetposition.h"
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

class Chain
{
public:
    typedef TargetReport::DataID                                  DataID;
    typedef TargetReport::IndexMap                                IndexMap;

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
    std::uint64_t recordNumber(const DataID& id) const;

    TargetPosition pos(const DataID& id) const;
    boost::optional<TargetPosition> posOpt(const DataID& id) const;

    std::vector<TargetPosition> positionsBetween(const boost::posix_time::ptime& timestamp0,
                                                 const boost::posix_time::ptime& timestamp1,
                                                 bool include_t0,
                                                 bool include_t1) const;

    boost::optional<TargetPositionAccuracy> posAccuracy(const DataID& id) const;

    /// CAT020 I020/400 Contributing Receivers - list of 1-based RU indices
    /// referring to the source's RemoteUnitDefinition. Empty optional when the
    /// dbcontent does not carry the variable or the record is null.
    boost::optional<std::vector<unsigned int>> contribRecv(const DataID& id) const;

    boost::optional<TargetVelocity> speed(const DataID& id) const;
    // estimate ref baro alt at tod,index TODO should be replaced by real altitude reconstructor

    boost::optional<TargetVelocityAccuracy> speedAccuracy(const DataID& id) const;

    boost::optional<std::string> acid(const DataID& id) const;

    boost::optional<unsigned int> modeA(
            const DataID& id, bool ignore_invalid = true, bool ignore_garbled = true) const; // only if set

    boost::optional<float> modeC(
            const DataID& id, bool ignore_invalid = true, bool ignore_garbled = true) const; // only if set

    boost::optional<unsigned int> acad(const DataID& id) const;

    // CAT021 ADS-B quality indicators (only set for the CAT021 chain).
    boost::optional<unsigned int> mopsVersion(const DataID& id) const; // I021/210 VN
    boost::optional<unsigned int> nacp(const DataID& id) const;        // I021/090 NACp
    boost::optional<unsigned int> nucpNic(const DataID& id) const;     // I021/090 NUCp (v0) / NIC (v1/v2)

    // CAT021 time stamps in seconds of day (only set for the CAT021 chain).
    boost::optional<float> tomrPosition(const DataID& id) const; // I021/073 Time of Message Reception for Position
    boost::optional<float> tort(const DataID& id) const;         // I021/077 Time of Report Transmission

    boost::optional<bool> groundBit(const DataID& id) const; // gbs

    boost::optional<unsigned int> tstTrackNum(const DataID& id) const;
    boost::optional<bool> trackBegin(const DataID& id) const;     // first message of the track
    boost::optional<bool> trackConfirmed(const DataID& id) const; // false = tentative

    boost::optional<float> groundSpeed(const DataID& id) const; // m/s

    boost::optional<float> trackAngle(const DataID& id) const; // deg

    boost::optional<double> acceleration(const DataID& id) const; // m/s2
    boost::optional<float> rocd(const DataID& id) const; // ft/min
    boost::optional<unsigned char> momLongAcc(const DataID& id) const;
    boost::optional<unsigned char> momTransAcc(const DataID& id) const;
    boost::optional<unsigned char> momVertRate(const DataID& id) const;

    boost::optional<unsigned char> trackCoasting(const DataID& id) const;

    // Radar polar position (I0xx/040) and CAT010 SMR items. Empty optional when
    // the dbcontent does not carry the variable or the record is null.
    boost::optional<double> radarRange(const DataID& id) const;    // NM
    boost::optional<double> radarAzimuth(const DataID& id) const;  // deg
    boost::optional<unsigned char> detectionType(const DataID& id) const; // CAT010 I010/020 TYP
    boost::optional<bool> slantRangeCorrected(const DataID& id) const;    // CAT010 I010/020 TCC
    boost::optional<float> targetLength(const DataID& id) const;          // m, I010/270
    boost::optional<float> targetWidth(const DataID& id) const;           // m, I010/270
    boost::optional<double> targetOrientation(const DataID& id) const;    // deg, I010/270
    boost::optional<unsigned char> psrAmplitude(const DataID& id) const;  // I010/131

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

    IndexMap timestamp_index_lookup_; // timestamp -> index
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
