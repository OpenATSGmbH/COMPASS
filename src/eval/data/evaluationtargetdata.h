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

#include "dbcontent/target/targetreportchain.h"
#include "evaluationdefs.h"
#include "timewindow.h"

#include "boost/date_time/posix_time/ptime.hpp"
#include <boost/optional.hpp>

#include <vector>
#include <map>
#include <memory>
#include <vector>
#include <set>
#include <string>

#include <Eigen/Core>

#include <QColor>

class Buffer;
class EvaluationTarget;
class EvaluationData;
class EvaluationCalculator;
class EvaluationManager;
class DBContentManager;
class SectorLayer;

class QAction;

/**
 */
class EvaluationTargetData
{
public:
    typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>            InsideCheckMatrix;
    typedef std::function<bool(const Evaluation::RequirementSumResultID&)> InterestEnabledFunc; 
    typedef std::map<Evaluation::RequirementSumResultID, double>           InterestMap;

    EvaluationTargetData(unsigned int utn, 
                         EvaluationData& eval_data,
                         std::shared_ptr<dbContent::DBContentAccessor> accessor,
                         EvaluationCalculator& calculator,
                         EvaluationManager& eval_man,
                         DBContentManager& dbcont_man);
    virtual ~EvaluationTargetData();

    void addRefIndex (boost::posix_time::ptime timestamp, unsigned int index);
    void addTstIndex (boost::posix_time::ptime timestamp, unsigned int index);

    bool hasData() const;
    bool hasRefData () const;
    bool hasTstData () const;

    void finalize () const;
    void updateToChanges() const;

    const unsigned int utn_{0};

    unsigned int numUpdates () const;
    unsigned int numRefUpdates () const;
    unsigned int numTstUpdates () const;

    boost::posix_time::ptime timeBegin() const;
    std::string timeBeginStr() const;
    boost::posix_time::ptime timeEnd() const;
    std::string timeEndStr() const;
    boost::posix_time::time_duration timeDuration() const;
    std::string timeDurationStr() const;

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
    bool isModeS () const;
    bool isModeACOnly () const;

    void updateUseInfo() const; // updates evaluation use information
    bool use() const;
    const Utils::TimeWindowCollection& excludedTimeWindows() const;
    bool isTimeStampNotExcluded(const boost::posix_time::ptime& ts) const;

    const std::set<std::string>& excludedRequirements() const;

    const dbContent::TargetReport::Chain& refChain() const;
    const dbContent::TargetReport::Chain& tstChain() const;

    double latitudeMin() const;
    double latitudeMax() const;
    double longitudeMin() const;
    double longitudeMax() const;

    bool hasPos() const;

    bool hasADSBInfo() const;
    bool hasMOPSVersion() const;
    std::set<unsigned int> mopsVersions() const;
    std::string mopsVersionStr() const;

    // ref
    bool hasMappedRefData(const dbContent::TargetReport::Chain::DataID& tst_id,
                          boost::posix_time::time_duration d_max) const;
    std::pair<boost::posix_time::ptime, boost::posix_time::ptime> mappedRefTimes(
            const dbContent::TargetReport::Chain::DataID& tst_id,
            boost::posix_time::time_duration d_max) const;
    // lower/upper times, {} if not existing

    boost::optional<dbContent::TargetPosition> mappedRefPos(
            const dbContent::TargetReport::Chain::DataID& tst_id) const;
    boost::optional<dbContent::TargetPosition> mappedRefPos(
            const dbContent::TargetReport::Chain::DataID& tst_id, boost::posix_time::time_duration d_max,
        bool debug=false) const;
    boost::optional<double> mappedRefMinAcc(
            const dbContent::TargetReport::Chain::DataID& tst_id, boost::posix_time::time_duration d_max,
        bool debug=false) const;

    // bool ok
    boost::optional<dbContent::TargetVelocity> mappedRefSpeed(
            const dbContent::TargetReport::Chain::DataID& tst_id, boost::posix_time::time_duration d_max) const;

    boost::optional<bool> mappedRefGroundBit(
            const dbContent::TargetReport::Chain::DataID& tst_id, boost::posix_time::time_duration d_max) const; // gbs

    // test
    unsigned int tstDSID(const dbContent::TargetReport::Chain::DataID& ref_id) const;
    boost::optional<bool> tstGroundBitInterpolated(const dbContent::TargetReport::Chain::DataID& tst_id,
                                                   const boost::posix_time::time_duration& d_max) const; // gds

    // TODO d_max not used
    boost::optional<bool> availableRefGroundBit(const dbContent::TargetReport::Chain::DataID& id,
                                                const boost::posix_time::time_duration& d_max) const;
    boost::optional<bool> availableTstGroundBit(const dbContent::TargetReport::Chain::DataID& id,
                                                const boost::posix_time::time_duration& d_max) const;

    // bool refPosAbove(const dbContent::TargetReport::Chain::DataID& id) const;
    // bool refPosGroundBitAvailable(const dbContent::TargetReport::Chain::DataID& id) const;
    bool refPosInside(const SectorLayer& layer,
                      const dbContent::TargetReport::Chain::DataID& id) const;
    // bool tstPosAbove(const dbContent::TargetReport::Chain::DataID& id) const;
    // bool tstPosGroundBitAvailable(const dbContent::TargetReport::Chain::DataID& id) const;
    bool tstPosInside(const SectorLayer& layer,
                      const dbContent::TargetReport::Chain::DataID& id) const;
    // bool mappedRefPosAbove(const dbContent::TargetReport::Chain::DataID& id) const;
    // bool mappedRefPosGroundBitAvailable(const dbContent::TargetReport::Chain::DataID& id) const;
    bool mappedRefPosInside(const SectorLayer& layer, 
                            const dbContent::TargetReport::Chain::DataID& id) const;

    static const int InterpGroundBitMaxSeconds = 15;
    static const int InterestFactorPrecision   = 3;

    // targets of interest
    void clearInterestFactors() const;
    void addInterestFactor(const Evaluation::RequirementSumResultID& id, 
                           double factor,
                           bool reset) const;
    const InterestMap& interestFactors() const;

    EvaluationTarget toTarget() const;
    static void updateTarget(DBContentManager& dbcontent_manager,
                             EvaluationTarget& target); // updates only comment, nothing more

    static std::string stringForInterestFactor(const Evaluation::RequirementSumResultID& id, 
                                               double factor);
    static QColor bgColorForInterestFactorRequirement(double factor);
    static QColor fgColorForInterestFactorRequirement(double factor);
    static QColor bgColorForInterestFactorSum(double factor);
    static QColor fgColorForInterestFactorSum(double factor);
    static unsigned int bgStyleForInterestFactorSum(double factor);
    static unsigned int fgStyleForInterestFactorSum(double factor);

    static std::string enabledInterestFactorsString(const InterestMap& interest_factors,
                                                    const InterestEnabledFunc& interest_enabled_func);
    static QAction* interestFactorAction(const Evaluation::RequirementSumResultID& id, 
                                         double interest_factor);

    static double interest_thres_req_high_, interest_thres_req_mid_;
    static double interest_thres_sum_high_, interest_thres_sum_mid_;

protected:
    void updateACIDs() const;
    void updateACADs() const;
    void updateModeACodes() const;
    void updateModeCMinMax() const;
    void updatePositionMinMax() const;
//    //void updateADSBInfo() const;

    void calculateTestDataMappings() const;
    void computeSectorInsideInfo() const;
    void computeSectorInsideInfo(InsideCheckMatrix& mat, 
                                 const dbContent::TargetPosition& pos,
                                 unsigned int idx_internal,
                                 const boost::optional<bool>& ground_bit,
                                 const SectorLayer* min_height_filter = nullptr) const;
    // bool checkAbove(const InsideCheckMatrix& mat,
    //                 const dbContent::TargetReport::Index& index) const;
    // bool checkGroundBit(const InsideCheckMatrix& mat,
    //                     const dbContent::TargetReport::Index& index) const;
    bool checkInside(const SectorLayer& layer,
                     const InsideCheckMatrix& mat,
                     const dbContent::TargetReport::Index& index) const;
    
    EvaluationData& eval_data_;
    std::shared_ptr<dbContent::DBContentAccessor> accessor_;
    EvaluationCalculator& calculator_;
    EvaluationManager& eval_man_;
    DBContentManager& dbcont_man_;

    dbContent::TargetReport::Chain ref_chain_;
    dbContent::TargetReport::Chain tst_chain_;

    mutable std::vector<dbContent::TargetReport::DataMapping> tst_data_mappings_;
    
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

    mutable bool has_adsb_info_ {false};
    mutable bool has_mops_versions_ {false};
    mutable std::set<unsigned int> mops_versions_;

    mutable bool use_in_eval_;
    mutable Utils::TimeWindowCollection excluded_time_windows_;
    mutable std::set<std::string> excluded_requirements_;

    mutable InsideCheckMatrix                    inside_ref_;
    mutable InsideCheckMatrix                    inside_tst_;
    mutable InsideCheckMatrix                    inside_map_;
    mutable std::map<const SectorLayer*, size_t> inside_sector_layers_;

    mutable InterestMap interest_factors_;
};
