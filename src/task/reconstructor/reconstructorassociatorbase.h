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
#include "reconstructortarget.h"
#include "reconstructorbase.h"

// used settings from ReconstructorBaseSettings
// max_time_diff_
// track_max_time_diff_
// max_altitude_diff_
// target_max_positions_dubious_verified_rate_
// target_max_positions_dubious_unknown_rate_
// target_min_updates_
// target_prob_min_time_overlap_

class ReconstructorAssociatorBase
{
  public:

    enum DistanceClassification
    {
        //Distance_Unknown=0, // check not possible
        Distance_Good=0,
        Distance_Acceptable,
        Distance_Dubious,
        Distance_NotOK
    };

    ReconstructorAssociatorBase();

    virtual void associateNewData();
    virtual void reset();

    const std::map<unsigned int, std::map<unsigned int,
                                          std::pair<unsigned int, unsigned int>>>& assocAounts() const;

    struct AssociationOption
    {
        AssociationOption(){}

        AssociationOption(bool usable, unsigned int other_utn, unsigned int num_updates,
                          bool associate, float avg_distance)
            : usable_(usable), other_utn_(other_utn), num_updates_(num_updates),
            associate_based_on_secondary_attributes_(associate), avg_distance_(avg_distance)
        {}

        bool usable_ {false};
        unsigned int other_utn_ {0};
        unsigned int num_updates_{0};

        bool associate_based_on_secondary_attributes_ {false};
        float avg_distance_{0};
    };

    struct BatchStats
    {
        double batchSizeMean() const { return num_batches == 0 ? 0.0 : (double)batch_size_mean / (double)num_batches; } 
        double batchSizeInSliceMean() const { return num_batches_slice == 0 ? 0.0 : (double)batch_slice_size_mean / (double)num_batches_slice; } 
        double batchSizePrimaryOnlyMean() const { return num_batches_po == 0 ? 0.0 : (double)batch_po_size_mean / (double)num_batches_po; } 

        size_t num_batches       = 0;
        size_t num_batches_slice = 0;
        size_t num_batches_po    = 0;

        size_t batch_size_min  = std::numeric_limits<unsigned int>::max();
        size_t batch_size_max  = std::numeric_limits<unsigned int>::lowest();
        size_t batch_size_mean = 0;

        size_t batch_slice_size_min  = std::numeric_limits<unsigned int>::max();
        size_t batch_slice_size_max  = std::numeric_limits<unsigned int>::lowest();
        size_t batch_slice_size_mean = 0;

        size_t batch_po_size_min  = std::numeric_limits<unsigned int>::max();
        size_t batch_po_size_max  = std::numeric_limits<unsigned int>::lowest();
        size_t batch_po_size_mean = 0;
    };

    virtual bool canGetPositionOffsetTR(
        const dbContent::targetReport::ReconstructorInfo& tr,
        const dbContent::ReconstructorTarget& target, bool use_max_distance=true) = 0;
    virtual boost::optional<std::tuple<double, double, double>> getPositionOffsetTR(
        const dbContent::targetReport::ReconstructorInfo& tr,
        const dbContent::ReconstructorTarget& target,
        bool do_debug,
        const boost::optional<unsigned int>& thread_id,
        reconstruction::PredictionStats* stats = nullptr) = 0;

    const std::vector<unsigned long>& unassociatedRecNums() const;
    const std::map<unsigned int, BatchStats>& batchStatistics() const;

protected:

    boost::posix_time::time_duration max_time_diff_;

    std::map<unsigned int, std::map<unsigned int, std::pair<unsigned int, unsigned int>>> assoc_counts_;
    // ds_id -> dbcont id -> (assoc, unassoc cnt)
    std::vector<unsigned long> unassoc_rec_nums_;
    std::vector<unsigned long> unassoc_rec_nums_no_retry_;

    unsigned int num_merges_ {0};

    boost::posix_time::time_duration time_assoc_trs_;
    boost::posix_time::time_duration time_assoc_new_utns_;
    boost::posix_time::time_duration time_retry_assoc_trs_;

    std::map<unsigned int, BatchStats> batch_stats_;

    void associateTargetReports();
    void associateTargetReportBatch(const boost::posix_time::ptime& ts, 
                                    const ReconstructorBase::TargetReportBatch& batch);
    void associateTargetReports(std::set<unsigned int> dbcont_ids);

    virtual void associateUnreliablePrimaryOnly(const boost::posix_time::ptime& ts,
                                                const std::vector<unsigned long>& rec_nums,
                                                bool debug);

    void selfAssociateNewUTNs();
    void retryAssociateTargetReports();
    void associate(dbContent::targetReport::ReconstructorInfo& tr, int utn);
    virtual void postAssociate(dbContent::targetReport::ReconstructorInfo& tr, unsigned int utn) {};
    //void checkACADLookup();
    virtual void countUnAssociated();
    void countUnAssociated(const std::vector<unsigned long>& rec_nums);

    int findUTNFor (dbContent::targetReport::ReconstructorInfo& tr);

            // tries to find existing utn for target report, based on mode a/c and position, -1 if failed
    int findUTNByModeACPos (const dbContent::targetReport::ReconstructorInfo& tr);

    // score -> utn, other_utn
    std::pair<float, std::pair<unsigned int, unsigned int>> findUTNsForTarget (
        unsigned int utn); //  const std::set<unsigned int>& utns_to_ignore

    //unsigned int createNewTarget(const dbContent::targetReport::ReconstructorInfo& tr);

    virtual bool canGetPositionOffsetTargets(
        const boost::posix_time::ptime& ts,
        const dbContent::ReconstructorTarget& target0,
        const dbContent::ReconstructorTarget& target1) = 0;
    virtual boost::optional<std::tuple<double, double, double>> getPositionOffsetTargets(
        const boost::posix_time::ptime& ts,
        const dbContent::ReconstructorTarget& target0,
        const dbContent::ReconstructorTarget& target1,
        bool do_debug,
        const boost::optional<unsigned int>& thread_id,
        reconstruction::PredictionStats* stats = nullptr) = 0;

    virtual boost::optional<bool> checkTrackPositionOffsetAcceptable (
        dbContent::targetReport::ReconstructorInfo& tr, unsigned int utn,
        bool secondary_verified, bool do_debug) = 0;
    // empty if not possible, else check passed or failed returned
    virtual void doOutlierDetection (
        dbContent::targetReport::ReconstructorInfo& tr,
        unsigned int utn, bool do_debug) {};

    virtual boost::optional<std::pair<bool, double>> calculatePositionOffsetScore (
        const dbContent::targetReport::ReconstructorInfo& tr, unsigned int other_utn,
        double distance_m, double tgt_est_std_dev, double tr_est_std_dev, bool secondary_verified,
        bool do_debug) = 0;
    // check passed + score (larger is better) returned
    virtual std::tuple<DistanceClassification, double> checkPositionOffsetScore
        (double distance_m, double sum_stddev_est, bool secondary_verified) = 0;

    //virtual bool isTargetAverageDistanceAcceptable(double distance_score_avg, bool secondary_verified) = 0;

    virtual ReconstructorBase& reconstructor() = 0;
};

