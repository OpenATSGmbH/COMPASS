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

#include "dbcontentaccessor.h"
#include "accuracyestimatorbase.h"
#include "configurable.h"
#include "targetreportdefs.h"
#include "reconstructortarget.h"
#include "referencecalculator.h"
#include "kalman_chain.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <map>
#include <string>
#include <memory>

#include <QObject>

#define DO_RECONSTRUCTOR_PEDANTIC_CHECKING 0

namespace dbContent
{
class VariableSet;
class ReconstructorTarget;
}

namespace reconstruction
{
struct Measurement;
class KalmanChainPredictors;
}

class Buffer;
class ReconstructorTask;
class ReconstructorAssociatorBase;

struct AltitudeState;

class ReconstructorBaseSettings
{
  public:
    ReconstructorBaseSettings() {};

    boost::posix_time::time_duration sliceDuration() const
    {
        return boost::posix_time::minutes(slice_duration_in_minutes);
    }
    boost::posix_time::time_duration outdatedDuration() const
    {
        return boost::posix_time::minutes(outdated_duration_in_minutes);
    }

    // output
    std::string  ds_name {"CalcRef"};
    unsigned int ds_sac  {REC_DS_SAC};
    unsigned int ds_sic  {REC_DS_SIC};
    unsigned int ds_line {0};

    //timeframe
    boost::posix_time::ptime data_timestamp_min;
    boost::posix_time::ptime data_timestamp_max;

    // slicing
    unsigned int slice_duration_in_minutes    {15};
    unsigned int outdated_duration_in_minutes {2};

    bool delete_all_calc_reftraj {true};
    bool ignore_calculated_references {true};

    // maximum time difference in target reports to do comparisons
    float max_time_diff_ {10}; // sec
    // maximum altitude difference to consider mode c the "same"
    float max_altitude_diff_ {300.0};
    // maximimum time difference between track updates, otherwise considered new track
    double track_max_time_diff_ {300.0};
    // do track number disassociation based on distance
    bool do_track_number_disassociate_using_distance_ {false};
    // if do tn disassc, factor for "normal" assoc threshold to calc threshold
    float tn_disassoc_distance_factor_ {3};

    // compare targets related
    double target_prob_min_time_overlap_ {0.1};
    unsigned int target_min_updates_ {2}; // TODO HP
    double target_max_positions_dubious_verified_rate_ {0.5};
    double target_max_positions_dubious_unknown_rate_ {0.3};

    double target_max_positions_not_ok_verified_rate_ {0.1};
    double target_max_positions_not_ok_unknown_rate_ {0.05};

    double targets_min_assoc_score_ {100};

    static const unsigned int REC_DS_SAC = 255;
    static const unsigned int REC_DS_SIC = 1;

    // target classification
    float min_aircraft_modec_ {1000};

    std::string vehicle_acids_;
    std::set<std::string> vehicle_acids_set_;
    std::string vehicle_acads_;
    std::set<unsigned int> vehicle_acads_set_;

    // fallback accuracies
    double numerical_min_std_dev_ {1E-3};

    float unspecific_pos_acc_fallback_ {1000};
    float unspecifc_vel_acc_fallback_ {50};
    float unspecifc_acc_acc_fallback_ {10};

    float no_value_acc_fallback_ {10000};

    bool use_stopped_adsb_tracking_ {true};

    // experimental switches (most likely to be removed)
    bool do_not_merge_utns_ {false};
    bool do_not_merge_targets_with_tentative_origin_ {false}; // used for testing primary only tracking
    bool do_not_associate_systracks_ {false};                 // used for testing primary only tracking

    void setVehicleACIDs(const std::string& value);
    void setVehicleACADs(const std::string& value);
};

/**
 */
class ReconstructorBase : public QObject, public Configurable
{
    Q_OBJECT
public:
    struct DataSlice
    {
        bool isWritten(const boost::posix_time::ptime& t) const
        {
            return (t >= remove_before_time_ && t < write_before_time_);
        }
        bool inSlice(const boost::posix_time::ptime& t) const
        {
            return (t >= slice_begin_ && t < next_slice_begin_);
        }

        unsigned int slice_count_;
        boost::posix_time::ptime slice_begin_;
        boost::posix_time::ptime next_slice_begin_;
        boost::posix_time::ptime timestamp_min_, timestamp_max_;
        bool first_slice_ {false};
        bool is_last_slice_ {false};

        boost::posix_time::ptime remove_before_time_;
        boost::posix_time::ptime write_before_time_;

        std::map<std::string, std::shared_ptr<Buffer>> data_;
        bool loading_done_ {false}; // set if data_ is set correctly and can be processed

        std::map<std::string, std::shared_ptr<Buffer>> assoc_data_;
        std::map<std::string, std::shared_ptr<Buffer>> reftraj_data_;

        bool processing_done_ {false}; // set if assoc_data_, reftraj_data_ are set correctly and can be written
        bool write_done_ {false}; // set if data has been written

        unsigned int run_count_ {0};
    };

    struct TargetsContainer
    {
        static std::set<std::string> unspecific_acids_;

        TargetsContainer(ReconstructorBase* reconstructor)
            :reconstructor_(reconstructor) { traced_assert(reconstructor_); }

        ReconstructorBase* reconstructor_;

        std::vector<unsigned int> utn_vec_;

        std::map<unsigned int, unsigned int> acad_2_utn_; // acad dec -> utn
        std::map<std::string, unsigned int> acid_2_utn_; // acid trim -> utn

        // ds_id -> line id -> track num -> utn, last tod
        std::map<unsigned int, std::map<unsigned int,
                                        std::map<unsigned int,
                                                 std::pair<unsigned int, boost::posix_time::ptime>>>> tn2utn_;

        std::map<unsigned int, dbContent::ReconstructorTarget> targets_; // utn -> tgt
        //std::vector<unsigned int> removed_utns_;

        unsigned int createNewTarget(const dbContent::targetReport::ReconstructorInfo& tr);

        void removeUTN(unsigned int other_utn);
        void replaceInLookup(unsigned int other_utn, unsigned int utn);
        void addToLookup(unsigned int utn, dbContent::targetReport::ReconstructorInfo& tr);
        void checkACADLookup();

        bool canAssocByACAD(dbContent::targetReport::ReconstructorInfo& tr, bool do_debug);
        int assocByACAD(dbContent::targetReport::ReconstructorInfo& tr, bool do_debug); // -1 if failed, else utn
        boost::optional<unsigned int> utnForACAD(unsigned int acad);

        bool canAssocByACID(dbContent::targetReport::ReconstructorInfo& tr, bool do_debug);
        int assocByACID(dbContent::targetReport::ReconstructorInfo& tr, bool do_debug); // -1 if failed, else utn

        bool canAssocByTrackNumber(dbContent::targetReport::ReconstructorInfo& tr, bool do_debug);
        int assocByTrackNumber(
            dbContent::targetReport::ReconstructorInfo& tr,
            const boost::posix_time::time_duration& track_max_time_diff, bool do_debug);
        // -1 if failed, else utn

        void eraseTrackNumberLookup(dbContent::targetReport::ReconstructorInfo& tr);

        void clear();
    };

    struct TargetReportBatch
    {
        TargetReportBatch(unsigned int ds_id, unsigned int line_id, 
            boost::posix_time::ptime batch_time, std::vector<unsigned long> rec_nums)
        : ds_id_(ds_id), line_id_(line_id), batch_time_(batch_time), rec_nums_(rec_nums)
        {}

        unsigned int ds_id_{0};
        unsigned int line_id_{0};
        boost::posix_time::ptime batch_time_;
        std::vector<unsigned long> rec_nums_;
    };

    typedef std::map<std::string, std::shared_ptr<Buffer>> Buffers;

    ReconstructorBase(const std::string& class_id, 
                      const std::string& instance_id,
                      ReconstructorTask& task, 
                      std::unique_ptr<AccuracyEstimatorBase>&& acc_estimator);
    virtual ~ReconstructorBase();

    const boost::posix_time::ptime& timestampMin() const { return timestamp_min_; }
    const boost::posix_time::ptime& timestampMax() const { return timestamp_max_; }

    bool hasNextTimeSlice();
    std::unique_ptr<ReconstructorBase::DataSlice> getNextTimeSlice();

    int numSlices();

    void processSlice();
    ReconstructorBase::DataSlice& currentSlice();
    const ReconstructorBase::DataSlice& currentSlice() const;

    virtual ReconstructorAssociatorBase& associator()=0;

    virtual dbContent::VariableSet getReadSetFor(const std::string& dbcontent_name) const = 0;

    virtual void reset();

    virtual ReconstructorBaseSettings& settings()=0;
    virtual const ReconstructorBaseSettings& settings() const=0;

    ReferenceCalculatorSettings& referenceCalculatorSettings() { return ref_calc_settings_; }
    const ReferenceCalculatorSettings& referenceCalculatorSettings() const { return ref_calc_settings_; }

    void createMeasurement(reconstruction::Measurement& mm, 
                           const dbContent::targetReport::ReconstructorInfo& ri,
                           const dbContent::ReconstructorTarget* target = nullptr);
    void createMeasurement(reconstruction::Measurement& mm, 
                           unsigned long rec_num,
                           const dbContent::ReconstructorTarget* target = nullptr);
    
    const dbContent::targetReport::ReconstructorInfo* getInfo(unsigned long rec_num) const;

    const dbContent::TargetReportAccessor& accessor(const dbContent::targetReport::ReconstructorInfo& tr) const;

    virtual void updateWidgets() = 0;

    ReconstructorTask& task() const;

    void cancel();
    bool isCancelled() { return cancelled_; };

    bool isInit() { return init_; }

    void saveTargets();

    // our data structures
    std::map<unsigned long, dbContent::targetReport::ReconstructorInfo> target_reports_;
    unsigned int num_new_target_reports_in_slice_{0};
    unsigned int num_new_target_reports_total_{0};
    unsigned int num_unassociated_target_reports_total_{0};

    // all sources, record_num -> base info
    std::multimap<boost::posix_time::ptime, unsigned long> tr_timestamps_;
    // all sources sorted by time, ts -> record_num
    std::map<unsigned int, std::map<unsigned int, std::map<unsigned int, std::vector<unsigned long>>>> tr_ds_;
    // dbcontent id -> ds_id -> line id -> record_num, sorted by ts
    std::multimap<boost::posix_time::ptime, TargetReportBatch> tr_batches_;
    
    TargetsContainer targets_container_;

    std::unique_ptr<AccuracyEstimatorBase> acc_estimator_;

    bool processing() const;

    virtual const std::map<unsigned int, std::map<unsigned int,
                                                  std::pair<unsigned int, unsigned int>>>& assocCounts() const = 0;
    // ds_id -> dbcont id -> (assoc, not assoc cnt)

    virtual void createAdditionalAnnotations() {}

    virtual bool doFurtherSliceProcessing() { return false; }     // called for repeat checking
    virtual bool isLastRunInSlice() { return true; }      // called to check if another repeat run is planned
    virtual unsigned int currentSliceRepeatRun() { return currentSlice().run_count_; }    // current repeat run

    virtual std::string reconstructorInfoString() { return ""; }
    virtual std::pair<boost::posix_time::ptime, boost::posix_time::ptime> timeFrame() const;

    reconstruction::KalmanChainPredictors& chainPredictors();

    boost::optional<unsigned int> utnForACAD(unsigned int acad);

    std::unique_ptr<reconstruction::KalmanChain>& chain(unsigned int utn);

    void informConfigChanged();
    void resetTimeframeSettings();

    bool isVehicleACID(const std::string& acid);
    bool isVehicleACAD(unsigned int value);

    std::unique_ptr<reconstruction::KalmanChain> createConfiguredChain(bool dynamic_insertions) const;
    std::unique_ptr<reconstruction::KalmanEstimator> createConfiguredEstimator(bool extract_wgs84_pos) const;
    std::unique_ptr<reconstruction::KalmanOnlineTracker> createConfiguredOnlineEstimator(bool extract_wgs84_pos) const;

signals:
    void configChanged(); 

protected:
    ReconstructorTask& task_;

    std::map<unsigned int, dbContent::TargetReportAccessor> accessors_;

    std::shared_ptr<dbContent::DBContentAccessor> accessor_;

    //ReconstructorBaseSettings& base_settings_;

    bool cancelled_ {false};

    std::map<unsigned int, std::unique_ptr<reconstruction::KalmanChain>> chains_; // utn -> chain

    unsigned int num_target_reports_ {0};
    unsigned int num_target_reports_associated_ {0};
    unsigned int num_target_reports_unassociated_ {0};

    void registerBaseSettings(ReconstructorBaseSettings& settings);

    void removeOldBufferData(); // remove all data before current_slice_begin_
    virtual void processSlice_impl() = 0;

    virtual void init_impl() {}

    void clearOldTargetReports();
    void createTargetReports();
    void createTargetReportBatches();
    void removeTargetReportsLaterOrEqualThan(const boost::posix_time::ptime& ts); // for slice recalc

    std::map<unsigned int, std::map<unsigned long, unsigned int>> createAssociations();
    std::map<std::string, std::shared_ptr<Buffer>> createAssociationBuffers(
        std::map<unsigned int, std::map<unsigned long, unsigned int>> associations);
    std::map<std::string, std::shared_ptr<Buffer>> createReferenceBuffers();

    void doUnassociatedAnalysis();
    void doOutlierAnalysis();
    void doReconstructionStatistics();

private:
    void init();
    void initIfNeeded();
    void initChainPredictors();

    void resetTimeframe();
    void applyTimeframeLimits();

    double determineProcessNoise(const dbContent::targetReport::ReconstructorInfo& ri,
                                 const dbContent::ReconstructorTarget& target,
                                 const ReferenceCalculatorSettings::ProcessNoise& Q);
    double determineProcessNoiseVariance(const dbContent::targetReport::ReconstructorInfo& ri,
                                         const dbContent::ReconstructorTarget& target,
                                         const ReferenceCalculatorSettings::ProcessNoise& Q);

    ReferenceCalculatorSettings ref_calc_settings_;

    unsigned int slice_cnt_ {0};
    boost::posix_time::ptime current_slice_begin_;
    boost::posix_time::ptime next_slice_begin_;
    boost::posix_time::ptime timestamp_min_, timestamp_max_;
    bool first_slice_ {false};

    boost::posix_time::ptime remove_before_time_;
    boost::posix_time::ptime write_before_time_;

    bool processing_ {false};
    bool init_       {false};

    std::unique_ptr<reconstruction::KalmanChainPredictors> chain_predictors_; // relic, not used now
};
