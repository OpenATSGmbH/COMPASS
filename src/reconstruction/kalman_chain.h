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

#include "reconstruction_defs.h"
#include "kalman_estimator.h"

#include <memory>
#include <map>

namespace reconstruction
{

class KalmanOnlineTracker;
class KalmanEstimator;
class KalmanInterface;

/**
*/
class KalmanChainPredictors
{
public:
    size_t size() const;
    bool isInit() const;

    KalmanEstimator& predictor(size_t idx);

    void init(std::unique_ptr<KalmanInterface>&& interface,
              const KalmanEstimator::Settings& settings,
              unsigned int max_threads);
    void init(kalman::KalmanType ktype,
              const KalmanEstimator::Settings& settings,
              unsigned int max_threads);
private:
    std::vector<std::unique_ptr<KalmanEstimator>> predictors;
};

/**
 * Chain of kalman updates in which new measurements can be inserted and 
 * a reestimation of kalman states can be triggered.
 * - Utilizes a KalmanOnlineTracker which is reinitialized to certain chain updates as needed
 * - The number of states reestimated following the insertion of a new measurement can be specified using multiple criteria
 */
class KalmanChain
{
public:
    struct Settings
    {
        typedef boost::posix_time::time_duration TD;

        enum class Mode
        {
            DynamicInserts = 0,  // allows dynamic inserts and reestimation, but needs to keep track of all states (more memory)
            StaticAdd            // allows adding to the end of the chain, and predictions at the end, only the current state is kept
        };

        enum class PredictionMode
        {
            NearestBefore = 0,   // predict from nearest previous update
            Interpolate,         // predict by interpolating previous and next update
            LastUpdate           // predict from last update in the chain
        };

        Mode           mode            = Mode::DynamicInserts;
        PredictionMode prediction_mode = PredictionMode::NearestBefore;

        TD     reestim_max_duration       = boost::posix_time::seconds(30); // maximum timeframe reestimated after a new mm has been inserted
        int    reestim_max_updates        = 500;                            // maximum updates reestimated after a new mm has been inserted
        double reestim_residual_state_sqr = 100;                            // 10  * 10  - reestimation stop criterion based on state change residual
        double reestim_residual_cov_sqr   = 10000;                          // 100 * 100 - reestimation stop criterion based on cov mat change residual

        TD     prediction_max_tdiff       = boost::posix_time::seconds(10); // maximum difference in time which can be predicted
        double prediction_max_wgs84_diff  = 0.5;

        int    verbosity = 0;
        bool   debug     = false;
    };

    struct Update
    {
        Update() {}
        Update(unsigned long id, 
               const boost::posix_time::ptime& ts, 
               bool check_timestamp,
               const kalman::KalmanUpdateMinimal& update = kalman::KalmanUpdateMinimal()) 
        :   mm_id        (id)
        ,   t            (ts)
        ,   kalman_update(update)
        ,   check_ts     (check_timestamp) {}

        unsigned long               mm_id;
        boost::posix_time::ptime    t;
        kalman::KalmanUpdateMinimal kalman_update;

        bool                        init     = false;
        bool                        check_ts = true;
    };

    typedef std::pair<int, int>                              Interval;
    typedef std::unique_ptr<KalmanOnlineTracker>             TrackerPtr;
    typedef std::unique_ptr<KalmanEstimator>                 EstimatorPtr;
    typedef std::function<const Measurement&(unsigned long)> MeasurementGetFunc;
    typedef std::function<void(Measurement&, unsigned long)> MeasurementAssignFunc;
    typedef std::function<bool(unsigned long)>               MeasurementCheckFunc;

    KalmanChain();
    virtual ~KalmanChain();

    void reset();
    bool hasData() const;

    boost::posix_time::ptime timestampMin() const;
    boost::posix_time::ptime timestampMax() const;

    bool isInit() const;
    void init(std::unique_ptr<KalmanInterface>&& interface);
    void init(kalman::KalmanType ktype);

    void configureEstimator(const KalmanEstimator::Settings& settings);
    void setMeasurementGetFunc(const MeasurementGetFunc& get_func);
    void setMeasurementAssignFunc(const MeasurementAssignFunc& assign_func);
    void setMeasurementCheckFunc(const MeasurementCheckFunc& check_func);

    bool add(unsigned long mm_id,
             const boost::posix_time::ptime& ts,
             bool reestim,
             bool check_ts = true,
             UpdateStats* stats = nullptr);
    bool add(const std::vector<std::pair<unsigned long, boost::posix_time::ptime>>& mms,
             bool reestim,
             bool check_ts = true,
             UpdateStats* stats = nullptr);
    bool insert(unsigned long mm_id,
                const boost::posix_time::ptime& ts,
                bool reestim,
                bool check_ts = true,
                UpdateStats* stats = nullptr);
    bool insert(const std::vector<std::pair<unsigned long, boost::posix_time::ptime>>& mms,
                bool reestim,
                bool check_ts = true,
                UpdateStats* stats = nullptr);
    bool remove(size_t idx,
                bool reestim,
                UpdateStats* stats = nullptr);

    std::pair<int, int> indicesNear(const boost::posix_time::ptime& ts, double dt) const;

    void removeUpdatesBefore(const boost::posix_time::ptime& ts);
    void removeUpdatesLaterOrEqualThan(const boost::posix_time::ptime& ts);

    kalman::KalmanUpdateMinimal lastKalmanUpdate() const;
    const kalman::KalmanUpdateMinimal& getKalmanUpdate(size_t idx) const;
    const Update& getUpdate(size_t idx);

    bool hasUpdateFor(const boost::posix_time::ptime& ts) const;
    bool hasUpdateFor(unsigned int mm_id) const;

    bool canReestimate() const;
    bool needsReestimate() const;
    bool reestimate(UpdateStats* stats = nullptr);

    bool canPredict(const boost::posix_time::ptime& ts) const;
    bool predictMT(Measurement* mm,
                   kalman::GeoProbState* gp_state,
                   kalman::GeoProbState* gp_state_mm,
                   const boost::posix_time::ptime& ts,
                   KalmanChainPredictors& predictors,
                   unsigned int thread_id,
                   PredictionStats* stats = nullptr) const;
    bool predict(Measurement* mm,
                 kalman::GeoProbState* gp_state,
                 kalman::GeoProbState* gp_state_mm,
                 const boost::posix_time::ptime& ts,
                 PredictionStats* stats = nullptr) const;
    bool predictPositionClose(boost::posix_time::ptime ts, double lat, double lon) const;
    bool getChainState(Measurement& mm,
                       const boost::posix_time::ptime& ts,
                       PredictionStats* stats = nullptr) const;
    
    size_t size() const;
    int count() const;

    bool checkMeasurementAvailability() const;

    Settings& settings();

private:
    struct Tracker
    {
        void reset();

        TrackerPtr                     tracker_ptr;
        boost::optional<unsigned long> tracked_mm_id;
    };

    struct Predictor
    {
        void reset();

        EstimatorPtr                   estimator_ptr;
        boost::optional<unsigned long> ref_mm_id = -1;
    };

    bool checkIntegrity() const;

    const Measurement& getMeasurement(unsigned long mm_id) const;

    bool addToTracker(unsigned long mm_id,
                      const boost::posix_time::ptime& ts,
                      bool check_ts,
                      UpdateStats* stats = nullptr);
    void addToEnd(unsigned long mm_id,
                  const boost::posix_time::ptime& ts,
                  bool check_ts);
    void insertAt(int idx, 
                  unsigned long mm_id,
                  const boost::posix_time::ptime& ts,
                  bool check_ts);
    bool reinit(int idx) const;
    bool reestimate(int idx, 
                    KalmanEstimator::StepInfo* info = nullptr);
    bool reestimate(int idx, 
                    double& d_state_sqr, 
                    double& d_cov_sqr, 
                    KalmanEstimator::StepInfo* info = nullptr);
    void removeUpdate(int idx);

    int lastIndex() const;

    Interval interval(const boost::posix_time::ptime& ts) const;
    int insertionIndex(const boost::posix_time::ptime& ts) const;
    int predictionRefIndex(const boost::posix_time::ptime& ts) const;
    Interval predictionRefInterval(const boost::posix_time::ptime& ts) const;

    bool predictInternal(Measurement* mm, 
                         kalman::GeoProbState* gp_state,
                         kalman::GeoProbState* gp_state_mm,
                         const boost::posix_time::ptime& ts,
                         KalmanChainPredictors* predictors,
                         unsigned int thread_id,
                         PredictionStats* stats) const;

    Settings              settings_;

    MeasurementGetFunc    get_func_;
    MeasurementAssignFunc assign_func_;
    MeasurementCheckFunc  check_func_;
    mutable Measurement   mm_tmp_;

    mutable Predictor     predictor_;
    mutable Tracker       tracker_;
    std::vector<Update>   updates_;

    bool                  needs_reestimate_ = false;
    std::vector<int>      fresh_indices_;
};

} // reconstruction
