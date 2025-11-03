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

#include "kalman_chain.h"
#include "kalman_online_tracker.h"
#include "kalman_interface.h"

#include "logger.h"
#include "timeconv.h"


#include <cmath>

namespace reconstruction
{

/**
*/
size_t KalmanChainPredictors::size() const
{
    return predictors.size();
}

/**
*/
bool KalmanChainPredictors::isInit() const
{
    if (size() == 0)
        return false;

    for (const auto& p : predictors)
        if (!p->isInit())
            return false;

    return true;
}

/**
*/
KalmanEstimator& KalmanChainPredictors::predictor(size_t idx)
{
    auto& p = predictors.at(idx);
    traced_assert(p);

    return *p;
}

/**
*/
void KalmanChainPredictors::init(std::unique_ptr<KalmanInterface>&& interface,
                                 const KalmanEstimator::Settings& settings,
                                 unsigned int max_threads)
{
    traced_assert(max_threads > 0);

    predictors.clear();
    predictors.resize(max_threads);

    for (unsigned int i = 0; i < max_threads; ++i)
    {
        predictors[ i ].reset(new KalmanEstimator);
        predictors[ i ]->settings() = settings;

        

        predictors[ i ]->init(std::unique_ptr<KalmanInterface>(interface->clone()));
    }
}

/**
*/
void KalmanChainPredictors::init(kalman::KalmanType ktype,
                                 const KalmanEstimator::Settings& settings,
                                 unsigned int max_threads)
{
    traced_assert(max_threads > 0);

    predictors.clear();
    predictors.resize(max_threads);

    for (unsigned int i = 0; i < max_threads; ++i)
    {
        predictors[ i ].reset(new KalmanEstimator);
        predictors[ i ]->settings() = settings;
        predictors[ i ]->init(ktype);
    }
}

/**
*/
void KalmanChain::Tracker::reset()
{
    traced_assert(tracker_ptr);
    tracker_ptr->reset();

    tracked_mm_id.reset();
}

/**
*/
void KalmanChain::Predictor::reset()
{
    ref_mm_id.reset();
}

namespace
{
    /**
    */
    void addToStats(PredictionStats& stats, 
                    kalman::KalmanError err,
                    size_t num_predictions,
                    size_t num_fixed,
                    size_t num_proj_changed)
    {
        bool ok = (err == kalman::KalmanError::NoError);

        stats.num_predictions  = num_predictions;
        stats.num_failed       = !ok ? num_predictions : 0;
        stats.num_fixed        = num_fixed;
        stats.num_proj_changed = num_proj_changed;

        if (!ok)
        {
            if (err == kalman::KalmanError::Numeric) 
                ++stats.num_failed_numeric;
            else if (err == kalman::KalmanError::InvalidState) 
                ++stats.num_failed_badstate;
            else
                ++stats.num_failed_other;
        }
    }

    /**
    */
    void addToStats(UpdateStats& stats,
                    bool ok,
                    const KalmanEstimator::StepInfo& info)
    {
        stats.set = true;

        ++stats.num_updated;

        if (ok)
        {
            ++stats.num_valid;
        }
        else if (info.result == KalmanEstimator::StepResult::FailStepTooSmall)
        {
            ++stats.num_skipped;
        }
        else if (info.result == KalmanEstimator::StepResult::FailKalmanError)
        {
            ++stats.num_failed;

            if (info.kalman_error == kalman::KalmanError::Numeric) 
                ++stats.num_failed_numeric;
            else if (info.kalman_error == kalman::KalmanError::InvalidState) 
                ++stats.num_failed_badstate;
            else 
                ++stats.num_failed_other;
        }
        else
        {
            ++stats.num_failed;
            ++stats.num_failed_other;
        }

        if (info.proj_changed)
            ++stats.num_proj_changed;
    }
}

/**
*/
KalmanChain::KalmanChain()
{
    predictor_.estimator_ptr.reset(new KalmanEstimator);
    tracker_.tracker_ptr.reset(new KalmanOnlineTracker);

    reset();
}

/**
*/
KalmanChain::~KalmanChain() = default;

/**
*/
bool KalmanChain::checkIntegrity() const
{
    for (size_t i = 1; i < size(); ++i)
        if (updates_[ i ].t < updates_[ i - 1 ].t)
            return false;

    return true;
}

/**
 * Resets the chain.
*/
void KalmanChain::reset()
{
    updates_.clear();
    tracker_.reset();
    predictor_.reset();

    fresh_indices_.clear();

    needs_reestimate_ = false;
}

/**
*/
bool KalmanChain::hasData() const
{
    return (canReestimate() ? size() > 0 : tracker_.tracker_ptr->isTracking());
}

/**
 * Checks if the chain has been initialized.
*/
bool KalmanChain::isInit() const
{
    if (!tracker_.tracker_ptr->isInit())
        return false;
    
    if (!predictor_.estimator_ptr->isInit())
        return false;
    
    return true;
}

/**
 * Initializes the chain by passing a kalman interface.
*/
void KalmanChain::init(std::unique_ptr<KalmanInterface>&& interface)
{
    traced_assert(interface);

    predictor_.estimator_ptr->init(std::unique_ptr<KalmanInterface>(interface->clone()));
    tracker_.tracker_ptr->init(std::move(interface));
}

/**
 * Initializes the chain to the given kalman type.
*/
void KalmanChain::init(kalman::KalmanType ktype)
{
    predictor_.estimator_ptr->init(ktype);
    tracker_.tracker_ptr->init(ktype);
}

/**
 * Returns the current number of chain elements.
*/
size_t KalmanChain::size() const
{
    return updates_.size();
}

/**
*/
int KalmanChain::count() const
{
    return (int)updates_.size();
}

/**
*/
int KalmanChain::lastIndex() const
{
    return count() - 1;
}

/**
*/
KalmanChain::Settings& KalmanChain::settings()
{
    return settings_;
}

/**
*/
void KalmanChain::configureEstimator(const KalmanEstimator::Settings& settings)
{
    KalmanEstimator::Settings settings_override = settings;

    //set all needed chain-specific overrides
    settings_override.extract_wgs84_pos = true; //essential for chain

    predictor_.estimator_ptr->settings()      = settings_override;
    tracker_.tracker_ptr->estimatorSettings() = settings_override;
}

/**
*/
void KalmanChain::setMeasurementGetFunc(const MeasurementGetFunc& get_func)
{
    traced_assert(get_func);
    get_func_ = get_func;
}

/**
*/
void KalmanChain::setMeasurementAssignFunc(const MeasurementAssignFunc& assign_func)
{
    traced_assert(assign_func);
    assign_func_ = assign_func;
}

/**
*/
void KalmanChain::setMeasurementCheckFunc(const MeasurementCheckFunc& check_func)
{
    traced_assert(check_func);
    check_func_ = check_func;
}

/**
*/
const Measurement& KalmanChain::getMeasurement(unsigned long mm_id) const
{
    traced_assert(get_func_ || assign_func_);

    if (get_func_)
        return get_func_(mm_id);

    assign_func_(mm_tmp_, mm_id);
    return mm_tmp_;
}

/**
 * Returns an interval containing the given timestamp.
 * (Note: in case of multiple equal timestamps the datum is located at the upper bound - as in std::multimap)
*/
KalmanChain::Interval KalmanChain::interval(const boost::posix_time::ptime& ts) const
{
    //empty case?
    if (updates_.empty())
        return { -1, -1 };

    //begin of range?
    if (ts < updates_.begin()->t)
        return { -1, 0 };

    //end of range?
    if (ts >= updates_.rbegin()->t)
        return { lastIndex(), -1 };

    //find interval via upper bound
    auto it = std::upper_bound(updates_.begin(), updates_.end(), ts, 
        [ & ] (const boost::posix_time::ptime& t,
               const Update& u) { return t < u.t; } );

    //checked beforehand
    traced_assert(it != updates_.begin());
    
    int idx = it - updates_.begin();

    //        <= ts    > ts = insert index
    return { idx - 1, idx };
}

/**
 * Returns the index at which the given timestamp should be inserted, 
 * or -1 if the index is to be added at the end of the chain.
 */
int KalmanChain::insertionIndex(const boost::posix_time::ptime& ts) const
{
    auto iv = interval(ts);

    if (iv.first < 0 && iv.second < 0)
        return -1;        // empty - add to end of chain
    else if (iv.first < 0 && iv.second == 0)
        return 0;         // add at begin of chain
    else if (iv.first >= 0 && iv.second < 0)
        return -1;        // add to end of chain
    else if (iv.first >= 0 && iv.second >= 0)
        return iv.second; // insert into the chain
    else
        traced_assert(false);    // should-never-happen-case
    
    return -1;
}

/**
 * Returns the reference index of the interval in which the timestamp should be interpolated,
 * or -1 if no interpolation is possible. 
 */
int KalmanChain::predictionRefIndex(const boost::posix_time::ptime& ts) const
{
    auto iv = interval(ts);

    if (iv.first < 0 && iv.second < 0)
        return -1;                       // empty
    else if (iv.first < 0 && iv.second == 0)
        return 0;                        // begin
    else if (iv.first >= 0 && iv.second < 0)
        return iv.first;                 // end
    else if (iv.first >= 0 && iv.second >= 0)
        return iv.first;                 // mid
    else
        traced_assert(false);            // should-never-happen-case
    
    return -1;
}

/**
*/
KalmanChain::Interval KalmanChain::predictionRefInterval(const boost::posix_time::ptime& ts) const
{
    auto iv = interval(ts);

    if (iv.first < 0 && iv.second < 0)
        return Interval(-1, -1);          // empty
    else if (iv.first < 0 && iv.second == 0)
        return Interval(-1, 0);           // begin
    else if (iv.first >= 0 && iv.second < 0)
        return Interval(lastIndex(), -1); // end
    else if (iv.first >= 0 && iv.second >= 0)
        return iv;                        // mid
    else
        traced_assert(false); // should-never-happen-case
    
    return Interval(-1, -1);
}

/**
*/
std::pair<int, int> KalmanChain::indicesNear(const boost::posix_time::ptime& ts, double dt) const
{
    auto iv = interval(ts);

    if (iv.first >= 0 && KalmanInterface::timestep(updates_[iv.first].t, ts) >= dt)
        iv.first = -1;
    if (iv.second >= 0 && KalmanInterface::timestep(ts, updates_[iv.second].t) >= dt)
        iv.second = -1;

    return iv;
}

/**
 * Internally used.
*/
bool KalmanChain::addToTracker(unsigned long mm_id,
                               const boost::posix_time::ptime& ts,
                               const boost::optional<boost::posix_time::ptime>& ts_mm,
                               UpdateStats* stats)
{
    traced_assert(!canReestimate());

    auto t_mm = ts_mm.has_value() ? ts_mm.value() : ts;

    const auto& mm = getMeasurement(mm_id);
    traced_assert(mm.t == t_mm);

    //just track measurement
    bool ok = tracker_.tracker_ptr->track(ts_mm.has_value() ? mm.replaceTimestamp(ts) : mm);

    if (stats)
    {
        const auto& step_info = tracker_.tracker_ptr->stepInfo();

        if (stats)
        {
            ++stats->num_fresh;
            addToStats(*stats, ok, step_info);
        }
    }

    return ok;
}

/**
 * Internally used.
*/
void KalmanChain::addToEnd(unsigned long mm_id,
                           const boost::posix_time::ptime& ts,
                           const boost::optional<boost::posix_time::ptime>& ts_mm)
{
    traced_assert(canReestimate());
    traced_assert(updates_.empty() || ts >= updates_.rbegin()->t);
    
    updates_.emplace_back(mm_id, ts, ts_mm.has_value() ? ts_mm.value() : ts);
    needs_reestimate_ = true;
}

/**
 * Adds a new measurement to the end of the chain.
*/
bool KalmanChain::add(unsigned long mm_id,
                      const boost::posix_time::ptime& ts,
                      const boost::optional<boost::posix_time::ptime>& ts_mm,
                      bool reestim,
                      UpdateStats* stats)
{
    if (stats)
        *stats = {};

    if (canReestimate())
    {
        addToEnd(mm_id, ts, ts_mm);

        if (reestim && !reestimate(stats))
            return false;
    }
    else
    {
        if (!addToTracker(mm_id, ts, ts_mm, stats))
            return false;
    }

    return true;
}

/**
 * Adds new measurements to the end of the chain.
*/
bool KalmanChain::add(const std::vector<MMDefinition>& mms,
                      bool reestim,
                      UpdateStats* stats)
{
    if (stats)
        *stats = {};

    if (mms.empty())
        return true;

    if (canReestimate())
    {
        //presort for safety
        std::vector<MMDefinition> mms_sorted = mms;
        std::sort(mms_sorted.begin(), mms_sorted.end(), [ & ] (const MMDefinition& mm0, const MMDefinition& mm1) { return mm0.second.first < mm1.second.first; });

        //check correct time ordering
        if (!updates_.empty())
            traced_assert(mms_sorted.begin()->second.first >= updates_.rbegin()->t);

        size_t n  = updates_.size();
        size_t ni = mms_sorted.size();

        updates_.resize(n + ni);
        needs_reestimate_ = true;

        for (size_t idx = 0, idx0 = n; idx < ni; ++idx, ++idx0)
        {
            updates_[ idx0 ].mm_id = mms_sorted[ idx ].first;
            updates_[ idx0 ].t     = mms_sorted[ idx ].second.first;
            updates_[ idx0 ].t_mm  = mms_sorted[ idx ].second.second.has_value() ? mms_sorted[ idx ].second.second.value() : mms_sorted[ idx ].second.first;
        }

        if (reestim && !reestimate(stats))
            return false;
    }
    else
    {
        //just track measurements one after another
        bool has_failed_updates = false;
        for (const auto& mm : mms)
            if (!addToTracker(mm.first, mm.second.first, mm.second.second, stats))
                has_failed_updates = true;

        if (has_failed_updates)
            return false;
    }

    return true;
}

/**
 * Inserts a new measurement at the given index.
*/
void KalmanChain::insertAt(int idx, 
                           unsigned long mm_id,
                           const boost::posix_time::ptime& ts, 
                           const boost::optional<boost::posix_time::ptime>& ts_mm)
{
    traced_assert(canReestimate());

    // if (settings_.verbosity >= 2 && idx >= 0)
    // {
    //     boost::posix_time::ptime t0, t1;
    //     if (count() > 0)
    //     {
    //         t0 = idx < 0 ? updates_.rbegin()->measurement.t : updates_.at(idx).t;
    //         t1 = (idx < 0 || idx == lastIndex()) ? boost::posix_time::ptime() : updates_.at(idx + 1).t;
    //     }

    //     loginf << "Inserting t = " << Utils::Time::toString(ts)
    //           << " @idx " << idx << ", last_index = " << lastIndex() << " [" 
    //           << Utils::Time::toString(t0) << "," 
    //           << Utils::Time::toString(t1) << "]";
    // }

    if (idx < 0)
    {
        //just add to end
        addToEnd(mm_id, ts, ts_mm);
    }
    else
    {
        //insert
        traced_assert(idx <= count());
        updates_.insert(updates_.begin() + idx, Update(mm_id, ts, ts_mm.has_value() ? ts_mm.value() : ts));
        needs_reestimate_ = true;
    }

    traced_assert(checkIntegrity());
}

/**
 * Inserts a new measurement into the chain.
*/
bool KalmanChain::insert(unsigned long mm_id,
                         const boost::posix_time::ptime& ts, 
                         const boost::optional<boost::posix_time::ptime>& ts_mm,
                         bool reestim,
                         UpdateStats* stats)
{
    //mode does not support inserts? => add instead
    if (!canReestimate())
        return add(mm_id, ts, ts_mm, reestim, stats);

    int idx = insertionIndex(ts);

    insertAt(idx, mm_id, ts, ts_mm);

    bool ok = true;

    if (reestim)
        ok = reestimate(stats);

    return ok;
}

/**
 * Inserts new measurements into the chain.
*/
bool KalmanChain::insert(const std::vector<MMDefinition>& mms,
                         bool reestim,
                         UpdateStats* stats)
{
    //mode does not support inserts? => add instead
    if (!canReestimate())
        return add(mms, reestim, stats);

    for (const auto& mm : mms)
    {
        int idx = insertionIndex(mm.second.first);
        insertAt(idx, mm.first, mm.second.first, mm.second.second);
    }

    if (reestim)
        return reestimate(stats);

    return true;
}

/**
*/
bool KalmanChain::remove(size_t idx,
                         bool reestim,
                         UpdateStats* stats)
{
    removeUpdate(idx);

    if (idx < updates_.size())
    {
        updates_.at(idx).init = false;
        needs_reestimate_ = true;
    }

    if (reestim)
        return reestimate(stats);

    return true;
}

/**
*/
void KalmanChain::removeUpdatesBefore(const boost::posix_time::ptime& ts)
{
    traced_assert(!needsReestimate());

    if (size() == 0)
        return;

    auto it = std::remove_if(updates_.begin(), updates_.end(), [ & ] (const Update& u) { return u.t_mm < ts; });
    updates_.erase(it, updates_.end());
    updates_.shrink_to_fit();

    //current mm ids might be outdated now => reset
    tracker_.tracked_mm_id.reset();
    predictor_.ref_mm_id.reset();
}

/**
*/
void KalmanChain::removeUpdatesLaterOrEqualThan(const boost::posix_time::ptime& ts)
{
    traced_assert(!needsReestimate());

    if (size() == 0)
        return;

    auto it = std::remove_if(updates_.begin(), updates_.end(), [ & ] (const Update& u) { return u.t_mm >= ts; });
    updates_.erase(it, updates_.end());
    updates_.shrink_to_fit();

    //current mm ids might be outdated now => reset
    tracker_.tracked_mm_id.reset();
    predictor_.ref_mm_id.reset();
}

/**
*/
const kalman::KalmanUpdateMinimal& KalmanChain::getKalmanUpdate(size_t idx) const
{
    traced_assert(canReestimate());
    return updates_.at(idx).kalman_update;
}

/**
*/
const kalman::KalmanUpdateMinimal& KalmanChain::getKalmanUpdateAtEnd() const
{
    traced_assert(canReestimate());
    return updates_.back().kalman_update;
}

/**
*/
kalman::KalmanUpdateMinimal KalmanChain::lastKalmanUpdate() const
{
    traced_assert(hasData());
    return (canReestimate() ? updates_.rbegin()->kalman_update : tracker_.tracker_ptr->currentState().value().minimalInfo());
}

/**
*/
const KalmanChain::Update& KalmanChain::getUpdate(size_t idx)
{
    traced_assert(canReestimate());
    return updates_.at(idx);
}

/**
 */
double KalmanChain::timeDiffSec(const boost::posix_time::ptime& ts) const
{
    if (!canReestimate())
        return tracker_.tracker_ptr->timeDiffSec(ts);

    //no updates no prediction
    if (updates_.empty())
        return std::numeric_limits<double>::max();

    if (settings_.prediction_mode == Settings::PredictionMode::LastUpdate ||
        settings_.prediction_mode == Settings::PredictionMode::NearestBefore)
    {
        //find reference update
        int idx = (settings_.prediction_mode == Settings::PredictionMode::LastUpdate ? lastIndex() : predictionRefIndex(ts));
        traced_assert(idx >= 0);

        auto t0 = updates_[ idx ].t;

        return KalmanInterface::timestep(t0, ts);
    }
    else if (settings_.prediction_mode == Settings::PredictionMode::Interpolate)
    {
        //find reference interval
        auto iv = predictionRefInterval(ts);
        traced_assert(iv.first >= 0 || iv.second >= 0); //!updates not empty => non-empty interval needs to exist!

        double dt0 = std::numeric_limits<double>::max();
        double dt1 = std::numeric_limits<double>::max();

        if (iv.first >= 0)
            dt0 = KalmanInterface::timestep(updates_[ iv.first ].t, ts);
        if (iv.second >= 0)
            dt1 = KalmanInterface::timestep(ts, updates_[ iv.second ].t);

        return std::fabs(dt0) < std::fabs(dt1) ? dt0 : dt1;
    }

    return std::numeric_limits<double>::max();
}

/**
 * Checks if a prediction is possible at the given timestamp.
*/
bool KalmanChain::canPredict(const boost::posix_time::ptime& ts) const
{
    //static mode? => just ask tracker
    if (!canReestimate())
        return tracker_.tracker_ptr->canPredict(ts, settings_.prediction_max_tdiff_sec);

    //no updates no prediction
    if (updates_.empty())
        return false;

    if (settings_.prediction_mode == Settings::PredictionMode::LastUpdate ||
        settings_.prediction_mode == Settings::PredictionMode::NearestBefore)
    {
        //find reference update
        int idx = (settings_.prediction_mode == Settings::PredictionMode::LastUpdate ? lastIndex() : predictionRefIndex(ts));
        traced_assert(idx >= 0);

        //check time diff
        auto t0 = updates_[ idx ].t;

        double dt = KalmanInterface::timestep(t0, ts);
        if (std::fabs(dt) > settings_.prediction_max_tdiff_sec)
            return false;

        return true;
    }
    else if (settings_.prediction_mode == Settings::PredictionMode::Interpolate)
    {
        //find reference interval
        auto iv = predictionRefInterval(ts);
        traced_assert(iv.first >= 0 || iv.second >= 0); //!updates not empty => non-empty interval needs to exist!

        bool first_ok  = false;
        bool second_ok = false;

        if (iv.first >= 0 && std::fabs(KalmanInterface::timestep(updates_[ iv.first ].t, ts)) <= settings_.prediction_max_tdiff_sec)
            first_ok = true;
        if (iv.second >= 0 && std::fabs(KalmanInterface::timestep(ts, updates_[ iv.second ].t)) <= settings_.prediction_max_tdiff_sec)
            second_ok = true;
        
        //not close enough to at least one interval border?
        if (!first_ok && !second_ok)
            return false;

        return true;
    }

    return false;
}

/**
*/
bool KalmanChain::predictMT(Measurement* mm,
                            kalman::GeoProbState* gp_state,
                            kalman::GeoProbState* gp_state_mm,
                            const boost::posix_time::ptime& ts,
                            KalmanChainPredictors& predictors,
                            unsigned int thread_id,
                            PredictionStats* stats) const
{
    if (thread_id >= predictors.size())
        logerr << "thread_id " << thread_id << " >= predictors.size() " << predictors.size();

    traced_assert(thread_id < predictors.size());

    return predictInternal(mm, gp_state, gp_state_mm, ts, &predictors, (int)thread_id, stats);
}

/**
*/
bool KalmanChain::predict(Measurement* mm,
                          kalman::GeoProbState* gp_state,
                          kalman::GeoProbState* gp_state_mm,
                          const boost::posix_time::ptime& ts,
                          PredictionStats* stats) const
{
    return predictInternal(mm, gp_state, gp_state_mm, ts, nullptr, 0, stats);
}

/**
 * Predicts the given timestamp from the nearest existing update in the chain.
*/
bool KalmanChain::predictInternal(Measurement* mm,
                                  kalman::GeoProbState* gp_state,
                                  kalman::GeoProbState* gp_state_mm,
                                  const boost::posix_time::ptime& ts,
                                  KalmanChainPredictors* predictors,
                                  unsigned int thread_id,
                                  PredictionStats* stats) const
{
    //static mode? => just ask tracker
    if (!canReestimate())
    {
        traced_assert(!predictors);

        bool fixed;
        auto err = tracker_.tracker_ptr->predict(mm, gp_state, gp_state_mm, ts, &fixed);

        bool ok = (err == kalman::KalmanError::NoError);

        if (stats)
            addToStats(*stats, err, 1, fixed ? 1 : 0, 0);
        
        return ok;
    }

    traced_assert(!predictors || predictors->isInit()); //!predictors must be init!
    traced_assert(!predictors || thread_id < predictors->size()); //!thread id must be in range!
    traced_assert(!needsReestimate()); //!no predictions if chain is out of date!
    traced_assert(!updates_.empty());  //!no prediction from empty chains!

    //predicts from a single index
    auto predictFromIndex = [ & ] (int idx)
    {
        traced_assert(idx >= 0);

        const auto& update = updates_[ idx ];

        traced_assert(update.mm_id >= 0);
        traced_assert(update.init);

        auto& p = predictors ? predictors->predictor(thread_id) : *predictor_.estimator_ptr;
        
        bool ref_changed = true;
        
        if (!predictors)
            ref_changed = !predictor_.ref_mm_id.has_value() || update.mm_id != predictor_.ref_mm_id.value();

        //predict
        bool fixed;
        bool proj_changed = false;
        auto err = ref_changed ? p.kalmanPrediction(mm, gp_state, gp_state_mm, update.kalman_update, ts, &fixed, &proj_changed) :
                                 p.kalmanPrediction(mm, gp_state, gp_state_mm, ts, &fixed);

        bool ok = (err == kalman::KalmanError::NoError);

        if (stats)
            addToStats(*stats, err, 1, fixed ? 1 : 0, proj_changed ? 1 : 0);

        //remember current mm id?
        if (!predictors)
            predictor_.ref_mm_id = update.mm_id;

        return ok;
    };

    //predicts from an interval
    auto predictFromInterval =  [ & ] (int idx0, int idx1)
    {
        traced_assert(idx0 >= 0 && idx1 >= 0);

        const auto& update0 = updates_[ idx0 ];
        const auto& update1 = updates_[ idx1 ];

        traced_assert(update0.mm_id >= 0);
        traced_assert(update1.mm_id >= 0);
        traced_assert(update0.init);
        traced_assert(update1.init);

        auto& p = predictors ? predictors->predictor(thread_id) : *predictor_.estimator_ptr;

        //@TODO: interpolate
        size_t num_fixed;
        size_t num_proj_changed;
        auto err = p.kalmanPrediction(mm, gp_state, gp_state_mm, update0.kalman_update, update1.kalman_update, ts, &num_fixed, &num_proj_changed);

        bool ok = (err == kalman::KalmanError::NoError);

        if (stats)
            addToStats(*stats, err, 2, num_fixed, num_proj_changed);

        if (!predictors)
            predictor_.ref_mm_id.reset();

        return ok;
    };

    if (settings_.prediction_mode == Settings::PredictionMode::LastUpdate ||
        settings_.prediction_mode == Settings::PredictionMode::NearestBefore)
    {
        //find reference update
        int idx =  (settings_.prediction_mode == Settings::PredictionMode::LastUpdate ? lastIndex() : predictionRefIndex(ts));

        //predict
        return predictFromIndex(idx);
    }
    else if (settings_.prediction_mode == Settings::PredictionMode::Interpolate)
    {
        //find reference interval
        auto iv = predictionRefInterval(ts);
        traced_assert(iv.first >= 0 || iv.second >= 0); //!updates not empty => non-empty interval needs to exist!

        //check distance to interval borders
        bool first_ok  = false;
        bool second_ok = false;

        if (iv.first >= 0 && std::fabs(KalmanInterface::timestep(updates_[ iv.first ].t, ts)) <= settings_.prediction_max_tdiff_sec)
            first_ok = true;
        if (iv.second >= 0 && std::fabs(KalmanInterface::timestep(ts, updates_[ iv.second ].t)) <= settings_.prediction_max_tdiff_sec)
            second_ok = true;

        //!assured beforehand by canPredict()!
        traced_assert(first_ok || second_ok);

        //predict
        if (first_ok && second_ok)
            return predictFromInterval(iv.first, iv.second);
        else if (first_ok)
            return predictFromIndex(iv.first);
        else if (second_ok)
            return predictFromIndex(iv.second);
    }

    return false;
}

/**
*/
boost::posix_time::ptime KalmanChain::timestampMin() const
{
    if (updates_.empty())
        return boost::posix_time::ptime();

    return updates_.begin()->t;
}

/**
*/
boost::posix_time::ptime KalmanChain::timestampMax() const
{
    if (updates_.empty())
        return boost::posix_time::ptime();

    return updates_.rbegin()->t;
}

/**
*/
bool KalmanChain::hasUpdateFor(const boost::posix_time::ptime& ts) const
{
    if (!canReestimate())
        return (tracker_.tracker_ptr->currentTime() == ts);

    auto it = std::lower_bound(updates_.begin(), updates_.end(), ts, 
        [ & ] (const Update& u, const boost::posix_time::ptime& t) { return u.t < t; });

    if (it == updates_.end())
        return false;

    if (it->t != ts)
        return false;

    return true;
}

/**
*/
bool KalmanChain::hasUpdateFor(unsigned int mm_id) const
{
    auto it = std::find_if(updates_.begin(), updates_.end(), [ & ] (const Update& u) { return u.mm_id == mm_id; });
    return (it != updates_.end());
}

/**
 * Gets the chain state at the given timestamp as a measurement.
*/
bool KalmanChain::getChainState(Measurement& mm,
                                const boost::posix_time::ptime& ts,
                                PredictionStats* stats) const
{
    //static mode? => just ask tracker
    if (!canReestimate())
    {
        //memoryless => check against currently tracked timestamp
        if (tracker_.tracker_ptr->currentTime() != ts)
            return false;

        const auto& update = tracker_.tracker_ptr->currentState();
        if (!update.has_value())
            return false;

        tracker_.tracker_ptr->estimator().storeUpdate(mm, update.value());
        
        return true;
    }

    //get lower bound
    auto it = std::lower_bound(updates_.begin(), updates_.end(), ts, 
        [ & ] (const Update& u, const boost::posix_time::ptime& t) { return u.t < t; });

    //not found?
    if (it == updates_.end())
        return false;

    //did not find update for timestamp?
    if (it->t != ts)
        return false;

    //retrieve update and store to mm
    size_t idx = it - updates_.begin();

    const auto& update = updates_[ idx ];

    tracker_.tracker_ptr->estimator().storeUpdate(mm, update.kalman_update);

    return true;
}

/**
*/
bool KalmanChain::predictPositionClose(boost::posix_time::ptime ts, double lat, double lon) const
{
    int idx =  predictionRefIndex(ts);

    if (idx == -1)
        return false;

    const auto& update = updates_[ idx ];

    traced_assert(update.kalman_update.has_wgs84_pos);

    return std::sqrt(std::pow(update.kalman_update.lat-lat, 2)+std::pow(update.kalman_update.lon-lon, 2))
           < settings_.prediction_max_wgs84_diff;
}

/**
 * Reinits the tracker to the given stored update.
*/
bool KalmanChain::reinit(int idx) const
{
    traced_assert(canReestimate());
    traced_assert(idx >= 0 && idx < count());
    traced_assert(updates_[ idx ].init); //!no freshly added measurements!
    traced_assert(updates_[ idx ].mm_id >= 0);

    const auto& update = updates_[ idx ];

    //update is the currently tracked update => nothing to do
    if (!tracker_.tracked_mm_id.has_value() || 
        update.mm_id != tracker_.tracked_mm_id.value() ||
        update.t != update.t_mm)
    {
        //if (settings_.verbosity > 0)
        //    loginf << "Reinit at idx=" << idx << " t=" << Utils::Time::toString(updates_[ idx ].kalman_update.t);

        //reinit tracker
        tracker_.tracker_ptr->reset();
        if (!tracker_.tracker_ptr->track(update.kalman_update))
            return false; //should not happen

        tracker_.tracked_mm_id = update.mm_id;

        traced_assert(tracker_.tracker_ptr->currentTime() == update.t);
    }

    traced_assert(tracker_.tracker_ptr->currentTime() == update.t);

    return true;
}

/**
 * Removes the update.
*/
void KalmanChain::removeUpdate(int idx)
{
    const auto& update_tbr = updates_.at(idx);

    //reset now outdated measurement id
    if (tracker_.tracked_mm_id.has_value() && update_tbr.mm_id == tracker_.tracked_mm_id.value())
        tracker_.tracked_mm_id.reset();

    if (predictor_.ref_mm_id.has_value() && update_tbr.mm_id == predictor_.ref_mm_id.value())
        predictor_.ref_mm_id.reset();

    //erase update
    updates_.erase(updates_.begin() + idx);
}

/**
*/
bool KalmanChain::canReestimate() const
{
    return (settings_.mode == Settings::Mode::DynamicInserts);
}

/**
 * Checks if new measurements have been added and a reestimation is needed.
*/
bool KalmanChain::needsReestimate() const
{
    return needs_reestimate_;
}

/**
 * Reestimates the kalman state of a measurement based on the current tracker state.
*/
bool KalmanChain::reestimate(int idx, 
                             KalmanEstimator::StepInfo* info)
{
    traced_assert(canReestimate());
    traced_assert(idx >= 0 && idx < count());
    traced_assert(updates_[ idx ].mm_id >= 0);

    auto&       update = updates_[ idx ];
    const auto& mm     = getMeasurement(update.mm_id);

    traced_assert(update.t_mm == mm.t);

    bool chain_input_mm_check = tracker_.tracker_ptr->estimator().checkPrediction(mm);
    if (!chain_input_mm_check)
    {
        logerr << "invalid measurement retrieved\n\n"
               << mm.asString() << "\n";
        traced_assert(chain_input_mm_check);
    }

    bool ok = tracker_.tracker_ptr->track(mm.t != update.t ? mm.replaceTimestamp(update.t) : mm);

    if (info) *info = tracker_.tracker_ptr->stepInfo();

    if (!ok)
        return false;

    tracker_.tracker_ptr->currentState().value().minimalInfo(update.kalman_update);
    update.init = true;

    tracker_.tracked_mm_id = update.mm_id;

    traced_assert(tracker_.tracker_ptr->currentTime() == update.t);

    return true;
}

/**
 * Reestimates the kalman state of a measurement based on the current tracker state.
 * Additionally returns a residual as norm of difference in covariance matrices before and after the reestimate.
*/
bool KalmanChain::reestimate(int idx, 
                             double& d_state_sqr, 
                             double& d_cov_sqr, 
                             KalmanEstimator::StepInfo* info)
{
    traced_assert(canReestimate());
    traced_assert(idx >= 0 && idx < count());
    traced_assert(updates_[ idx ].mm_id >= 0);

    //@TODO: triggered when adding system tracks in different lines!?
    traced_assert(updates_[ idx ].init); //!no freshly added measurements!

    auto x0 = updates_[ idx ].kalman_update.x;
    auto P0 = updates_[ idx ].kalman_update.P;

    if (!reestimate(idx, info))
        return false;

    const auto& x1 = updates_[ idx ].kalman_update.x;
    const auto& P1 = updates_[ idx ].kalman_update.P;

    d_state_sqr = (x0 - x1).squaredNorm();
    d_cov_sqr   = (P0.diagonal() - P1.diagonal()).squaredNorm(); // = squared frobenius norm

    return true;
}

/**
 * Reestimates the chain after new measurements have been added, based on various criteria.
*/
bool KalmanChain::reestimate(UpdateStats* stats)
{
    if (stats)
    {
        stats->resetChainInternals();
        stats->set = true;
    }

    if (!needsReestimate() || !canReestimate())
        return true;

    int n       = count();
    int n_fresh = 0;

    fresh_indices_.clear();
    for (int i = 0; i < n; ++i)
    {
        if (!updates_[ i ].init)
        {
            fresh_indices_.push_back(i);
            ++n_fresh;
        }
    }

    if (n_fresh > 1)
        std::sort(fresh_indices_.begin(), fresh_indices_.end());

    if (settings_.debug)
    {
        loginf << "n_fresh = " << n_fresh;

        std::string str;
        for (int idx : fresh_indices_)
            str += std::to_string(idx) + " ";
        str += "\n";

        loginf << str;
    }

    bool is_last = (n_fresh == 1 && fresh_indices_[ 0 ] == lastIndex());

    std::vector<int> tbr;

    int last_valid_idx = -1;

    auto reestimateRange = [ & ] (int idx_start, int idx_end)
    {
        if (stats)
            ++stats->num_fresh;

        if (idx_end < 0)
            idx_end = n;

        int  idx_cutoff = std::min(idx_end, idx_start + 1 + settings_.reestim_max_updates); // cutoff range based on settings
        auto tstart     = updates_[ idx_start ].t;

        if (!is_last && settings_.verbosity >= 2)
            loginf << "reestimating range [" << idx_start << "," << idx_end << "): "
                   << "idx cutoff = " << idx_cutoff; 

        if (settings_.debug)
            loginf << "idx_start = " << idx_start << ", idx_end = " << idx_end << ", idx_cutoff = " << idx_cutoff;

        //reinit tracker
        if (idx_start == 0)
        {
            //first uninit index is start of chain => start from scratch
            tracker_.reset();

            if (settings_.debug)
                loginf << "idx = 0 => ressetting tracker";
        }
        else
        {
            int reinit_idx = idx_start - 1;
            if (!tbr.empty() && reinit_idx == tbr.back())
                reinit_idx = last_valid_idx;

            if (settings_.debug)
                loginf << "reinit_idx = " << reinit_idx;

            if (reinit_idx < 0)
            {
                tbr.push_back(idx_start);
                return 0;
            }

            //reinit to last valid update
            bool ok = reinit(reinit_idx);
            traced_assert(ok); // !reinit should always succeed! 
        }

        //iterate over indices, reestimate, and stop if one of the criteria hits
        int reestimations = 0;
        double d_state_sqr, d_cov_sqr;
        for (int idx = idx_start; idx < idx_cutoff; ++idx)
        {
            auto& u = updates_[ idx ];

            //max reestimation duration hit?
            traced_assert(u.t >= tstart);
            if (KalmanInterface::timestep(tstart, u.t) > settings_.reestim_max_duration_sec)
                break;

            //we check the norm starting from the extended range
            bool check_norm = (idx > idx_start);

            //reestim mm
            KalmanEstimator::StepInfo info;
            bool ok = check_norm ? reestimate(idx, d_state_sqr, d_cov_sqr, &info) : reestimate(idx, &info);

            if (stats)
                addToStats(*stats, ok, info);

            if (!ok)
            {
                //reestimate (=kalman step) failed => collect update for removal
                tbr.push_back(idx);
                continue;
            }

            last_valid_idx = idx;

            ++reestimations;

            if (settings_.verbosity >= 2 && check_norm)
               loginf << "start" 
                      << " idx_start = " << idx_start << " idx = " << idx
                      << " ds = " << std::sqrt(d_state_sqr) << " / " << std::sqrt(settings_.reestim_residual_state_sqr)
                      << " dc = " << std::sqrt(d_cov_sqr  ) << " / " << std::sqrt(settings_.reestim_residual_cov_sqr  );

            //check residuals, small changes? => stop reestimation 
            if (check_norm && 
                d_state_sqr <= settings_.reestim_residual_state_sqr && 
                d_cov_sqr   <= settings_.reestim_residual_cov_sqr)
                break;
        }

        return reestimations;
    };

    int reestimations = 0;
    
    //reestimate from each fresh index up to the next one at max
    for (int i = 0; i < n_fresh; ++i)
    {
        int idx      = fresh_indices_[ i ];
        int idx_next = i < n_fresh - 1 ? fresh_indices_[ i + 1 ] : n;

        int ret = reestimateRange(idx, idx_next);
        traced_assert(ret >= 0);

        reestimations += ret;
    }

    //remove any failed (=skipped) updates
    for (auto itr = tbr.rbegin(); itr != tbr.rend(); ++itr)
    {
        if (settings_.verbosity >= 2)
            logwrn << "removing update " << *itr;

        removeUpdate(*itr);
    }

    if (settings_.verbosity >= 2 && !is_last)
        loginf << "refreshed " << reestimations << " measurement(s)";

    //had to remove error-step updates?
    bool ok = tbr.empty();

    needs_reestimate_ = false;

    // n_fresh = 0;
    // for (int i = 0; i < n; ++i)
    //     if (!updates_[ i ].init)
    //         ++n_fresh;
    // loginf << "nfresh = " << n_fresh;

    return ok;
}

/**
*/
bool KalmanChain::checkMeasurementAvailability() const
{
    traced_assert(check_func_);

    for (const auto& u : updates_)
    {
        if (!check_func_(u.mm_id))
        {
            logwrn << "start" << u.mm_id << " not available @t=" << Utils::Time::toString(u.t);
            return false;
        }
    }

    return true;
}

} // reconstruction
