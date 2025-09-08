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

#include "referencecalculator.h"
#include "spline_interpolator.h"
#include "reconstructorbase.h"
#include "reconstructortask.h"
#include "targetreportdefs.h"

#include "kalman_estimator.h"
#include "kalman_interface.h"
#include "kalman_projection.h"

#include "viewpointgenerator.h"
#include "histograminitializer.h"
#include "scatterseries.h"
#include "grid2d.h"
#include "grid2dlayer.h"

#include "util/timeconv.h"
#include "util/number.h"
#include "stringconv.h"
#include "logger.h"
#include "global.h"

#include "projector.h"

#include <ogr_spatialref.h>

#include "tbbhack.h"

using namespace Utils;

const QColor ReferenceCalculator::ColorMeasurements    = QColor(102, 178, 255);
const QColor ReferenceCalculator::ColorKalman          = QColor(255, 102, 178);
const QColor ReferenceCalculator::ColorKalmanSmoothed  = QColor(255, 178, 102);
const QColor ReferenceCalculator::ColorKalmanResampled = QColor(200, 200, 200);

const float ReferenceCalculator::PointSizeOSG             = 10.0f;
const float ReferenceCalculator::PointSizeMeasurements    = 6.0f;
const float ReferenceCalculator::PointSizeKalman          = 4.0f;
const float ReferenceCalculator::PointSizeKalmanSmoothed  = 2.0f;
const float ReferenceCalculator::PointSizeKalmanResampled = 2.0f;

const float ReferenceCalculator::LineWidthBase = 1.0f;

/**
 */
void ReferenceCalculator::TargetReferences::reset()
{
    measurements.resize(0);
    references.resize(0);
    init_update.reset();
    start_index.reset();

    resetCounts();
}

/**
 */
void ReferenceCalculator::TargetReferences::resetCounts()
{
    num_updates                 = 0;
    num_updates_ccoeff_corr     = 0;
    num_updates_valid           = 0;
    num_updates_raf             = 0;
    num_updates_raf_numeric     = 0;
    num_updates_raf_badstate    = 0;
    num_updates_raf_other       = 0;
    num_updates_failed          = 0;
    num_updates_failed_numeric  = 0;
    num_updates_failed_badstate = 0;
    num_updates_failed_other    = 0;
    num_updates_skipped         = 0;
    num_smooth_steps_failed     = 0;
    num_smoothing_failed        = 0;
    num_interp_steps_failed     = 0;
}

/**
 */
ReferenceCalculator::ReferenceCalculator(ReconstructorBase& reconstructor)
:   reconstructor_(reconstructor)
{
}

/**
 */
ReferenceCalculator::~ReferenceCalculator() = default;

/**
 */
void ReferenceCalculator::prepareForNextSlice()
{
    if (!reconstructor_.currentSlice().first_slice_)
    {
        prepareForCurrentSlice(); // also does resetDataStructs()

        ++slice_idx_;
    }
    else
    {
        slice_idx_ = 0;

        //reset data structs
        resetDataStructs();
    }

    is_last_slice_ = reconstructor_.currentSlice().is_last_slice_;
}

/**
 */
void ReferenceCalculator::prepareForCurrentSlice()
{
    auto ThresRemove = reconstructor_.currentSlice().remove_before_time_;
    auto ThresJoin   = getJoinThreshold();

    //remove targets & previous updates which are no longer needed (either too old or above the join threshold)
    for (auto ref_it = references_.begin(); ref_it != references_.end(); )
    {
        if (!reconstructor_.targets_container_.targets_.count(ref_it->first))
        {
            // deleted target, remove
            ref_it = references_.erase(ref_it);
        }
        else 
        {
            // target still exists => remove updates out of join region
            auto it = std::remove_if(ref_it->second.updates.begin(),
                                     ref_it->second.updates.end(),
                                     [ & ] (const kalman::KalmanUpdate& update) { return update.t <  ThresRemove ||
                                                                                         update.t >= ThresJoin; });
            ref_it->second.updates.erase(it, ref_it->second.updates.end());

            // remove smooth updates out of join region
            auto it_s = std::remove_if(ref_it->second.updates_smooth.begin(),
                                       ref_it->second.updates_smooth.end(),
                                       [ & ] (const kalman::KalmanUpdate& update) { return update.t <  ThresRemove ||
                                                                                           update.t >= ThresJoin; });
            ref_it->second.updates_smooth.erase(it_s, ref_it->second.updates_smooth.end());

            // remove input infos out of join region
            auto& input_infos = ref_it->second.input_infos;
            for (auto first = input_infos.begin(), last = input_infos.end(); first != last;)
            {
                if (first->first.second < ThresRemove || first->first.second >= ThresJoin)
                    first = input_infos.erase(first);
                else
                    ++first;
            }

            // remove target report usage flags out of join region out of join region
            auto it_u = std::remove_if(ref_it->second.measurement_usage.begin(),
                                       ref_it->second.measurement_usage.end(),
                                       [ & ] (const reconstruction::TRUsage& usage) { return usage.t <  ThresRemove ||
                                                                                             usage.t >= ThresJoin; });
            ref_it->second.measurement_usage.erase(it_u, ref_it->second.measurement_usage.end());

            ++ref_it;
        }
    }

    //reset data structs
    resetDataStructs();
}

/**
 */
void ReferenceCalculator::reset()
{
    loginf;

    references_.clear();
    interp_options_.clear();

    slice_idx_ = 0;
}

/**
 */
void ReferenceCalculator::resetDataStructs()
{
    for (auto& ref : references_)
        ref.second.reset();

    updateInterpOptions();
}

/**
 */
void ReferenceCalculator::updateInterpOptions()
{
    if (settings_.resample_systracks)
    {
        interp_options_[ 62 ].sample_dt = settings_.resample_systracks_dt;
        interp_options_[ 62 ].max_dt    = settings_.resample_systracks_max_dt;
    }
    else if (interp_options_.count(62))
    {
        interp_options_.erase(62);
    }
}

/**
 */
bool ReferenceCalculator::computeReferences()
{
    loginf;

    resetDataStructs();

    //skip slice? (debug)
    if (settings_.max_slice_index >= 0 && slice_idx_ > settings_.max_slice_index)
        return true;

    generateMeasurements();
    reconstructMeasurements();
    updateReferences();

    return true;
}

/**
 */
void ReferenceCalculator::generateMeasurements()
{
    for (const auto& target : reconstructor_.targets_container_.targets_)
        generateTargetMeasurements(target.second);
}

/**
 */
void ReferenceCalculator::generateTargetMeasurements(const dbContent::ReconstructorTarget& target)
{
    for (const auto& dbcontent_targets : target.tr_ds_timestamps_)
        for (const auto& ds_targets : dbcontent_targets.second)
            for (const auto& line_targets : ds_targets.second)
                generateLineMeasurements(target, 
                                         dbcontent_targets.first, 
                                         ds_targets.first, 
                                         line_targets.first, 
                                         line_targets.second);
}

/**
 */
void ReferenceCalculator::generateLineMeasurements(const dbContent::ReconstructorTarget& target,
                                                   unsigned int dbcontent_id,
                                                   unsigned int sensor_id,
                                                   unsigned int line_id,
                                                   const TargetReports& target_reports)
{
    //compatibility mode: only use systracks and adsb (debug)
    if (settings_.compat_mode && dbcontent_id != 21 && dbcontent_id != 62)
        return;

    std::vector<reconstruction::Measurement> line_measurements;

    traced_assert(reconstructor_.acc_estimator_);

    for (const auto& elem : target_reports)
    {
        const auto& tr_info = reconstructor_.target_reports_.at(elem.second);

        if (tr_info.doNotUsePosition())
            continue;

        reconstruction::Measurement mm;
        reconstructor_.createMeasurement(mm, tr_info, &target);

        // if (tr_info.track_number_.value() == 69 || target.utn_ == 69)
        // {
        //     loginf << "POS (" << mm.lat << "," << mm.lon << ") " << "(" << (mm.vx.has_value() ? mm.vx.value() : 666) << "," << (mm.vy.has_value() ? mm.vy.value() : 666) << ")";
        //     loginf << "ACC (" << mm.x_stddev.value() << "," << mm.y_stddev.value() << "," << mm.xy_cov.value() << ") " << "(" << mm.vx_stddev.value() << "," << mm.vy_stddev.value() << ")";
        // }

        line_measurements.push_back(mm);
    }

    addMeasurements(target.utn_, dbcontent_id, line_measurements);
}

/**
 */
void ReferenceCalculator::addMeasurements(unsigned int utn,
                                          unsigned int dbcontent_id,
                                          Measurements& measurements)
{
    //preprocess
    preprocessMeasurements(dbcontent_id, measurements);

    //add to utn measurements
    auto& utn_ref = references_[ utn ];
    utn_ref.utn = utn;
    utn_ref.annotations.setReconstructor(&reconstructor_);

    if (!measurements.empty()) // TODO UGA
        utn_ref.measurements.insert(utn_ref.measurements.end(), measurements.begin(), measurements.end());

    //final check on available source ids
    for (const auto& mm : utn_ref.measurements)
        traced_assert(mm.source_id.has_value());
}

/**
 */
void ReferenceCalculator::preprocessMeasurements(unsigned int dbcontent_id, 
                                                 Measurements& measurements)
{
    //interpolate if options are set for dbcontent
    if (interp_options_.count(dbcontent_id))
        interpolateMeasurements(measurements, interp_options_.at(dbcontent_id));
}

namespace
{
bool mmSortPred(const reconstruction::Measurement& mm0,
                const reconstruction::Measurement& mm1)
{
    //sort by time first
    if (mm0.t != mm1.t)
        return mm0.t < mm1.t;

    //otherwise sort by source at least
    return mm0.source_id < mm1.source_id;
}
}

/**
 */
void ReferenceCalculator::interpolateMeasurements(Measurements& measurements, 
                                                  const reconstruction::InterpOptions& options) const
{
    //sort measurements by timestamp
    std::sort(measurements.begin(), measurements.end(), mmSortPred);

    reconstruction::SplineInterpolator interp;
    interp.config().check_fishy_segments = true;
    interp.config().interpolate_cart     = false;
    interp.config().covmat_interp_mode   = reconstruction::SplineInterpolator::CovMatInterpMode::Linear;
    interp.config().sample_dt            = options.sample_dt;
    interp.config().max_dt               = options.max_dt;

    //interpolate using desired timestep
    auto mms_interp = interp.interpolate(measurements);
    
    size_t ni = mms_interp.size();
    if (ni < 1)
        return;

    //assign new measurements
    measurements.resize(ni);

    for (size_t i = 0; i < ni; ++i)
        measurements[ i ] = mms_interp[ i ];
}

/**
 */
void ReferenceCalculator::reconstructMeasurements()
{
    if (references_.empty())
        return;

    std::vector<TargetReferences*> refs;

    //collect jobs
    for (auto& ref : references_)
        refs.push_back(&ref.second);

    unsigned int num_targets = refs.size();

    loginf << "reconstructing " << num_targets
           << " target(s) " << (settings_.multithreading ? "multithreaded" : "") << "...";

    //compute references in parallel
    if (settings_.multithreading)
    {
        tbb::parallel_for(uint(0), num_targets, [&](unsigned int tgt_cnt)
        {
            reconstructMeasurements(*refs[ tgt_cnt ]);
        });
    }
    else
    {
        for (unsigned int i = 0; i < num_targets; ++i)
        {
            reconstructMeasurements(*refs[ i ]);
        }
    }

    loginf << "done";
}

/**
 */
boost::posix_time::ptime ReferenceCalculator::getJoinThreshold() const
{
    const auto ThresJoin = reconstructor_.currentSlice().remove_before_time_
                           + (reconstructor_.currentSlice().slice_begin_ - reconstructor_.currentSlice().remove_before_time_) / 2;
    return ThresJoin;
}

/**
 */
ReferenceCalculator::InitRecResult ReferenceCalculator::initReconstruction(TargetReferences& refs)
{
    refs.start_index.reset();
    refs.init_update.reset();

    if (refs.measurements.size() < 1)
        return InitRecResult::NoMeasurements;

    //sort measurements by timestamp
    std::sort(refs.measurements.begin(), refs.measurements.end(), mmSortPred);

    if (reconstructor_.currentSlice().first_slice_)
    {
        refs.start_index = 0;
        return InitRecResult::Success;
    }

    //join old and new updates in the mid of the overlap timeframe
    const auto ThresJoin = getJoinThreshold();

    //get start index of new measurements (first measurement above join threshold)
    for (size_t i = 0; i < refs.measurements.size(); ++i)
    {
        if (refs.measurements[ i ].t >= ThresJoin)
        {
            refs.start_index = i;
            break;
        }
    }

    //no new measurements to add?
    if (!refs.start_index.has_value())
    {
        return InitRecResult::NoStartIndex;
    }

    //get last slice's last update for init
    if (!refs.updates.empty())
    {
        refs.init_update = *refs.updates.rbegin();

        //first measurement of new slice must be after the last slice's last update
        traced_assert(refs.init_update->t <= refs.measurements[ refs.start_index.value() ].t);
    }

    return InitRecResult::Success;
}

/**
 */
void ReferenceCalculator::reconstructSmoothMeasurements(std::vector<kalman::KalmanUpdate>& updates,
                                                        TargetReferences& refs,
                                                        reconstruction::KalmanEstimator& estimator)
{
    bool general_debug = reconstructor_.task().debugSettings().debug_ &&
                         reconstructor_.task().debugSettings().debug_reference_calculation_;
    bool debug_target  = general_debug && reconstructor_.task().debugSettings().debugUTN(refs.utn);

    const auto& debug_rec_nums = reconstructor_.task().debugSettings().debug_rec_nums_;

    const auto& debug_ts_min  = reconstructor_.task().debugSettings().debug_timestamp_min_;
    const auto& debug_ts_max  = reconstructor_.task().debugSettings().debug_timestamp_max_;

    if (settings_.activeVerbosity() > 0) 
    {
        loginf << "    #measurements: " << refs.measurements.size();
        loginf << "    #old updates:  " << refs.updates.size();
        loginf << "    start index:   " << refs.start_index.value();
        loginf << "    init update:   " << (refs.init_update.has_value() ? "yes" : "no");
    }

    traced_assert(refs.start_index.has_value());

    //if (refs.utn == 21)
    //    writeTargetData(refs, "/home/mcphatty/track_utn21.json");

    kalman::KalmanUpdate update;
    size_t offs     = 0;
    size_t n_before = refs.updates.size();
    size_t n_mm     = refs.measurements.size() - refs.start_index.value() + 1;

    refs.updates.reserve(n_before + n_mm);

    std::vector<QPointF> failed_updates;
    std::vector<QPointF> skipped_updates;
    std::vector<size_t>  update_usages;

    if (debug_target && shallAddAnnotationData())
    {
        refs.annotations.addAnnotationData("Input Measurements",
                                           refcalc_annotations::AnnotationStyle(ColorMeasurements, PointSizeMeasurements, LineWidthBase),
                                           refcalc_annotations::AnnotationStyle(ColorMeasurements, PointSizeOSG, LineWidthBase),
                                           refs.measurements);
    }

    auto collectUsage = [ & ] (const reconstruction::Measurement& mm, int usage_flags)
    {
        reconstruction::TRUsage tr_usage;
        tr_usage.rec_num = mm.source_id.value();
        tr_usage.t       = mm.t;
        tr_usage.flags   = usage_flags;
        tr_usage.used    = usage_flags == 0;

        //flag if this update stems from an interpolation and is not the first in the interpolated interval
        if (mm.mm_interp && !mm.mm_interp_first)
            tr_usage.flags |= reconstruction::TRUsage::Flags::Repeated;

        size_t idx = refs.measurement_usage.size();

        refs.measurement_usage.emplace_back(tr_usage);

        return idx;
    };

    auto collectUpdate = [ & ] (const kalman::KalmanUpdate& update, 
                                const reconstruction::Measurement& mm,
                                unsigned int mm_idx,
                                bool debug)
    {
        refs.updates.push_back(update);
        refs.updates.back().Q_var_interp = mm.Q_var_interp;

        //collect usage and separate info where the usage is located for the kalman update
        auto idx = collectUsage(mm, 0);
        update_usages.push_back(idx);

        if (debug_target)
        {
            auto& input_info = refs.input_infos[ mm.uniqueID() ];

            input_info.lat    = mm.lat;
            input_info.lon    = mm.lon;
            input_info.interp = mm.mm_interp;
        }

        if (debug)
        {
            loginf << "Debugging recnum " << mm.source_id.value() 
                   << " @t " << Utils::Time::toString(mm.t) 
                   << " interp " << mm.mm_interp;
            refs.updates.back().debugUpdate();
        }
    };


    auto debugMM = [ & ] (const reconstruction::Measurement& mm)
    {
        bool debug_mm = debug_target && debug_rec_nums.count(mm.source_id.value());

        if (debug_target && !debug_ts_min.is_not_a_date_time() && mm.t >= debug_ts_min
                         && !debug_ts_max.is_not_a_date_time() && mm.t <= debug_ts_max)
            debug_mm = true;

        return debug_mm;
    };

    //estimator.enableDebugging(refs.utn == 137);

    //init kalman (either from last slice's update or from new measurement)
    if (refs.init_update.has_value())
    {
        if (settings_.activeVerbosity() > 0) 
        {
            auto& mm0 = refs.measurements[ refs.start_index.value() ];
            loginf << "    initializing to update t=" << Utils::Time::toString(refs.init_update.value().t) << ", mm0 t=" << Utils::Time::toString(mm0.t);
        }

        //init kalman from last slice's update
        estimator.kalmanInit(refs.init_update.value());

        //continue with first measurement
    }
    else
    {
        auto& mm0      = refs.measurements[ refs.start_index.value() ];
        bool  debug_mm = debugMM(mm0);

        if (settings_.activeVerbosity() > 0) 
        {
            loginf << "    initializing to mm t=" << Utils::Time::toString(mm0.t);
        }

        //reinit kalman with first measurement
        estimator.kalmanInit(update, mm0);
        traced_assert(update.valid);

        collectUpdate(update, mm0, refs.start_index.value(), debug_mm);

        ++offs; //continue with second measurement
    }

    //add new measurements to kalman and collect updates
    for (size_t i = refs.start_index.value() + offs; i < refs.measurements.size(); ++i)
    {
        const auto& mm       = refs.measurements[ i ];
        bool        debug_mm = debugMM(mm);

        if (mm.pos_acc_corrected)
            ++refs.num_updates_ccoeff_corr;

        if (debug_mm)
        {
            loginf << "[ Debugging UTN " << refs.utn << " ID " << mm.source_id.value() << " TS " << Utils::Time::toString(mm.t) << " ]\n\n"
                   << " * Before State:               \n\n" << estimator.asString()                             << "\n\n"
                   << " * Before State as Measurement:\n\n" << estimator.currentStateAsMeasurement().asString() << "\n\n"
                   << " * Measurement:                \n\n" << mm.asString()                                    << "\n\n";
        }

        //do kalman step
        auto res = estimator.kalmanStep(update, mm);

        if (debug_mm)
        {
            const auto& step_info = estimator.stepInfo();

            loginf << "Step Result:        " << (int)step_info.result;
            loginf << "Kalman Error:       " << (int)step_info.kalman_error;
            loginf << "Reinit After Fail:  " << step_info.reinit_after_fail;
            loginf << "Projection Changed: " << step_info.proj_changed << "\n";

            loginf << " * After State:                \n\n" << estimator.asString()                             << "\n\n"
                   << " * After State as Measurement: \n\n" << estimator.currentStateAsMeasurement().asString() << "\n";
        }

        //check result
        if (res == reconstruction::KalmanEstimator::StepResult::Success)
        {
            traced_assert(update.valid);

            const auto& step_info = estimator.stepInfo();

            if (step_info.reinit_after_fail)
            {
                //log reinits after fail separately
                ++refs.num_updates_raf;

                if (step_info.kalman_error == kalman::KalmanError::Numeric)
                    ++refs.num_updates_raf_numeric;
                else if (step_info.kalman_error == kalman::KalmanError::InvalidState)
                    ++refs.num_updates_raf_badstate;
                else
                    ++refs.num_updates_raf_other;
            }

            ++refs.num_updates_valid;
        }
        else if (res == reconstruction::KalmanEstimator::StepResult::FailStepTooSmall)
        {
            traced_assert(!update.valid);

            if (debug_target)
                skipped_updates.push_back(estimator.currentPositionWGS84());
            
            ++refs.num_updates_skipped;
            collectUsage(mm, reconstruction::TRUsage::SkippedTimeStep);
        }
        else if (res == reconstruction::KalmanEstimator::StepResult::FailKalmanError)
        {
            traced_assert(!update.valid);

            if (debug_target)
                failed_updates.push_back(estimator.currentPositionWGS84());
            
            ++refs.num_updates_failed;

            const auto& step_info = estimator.stepInfo();

            if (step_info.kalman_error == kalman::KalmanError::Numeric)
            {
                ++refs.num_updates_failed_numeric;
                collectUsage(mm, reconstruction::TRUsage::SkippedNumeric);
            }
            else if (step_info.kalman_error == kalman::KalmanError::InvalidState)
            {
                ++refs.num_updates_failed_badstate;
                collectUsage(mm, reconstruction::TRUsage::SkippedInvalid);
            }
            else
            {
                ++refs.num_updates_failed_other;
                collectUsage(mm, reconstruction::TRUsage::SkippedUnknown);
            }
        }
        else
        {
            traced_assert(!update.valid);

            ++refs.num_updates_failed;
            ++refs.num_updates_failed_other;
            collectUsage(mm, reconstruction::TRUsage::SkippedUnknown);
        }

        //!only add update if valid!
        if (update.valid) 
            collectUpdate(update, mm, i, debug_mm);

        ++refs.num_updates;
    }

    if (settings_.activeVerbosity() > 0)
    {
        loginf << "    #new updates: " << refs.updates.size() - n_before;
        loginf << "    #updates: " << refs.updates.size();
    }

    if (debug_target && shallAddAnnotationData())
    {
        refs.annotations.addAnnotationData(estimator, 
                                           "Kalman", 
                                           refcalc_annotations::AnnotationStyle(ColorKalman, PointSizeKalman, LineWidthBase),
                                           refcalc_annotations::AnnotationStyle(ColorKalman, PointSizeOSG, LineWidthBase),
                                           refs.updates,
                                           true,
                                           &refs.input_infos,
                                           &failed_updates,
                                           &skipped_updates, 
                                           nullptr,
                                           debug_target);
    }

    //start with joined kalman updates
    updates = refs.updates;
    
    //run rts smoothing?
    std::map<kalman::UniqueUpdateID, kalman::RTSDebugInfo> rts_debug_info_map;

    if (settings_.smooth_rts)
    {
        //jointly smooth old + new kalman updates RTS
        std::vector<kalman::RTSDebugInfo> rts_debug_infos;
        bool ok = estimator.smoothUpdates(updates, 
                                          kalman::SmoothFailStrategy::SetInvalid,
                                          debug_target ? &rts_debug_infos : nullptr);
        //assert(ok);
        traced_assert(updates.size() == refs.updates.size());

        if (!ok)
        {
            //revert to unsmoothed updates
            updates = refs.updates;
            ++refs.num_smoothing_failed;
        }

        //collect debug infos
        for (const auto& di : rts_debug_infos)
        {
            const auto& u = updates.at(di.update_idx);
            if (!u.valid)
                continue;

            rts_debug_info_map[ u.uniqueID() ] = di;
        }

        //combine old rts updates with new rts updates
        refs.updates_smooth.reserve(n_before + n_mm);

        size_t n_updates = updates.size();
        for (size_t i = n_before; i < n_updates; ++i)
        {
            if (updates[ i ].valid)
            {
                refs.updates_smooth.push_back(updates[ i ]);
            }
            else
            {
                ++refs.num_smooth_steps_failed;

                //flag usages of failed smoothing updates
                size_t new_update_idx = i - n_before;
                traced_assert(new_update_idx < update_usages.size());

                size_t usage_idx = update_usages[ new_update_idx ];
                traced_assert(usage_idx < refs.measurement_usage.size());

                refs.measurement_usage[ usage_idx ].flags |= reconstruction::TRUsage::SkippedSmoothing;
            }
        }

        updates = refs.updates_smooth;

        if (settings_.activeVerbosity() > 0)
        {
            loginf << "    #updates (smoothed): " << updates.size();
        }
    }

    if (debug_target && shallAddAnnotationData())
    {
        loginf << "Collected " << rts_debug_info_map.size() << " debug target(s)";

        refs.annotations.addAnnotationData(estimator, 
                                           "Kalman (RTS)", 
                                           refcalc_annotations::AnnotationStyle(ColorKalmanSmoothed, PointSizeKalmanSmoothed, LineWidthBase),
                                           refcalc_annotations::AnnotationStyle(ColorKalmanSmoothed, PointSizeOSG, LineWidthBase),
                                           updates,
                                           false,
                                           &refs.input_infos,
                                           nullptr,
                                           nullptr,
                                           &rts_debug_info_map,
                                           debug_target);
    }
}

/**
 */
void ReferenceCalculator::obtainRemainingUpdates(std::vector<kalman::KalmanUpdate>& updates,
                                                 TargetReferences& refs,
                                                 reconstruction::KalmanEstimator& estimator)
{
    if (refs.updates.empty())
        return;

    bool general_debug = reconstructor_.task().debugSettings().debug_ &&
                         reconstructor_.task().debugSettings().debug_reference_calculation_;
    bool debug_target  = general_debug && reconstructor_.task().debugSettings().debugUTN(refs.utn);

    //process remaining updates of last slice
    updates = settings_.smooth_rts ? refs.updates_smooth : refs.updates;

    //add missing annotations
    //note: RTS or IMM debug infos for these updates mights already have been added in the last slice
    if (debug_target && shallAddAnnotationData())
    {
        refs.annotations.addAnnotationData(estimator, 
                                            "Kalman", 
                                            refcalc_annotations::AnnotationStyle(ColorKalman, PointSizeKalman, LineWidthBase),
                                            refcalc_annotations::AnnotationStyle(ColorKalman, PointSizeOSG, LineWidthBase),
                                            refs.updates,
                                            true,
                                            &refs.input_infos,
                                            nullptr,
                                            nullptr, 
                                            nullptr);

        refs.annotations.addAnnotationData(estimator, 
                                            "Kalman (RTS)", 
                                            refcalc_annotations::AnnotationStyle(ColorKalmanSmoothed, PointSizeKalmanSmoothed, LineWidthBase),
                                            refcalc_annotations::AnnotationStyle(ColorKalmanSmoothed, PointSizeOSG, LineWidthBase),
                                            refs.updates_smooth,
                                            false,
                                            &refs.input_infos,
                                            nullptr,
                                            nullptr,
                                            nullptr);
    }
}

/**
 */
void ReferenceCalculator::reconstructMeasurements(TargetReferences& refs)
{
    refs.resetCounts();

    bool general_debug = reconstructor_.task().debugSettings().debug_ &&
                         reconstructor_.task().debugSettings().debug_reference_calculation_;
    bool debug_target  = general_debug && reconstructor_.task().debugSettings().debugUTN(refs.utn);

    if(settings_.activeVerbosity() > 0 || debug_target) 
        loginf << "[UTN = " << refs.utn << "]";

    std::vector<kalman::KalmanUpdate> updates;

    //configure and init estimator
    reconstruction::KalmanEstimator estimator;
    estimator.settings() = settings_.kalmanEstimatorSettings();

#if USE_EXPERIMENTAL_SOURCE == false
    traced_assert(settings_.kalman_type_final == kalman::KalmanType::UMKalman2D);
#endif

    estimator.init(settings_.kalman_type_final);

    //try to init
    auto res = initReconstruction(refs);
    if (res == InitRecResult::Success)
    {
        reconstructSmoothMeasurements(updates, refs, estimator);
    }
    else
    {
        //init failed
        if (settings_.activeVerbosity() > 0 || debug_target)
        {
            if (res == InitRecResult::NoMeasurements)
                loginf << "    no measurements to add...";
            else if (res == InitRecResult::NoStartIndex)
                loginf << "    no start index found...";
        }

        if (refs.updates.empty())
            return;

        if (settings_.activeVerbosity() > 0 || debug_target)
            loginf << "    adding remaining updates";
        
        obtainRemainingUpdates(updates, refs, estimator);
    }

    //@TODO: remove check
    for (const auto& u : updates)
        traced_assert(u.source_id.has_value());

    //resample?
    if (settings_.resample_result)
    {
        reconstruction::KalmanEstimator estimator_resample;

        //init resample estimator
        kalman::KalmanType type_resample = 
            settings_.kalman_type_final == kalman::KalmanType::IMMKalman2D ? kalman::KalmanType::AMKalman2D : settings_.kalman_type_final;

        estimator_resample.settings() = settings_.kalmanEstimatorSettings();
        estimator_resample.init(type_resample);

        //interpolate measurements
        size_t num_failed_steps;
        std::vector<kalman::KalmanUpdate> updates_interp;
        estimator_resample.interpUpdates(updates_interp, updates, &num_failed_steps);

        refs.num_interp_steps_failed += num_failed_steps;

        updates = updates_interp;

        if (settings_.activeVerbosity() > 0)
        {
            loginf << "    #updates (resampled): " << updates.size();
        }
    }

    if (debug_target && shallAddAnnotationData())
    {
        refs.annotations.addAnnotationData(estimator, 
                                           "Kalman (Resampled)", 
                                           refcalc_annotations::AnnotationStyle(ColorKalmanResampled, PointSizeKalmanResampled, LineWidthBase),
                                           refcalc_annotations::AnnotationStyle(ColorKalmanResampled, PointSizeOSG, LineWidthBase),
                                           updates,
                                           false,
                                           nullptr,
                                           nullptr,
                                           nullptr, 
                                           nullptr);
    }

    //generate references
    estimator.storeUpdates(refs.references, updates);
}

/**
 */
void ReferenceCalculator::updateReferences()
{
    for (auto& ref : references_)
    {
        traced_assert(reconstructor_.targets_container_.targets_.count(ref.first));

        auto& target = reconstructor_.targets_container_.targets_.at(ref.first);

        target.references_.clear();
        target.reference_tr_usages_.clear();

        //store references to target
        for (auto& ref_it : ref.second.references)
            target.references_[ref_it.t] = ref_it;

        ref.second.references.clear();

        //store target report usages to target
        for (auto& tru_it : ref.second.measurement_usage)
                target.reference_tr_usages_.insert({tru_it.t, tru_it});

        dbContent::ReconstructorTarget::globalStats().num_rec_updates                 += ref.second.num_updates;
        dbContent::ReconstructorTarget::globalStats().num_rec_updates_ccoeff_corr     += ref.second.num_updates_ccoeff_corr;

        dbContent::ReconstructorTarget::globalStats().num_rec_updates_valid           += ref.second.num_updates_valid;

        dbContent::ReconstructorTarget::globalStats().num_rec_updates_raf             += ref.second.num_updates_raf;
        dbContent::ReconstructorTarget::globalStats().num_rec_updates_raf_numeric     += ref.second.num_updates_raf_numeric;
        dbContent::ReconstructorTarget::globalStats().num_rec_updates_raf_badstate    += ref.second.num_updates_raf_badstate;
        dbContent::ReconstructorTarget::globalStats().num_rec_updates_raf_other       += ref.second.num_updates_raf_other;

        dbContent::ReconstructorTarget::globalStats().num_rec_updates_failed          += ref.second.num_updates_failed;
        dbContent::ReconstructorTarget::globalStats().num_rec_updates_failed_numeric  += ref.second.num_updates_failed_numeric;
        dbContent::ReconstructorTarget::globalStats().num_rec_updates_failed_badstate += ref.second.num_updates_failed_badstate;
        dbContent::ReconstructorTarget::globalStats().num_rec_updates_failed_other    += ref.second.num_updates_failed_other;
        dbContent::ReconstructorTarget::globalStats().num_rec_updates_skipped         += ref.second.num_updates_skipped;

        dbContent::ReconstructorTarget::globalStats().num_rec_smooth_steps_failed     += ref.second.num_smooth_steps_failed;
        dbContent::ReconstructorTarget::globalStats().num_rec_smooth_target_failed    += ref.second.num_smoothing_failed;

        dbContent::ReconstructorTarget::globalStats().num_rec_interp_failed           += ref.second.num_interp_steps_failed;
    }
}

/**
*/
bool ReferenceCalculator::writeTargetData(TargetReferences& refs,
                                          const std::string& fn)
{
    if (refs.measurements.empty())
        return true;

    FrameProjector p;
    if (!refs.export_data)
    {
        refs.export_data.reset(new ReferenceCalculatorTargetExportData);

        refs.export_data->t0   = refs.measurements[ 0 ].t;
        refs.export_data->lat_ = refs.measurements[ 0 ].lat;
        refs.export_data->lon_ = refs.measurements[ 0 ].lon;
    }
    p.update(refs.export_data->lat_, refs.export_data->lon_);

    for (const auto& mm : refs.measurements)
    {
        if (mm.t <  reconstructor_.currentSlice().slice_begin_ ||
            mm.t >= reconstructor_.currentSlice().next_slice_begin_)
            continue;

        TestTarget::Measurement mm_tt;
        p.project(mm_tt.x, mm_tt.y, mm.lat, mm.lon);

        mm_tt.vx = mm.vx.has_value() ? mm.vx.value() : 0.0;
        mm_tt.vy = mm.vy.has_value() ? mm.vy.value() : 0.0;

        mm_tt.ax = mm.ax.has_value() ? mm.ax.value() : 0.0;
        mm_tt.ay = mm.ay.has_value() ? mm.ay.value() : 0.0;

        mm_tt.x_stddev = mm.x_stddev;
        mm_tt.y_stddev = mm.y_stddev;
        mm_tt.xy_cov = mm.xy_cov;

        mm_tt.vx_stddev = mm.vx_stddev;
        mm_tt.vy_stddev = mm.vy_stddev;

        mm_tt.ax_stddev = mm.ax_stddev;
        mm_tt.ay_stddev = mm.ay_stddev;

        mm_tt.t = Utils::Time::partialSeconds(mm.t - refs.export_data->t0.value());

        refs.export_data->test_target.addMeasurement(mm_tt);
    }

    return refs.export_data->test_target.exportJSON(fn);
}

/**
*/
bool ReferenceCalculator::shallAddAnnotationData() const
{
    //add only if in last iteration
    return reconstructor_.isLastRunInSlice();
}

/**
*/
void ReferenceCalculator::createAnnotations()
{
    loginf << "creating annotations from " << references_.size() << " reference(s)";

    if (references_.empty())
        return;

    std::vector<TargetReferences*>       refs;
    std::vector<ViewPointGenAnnotation*> annotations;

    const auto& task = reconstructor_.task();

    for (auto& ref : references_)
    {
        if (!ref.second.annotations.hasAnnotations())
            continue;

        loginf << "creating annotation for UTN " << ref.second.utn;

        auto vp   = task.getDebugViewpointForUTN(ref.second.utn);
        auto anno = vp->annotations().getOrCreateAnnotation("Final Reconstruction");
        
        refs.push_back(&ref.second);
        annotations.push_back(anno);
    }

    unsigned int num_targets = refs.size();

    loginf << "creating annotations in parallel for " << refs.size() << " target(s)";

    tbb::parallel_for(uint(0), num_targets, [&](unsigned int tgt_cnt)
    {
        const auto& r = *refs[ tgt_cnt ];

        r.annotations.createAnnotations(annotations[ tgt_cnt ]);
    });
}

#if 0

/**
*/
std::vector<std::vector<Measurement>> Reconstructor::splitMeasurements(const std::vector<Measurement>& measurements,
                                                                       double max_dt)
{
    size_t n = measurements.size();
    if (n == 0)
        return {};

    std::vector<std::vector<Measurement>> split_measurements;
    std::vector<Measurement> current;

    current.push_back(measurements[ 0 ]);

    for (size_t i = 1; i < n; ++i)
    {
        const auto& mm0 = measurements[i - 1];
        const auto& mm1 = measurements[i    ];

        double dt = timestep(mm0, mm1);

        if (dt > max_dt)
        {
            if (current.size() > 0)
            {
                split_measurements.push_back(current);
                current.clear();
            }
        }

        current.push_back(mm1);
    }

    if (current.size() > 0)
        split_measurements.push_back(current);

    return split_measurements;
}

#endif
