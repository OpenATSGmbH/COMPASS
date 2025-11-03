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

/**
*/
struct ReferenceCalculatorSettings
{
    struct ProcessNoise
    {
        ProcessNoise() = default;
        ProcessNoise(double stddev_static,
                     double stddev_ground,
                     double stddev_air,
                     double stddev_unknown)
        :   Q_std_static (stddev_static)
        ,   Q_std_ground (stddev_ground)
        ,   Q_std_air    (stddev_air)
        ,   Q_std_unknown(stddev_unknown) {}

        double Q_std_static;
        double Q_std_ground;
        double Q_std_air;
        double Q_std_unknown;
    };

    /**
    */
    reconstruction::KalmanEstimator::Settings kalmanEstimatorSettings() const
    {
        reconstruction::KalmanEstimator::Settings settings;

        //settings.min_chain_size    = min_chain_size;
        settings.min_dt            = min_dt;
        settings.max_dt            = max_dt;
        settings.max_distance_cart = max_distance;

        settings.Q_var       = Q_std.Q_std_static * Q_std.Q_std_static;
        settings.R_var_undef = R_std_undef * R_std_undef;

        settings.min_pred_dt = min_dt;
        
        settings.resample_dt          = resample_dt;
        settings.resample_Q_var       = resample_Q_std.Q_std_static * resample_Q_std.Q_std_static;
        settings.resample_interp_mode = resample_blend_mode;

        settings.smoothing_scale = smooth_scale;

        settings.max_proj_distance_cart = max_proj_distance_cart;

        settings.verbosity = activeVerbosity() >= 2 ? activeVerbosity() - 1 : 0;

        settings.step_fail_strategy = allow_invalid_updates ? reconstruction::KalmanEstimator::Settings::StepFailStrategy::Reinit :
                                                              reconstruction::KalmanEstimator::Settings::StepFailStrategy::Assert;

        settings.fix_predictions        = fix_predictions;
        settings.fix_predictions_interp = fix_predictions_interp;

        settings.imm_prob_transition_likely   = imm_prob_transition_likely;
        settings.imm_prob_transition_unlikely = imm_prob_transition_unlikely;

        return settings;
    }

    /**
    */
    reconstruction::KalmanEstimator::Settings chainEstimatorSettings() const
    {
        reconstruction::KalmanEstimator::Settings settings = kalmanEstimatorSettings();

        //override settings for chain if needed

        return settings;
    }

    int activeVerbosity() const
    {
        return (multithreading ? 0 : verbosity);
    }

    int verbosity = 0;

    kalman::KalmanType kalman_type_assoc = kalman::KalmanType::UMKalman2D;
    kalman::KalmanType kalman_type_final = kalman::KalmanType::IMMKalman2D;

    //default noise
    double       R_std_undef       = reconstruction::KalmanEstimator::HighStdDev;
    ProcessNoise Q_std             = ProcessNoise(7.0, 5.0, 10.0, 7.0); // (static, ground, air, unknown)
    double       Q_altitude_min_ft = 0.0;
    double       Q_altitude_max_ft = 30000.0;

    //imm related
    double imm_prob_transition_likely   = reconstruction::KalmanEstimator::Settings().imm_prob_transition_likely; 
    double imm_prob_transition_unlikely = reconstruction::KalmanEstimator::Settings().imm_prob_transition_unlikely;

    //reinit related
    //int    min_chain_size = 2;
    double min_dt         = 0.001;
    double max_dt         = 11.0;
    double max_distance   = 50000.0;

    bool   smooth_rts   = true;
    double smooth_scale = 1.0;

    //stopped adsb tracking related
    bool track_stopped_adsb = true;

    //result resampling related
    bool                            resample_result     = true;
    ProcessNoise                    resample_Q_std      = ProcessNoise(3.0, 2.0, 5.0, 3.0); // (static, ground, air, unknown)
    double                          resample_dt         = 2.0;
    reconstruction::StateInterpMode resample_blend_mode = reconstruction::StateInterpMode::BlendVar;

    //dynamic projection change
    double max_proj_distance_cart  = 20000.0;

    //systrack resampling related
    bool   resample_systracks        = true; // resample system tracks using spline interpolation
    double resample_systracks_dt     = 1.0;  // resample interval in seconds
    double resample_systracks_max_dt = 30.0; // maximum timestep to interpolate in seconds

    bool multithreading        = true;
    bool allow_invalid_updates = true;

    bool fix_predictions        = true;
    bool fix_predictions_interp = false;

    bool dynamic_process_noise = true;

    //final reference filter related
    bool   filter_references_max_stddev_ {false};
    double filter_references_max_stddev_m_ {150.0};

    //debug options
    bool compat_mode     = false;
    int  max_slice_index = -1;
};
