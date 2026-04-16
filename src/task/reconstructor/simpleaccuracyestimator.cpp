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

#include "simpleaccuracyestimator.h"
#include "radarbiascorrection.h"
#include "reconstructorbase.h"
#include "reconstructortask.h"
#include "reconstructortarget.h"
#include "targetreportaccessor.h"
#include "compass.h"
#include "db_context_manager.h"
#include "data_source.h"
#include "datasourcebase.h"
#include "projectionmanager.h"
#include "taskmanager.h"
#include "number.h"
#include "timeconv.h"
#include "global.h"
#include "logger.h"
#include "stringconv.h"

#include <osgEarth/GeoMath>

using namespace Utils;

SimpleAccuracyEstimator::SimpleAccuracyEstimator()
{
}

void SimpleAccuracyEstimator::init(ReconstructorBase* reconstructor_ptr)
{
    AccuracyEstimatorBase::init(reconstructor_ptr);

    radar_sources_.clear();

    auto& ctx_man = reconstructor_->task().manager().compass().dbContextManager();

    for (const auto& [ds_id, type_str] : ctx_man.dsTypes())
    {
        if (type_str != "Radar")
            continue;

        const context::DataSource* ds = ctx_man.dataSource(ds_id);
        if (!ds || !ds->hasPosition())
            continue;

        bool has_bias = ds->hasRadarBias();
        bool has_acc  = ds->hasRadarAccuracies();

        if (!has_bias && !has_acc)
            continue;

        RadarBiasInfo bias;
        if (has_bias)
            bias = ds->radarBiasInfo();

        if (!bias.azimuth_bias_valid_ && !bias.range_bias_valid_ && !has_acc)
            continue;

        RadarSourceInfo info;
        info.bias_info = bias;
        info.ground_only = ds->groundOnly();
        info.ground_altitude_m = ds->altitude();
        info.ignore_range_azimuth = ds->ignoreRadarAzmRange();

        // load per-channel accuracies if configured
        if (ds->hasRadarAccuracies())
        {
            using DSB = dbContent::DataSourceBase;

            auto acc = ds->radarAccuracies();

            auto loadChannel = [&](ChannelAccuracy& ch, const std::string& azm_key, const std::string& rng_key)
            {
                if (acc.count(azm_key) && acc.count(rng_key))
                {
                    ch.azimuth_stddev_deg = acc.at(azm_key);
                    ch.range_stddev_m = acc.at(rng_key);
                    ch.valid = true;
                }
            };

            loadChannel(info.primary,   DSB::PSRAzmSDKey,   DSB::PSRRngSDKey);
            loadChannel(info.secondary, DSB::SSRAzmSDKey,   DSB::SSRRngSDKey);
            loadChannel(info.mode_s,    DSB::ModeSAzmSDKey, DSB::ModeSRngSDKey);
        }

        radar_sources_[ds_id] = info;

        loginf << "SimpleAccuracyEstimator: loaded radar ds " << ds_id
               << " bias azm " << String::doubleToStringPrecision(bias.azimuth_bias_deg_, 4)
               << " rng " << String::doubleToStringPrecision(bias.range_bias_m_, 1)
               << " gain " << String::doubleToStringPrecision(bias.range_gain_, 5)
               << " acc PSR " << (info.primary.valid
                    ? String::doubleToStringPrecision(info.primary.azimuth_stddev_deg, 4) + "/"
                      + String::doubleToStringPrecision(info.primary.range_stddev_m, 1)
                    : "n/a")
               << " SSR " << (info.secondary.valid
                    ? String::doubleToStringPrecision(info.secondary.azimuth_stddev_deg, 4) + "/"
                      + String::doubleToStringPrecision(info.secondary.range_stddev_m, 1)
                    : "n/a")
               << " ModeS " << (info.mode_s.valid
                    ? String::doubleToStringPrecision(info.mode_s.azimuth_stddev_deg, 4) + "/"
                      + String::doubleToStringPrecision(info.mode_s.range_stddev_m, 1)
                    : "n/a");
    }

    if (radar_sources_.size())
        loginf << "SimpleAccuracyEstimator: loaded " << radar_sources_.size() << " radar source(s)";
}

void SimpleAccuracyEstimator::postProccessNewSlice()
{
    if (radar_sources_.empty())
        return;

    const boost::posix_time::time_duration max_time_diff =
        Time::partialSeconds(reconstructor_->settings().max_time_diff_);

    // per data source: original and corrected distance errors
    std::map<unsigned int, std::vector<double>> org_errors; // ds_id -> errors
    std::map<unsigned int, std::vector<double>> cor_errors;

    for (auto& [utn, target] : reconstructor_->targets_container_.targets_)
    {
        for (auto& [ts, rec_num] : target.tr_timestamps_)
        {
            auto tr_it = reconstructor_->target_reports_.find(rec_num);
            if (tr_it == reconstructor_->target_reports_.end())
                continue;

            auto& tr = tr_it->second;

            if (!tr.position_ || !tr.position_corrected_)
                continue;

            if (!radar_sources_.count(tr.ds_id_))
                continue;

            auto [ref_pos, ref_pos_acc] = target.interpolatedRefPosForTime(ts, max_time_diff);
            if (!ref_pos)
                continue;

            double org_dist = osgEarth::GeoMath::distance(
                tr.position_->latitude_ * DEG2RAD, tr.position_->longitude_ * DEG2RAD,
                ref_pos->latitude_ * DEG2RAD, ref_pos->longitude_ * DEG2RAD);

            double cor_dist = osgEarth::GeoMath::distance(
                tr.position_corrected_->latitude_ * DEG2RAD, tr.position_corrected_->longitude_ * DEG2RAD,
                ref_pos->latitude_ * DEG2RAD, ref_pos->longitude_ * DEG2RAD);

            org_errors[tr.ds_id_].push_back(org_dist);
            cor_errors[tr.ds_id_].push_back(cor_dist);
        }
    }

    for (auto& [ds_id, org_vec] : org_errors)
    {
        if (org_vec.empty())
            continue;

        auto org_stat = Number::getMedianStatistics(org_vec);
        auto cor_stat = Number::getMedianStatistics(cor_errors[ds_id]);

        loginf << "SimpleAccuracyEstimator: ds " << ds_id << " bias correction stats"
               << " cnt " << org_vec.size()
               << "\n\t org median " << String::doubleToStringPrecision(std::get<0>(org_stat), 2)
               << " mad " << String::doubleToStringPrecision(std::get<2>(org_stat), 2)
               << "\n\t cor median " << String::doubleToStringPrecision(std::get<0>(cor_stat), 2)
               << " mad " << String::doubleToStringPrecision(std::get<2>(cor_stat), 2);
    }
}

void SimpleAccuracyEstimator::validate (
    dbContent::targetReport::ReconstructorInfo& tr)
{
    traced_assert(reconstructor_);

    if (tr.isRiskyADSB())
        tr.invalidated_pos_ = true;
}

bool SimpleAccuracyEstimator::canCorrectPosition(
    const dbContent::targetReport::ReconstructorInfo& tr)
{
    if (!tr.position_)
        return false;

    return radar_sources_.count(tr.ds_id_) > 0;
}

void SimpleAccuracyEstimator::correctPosition(
    dbContent::targetReport::ReconstructorInfo& tr)
{
    auto it = radar_sources_.find(tr.ds_id_);
    if (it == radar_sources_.end())
        return;

    auto& src = it->second;

    auto [alt_given, alt_ft] = radar_bias::getBaroAltitude(tr, src.ground_only, src.ground_altitude_m);

    Projection& projection = reconstructor_->task().manager().compass().projectionManager().currentProjection();

    auto corrected = radar_bias::correctPosition(
        tr, reconstructor_->accessor(tr), projection, tr.ds_id_,
        src.bias_info, alt_given, alt_ft, src.ignore_range_azimuth);

    if (corrected)
        tr.position_corrected_ = corrected;
}

dbContent::targetReport::PositionAccuracy SimpleAccuracyEstimator::positionAccuracy (
    const dbContent::targetReport::ReconstructorInfo& tr)
{
    if (tr.position_accuracy_)
    {
        if (tr.position_accuracy_->minStdDev() < reconstructor_->settings().numerical_min_std_dev_)
            return tr.position_accuracy_->getScaledToMinStdDev(reconstructor_->settings().numerical_min_std_dev_);

        return *tr.position_accuracy_;
    }

    // no reported accuracy — try polar model from data source radar accuracies
    if (tr.position_)
    {
        auto it = radar_sources_.find(tr.ds_id_);
        if (it != radar_sources_.end())
        {
            const auto& ch = bestChannelForTR(it->second, tr);

            if (ch.valid)
            {
                boost::optional<double> range_nm = reconstructor_->accessor(tr).radarRange(tr.buffer_index_);
                boost::optional<double> azimuth_deg = reconstructor_->accessor(tr).radarAzimuth(tr.buffer_index_);

                if (range_nm && azimuth_deg)
                {
                    double range_m = *range_nm * NM2M;
                    double bearing_rad = *azimuth_deg * DEG2RAD;

                    // bias-correct range/azimuth if available
                    if (it->second.bias_info.azimuth_bias_valid_
                        && it->second.bias_info.range_bias_valid_)
                    {
                        range_m = (range_m - it->second.bias_info.range_bias_m_)
                                  / (1.0 + it->second.bias_info.range_gain_);
                        bearing_rad = (*azimuth_deg - it->second.bias_info.azimuth_bias_deg_) * DEG2RAD;
                    }

                    // for combined detections, use the channel with smaller azimuth stddev
                    double azm_sd = ch.azimuth_stddev_deg;
                    double rng_sd = ch.range_stddev_m;

                    auto acc = polarToCartesianAccuracy(azm_sd, rng_sd, range_m, bearing_rad);

                    if (acc.minStdDev() < reconstructor_->settings().numerical_min_std_dev_)
                        return acc.getScaledToMinStdDev(reconstructor_->settings().numerical_min_std_dev_);

                    return acc;
                }
            }
        }

        return unspecific_pos_acc_fallback_;
    }

    return no_pos_acc_fallback_;
}

dbContent::targetReport::VelocityAccuracy SimpleAccuracyEstimator::velocityAccuracy (
    const dbContent::targetReport::ReconstructorInfo& tr)
{
    if (tr.velocity_accuracy_)
    {
        if (tr.velocity_accuracy_->minStdDev() < reconstructor_->settings().numerical_min_std_dev_)
            return tr.velocity_accuracy_->getScaledToMinStdDev(reconstructor_->settings().numerical_min_std_dev_);

        return *tr.velocity_accuracy_;
    }

    if (tr.velocity_)
        return unspecifc_vel_acc_fallback_;
    else
        return no_vel_acc_fallback_;
}

dbContent::targetReport::AccelerationAccuracy SimpleAccuracyEstimator::accelerationAccuracy (
    const dbContent::targetReport::ReconstructorInfo& tr)
{
    return no_acc_acc_fallback_;
}

const SimpleAccuracyEstimator::ChannelAccuracy& SimpleAccuracyEstimator::bestChannelForTR(
    const RadarSourceInfo& src,
    const dbContent::targetReport::ReconstructorInfo& tr) const
{
    if (tr.isModeSDetection())
    {
        // Mode S combined: prefer channel with smaller azimuth stddev
        if (src.mode_s.valid && src.primary.valid)
            return src.primary.azimuth_stddev_deg < src.mode_s.azimuth_stddev_deg
                ? src.primary : src.mode_s;
        return src.mode_s.valid ? src.mode_s : src.primary;
    }

    if (tr.isModeACDetection())
    {
        // Mode AC combined: prefer channel with smaller azimuth stddev
        if (src.secondary.valid && src.primary.valid)
            return src.primary.azimuth_stddev_deg < src.secondary.azimuth_stddev_deg
                ? src.primary : src.secondary;
        return src.secondary.valid ? src.secondary : src.primary;
    }

    // primary only
    return src.primary;
}

dbContent::targetReport::PositionAccuracy SimpleAccuracyEstimator::polarToCartesianAccuracy(
    double azimuth_stddev_deg, double range_stddev_m,
    double range_m, double bearing_rad)
{
    // convert azimuth stddev from degrees to meters at this range
    double azm_stddev_m = azimuth_stddev_deg * 2.0 * M_PI * range_m / 360.0;

    // polar covariance -> Cartesian via rotation
    double rng_var = range_stddev_m * range_stddev_m;
    double azm_var = azm_stddev_m * azm_stddev_m;

    double sin_b = sin(bearing_rad);
    double cos_b = cos(bearing_rad);

    // rotation matrix A = [sin(b) cos(b); cos(b) -sin(b)]
    // C_cart = A * diag(rng_var, azm_var) * A^T
    double xx = rng_var * sin_b * sin_b + azm_var * cos_b * cos_b;
    double yy = rng_var * cos_b * cos_b + azm_var * sin_b * sin_b;
    double xy = (rng_var - azm_var) * sin_b * cos_b;

    return dbContent::targetReport::PositionAccuracy(sqrt(xx), sqrt(yy), xy);
}
