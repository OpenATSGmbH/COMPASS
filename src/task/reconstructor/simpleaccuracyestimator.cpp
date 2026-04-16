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
#include "projectionmanager.h"
#include "taskmanager.h"
#include "number.h"
#include "timeconv.h"
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
        if (!ds || !ds->hasRadarBias() || !ds->hasPosition())
            continue;

        RadarBiasInfo bias = ds->radarBiasInfo();

        if (!bias.azimuth_bias_valid_ && !bias.range_bias_valid_)
            continue;

        RadarSourceInfo info;
        info.bias_info = bias;
        info.ground_only = ds->groundOnly();
        info.ground_altitude_m = ds->altitude();
        info.ignore_range_azimuth = ds->ignoreRadarAzmRange();

        radar_sources_[ds_id] = info;

        loginf << "SimpleAccuracyEstimator: loaded radar bias for ds " << ds_id
               << " azm " << String::doubleToStringPrecision(bias.azimuth_bias_deg_, 4)
               << " rng " << String::doubleToStringPrecision(bias.range_bias_m_, 1)
               << " gain " << String::doubleToStringPrecision(bias.range_gain_, 5);
    }

    if (radar_sources_.size())
        loginf << "SimpleAccuracyEstimator: loaded bias for " << radar_sources_.size() << " radar source(s)";
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

    if (tr.position_)
        return unspecific_pos_acc_fallback_;
    else
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
