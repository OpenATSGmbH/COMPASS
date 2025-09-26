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

#include "accuracyestimatorbase.h"
#include "datasourcemanager.h"
#include "dbdatasource.h"
#include "logger.h"
#include "stringconv.h"
#include "reconstructorbase.h"

using namespace Utils;

// const double AccuracyEstimatorBase::PosAccStdDevMin = 1.0;
// const dbContent::targetReport::PositionAccuracy AccuracyEstimatorBase::PosAccStdMin
//     { AccuracyEstimatorBase::PosAccStdDevMin, AccuracyEstimatorBase::PosAccStdDevMin, 0};

// const double AccuracyEstimatorBase::VelAccStdDevMin = 2.0;
// const dbContent::targetReport::VelocityAccuracy AccuracyEstimatorBase::VelAccStdMin
//     { AccuracyEstimatorBase::VelAccStdDevMin, AccuracyEstimatorBase::VelAccStdDevMin};

// const double AccuracyEstimatorBase::AccAccStdDevMin = 3.0;
// const dbContent::targetReport::AccelerationAccuracy AccuracyEstimatorBase::AccAccStdMin
//     { AccuracyEstimatorBase::AccAccStdDevMin, AccuracyEstimatorBase::AccAccStdDevMin};

// const double AccuracyEstimatorBase::PosAccStdDevFallback = 100.0;
// const dbContent::targetReport::PositionAccuracy AccuracyEstimatorBase::PosAccStdFallback
//     { AccuracyEstimatorBase::PosAccStdDevFallback, AccuracyEstimatorBase::PosAccStdDevFallback, 0};

// const double AccuracyEstimatorBase::PosAccStdDevMax = 5000.0;
// const dbContent::targetReport::PositionAccuracy AccuracyEstimatorBase::PosAccStdMax
//     { AccuracyEstimatorBase::PosAccStdDevMax, AccuracyEstimatorBase::PosAccStdDevMax, 0};

// const double AccuracyEstimatorBase::VelAccStdDevFallback = 10.0;

// const dbContent::targetReport::VelocityAccuracy AccuracyEstimatorBase::VelAccStdFallback
//     { AccuracyEstimatorBase::VelAccStdDevFallback, AccuracyEstimatorBase::VelAccStdDevFallback};


// const double AccuracyEstimatorBase::AccAccStdDevFallback = 1000.0;

// const dbContent::targetReport::AccelerationAccuracy AccuracyEstimatorBase::AccAccStdFallback
//     { AccuracyEstimatorBase::AccAccStdDevFallback, AccuracyEstimatorBase::AccAccStdDevFallback};

AccuracyEstimatorBase::AccuracyEstimatorBase()
{
}

void AccuracyEstimatorBase::init(ReconstructorBase* reconstructor_ptr)
{
    logdbg;

    traced_assert(reconstructor_ptr);
    reconstructor_ = reconstructor_ptr;
    assoc_distances_.clear();

    unspecific_pos_acc_fallback_= dbContent::targetReport::PositionAccuracy(
          reconstructor_->settings().unspecific_pos_acc_fallback_,
        reconstructor_->settings().unspecific_pos_acc_fallback_, 0);
    no_pos_acc_fallback_ = dbContent::targetReport::PositionAccuracy(
          reconstructor_->settings().no_value_acc_fallback_,
        reconstructor_->settings().no_value_acc_fallback_, 0);
    unspecifc_vel_acc_fallback_ = dbContent::targetReport::VelocityAccuracy(
          reconstructor_->settings().unspecifc_vel_acc_fallback_,
        reconstructor_->settings().unspecifc_vel_acc_fallback_);
    no_vel_acc_fallback_ = dbContent::targetReport::VelocityAccuracy(
          reconstructor_->settings().no_value_acc_fallback_,
        reconstructor_->settings().no_value_acc_fallback_);
    no_acc_acc_fallback_  = dbContent::targetReport::AccelerationAccuracy(
        reconstructor_->settings().no_value_acc_fallback_,
        reconstructor_->settings().no_value_acc_fallback_);
}

void AccuracyEstimatorBase::addAssociatedDistance(
    dbContent::targetReport::ReconstructorInfo& tr, const AssociatedDistance& dist)
{
    assoc_distances_.push_back(dist);
}

void AccuracyEstimatorBase::analyzeAssociatedDistances() const
{
    if (!assoc_distances_.size())
        return;

    std::function<double(const AssociatedDistance&)> dist_lambda =
        [] (const AssociatedDistance& dist) { return dist.distance_m_; };

    std::function<double(const AssociatedDistance&)> estacc_lambda =
        [] (const AssociatedDistance& dist) { return dist.est_std_dev_; };

    std::function<double(const AssociatedDistance&)> ma_lambda =
        [] (const AssociatedDistance& dist) { return dist.mahalanobis_distance_; };

//    printStatistics("distance ", dist_lambda);
//    printStatistics("est acc ", estacc_lambda);
//    printStatistics("maha dist", ma_lambda);

//    loginf << "";
}

void AccuracyEstimatorBase::clearAssociatedDistances()
{
    assoc_distances_.clear();
}

void AccuracyEstimatorBase::printStatistics (
    const std::string name, std::function<double(const AssociatedDistance&)>& lambda)  const
{
    if (!assoc_distances_.size())
        return;

    double sum = std::accumulate(assoc_distances_.begin(), assoc_distances_.end(), 0.0,
                                 [lambda] (double sum, const AssociatedDistance& dist) { return sum + lambda(dist); });
    double mean = sum / assoc_distances_.size();

    std::vector<double> diff(assoc_distances_.size());
    std::transform(assoc_distances_.begin(), assoc_distances_.end(), diff.begin(),
                   [mean, lambda](const AssociatedDistance& dist) { return lambda(dist) - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / assoc_distances_.size());

    auto comp = [lambda] (const AssociatedDistance& a, const AssociatedDistance& b) { return lambda(a) < lambda(b); };

    double min = lambda(*std::min_element(assoc_distances_.begin(), assoc_distances_.end(), comp));
    double max = lambda(*std::max_element(assoc_distances_.begin(), assoc_distances_.end(), comp));

    loginf << name << " SRC " << name_ << ": "  << " avg " << String::doubleToStringPrecision(mean, 2)
           << " stddev " << String::doubleToStringPrecision(stdev, 2) << " min " << min << " max " << max;
}
