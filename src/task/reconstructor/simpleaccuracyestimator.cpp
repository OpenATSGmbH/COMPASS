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
#include "reconstructorbase.h"
#include "targetreportaccessor.h"
#include "number.h"

using namespace Utils;

SimpleAccuracyEstimator::SimpleAccuracyEstimator()
{
}

void SimpleAccuracyEstimator::validate (
    dbContent::targetReport::ReconstructorInfo& tr)
{
    traced_assert(reconstructor_);

    boost::optional<unsigned char> mops_version = reconstructor_->accessor(tr).mopsVersion(tr.buffer_index_);

    if (mops_version && *mops_version == 0)
        tr.invalidated_pos_ = true;

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
