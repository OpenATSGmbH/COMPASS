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

#include "simpleassociator.h"
#include "simplereconstructor.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "timeconv.h"

#include "util/tbbhack.h"

#include <boost/thread/mutex.hpp>
#include <boost/optional/optional_io.hpp>

#include <osgEarth/GeoMath>

#include <tuple>

using namespace std;
using namespace Utils;
using namespace dbContent;
using namespace boost::posix_time;

SimpleAssociator::SimpleAssociator(SimpleReconstructor& reconstructor)
    : reconstructor_(reconstructor)
{
}

void SimpleAssociator::associateNewData()
{
    unassoc_rec_nums_.clear();
    unassoc_rec_nums_no_retry_.clear();

    loginf << "associating RefTraj data";

    if (reconstructor().isCancelled())
        return;

    associateTargetReports({255});

    if (reconstructor().isCancelled())
        return;

    // create tracker targets

    loginf << "associating CAT062 data";

    associateTargetReports({62});

    unsigned int multiple_associated {0};
    unsigned int single_associated {0};

    for (auto& target_it : reconstructor_.targets_container_.targets_)
    {
        if (target_it.second.ds_ids_.size() > 1)
            ++multiple_associated;
        else
            ++single_associated;
    }

    if (reconstructor().isCancelled())
        return;

    loginf << "tracker targets " << reconstructor_.targets_container_.targets_.size()
           << " multiple " << multiple_associated << " single " << single_associated;

    // create non-tracker utns
    std::set<unsigned int> sensor_dbcont_ids;

    for (auto dbcont_it : COMPASS::instance().dbContentManager())
    {
        if (dbcont_it.second->id() != 62 && dbcont_it.second->id() != 255)
            sensor_dbcont_ids.insert(dbcont_it.second->id());
    }

    if (reconstructor().isCancelled())
        return;

    loginf << "associating remaining sensor data";

    associateTargetReports(sensor_dbcont_ids);

    if (reconstructor().isCancelled())
        return;

    multiple_associated = 0;
    single_associated = 0;

    for (auto& target_it : reconstructor_.targets_container_.targets_)
    {
        if (target_it.second.ds_ids_.size() > 1)
            ++multiple_associated;
        else
            ++single_associated;
    }

    reconstructor_.targets_container_.checkACADLookup();

    if (reconstructor().isCancelled())
        return;

    selfAssociateNewUTNs();

    if (reconstructor().isCancelled())
        return;

    reconstructor_.targets_container_.checkACADLookup();

    if (reconstructor().isCancelled())
        return;

    retryAssociateTargetReports();

    countUnAssociated();

    if (reconstructor().isCancelled())
        return;

    for (auto& tgt_it : reconstructor().targets_container_.targets_)
        tgt_it.second.created_in_current_slice_ = false;

    // unassoc_rec_nums_.clear(); moved to beginning for statistics

    loginf << "after non-tracker targets "
           << reconstructor_.targets_container_.targets_.size()
           << " multiple " << multiple_associated << " single " << single_associated;
}

bool SimpleAssociator::canGetPositionOffsetTR(
    const dbContent::targetReport::ReconstructorInfo& tr,
    const dbContent::ReconstructorTarget& target, bool use_max_distance)
{
    dbContent::targetReport::Position ref_pos;
    bool ok;

    tie(ref_pos, ok) = target.interpolatedPosForTimeFast(tr.timestamp_, max_time_diff_);

    return tr.position_ && ok;
}

// distance, target acc, tr acc
boost::optional<std::tuple<double, double, double>> SimpleAssociator::getPositionOffsetTR(
    const dbContent::targetReport::ReconstructorInfo& tr,
    const dbContent::ReconstructorTarget& target, 
    bool do_debug,
    const boost::optional<unsigned int>& thread_id,
    reconstruction::PredictionStats* stats)
{
    dbContent::targetReport::Position ref_pos;
    bool ok;

    tie(ref_pos, ok) = target.interpolatedPosForTimeFast(
        tr.timestamp_, max_time_diff_);

    traced_assert(ok);

    double distance_m = osgEarth::GeoMath::distance(tr.position_->latitude_ * DEG2RAD,
                                                    tr.position_->longitude_ * DEG2RAD,
                                                    ref_pos.latitude_ * DEG2RAD, ref_pos.longitude_ * DEG2RAD);

    // distance, target acc, tr acc
    return std::tuple<double, double, double>(distance_m, -1, -1);
}

bool SimpleAssociator::canGetPositionOffsetTargets(const boost::posix_time::ptime& ts,
                                                   const dbContent::ReconstructorTarget& target0,
                                                   const dbContent::ReconstructorTarget& target1)
{
    dbContent::targetReport::Position ref_pos;
    bool ok;

    tie(ref_pos, ok) = target0.interpolatedPosForTimeFast(ts, max_time_diff_);

    if (!ok)
        return false;

    tie(ref_pos, ok) = target1.interpolatedPosForTimeFast(ts, max_time_diff_);

    return ok;
}

// distance, target0+target0 acc
boost::optional<std::tuple<double, double>> SimpleAssociator::getPositionOffsetTargets(
    const boost::posix_time::ptime& ts,
    const dbContent::ReconstructorTarget& target0,
    const dbContent::ReconstructorTarget& target1,
    bool do_debug,
    const boost::optional<unsigned int>& thread_id,
    reconstruction::PredictionStats* stats)
{
    dbContent::targetReport::Position target0_pos;
    bool ok;

    tie(target0_pos, ok) = target0.interpolatedPosForTimeFast(ts, max_time_diff_);
    traced_assert(ok);

    dbContent::targetReport::Position target1_pos;
    tie(target1_pos, ok) = target1.interpolatedPosForTimeFast(ts, max_time_diff_);
    traced_assert(ok);

    double distance_m = osgEarth::GeoMath::distance(target0_pos.latitude_ * DEG2RAD, target0_pos.longitude_ * DEG2RAD,
                                                    target1_pos.latitude_ * DEG2RAD, target1_pos.longitude_ * DEG2RAD);

    // distance, target acc, tr acc
    return std::tuple<double, double>(distance_m, -1);
}

boost::optional<bool> SimpleAssociator::checkTrackPositionOffsetAcceptable (
    dbContent::targetReport::ReconstructorInfo& tr,
    unsigned int utn, bool secondary_verified, bool do_debug)
{
    auto pos_offs = getPositionOffsetTR(tr, reconstructor().targets_container_.targets_.at(utn), do_debug, {});
    if (!pos_offs.has_value())
        return false;

    double distance_m{0}, tgt_est_std_dev{0}, tr_est_std_dev{0};

    std::tie(distance_m, tgt_est_std_dev, tr_est_std_dev) = pos_offs.value();

    return distance_m <
           reconstructor_.settings().max_distance_acceptable_ * reconstructor_.settings().tn_disassoc_distance_factor_;
}

boost::optional<std::pair<bool, double>> SimpleAssociator::calculatePositionOffsetScore (
    const dbContent::targetReport::ReconstructorInfo& tr, unsigned int other_utn,
    double distance_m, double tgt_est_std_dev, double tr_est_std_dev, bool secondary_verified,
    bool do_debug)
{
    return std::pair<bool, double> (distance_m < reconstructor_.settings().max_distance_acceptable_,
                                   distance_m - reconstructor_.settings().max_distance_acceptable_);
}

std::tuple<ReconstructorAssociatorBase::DistanceClassification, double>
SimpleAssociator::checkPositionOffsetScore (double distance_m, double sum_stddev_est,
                                           bool secondary_verified)
{
    const auto& settings = reconstructor_.settings();

    DistanceClassification classif;

    if (distance_m > settings.max_distance_notok_)
        classif = DistanceClassification::Distance_NotOK;
    else if (distance_m > settings.max_distance_dubious_)
        classif = DistanceClassification::Distance_Dubious;
    else if (distance_m < settings.max_distance_acceptable_)
        classif = DistanceClassification::Distance_Good;
    else
        classif = DistanceClassification::Distance_Acceptable;

    return tuple<DistanceClassification, double>(
        classif, settings.max_distance_acceptable_ - distance_m);
}



// bool SimpleAssociator::isTargetAverageDistanceAcceptable(double distance_score_avg, bool secondary_verified)
// {
//     return distance_score_avg < reconstructor_.settings().max_distance_acceptable_;
// }

ReconstructorBase& SimpleAssociator::reconstructor()
{
    return reconstructor_;
}
