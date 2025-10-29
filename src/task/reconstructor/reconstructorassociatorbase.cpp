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

#include "reconstructorassociatorbase.h"
#include "logger.h"
#include "stringconv.h"
#include "timeconv.h"
#include "util/tbbhack.h"
#include "reconstructortask.h"
#include "number.h"

#include <QApplication>

#define FIND_UTN_FOR_TARGET_MT
#define FIND_UTN_FOR_TARGET_REPORT_MT

using namespace std;
using namespace dbContent;
using namespace Utils;

ReconstructorAssociatorBase::ReconstructorAssociatorBase()
{
}

#define DO_VALGRIND_BENCH 0

#if DO_VALGRIND_BENCH
#include <valgrind/callgrind.h>
#endif


void ReconstructorAssociatorBase::associateNewData()
{
    loginf << "slice " << reconstructor().currentSlice().slice_count_
           << " run " << reconstructor().currentSlice().run_count_;

    max_time_diff_ = Time::partialSeconds(reconstructor().settings().max_time_diff_);

    unassoc_rec_nums_.clear();

    if (reconstructor().isCancelled())
        return;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    loginf << "associateTargetReports";

#if DO_VALGRIND_BENCH
    CALLGRIND_START_INSTRUMENTATION;

    CALLGRIND_TOGGLE_COLLECT;
#endif

    associateTargetReports();

#if DO_VALGRIND_BENCH
    CALLGRIND_TOGGLE_COLLECT;

    CALLGRIND_STOP_INSTRUMENTATION;
#endif

    time_assoc_trs_ += boost::posix_time::microsec_clock::local_time() - start_time;

    reconstructor().targets_container_.checkACADLookup();

    // if (reconstructor().isCancelled())
    //     return;

    // start_time = boost::posix_time::microsec_clock::local_time();

    // retryAssociateTargetReports(); // try once

    // time_retry_assoc_trs_ += boost::posix_time::microsec_clock::local_time() - start_time;

    // if (reconstructor().isCancelled())
    //     return;

    // reconstructor().targets_container_.checkACADLookup();

    if (reconstructor().isCancelled())
        return;

    start_time = boost::posix_time::microsec_clock::local_time();

    loginf << "selfAssociateNewUTNs";

    selfAssociateNewUTNs(); // self-associate created utns

    time_assoc_new_utns_ += boost::posix_time::microsec_clock::local_time() - start_time;

    if (reconstructor().isCancelled())
        return;

    reconstructor().targets_container_.checkACADLookup();

    if (reconstructor().isCancelled())
        return;

    postAssociate();

    if (reconstructor().isCancelled())
        return;

    start_time = boost::posix_time::microsec_clock::local_time();

    loginf << "retryAssociateTargetReports";

    retryAssociateTargetReports();  // try second time

    time_retry_assoc_trs_ += boost::posix_time::microsec_clock::local_time() - start_time;

    loginf << "countUnAssociated";

    countUnAssociated();

    if (reconstructor().isCancelled())
        return;

    loginf << "clear new flags";

    // clear new flags
    for (auto& tgt_it : reconstructor().targets_container_.targets_)
        tgt_it.second.created_in_current_slice_ = false;

    // unassoc_rec_nums_.clear(); moved to beginning for statistics

    loginf << "time_assoc_trs " << Time::toString(time_assoc_trs_)
           << " time_assoc_new_utns " << Time::toString(time_assoc_new_utns_)
           << " time_retry_assoc_trs " << Time::toString(time_retry_assoc_trs_);

    if (reconstructor().currentSlice().is_last_slice_)
        loginf << "done, num_merges " << num_merges_;
}

void ReconstructorAssociatorBase::reset()
{
    logdbg;

    reconstructor().targets_container_.utn_vec_.clear();
    reconstructor().targets_container_.acad_2_utn_.clear();
    reconstructor().targets_container_.acid_2_utn_.clear();
    reconstructor().targets_container_.tn2utn_.clear();

    unassoc_rec_nums_.clear();
    assoc_counts_.clear();

    num_merges_ = 0;

    time_assoc_trs_ = {};
    time_assoc_new_utns_ = {};
    time_retry_assoc_trs_ = {};
}

void ReconstructorAssociatorBase::associateTargetReports()
{
    loginf << "num timestamps "
           << reconstructor().tr_timestamps_.size();

    reconstructor().targets_container_.checkACADLookup();

    boost::posix_time::ptime last_ts;
    auto five_min = boost::posix_time::seconds(5*60);
    unsigned int batch_cnt=0;

    for (auto& batch_it : reconstructor().tr_batches_)
    {
        if (reconstructor().isCancelled())
            return;

        if (last_ts.is_not_a_date_time())
        {
            last_ts = batch_it.first;
            loginf << "start time " << Time::toString(last_ts) << " batch_cnt " << batch_cnt;
        }

        if (batch_it.first - last_ts > five_min)
        {
            last_ts = batch_it.first;
            loginf << "processed time " << Time::toString(last_ts) << " batch_cnt " << batch_cnt;
        }

        associateTargetReportBatch(batch_it.first, batch_it.second);

        ++batch_cnt;
    }

    loginf << "done";
}

void ReconstructorAssociatorBase::associateTargetReportBatch(const boost::posix_time::ptime& ts, 
                                                             const ReconstructorBase::TargetReportBatch& batch)
{
    logdbg << "batch size " << batch.rec_nums_.size();

    bool do_debug_rec_num = false;
    
    unsigned long rec_num;
    int utn;
    bool is_unreliable_primary_only;

    //auto& batch_stats = batch_stats_[ batch.ds_id_ ];

    //batch_stats.num_batches     += 1;
    //batch_stats.batch_size_min   = std::min(batch_stats.batch_size_min, batch.rec_nums_.size());
    //batch_stats.batch_size_max   = std::max(batch_stats.batch_size_max, batch.rec_nums_.size());
    //batch_stats.batch_size_mean += batch.rec_nums_.size();

    std::vector<unsigned long> unreliable_primary_only_trs;
    size_t num_in_slice = 0;

    for (auto& rn_it : batch.rec_nums_)
    {
        rec_num = rn_it;

        logdbg << "rec_num " << rec_num;

        traced_assert(reconstructor().target_reports_.count(rec_num));

        dbContent::targetReport::ReconstructorInfo& tr =
            reconstructor().target_reports_.at(rec_num);

        do_debug_rec_num = reconstructor().task().debugSettings().debug_association_ &&
                            reconstructor().task().debugSettings().debugRecNum(rec_num);

        if (do_debug_rec_num)
            loginf << "DBG tr " << rec_num;

        if (!tr.in_current_slice_)
        {
            if (do_debug_rec_num)
                loginf << "DBG tr " << rec_num << " not in current slice";

            continue;
        }

        ++num_in_slice;

        if (reconstructor().acc_estimator_->canCorrectPosition(tr))
        {
            if (do_debug_rec_num)
                loginf << "DBG correcting position";

            reconstructor().acc_estimator_->correctPosition(tr);
        }

        is_unreliable_primary_only = tr.isUnreliablePrimaryOnlyDetection();

        if (do_debug_rec_num)
            loginf << "is_unreliable_primary_only " << is_unreliable_primary_only;

        if (is_unreliable_primary_only)
        {
            unreliable_primary_only_trs.push_back(rec_num);
            continue;
        }

        utn = -1;

        if (!is_unreliable_primary_only)  // if unreliable primary only, delay association until
                                            // retry
        {
            if (do_debug_rec_num)
                loginf << "DBG !is_unreliable_primary_only finding UTN";

            utn = findUTNFor(tr);

            if (do_debug_rec_num ||
                (reconstructor().task().debugSettings().debug_association_ &&
                    reconstructor().task().debugSettings().debugUTN(utn)))
                loginf << "DBG !is_unreliable_primary_only got UTN " << utn;
        }

        if (utn != -1)  // estimate accuracy and associate
        {
            bool do_debug_utn = reconstructor().task().debugSettings().debug_association_ &&
                                reconstructor().task().debugSettings().debugUTN(utn);

            if (do_debug_rec_num || do_debug_utn)
                loginf << "DBG associating tr " << rec_num << " to UTN " << utn;

            associate(tr, utn);

            if (do_debug_rec_num || do_debug_utn)
            {
                traced_assert(reconstructor().targets_container_.targets_.count(utn));

                loginf << "DBG target after assoc "
                        << reconstructor().targets_container_.targets_.at(utn).asStr();
            }
        }
        else  // not associated
        {
            if (do_debug_rec_num)
                loginf << "DBG adding to unassoc_rec_nums_";

            ReconstructorTarget::globalStats().num_sec_unassoc += 1;

            unassoc_rec_nums_.push_back(rec_num);
        }
    }

    // if (num_in_slice > 0)
    // {
    //     batch_stats.batch_slice_size_min   = std::min(batch_stats.batch_slice_size_min, num_in_slice);
    //     batch_stats.batch_slice_size_max   = std::max(batch_stats.batch_slice_size_max, num_in_slice);
    //     batch_stats.batch_slice_size_mean += num_in_slice;
    //     batch_stats.num_batches_slice     += 1;
    // }

    if (!unreliable_primary_only_trs.empty())
    {
        size_t n_po = unreliable_primary_only_trs.size();

        //batch_stats.batch_po_size_min   = std::min(batch_stats.batch_po_size_min, n_po);
        //batch_stats.batch_po_size_max   = std::max(batch_stats.batch_po_size_max, n_po);
        //batch_stats.batch_po_size_mean += n_po;
        //batch_stats.num_batches_po     += 1;

        ReconstructorTarget::globalStats().num_po_unassoc += unreliable_primary_only_trs.size();

        logdbg << "associating primary onlies";
        associateUnreliablePrimaryOnly(batch.ds_id_, ts, unreliable_primary_only_trs, do_debug_rec_num);
    }

    logdbg << "done";
}

void ReconstructorAssociatorBase::associateUnreliablePrimaryOnly(unsigned int ds_id,
                                                                 const boost::posix_time::ptime& ts,
                                                                 const std::vector<unsigned long>& rec_nums,
                                                                 bool debug)
{
    //default: just add to unassociated and retry later on
    for (auto rec_num : rec_nums) 
    {
        if (debug)
            loginf << "DBG adding tr " << rec_num << " to unassoc_rec_nums_";

        unassoc_rec_nums_.push_back(rec_num);
    }
}

void ReconstructorAssociatorBase::associateTargetReports(std::set<unsigned int> dbcont_ids)
{
    loginf << "dbcont_ids " << String::compress(dbcont_ids, ',');

    unsigned long rec_num;
    int utn;

    bool do_debug;

    reconstructor().targets_container_.checkACADLookup();

    for (auto& batch_it : reconstructor().tr_batches_)
    {
        if (reconstructor().isCancelled())
            return;

        for (auto& rn_it : batch_it.second.rec_nums_)
        {
            rec_num = rn_it;
            traced_assert(reconstructor().target_reports_.count(rec_num));

            do_debug = reconstructor().task().debugSettings().debug_association_ &&
                       reconstructor().task().debugSettings().debugRecNum(rec_num);

            if (do_debug)
                loginf << "DBG tr " << rec_num;

            dbContent::targetReport::ReconstructorInfo& tr =
                reconstructor().target_reports_.at(rec_num);

            if (!dbcont_ids.count(tr.dbcont_id_))
                continue;

            if (!tr.in_current_slice_)
            {
                if (do_debug)
                    loginf << "DBG tr " << rec_num << " not in current slice";

                continue;
            }

            utn = findUTNFor(tr);

            if (utn != -1)  // estimate accuracy and associate
            {
                if (reconstructor().task().debugSettings().debug_association_ &&
                    reconstructor().task().debugSettings().debugUTN(utn))
                    loginf << "DBG associating (dbcont_ids) tr " << rec_num << " to UTN " << utn;

                associate(tr, utn);
            }
            else  // not associated
            {
                unassoc_rec_nums_.push_back(rec_num);
            }
        }
    }
}

void ReconstructorAssociatorBase::selfAssociateNewUTNs()
{
    loginf;

    size_t num_tentative_origin = 0;
    size_t num_targets          = 0;
    for (auto utn : reconstructor().targets_container_.utn_vec_)
    {
        traced_assert(reconstructor().targets_container_.targets_.count(utn));
        dbContent::ReconstructorTarget& target = reconstructor().targets_container_.targets_.at(utn);

        if (!target.target_reports_.size()) // can not compare
            continue;

        ++num_targets;

        if (target.created_from_tentative_)
            ++num_tentative_origin;
    }

    loginf << "about to merge " << num_targets << " target(s), " << num_tentative_origin << " created from tentative(s)";

    if (reconstructor().settings().do_not_merge_utns_)
    {
        loginf << "skipping";
        return;
    }

    unsigned int loop_cnt {0};

    std::multimap<float, std::pair<unsigned int, unsigned int>> scored_utn_pairs;
    set<unsigned int> utns_to_remove;

    //float min_score = reconstructor().settings().targets_min_assoc_score_;
    //float score;
    //std::pair<unsigned int, unsigned int> utn_pair;
    unsigned int utn, other_utn;

RESTART_SELF_ASSOC:

    scored_utn_pairs.clear();
    utns_to_remove.clear();

    //std::map<std::pair<unsigned int, unsigned int>, float> assessed_scores; // utn_pair -> score

    if (reconstructor().isCancelled())
        return;

    logdbg << "loop " << loop_cnt;

    reconstructor().targets_container_.checkACADLookup();

    std::map<std::pair<unsigned int, unsigned int>, ReconstructorAssociatorBase::AssociationOption> assoc_option_cache;

    // collect scored pairs
    for (auto utn : reconstructor().targets_container_.utn_vec_)
    {
        traced_assert(reconstructor().targets_container_.targets_.count(utn));
        dbContent::ReconstructorTarget& target = reconstructor().targets_container_.targets_.at(utn);

        if (!target.target_reports_.size()) // can not compare
            continue;

        bool do_debug = reconstructor().task().debugSettings().debug_association_
                        && reconstructor().task().debugSettings().debugUTN(utn);

        if (do_debug)
            loginf << "checking utn " << utn;

        // tie(score, utn_pair) = findUTNsForTarget(utn); // , utns_to_remove

        // if (score != std::numeric_limits<float>::lowest())
        //     scored_utn_pairs.insert({-score, utn_pair});

        std::vector<ReconstructorAssociatorBase::AssociationOption> options = findUTNsForTarget(utn, assoc_option_cache);

        for (auto& option : options)
        {
            traced_assert (option.usable_);
            scored_utn_pairs.insert({-option.score_, {option.utn_, option.other_utn_}});
            assoc_option_cache[{option.utn_, option.other_utn_}] = option;
        }
    }

    if (reconstructor().isCancelled())
        return;

    loginf << "loop " << loop_cnt << " pairs to merge " << scored_utn_pairs.size();

    // reverse iterate and remove elements with the largest key
    while (!scored_utn_pairs.empty())
    {
        // get iterator to the last element (largest score)
        auto last_it = scored_utn_pairs.begin();
        float largest_score = last_it->first;

        // Access the key and value
        tie(utn, other_utn) = last_it->second;

        if (!utns_to_remove.count(utn) && !utns_to_remove.count(other_utn))
        {
            // join and schedule for remove
            traced_assert(reconstructor().targets_container_.targets_.count(utn));
            dbContent::ReconstructorTarget& target =
                reconstructor().targets_container_.targets_.at(utn);

            traced_assert(reconstructor().targets_container_.targets_.count(other_utn));
            dbContent::ReconstructorTarget& other_target =
                reconstructor().targets_container_.targets_.at(other_utn);

            loginf << "loop " << loop_cnt
                    << ": merging '" << target.asStr()
                    << "' with '" << other_target.asStr() << "' using score "
                    << String::doubleToStringPrecision(largest_score, 2);

            // move target reports
            target.addTargetReports(other_target);

            // schedule remove from targets
            utns_to_remove.insert(other_utn);

            reconstructor().targets_container_.replaceInLookup(other_utn, utn);

            ++num_merges_;

            assoc_option_cache.erase({other_utn, utn});
            assoc_option_cache.erase({utn, other_utn});
        }
        

        // Erase all elements with the largest key
        scored_utn_pairs.erase(last_it);
    }

    // remove scheduled from targets
    if (utns_to_remove.size())
    {
        for (auto other_utn : utns_to_remove)
        {
            traced_assert(reconstructor().targets_container_.targets_.count(other_utn));

            reconstructor().targets_container_.removeUTN(other_utn);
        }

        utns_to_remove.clear();

        ++loop_cnt;

        reconstructor().targets_container_.checkACADLookup();

        goto RESTART_SELF_ASSOC; // here we go again
    }

    loginf << "done at loop cnt " << loop_cnt;
}

void ReconstructorAssociatorBase::retryAssociateTargetReports()
{
    loginf;

    if (!unassoc_rec_nums_.size())
        return;

    unsigned long rec_num;
    unsigned int dbcont_id;
    int utn;

    bool do_debug;

    unsigned int assocated_cnt{0};

    for (auto rec_num_it = unassoc_rec_nums_.rbegin(); rec_num_it != unassoc_rec_nums_.rend();)
    {
        if (reconstructor().isCancelled())
            return;

        rec_num = *rec_num_it;

        dbcont_id = Number::recNumGetDBContId(rec_num);

        traced_assert(reconstructor().target_reports_.count(rec_num));

        do_debug = reconstructor().task().debugSettings().debug_association_
                   && reconstructor().task().debugSettings().debugRecNum(rec_num);

        dbContent::targetReport::ReconstructorInfo& tr = reconstructor().target_reports_.at(rec_num);
        bool is_unreliable_primary_only = tr.isUnreliablePrimaryOnlyDetection();

        //do_debug = tr.dbcont_id_ == 10;

        if (do_debug)
            loginf << "DBG tr " << rec_num;

        if (!tr.in_current_slice_)
        {
            if(do_debug)
                loginf << "DBG tr " << rec_num << " not in current slice";

            ++rec_num_it;
            continue;
        }

        // check if should have been associated
        traced_assert(!tr.acad_);
        traced_assert(!tr.acid_);

        if (dbcont_id == 62 || dbcont_id == 255)
            traced_assert(!tr.track_number_);

        utn = findUTNByModeACPos (tr);

        if (utn != -1) // estimate accuracy and associate
        {
            if (do_debug || reconstructor().task().debugSettings().debugUTN(utn))
                loginf << "DBG retry-associating tr " << rec_num << " to UTN " << utn;

            if (is_unreliable_primary_only)
                ReconstructorTarget::globalStats().num_po_reassociated += 1;
            else
                ReconstructorTarget::globalStats().num_sec_reassociated += 1;

            associate(tr, utn);

            ++assocated_cnt;

            // Erase the element and update the iterator
            rec_num_it = std::vector<unsigned long>::reverse_iterator(
                unassoc_rec_nums_.erase(std::next(rec_num_it).base()));
        }
        else
            ++rec_num_it;
    }

    loginf << "done with count " << assocated_cnt;
}

void ReconstructorAssociatorBase::associate(
    dbContent::targetReport::ReconstructorInfo& tr, int utn)
{
    traced_assert(utn >= 0);

    bool do_debug = reconstructor().task().debugSettings().debug_association_
                    && (reconstructor().task().debugSettings().debugRecNum(tr.record_num_)
                        || reconstructor().task().debugSettings().debugUTN(utn));

    unsigned int dbcont_id  = Number::recNumGetDBContId(tr.record_num_);
    //AccuracyEstimatorBase::AssociatedDistance dist;

    if (!reconstructor().targets_container_.targets_.count(utn))
        logerr << "utn " << utn << " missing";

    // add associated target reports
    traced_assert(reconstructor().targets_container_.targets_.count(utn));

    if(do_debug)
        loginf << "DBG associate tr " << tr.asStr() << " to utn "
               << reconstructor().targets_container_.targets_.at(utn).asStr();

    // check if position usable
    // if (do_debug)
    //     loginf << "DBG validate";

    reconstructor().acc_estimator_->validate(tr); // do counting

    // if (do_debug)
    //     loginf << "DBG outlier detect";

    tr.is_pos_outlier_ = false;

    if (!tr.doNotUsePosition())
        reconstructor().acc_estimator_->doOutlierDetection(tr, utn); //124976,129072

    // if (do_debug)
    //     loginf << "DBG addTargetReport";

    reconstructor().targets_container_.targets_.at(utn).addTargetReport(tr.record_num_);

    // if (do_debug)
    //     loginf << "DBG add to lookups";

    reconstructor().targets_container_.addToLookup(utn, tr);

    assoc_counts_[tr.ds_id_][dbcont_id].first++;

    // if (do_debug)
    //     loginf << "DBG postAssociate";

    postAssociate (tr, utn);
}

void ReconstructorAssociatorBase::countUnAssociated(const std::vector<unsigned long> &rec_nums)
{
    for (auto rn_it : rec_nums)
    {
        if (reconstructor().isCancelled())
            return;

        unsigned long rec_num;
        unsigned int dbcont_id;

        rec_num = rn_it;

        dbcont_id = Number::recNumGetDBContId(rec_num);

        traced_assert(reconstructor().target_reports_.count(rec_num));

        dbContent::targetReport::ReconstructorInfo &tr = reconstructor().target_reports_.at(rec_num);

        assoc_counts_[tr.ds_id_][dbcont_id].second++;
    }
}

void ReconstructorAssociatorBase::countUnAssociated()
{
    countUnAssociated(unassoc_rec_nums_);
}

int ReconstructorAssociatorBase::findUTNFor (dbContent::targetReport::ReconstructorInfo& tr)
{
    int utn {-1};

    bool do_debug = reconstructor().task().debugSettings().debug_association_
                    && reconstructor().task().debugSettings().debugRecNum(tr.record_num_);

    if (do_debug)
        loginf << "DBG findUTNFor tr " << tr.asStr();

    unsigned int dbcont_id = Number::recNumGetDBContId(tr.record_num_);

    vector<tuple<bool, unsigned int, double>> results;
    const boost::posix_time::time_duration track_max_time_diff =
        Time::partialSeconds(reconstructor().settings().track_max_time_diff_);

    traced_assert(reconstructor().targets_container_.targets_.size() == reconstructor().targets_container_.utn_vec_.size());

START_TR_ASSOC:

    utn = -1; // not yet found

    // try by mode-s address or trustworthy track number
    // create new if not already existing
    if (tr.acad_
        || tr.acid_
        || (tr.track_number_ && (dbcont_id == 62 || dbcont_id == 255)))
    {
        // find utn

        if (reconstructor().targets_container_.canAssocByACAD(tr, do_debug)) // already exists
            utn = reconstructor().targets_container_.assocByACAD(tr, do_debug);
        else if (reconstructor().targets_container_.canAssocByACID(tr, do_debug)) // already exists
            utn = reconstructor().targets_container_.assocByACID(tr, do_debug);
        else if (reconstructor().targets_container_.canAssocByTrackNumber(tr, do_debug))
        {
            if (do_debug)
                loginf << "DBG canAssocByTrackNumber";

            utn = reconstructor().targets_container_.assocByTrackNumber(tr, track_max_time_diff, do_debug);

            if (utn == -1)
            {
                if (do_debug)
                    loginf << "DBG restart assoc due to track_num disassoc";

                traced_assert(!reconstructor().targets_container_.canAssocByTrackNumber(tr, do_debug));

                goto START_TR_ASSOC;
            }

            if (do_debug)
                loginf << "DBG assoc by track_num, utn " << utn;

            if (utn != -1 && reconstructor().settings().do_track_number_disassociate_using_distance_
                && canGetPositionOffsetTR(tr, reconstructor().targets_container_.targets_.at(utn)))
            {
                traced_assert(reconstructor().targets_container_.targets_.count(utn));

                if (do_debug)
                    loginf << "DBG stored utn in tn2utn_ checking position offset";

                // check for position offsets
                boost::optional<bool> check_result = isTrackNumberPositionOffsetTooLarge(
                    tr, utn, true, do_debug);

                if (check_result && *check_result)
                {
                    if (do_debug)
                        loginf << "DBG stored utn in tn2utn_ position offset not acceptable " << *check_result;

                    reconstructor().targets_container_.eraseTrackNumberLookup(tr);

                    goto START_TR_ASSOC;
                }
            }

            // do assoc
            if (do_debug)
                loginf << "DBG use stored utn in tn2utn_: " << utn;
        }
        else // not yet existing, create & add target findUTNByModeACPosOrCreateNewTarget(tr)
        {
            // check if position match to other target would exist
            utn = findUTNByModeACPos (tr);

            if (utn == -1)
            {
                // create new and add
                utn = reconstructor().targets_container_.createNewTarget(tr);

                if (reconstructor().task().debugSettings().debugUTN(utn))
                    loginf << "created new utn " << utn << " with no mode AC match using tr " << tr.asStr();

                if (do_debug)
                    loginf << "DBG use mode a/c/pos assoc to new utn " << utn;

                traced_assert(reconstructor().targets_container_.targets_.count(utn));
            }
            else
            {
                traced_assert(reconstructor().targets_container_.targets_.count(utn));

                if (do_debug)
                    loginf << "DBG use mode a/c/pos assoc to existing utn " << utn;
            }
        }
        traced_assert(utn != -1);

        if(do_debug)
            loginf << "DBG tr " << tr.record_num_ << " utn by acad/acid/track num";
    }
    else // not associated by trustworty id
    {
        traced_assert(utn == -1);

        if(do_debug)
            loginf << "DBG tr " << tr.record_num_ << " no utn by acad, doing mode a/c + pos";

        utn = findUTNByModeACPos (tr);

        if (utn != -1)
            traced_assert(reconstructor().targets_container_.targets_.count(utn));
    }

    return utn;
}

int ReconstructorAssociatorBase::findUTNByModeACPos (
    const dbContent::targetReport::ReconstructorInfo& tr)
{
    unsigned int num_targets = reconstructor().targets_container_.targets_.size();
    traced_assert(reconstructor().targets_container_.utn_vec_.size() == num_targets);

    vector<tuple<bool, unsigned int, double>> results;
    vector<reconstruction::PredictionStats> prediction_stats;

    results.resize(num_targets);
    prediction_stats.resize(num_targets);
    boost::posix_time::ptime timestamp = tr.timestamp_;

    const float max_altitude_diff = reconstructor().settings().max_altitude_diff_;

    bool do_debug = reconstructor().task().debugSettings().debug_association_
                    && reconstructor().task().debugSettings().debugRecNum(tr.record_num_);

    if (do_debug)
        loginf << "rn " << tr.record_num_;

#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
    tbb::parallel_for(uint(0), num_targets, [&](unsigned int target_cnt)
#else
    for (unsigned int target_cnt = 0; target_cnt < num_targets; ++target_cnt)
#endif
                      {
                          unsigned int other_utn = reconstructor().targets_container_.utn_vec_.at(target_cnt);
                          //bool do_other_debug = false; //debug_utns.count(other_utn);

                          //do_debug = tr.dbcont_id_ == 10 && other_utn == 7;

                          ReconstructorTarget& other = reconstructor().targets_container_.targets_.at(other_utn);

                          results[target_cnt] = tuple<bool, unsigned int, double>(false, other.utn_, 0);

                          if (tr.acad_ && other.hasACAD()) // has to be covered outside
#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
                              return;
#else
            continue;
#endif

                          if (!other.isTimeInside(timestamp, max_time_diff_))
                          {
#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
                              return;
#else
            continue;
#endif
                          }

                          if (!other.canPredict(timestamp))
#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
                              return;
#else
            continue;
#endif

                          bool mode_a_verified = false;
                          bool mode_c_verified = false;

                          if (tr.mode_a_code_ || tr.barometric_altitude_) // mode a/c based
                          {
                              // check mode a code
                              if (tr.mode_a_code_)
                              {
                                  ComparisonResult ma_res = other.compareModeACode(tr, max_time_diff_, do_debug);

                                  if (ma_res == ComparisonResult::DIFFERENT)
                                  {
#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
                                      return;
#else
                    continue;
#endif
                                  }

                                  mode_a_verified = ma_res == ComparisonResult::SAME;
                              }

                              if (do_debug)
                                  loginf << "DBG tr " << tr.record_num_ << " other_utn "
                                         << other_utn << ": possible mode a match, verified "
                                         << mode_a_verified;


                              // check mode c code
                              if (tr.barometric_altitude_)
                              {
                                  ComparisonResult mc_res = other.compareModeCCode(
                                      tr, max_time_diff_, max_altitude_diff, false);

                                  if (mc_res == ComparisonResult::DIFFERENT)
                                  {
#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
                                      return;
#else
                    continue;
#endif
                                  }

                                  mode_c_verified = mc_res == ComparisonResult::SAME;
                              }

                              if (do_debug)
                                  loginf << "DBG tr " << tr.record_num_ << " other_utn "
                                         << other_utn << ": possible match";
                          }

                          bool secondary_verified = mode_a_verified || mode_c_verified;

                          if (do_debug)
                              loginf << "DBG tr " << tr.record_num_ << " other_utn " << other_utn
                                     << " mode_a_verified " << mode_a_verified
                                     << " mode_c_verified " << mode_c_verified
                                     << " secondary_verified " << secondary_verified;


                          // check positions

                          double distance_m{0}, tgt_est_std_dev{0}, tr_est_std_dev{0};

                          if (!canGetPositionOffsetTR(tr, other))
#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
                              return;
#else
            continue;
#endif
                          auto pos_offs = getPositionOffsetTR(tr, other, do_debug, {}, &prediction_stats[ target_cnt ]);

                          if (!pos_offs.has_value())
                          {
                              if (do_debug)
                                  loginf << "DBG tr " << tr.record_num_ << " other_utn " << other_utn
                                         << " no position offset";

#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
                              return;
#else
            continue;
#endif
                          }

                          std::tie(distance_m, tgt_est_std_dev, tr_est_std_dev) = pos_offs.value();

                          boost::optional<std::pair<bool, double>> check_ret = calculatePositionOffsetScore(
                              tr, other_utn, distance_m, tgt_est_std_dev, tr_est_std_dev, secondary_verified, do_debug);

                          if (check_ret && check_ret->first)
                          {
                              results[target_cnt] = tuple<bool, unsigned int, double> (
                                  true, other.utn_, check_ret->second);
                          }
                      }
#ifdef FIND_UTN_FOR_TARGET_REPORT_MT
                      );
#endif

    //log failed predictions
    for (const auto& s : prediction_stats)
        ReconstructorTarget::addPredictionToGlobalStats(s);

    // find best match
    bool usable;
    unsigned int other_utn;

    bool first = true;
    unsigned int best_other_utn {0};
    double best_mahalanobis_dist {0};
    double mahalanobis_dist {0};

    for (auto& res_it : results) // usable, other utn, num updates, avg distance
    {
        tie(usable, other_utn, mahalanobis_dist) = res_it;

        if (!usable)
            continue;

        if (first || mahalanobis_dist < best_mahalanobis_dist)
        {
            best_other_utn = other_utn;
            best_mahalanobis_dist = mahalanobis_dist;

            first = false;
        }
    }

    if (!first)
    {
        if (do_debug)
        {
            loginf << "DBG tr " << tr.record_num_ << " other_utn "
                   << other_utn << ": match with best_mahalanobis_dist " << best_mahalanobis_dist;
        }

        return best_other_utn;
    }

    return -1;
}

std::vector<ReconstructorAssociatorBase::AssociationOption> ReconstructorAssociatorBase::findUTNsForTarget (
    unsigned int utn,
    std::map<std::pair<unsigned int, unsigned int>, ReconstructorAssociatorBase::AssociationOption>& assoc_option_cache)
{
    std::vector<ReconstructorAssociatorBase::AssociationOption> ret_options;

    if (!reconstructor().targets_container_.targets_.size()) // check if targets exist
        return ret_options;

    traced_assert(reconstructor().targets_container_.targets_.count(utn));

    const dbContent::ReconstructorTarget& target = reconstructor().targets_container_.targets_.at(utn);

    // dont check by mode s, should never work

    // try to find by m a/c/pos

    vector<AssociationOption> results;

    vector<reconstruction::PredictionStats> prediction_stats;

    unsigned int num_utns = reconstructor().targets_container_.utn_vec_.size();
    results.resize(num_utns);
    prediction_stats.resize(num_utns);

    const auto& settings = reconstructor().settings();

#ifdef FIND_UTN_FOR_TARGET_MT
    tbb::parallel_for(uint(0), num_utns, [&](unsigned int cnt)
#else
    for (unsigned int cnt=0; cnt < num_utns; ++cnt)
#endif
                      {
                          unsigned int other_utn = reconstructor().targets_container_.utn_vec_.at(cnt);

                          auto assoc_option_it = assoc_option_cache.find({utn, other_utn});
                          if (assoc_option_it != assoc_option_cache.end())
                          {
                              results[cnt] = assoc_option_it->second;
#ifdef FIND_UTN_FOR_TARGET_MT
                              return;
#else
                              continue;
#endif
                          }

                          //bool print_debug_target = debug_utns.count(utn) && debug_utns.count(other_utn);

                          // only check for previous targets
                          const dbContent::ReconstructorTarget& other =
                              reconstructor().targets_container_.targets_.at(other_utn);

                          // do not merge tentative targets?
                          if (reconstructor().settings().do_not_merge_targets_with_tentative_origin_ &&
                              (target.createdFromTentative() || other.createdFromTentative()))
                              return;

                          results[cnt] = AssociationOption(false, utn, other.utn_, 0, false, 0);

                          bool do_debug = reconstructor().task().debugSettings().debug_association_
                                          && reconstructor().task().debugSettings().debugUTN(utn)
                                          && reconstructor().task().debugSettings().debugUTN(other_utn);

                          if (utn == other_utn)
#ifdef FIND_UTN_FOR_TARGET_MT
                              return;
#else
            continue;
#endif

                          if (!other.created_in_current_slice_)
                          {
                              if (do_debug)
                                  loginf << "\ttarget " << target.utn_ << " other " << other_utn
                                         << " other not in current slice";

#ifdef FIND_UTN_FOR_TARGET_MT
                              return;
#else
            continue;
#endif
                          }

                          if (do_debug)
                              loginf << "\ttarget '" << target.asStr() << "' compare to other '"
                                     << other.asStr() << "'";

                          if (target.hasACAD() && other.hasACAD())
                          {
                              if (do_debug)
                                  loginf << "\ttarget " << target.utn_ << " other " << other_utn
                                         << " both have acads, first " << target.hasACAD()
                                         << " " << target.acadsStr()
                                         << " other " << other.hasACAD()
                                         << " " << other.acadsStr();

#ifdef FIND_UTN_FOR_TARGET_MT
                              return;
#else
            continue;
#endif
                          }

                          if (do_debug)
                          {
                              loginf << "\ttarget " << target.utn_ << " " << target.timeStr()
                                     << " checking other " << other.utn_ << " " << other.timeStr()
                                     << " overlaps " << target.timeOverlaps(other)
                                     << " prob " << target.probTimeOverlaps(other);
                          }

                          if (target.timeOverlaps(other)
                              && target.probTimeOverlaps(other) >= settings.target_prob_min_time_overlap_)
                          {
                              // check based on mode a/c/pos

                              if (do_debug)
                                  loginf << "\ttarget " << target.utn_ << " other " << other.utn_ << " overlap passed";

                              vector<unsigned long> ma_unknown; // record numbers
                              vector<unsigned long> ma_same;
                              vector<unsigned long> ma_different;

                              tie (ma_unknown, ma_same, ma_different) = target.compareModeACodes(
                                  other, max_time_diff_, do_debug);

                              if (do_debug)
                              {
                                  loginf << "\ttarget " << target.utn_ << " other " << other.utn_
                                         << " ma unknown " << ma_unknown.size()
                                         << " same " << ma_same.size() << " diff " << ma_different.size();
                              }

                              if (ma_same.size() > ma_different.size()
                                  && ma_same.size() >= settings.target_min_updates_)
                              {
                                  if (do_debug)
                                      loginf << "\ttarget " << target.utn_ << " other " << other.utn_
                                             << " mode a check passed";

                                  // check mode c codes

                                  vector<unsigned long> mc_unknown;
                                  vector<unsigned long> mc_same;
                                  vector<unsigned long> mc_different;

                                  tie (mc_unknown, mc_same, mc_different) = target.compareModeCCodes(
                                      other, ma_same, max_time_diff_, settings.max_altitude_diff_, do_debug);

                                  if (do_debug)
                                  {
                                      loginf << "\ttarget " << target.utn_ << " other " << other.utn_
                                             << " ma same " << ma_same.size() << " diff " << ma_different.size()
                                             << " mc same " << mc_same.size() << " diff " << mc_different.size();
                                  }

                                  if (mc_same.size() > mc_different.size()
                                      && mc_same.size() >= settings.target_min_updates_)
                                  {
                                      if (do_debug)
                                          loginf << "\ttarget " << target.utn_ << " other " << other.utn_
                                                 << " mode c check passed";

                                      // check positions
                                      scoreUTN(target, mc_same, other, results[cnt], &prediction_stats[cnt], true, do_debug);
                                  }
                                  else if (!mc_different.size())
                                  {
                                      if (do_debug)
                                          loginf << "\ttarget " << target.utn_ << " other " << other.utn_
                                                 << " mode c check failed, checking on position only";

                                      // check based on pos only
                                      scoreUTN(target, target.target_reports_, other, results[cnt], &prediction_stats[cnt],false, do_debug);
                                  }
                              }
                              else if (!ma_different.size())
                              {
                                  if (do_debug)
                                      loginf << "\ttarget " << target.utn_ << " other " << other.utn_
                                             << " mode a check failed, checking on position only";

                                  // check based on pos only
                                  scoreUTN(target, target.target_reports_, other, results[cnt], &prediction_stats[cnt],false, do_debug);
                              }
                              else
                                  if (do_debug)
                                      loginf << "\ttarget " << target.utn_ << " other " << other.utn_
                                             << " mode a check failed";
                          }
                          else
                          {
                              if (do_debug)
                                  loginf << "\ttarget " << target.utn_ << " other " << other.utn_ << " no overlap";
                          }
#ifdef FIND_UTN_FOR_TARGET_MT
                      });
#else
        }
#endif

    //log failed predictions
    for (const auto& s : prediction_stats)
        ReconstructorTarget::addPredictionToGlobalStats(s);

    bool do_debug = false; //debug_utns.count(utn);
    //std::vector<unsigned int> utns_to_merge;

    // TODO rework to 1?

    // bool best_found = false;
    // unsigned int best_other_utn {0};
    //unsigned int best_num_updates {0};
    //unsigned int best_score {0};


    float min_score = reconstructor().settings().targets_min_assoc_score_;

    //float score;
    for (auto& res_it : results) // usable, other utn, num updates, avg distance
    {
        do_debug = reconstructor().task().debugSettings().debug_association_
                   && (reconstructor().task().debugSettings().debugUTN(utn)
                       || reconstructor().task().debugSettings().debugUTN(res_it.other_utn_));

        if (!res_it.usable_ || res_it.score_ < min_score)
        {
            // if (do_debug)
            //     loginf << "\ttarget " << target.utn_ << " other " << other_utn << " not usable";

            continue;
        }

        // score = res_it.num_updates_ / res_it.avg_distance_;

        // if (res_it.associate_based_on_secondary_attributes_)
        //     score *= 5;

        // if (do_debug)
            loginf << "\ttarget " << target.utn_ << " other " << res_it.other_utn_
                   << " assoc_sec " << res_it.associate_based_on_secondary_attributes_
                   << " avg dist " << String::doubleToStringPrecision(res_it.avg_distance_, 2)
                   << " score " << String::doubleToStringPrecision(res_it.score_, 2);

        // if (res_it.associate_based_on_secondary_attributes_ || res_it.avg_distance_ <= 0)
        // {
        //     if (do_debug)
        //         loginf << "\ttarget " << target.utn_ << " other " << res_it.other_utn_
        //                << " merging, based on sec.at. " << res_it.associate_based_on_secondary_attributes_
        //                << " avg dist " << String::doubleToStringPrecision(res_it.avg_distance_, 2);

            ret_options.push_back(res_it);

            // if (best_found)
            // {
            //     if (best_score < score)
            //     {
            //         best_other_utn = res_it.other_utn_;
            //         best_score = score;
            //     }
            // }
            // else
            // {
            //     best_other_utn = res_it.other_utn_;
            //     best_score = score;
            // }

            // best_found = true;
        }
    //}

    if (ret_options.size())
        loginf << "utn " << utn << " found " << ret_options.size() << " options";

    return ret_options;

    // if (best_found)
    //     return std::pair<float, std::pair<unsigned int, unsigned int>>{score, {utn,best_other_utn}};
    // else
    //     return std::pair<float, std::pair<unsigned int, unsigned int>>{std::numeric_limits<float>::lowest(), {0,0}};
}

 //computes a match score for the given other target
void ReconstructorAssociatorBase::scoreUTN(const dbContent::ReconstructorTarget& target, const std::vector<size_t>& rec_nums,
                                           const dbContent::ReconstructorTarget& other,
                                           AssociationOption& result_ref, reconstruction::PredictionStats* stats,
                                           bool secondary_verified, bool do_debug)
{
    const auto& settings = reconstructor().settings();

    traced_assert(stats);

    vector<pair<unsigned long, double>> distance_scores;
    double distance_scores_sum{0};

    unsigned int pos_not_ok_cnt{0};
    unsigned int pos_dubious_cnt{0};
    unsigned int pos_good_cnt{0};
    unsigned int pos_skipped_cnt{0};

    double distance_m, stddev_sum_targets;
    float pos_not_ok_rate, pos_dubious_rate;
    bool pos_not_ok_rate_acceptable, pos_dubious_rate_acceptable;

    if (do_debug)
        loginf << "\ttarget " << target.utn_ << " other utn " << other.utn_ << " rec_nums "
               << rec_nums.size();

    for (auto rn_it : rec_nums)
    {
        traced_assert(reconstructor().target_reports_.count(rn_it));

        const dbContent::targetReport::ReconstructorInfo& tr =
            reconstructor().target_reports_.at(rn_it);

        if (tr.doNotUsePosition() || !canGetPositionOffsetTargets(tr.timestamp_, target, other))
        {
            ++pos_skipped_cnt;
            continue;
        }

        //@TODO: debug flag
        auto pos_offs = getPositionOffsetTargets(tr.timestamp_, target, other, false, {}, stats);
        if (!pos_offs.has_value())
        {
            ++pos_skipped_cnt;
            continue;
        }

        tie(distance_m, stddev_sum_targets) = pos_offs.value();

        // double sum_stddev_est = stddev_est_target + stddev_est_other;
        ReconstructorAssociatorBase::DistanceClassification score_class;
        double distance_score;

        std::tie(score_class, distance_score) =
            checkPositionOffsetScore(distance_m, stddev_sum_targets, secondary_verified);

        // distance_score is supposed to be positve, the higher the better

        if (score_class == DistanceClassification::Distance_Dubious)
            ++pos_dubious_cnt;
        else if (score_class == DistanceClassification::Distance_Good)
            ++pos_good_cnt;
        else if (score_class == DistanceClassification::Distance_NotOK)
            ++pos_not_ok_cnt;

        // loginf << "\tdist " << distance;

        distance_scores.push_back({rn_it, distance_score});
        distance_scores_sum += distance_score;
    }

    if (pos_good_cnt)
    {
        pos_not_ok_rate = (float)pos_not_ok_cnt / (float)pos_good_cnt;
        pos_dubious_rate = (float)pos_dubious_cnt / (float)pos_good_cnt;
    }
    else
    {
        pos_dubious_rate = 0;
        pos_not_ok_rate = 0;
    }

    if (do_debug)
        loginf << "\ttarget " << target.utn_ << " other utn " << other.utn_ << " pos_not_ok_rate "
               << pos_not_ok_rate << " pos_not_ok_cnt " << pos_not_ok_cnt << " pos_dubious_rate "
               << pos_dubious_rate << " pos_dubious_cnt " << pos_dubious_cnt << " pos_good_cnt "
               << pos_good_cnt;

    pos_not_ok_rate_acceptable =
        secondary_verified ? pos_not_ok_rate < settings.target_max_positions_not_ok_verified_rate_
                           : pos_not_ok_rate < settings.target_max_positions_not_ok_unknown_rate_;

    pos_dubious_rate_acceptable =
        secondary_verified ? pos_dubious_rate < settings.target_max_positions_dubious_verified_rate_
                           : pos_dubious_rate < settings.target_max_positions_dubious_unknown_rate_;

    if (do_debug)
        loginf << "\ttarget " << target.utn_ << " other utn " << other.utn_ << " pos_good_cnt "
               << pos_good_cnt << " distance_scores_sum " << distance_scores_sum
               << " pos_not_ok_rate_acceptable " << pos_not_ok_rate_acceptable
               << " pos_dubious_rate_acceptable " << pos_dubious_rate_acceptable
               << " (distance_scores " << distance_scores.size() << " < target_min_updates "
               << settings.target_min_updates_ << ") "
               << (distance_scores.size() >= settings.target_min_updates_);

    if (pos_good_cnt && distance_scores_sum > 0 && pos_not_ok_rate_acceptable &&
        pos_dubious_rate_acceptable && distance_scores.size() >= settings.target_min_updates_)
    {
        double distance_score_avg = distance_scores_sum / (float)distance_scores.size();

        if (do_debug)
            loginf << "\ttarget " << target.utn_ << " other " << other.utn_ 
                   << " distance_score_avg " << distance_score_avg << " num "
                   << distance_scores.size();

        result_ref = AssociationOption(true, target.utn_, other.utn_, distance_scores.size(),
                                                secondary_verified, distance_score_avg);
    }
    else
    {
        if (do_debug)
            loginf << "\ttarget " << target.utn_ << " other " << other.utn_
                   << " same distances failed " << distance_scores.size() << " < "
                   << settings.target_min_updates_;
    }
}

const std::map<unsigned int, std::map<unsigned int,
                                      std::pair<unsigned int, unsigned int>>>& ReconstructorAssociatorBase::assocAounts() const
{
    return assoc_counts_;
}

const std::vector<unsigned long>& ReconstructorAssociatorBase::unassociatedRecNums() const
{
    return unassoc_rec_nums_;
}
