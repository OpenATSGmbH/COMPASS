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

#include "createartasassociationsjob.h"
#include "compass.h"
#include "buffer.h"
#include "createartasassociationstask.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
//#include "dbcontent/variable/metavariable.h"
#include "stringconv.h"
#include "util/timeconv.h"
#include "util/number.h"

#include "json.hpp"

#include <QThread>
#include <QCoreApplication>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <cmath>
#include <algorithm>

using namespace Utils;
using namespace std;
using namespace nlohmann;

CreateARTASAssociationsJob::CreateARTASAssociationsJob(
        CreateARTASAssociationsTask& task, DBInterface& db_interface,
        map<string, shared_ptr<Buffer>> buffers)
    : Job("CreateARTASAssociationsJob"), task_(task), db_interface_(db_interface), buffers_(buffers)
{
    misses_acceptable_time_ = Time::partialSeconds(task_.missesAcceptableTime());

    association_time_past_ = Time::partialSeconds(task_.associationTimePast());
    association_time_future_ = Time::partialSeconds(task_.associationTimeFuture());  // will be negative time diff

    associations_dubious_distant_time_ = Time::partialSeconds(task_.associationsDubiousDistantTime());
    association_dubious_close_time_past_ = Time::partialSeconds(task_.associationDubiousCloseTimePast());
    association_dubious_close_time_future_ =
            Time::partialSeconds(task_.associationDubiousCloseTimeFuture());  // will be negative time diff
}

CreateARTASAssociationsJob::~CreateARTASAssociationsJob() {}

void CreateARTASAssociationsJob::run_impl()
{
    logdbg;

    started_ = true;

    boost::posix_time::ptime start_time;
    boost::posix_time::ptime stop_time;

    start_time = boost::posix_time::microsec_clock::local_time();

    COMPASS::instance().logInfo("Create ARTAS Associations") << "started";

    // clear previous associations

    loginf << "clearing associations";

    emit statusSignal("Clearing Previous ARTAS Target Report Usage");
    removePreviousAssociations();

    // create utns
    emit statusSignal("Creating Unique ARTAS Tracks");
    createUniqueARTASTracks();

    // create associations for artas tracks
    emit statusSignal("Calculate ARTAS Target Report Usage");
    createARTASAssociations();

    // create associations for sensors
    createSensorAssociations();

    COMPASS::instance().logInfo("Create ARTAS Associations")
        << missing_hashes_cnt_ << " missing hashes and "
        << dubious_associations_cnt_ << " dubious associations ("
        << found_hashes_cnt_ << " found total";

    if (task_.allowUserInteractions() && (missing_hashes_cnt_ || dubious_associations_cnt_))
    {
        stringstream ss;
        ss << "There are " << missing_hashes_cnt_ << " missing hashes and "
           << dubious_associations_cnt_ << " dubious associations ("
           << found_hashes_cnt_ << " found total).\nDo you want to still save the associations?";

        emit saveAssociationsQuestionSignal(ss.str().c_str());

        while (!save_question_answered_)
            QThread::msleep(1);

        if (!save_question_answer_)  // nope
        {
            stop_time = boost::posix_time::microsec_clock::local_time();

            COMPASS::instance().logInfo("Create ARTAS Associations") << "save declined";

            double load_time;
            boost::posix_time::time_duration diff = stop_time - start_time;
            load_time = diff.total_milliseconds() / 1000.0;

            loginf << "done ("
                   << String::doubleToStringPrecision(load_time, 2) << " s).";
            done_ = true;

            return;
        }
        // else simply continue
    }

    // save associations
    emit statusSignal("Saving ARTAS Target Report Usage");

    saveAssociations();

    //    object_man.setAssociationsDataSource(tracker_dbcontent_name_, task_.currentDataSourceName());

    stop_time = boost::posix_time::microsec_clock::local_time();

    double load_time;
    boost::posix_time::time_duration diff = stop_time - start_time;
    load_time = diff.total_milliseconds() / 1000.0;

    loginf << "done ("
           << String::doubleToStringPrecision(load_time, 2) << " s).";

    COMPASS::instance().logInfo("Create ARTAS Associations")
        << "finished after "
        << String::timeStringFromDouble(diff.total_milliseconds() / 1000.0, false);

    done_ = true;
}

size_t CreateARTASAssociationsJob::missingHashes() const { return missing_hashes_cnt_; }

size_t CreateARTASAssociationsJob::foundHashes() const { return found_hashes_cnt_; }

size_t CreateARTASAssociationsJob::foundHashDuplicates() const
{
    return found_hash_duplicates_cnt_;
}

size_t CreateARTASAssociationsJob::dubiousAssociations() const { return dubious_associations_cnt_; }

size_t CreateARTASAssociationsJob::missingHashesAtBeginning() const
{
    return acceptable_missing_hashes_cnt_;
}

void CreateARTASAssociationsJob::createUniqueARTASTracks()
{
    loginf;

    if (!buffers_.count(tracker_dbcontent_name_))
    {
        logwrn << "no tracker data found";
        return;
    }

    shared_ptr<Buffer> buffer = buffers_.at(tracker_dbcontent_name_);
    size_t buffer_size = buffer->size();

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    traced_assert(buffer->has<unsigned int>(
               dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_track_num_).name()));
    traced_assert(buffer->has<bool>(
               dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_track_begin_).name()));
    traced_assert(buffer->has<bool>(
               dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_track_end_).name()));
    traced_assert(buffer->has<unsigned char>(
               dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_track_coasting_).name()));
    traced_assert(buffer->has<string>(dbcont_man.getVariable(tracker_dbcontent_name_, DBContent::var_cat062_tris_).name()));

    traced_assert(buffer->has<unsigned long>(
               dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_rec_num_).name()));
    traced_assert(buffer->has<boost::posix_time::ptime>(
               dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_timestamp_).name()));

    NullableVector<unsigned int>& track_num_vec = buffer->get<unsigned int>(
                dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_track_num_).name());
    NullableVector<bool>& track_begin_vec = buffer->get<bool>(
                dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_track_begin_).name());
    NullableVector<bool>& track_end_vec = buffer->get<bool>(
                dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_track_end_).name());
    NullableVector<unsigned char>& track_coasting_vec = buffer->get<unsigned char>(
                dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_track_coasting_).name());
    NullableVector<string>& tri_hash_vec = buffer->get<string>(
                dbcont_man.getVariable(tracker_dbcontent_name_, DBContent::var_cat062_tris_).name());

    NullableVector<unsigned long>& rec_num_vec = buffer->get<unsigned long>(
                dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_rec_num_).name());
    NullableVector<boost::posix_time::ptime>& ts_vec = buffer->get<boost::posix_time::ptime>(
                dbcont_man.metaGetVariable(tracker_dbcontent_name_, DBContent::meta_var_timestamp_).name());

    map<int, UniqueARTASTrack> current_tracks;  // utn -> unique track
    map<int, int> current_track_mappings;       // track_num -> utn

    int utn_cnt{0};

    bool tri_set;
    string tri;
    int track_num;
    bool track_begin_set;
    bool track_begin;
    bool track_end_set;
    bool track_end;
    bool track_coasting_set;
    bool track_coasting;

    bool ignore_track_end_associations = task_.ignoreTrackEndAssociations();
    bool ignore_track_coasting_associations = task_.ignoreTrackCoastingAssociations();

    unsigned long rec_num;
    boost::posix_time::ptime timestamp;

    int utn;
    // bool new_track_created;
    bool finish_previous_track;
    bool ignore_update;

    boost::posix_time::time_duration track_end_time = Time::partialSeconds(task_.endTrackTime());
    // time-delta after which begin a new track

    for (size_t cnt = 0; cnt < buffer_size; ++cnt)
    {
        // new_track_created = false;
        finish_previous_track = false;

        tri_set = !tri_hash_vec.isNull(cnt);
        if (tri_set)
            tri = tri_hash_vec.get(cnt);
        else
            tri = "";

        traced_assert(!track_num_vec.isNull(cnt));
        track_num = track_num_vec.get(cnt);

        track_begin_set = !track_begin_vec.isNull(cnt);
        if (track_begin_set)
            track_begin = track_begin_vec.get(cnt);
        else
            track_begin = false;

        track_end_set = !track_end_vec.isNull(cnt);
        if (track_end_set)
            track_end = track_end_vec.get(cnt);
        else
            track_end = false;

        track_coasting_set = !track_coasting_vec.isNull(cnt);
        if (track_coasting_set)
            track_coasting = track_coasting_vec.get(cnt) != 0;
        else
            track_coasting = false;

        traced_assert(!rec_num_vec.isNull(cnt));
        rec_num = rec_num_vec.get(cnt);

        traced_assert(!ts_vec.isNull(cnt));
        timestamp = ts_vec.get(cnt);

        // was loaded as sorted in time
        if (cnt == 0)  // store first time
            first_track_ts_ = timestamp;

        last_track_ts_ = timestamp;  // store last time

        if (track_begin_set && track_begin && current_track_mappings.count(track_num))
        {
            logdbg << "finalizing track utn " << utn
                   << " track end is set";

            finish_previous_track = true;
        }

        // check if existed but last update was more than 10m ago
        if (!finish_previous_track && current_track_mappings.count(track_num) &&
                timestamp - current_tracks.at(current_track_mappings.at(track_num)).last_ts_ >
                track_end_time)
        {
            logdbg << "finalizing track utn "
                   << current_track_mappings.at(track_num) << " since tod difference "
                   << timestamp - current_tracks.at(current_track_mappings.at(track_num)).last_ts_;

            finish_previous_track = true;
        }

        if (finish_previous_track)
        {
            // finalize old track
            utn = current_track_mappings.at(track_num);
            traced_assert(!finished_tracks_.count(utn));
            finished_tracks_[utn] = current_tracks.at(utn);
            current_tracks.erase(utn);
            current_track_mappings.erase(track_num);
        }

        if (!current_track_mappings.count(track_num))  // new track where none existed
        {
            logdbg << "new track utn " << utn_cnt
                   << " track num " << track_num << " tod " << Time::toString(timestamp)
                   << " begin " << (track_begin_set ? to_string(track_begin) : " not set");

            utn = utn_cnt;
            ++utn_cnt;

            current_track_mappings[track_num] = utn;

            current_tracks[utn].utn = utn;
            current_tracks[utn].track_num = track_num;
            current_tracks[utn].first_ts_ = timestamp;

            // new_track_created = true;
        }
        else
            utn = current_track_mappings.at(track_num);

        traced_assert(current_tracks.count(utn));
        UniqueARTASTrack& unique_track = current_tracks.at(utn);
        unique_track.last_ts_ = timestamp;

        // add tris if not to be ignored
        ignore_update =
                ((track_end_set && track_end && ignore_track_end_associations) ||
                 (track_coasting_set && track_coasting && ignore_track_coasting_associations));

        if (ignore_update)
        {
            logdbg << "ignoring rec num " << rec_num;
            // add empty tri so that at least track update is associated
            unique_track.rec_nums_tris_[rec_num] = make_pair("", timestamp);
            ++ignored_track_updates_cnt_;
        }
        else
            unique_track.rec_nums_tris_[rec_num] = make_pair(tri, timestamp);

        if (track_end_set && track_end)
        {
            logdbg << "finalizing track utn " << utn
                   << " since track end is set";

            // finalize old track
            traced_assert(!finished_tracks_.count(utn));
            finished_tracks_[utn] = current_tracks.at(utn);
            current_tracks.erase(utn);
            current_track_mappings.erase(track_num);
        }
    }

    // finish all and clean up
    for (auto& ut_it : current_tracks)
    {
        traced_assert(!finished_tracks_.count(ut_it.first));
        finished_tracks_[ut_it.first] = ut_it.second;
    }

    current_tracks.clear();
    current_track_mappings.clear();

    loginf << "found " << finished_tracks_.size()
           << " finished tracks ";
}

void CreateARTASAssociationsJob::createARTASAssociations()
{
    loginf;

    // set utns in tracker rec_nums

    for (auto& ut_it : finished_tracks_)                    // utn -> unique track
        for (auto& assoc_it : ut_it.second.rec_nums_tris_)  // rec_num -> (tri, timestamp)
            cat062_associations_[assoc_it.first] = std::vector<unsigned long>();
    // dbcontent -> cat062 rec_num -> <src rec_nums>
}

void CreateARTASAssociationsJob::saveAssociations()
{
    loginf;

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    // write association info to buffers

    unsigned long rec_num;

    unsigned int num_associated {0};
    unsigned int num_not_associated {0};

    //string dbcontent_name = cont_assoc_it.first;
    //std::map<unsigned long, std::vector<unsigned long>>& associations = cont_assoc_it.second;
    //cat062 rec_num -> <rec_num>

    traced_assert(buffers_.count(tracker_dbcontent_name_));

    traced_assert(dbcontent_man.metaCanGetVariable(tracker_dbcontent_name_, DBContent::meta_var_rec_num_));
    traced_assert(dbcontent_man.canGetVariable(tracker_dbcontent_name_, DBContent::var_cat062_tri_recnums_));

    dbContent::Variable& rec_num_var = dbcontent_man.metaGetVariable(
                tracker_dbcontent_name_, DBContent::meta_var_rec_num_);

    dbContent::Variable& tri_rec_num_var = dbcontent_man.getVariable(
                tracker_dbcontent_name_, DBContent::var_cat062_tri_recnums_);

    string rec_num_var_name = rec_num_var.name();
    string tri_rec_num_var_name = tri_rec_num_var.name();

    traced_assert(buffers_.at(tracker_dbcontent_name_)->has<unsigned long>(rec_num_var_name));
    traced_assert(buffers_.at(tracker_dbcontent_name_)->has<json>(tri_rec_num_var_name));

    NullableVector<unsigned long>& rec_num_vec =
            buffers_.at(tracker_dbcontent_name_)->get<unsigned long>(rec_num_var_name);
    NullableVector<json>& tri_rec_num_vec = buffers_.at(tracker_dbcontent_name_)->get<json>(tri_rec_num_var_name);

    for (unsigned int cnt=0; cnt < buffers_.at(tracker_dbcontent_name_)->size(); ++cnt)
    {
        traced_assert(!rec_num_vec.isNull(cnt));

        rec_num = rec_num_vec.get(cnt);

        if (cat062_associations_.count(rec_num))
        {
            tri_rec_num_vec.set(cnt, cat062_associations_.at(rec_num));

            // do association counts
            for (auto assoc_rec_num : cat062_associations_.at(rec_num))
            {
                const string& dbcont_name = dbcontent_man.dbContentWithId(Number::recNumGetDBContId(assoc_rec_num));

                if (!association_counts_.count(dbcont_name) && buffers_.count(dbcont_name))
                    association_counts_[dbcont_name] = {buffers_.at(dbcont_name)->size(),0};

                get<1>(association_counts_[dbcont_name])++;
            }

            ++num_associated;
        }
        else
            ++num_not_associated;
    }

    association_counts_[tracker_dbcontent_name_] = {buffers_.at(tracker_dbcontent_name_)->size(), num_associated};

    loginf << "dcontent " << tracker_dbcontent_name_
           <<  " assoc " << num_associated << " not assoc " << num_not_associated;

    auto buffer = buffers_.at(tracker_dbcontent_name_);
    buffers_.clear();

    // delete all data from buffer except rec_nums and associations, rename to db column names

    string rec_num_col_name = rec_num_var.dbColumnName();
    string tri_rec_num_col_name = tri_rec_num_var.dbColumnName();

    PropertyList properties = buffer->properties();

    for (auto& prop_it : properties.properties())
    {
        if (prop_it.name() == rec_num_var_name)
            buffer->rename<unsigned long>(rec_num_var_name, rec_num_col_name);
        else if (prop_it.name() == tri_rec_num_var_name)
            buffer->rename<json>(tri_rec_num_var_name, tri_rec_num_col_name);
        else
            buffer->deleteProperty(prop_it);
    }

    // actually save data, ok since DB job


    loginf << "saving for " << tracker_dbcontent_name_;

    DBContent& dbcontent = dbcontent_man.dbContent(tracker_dbcontent_name_);

    unsigned int chunk_size = 50000;

    unsigned int steps = buffer->size() / chunk_size;

    unsigned int index_from = 0;
    unsigned int index_to = 0;

    for (unsigned int cnt = 0; cnt <= steps; cnt++)
    {
        index_from = cnt * chunk_size;
        index_to = index_from + chunk_size;

        if (index_to > buffer->size() - 1)
            index_to = buffer->size() - 1;

        loginf << "step " << cnt << " steps " << steps << " from "
               << index_from << " to " << index_to;

        db_interface_.updateBuffer(dbcontent.dbTableName(), rec_num_col_name,
                                   buffer, index_from, index_to);

    }

    loginf << "done";
}

void CreateARTASAssociationsJob::createSensorAssociations()
{
    loginf;
    // for each rec_num + tri, find sensor hash + rec_num

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    for (auto& dbcont_it : dbcont_man)
    {
        if (dbcont_it.first != tracker_dbcontent_name_ && dbcont_it.second->hasData())
        {
            string status = "Creating " + dbcont_it.first + " Hash List";
            emit statusSignal(status.c_str());
            createSensorHashes(*dbcont_it.second);
        }
    }

    vector<string> tri_splits;
    bool match_found;
    bool best_match_dubious;
    string best_match_dubious_comment;

    //string best_match_dbcontent_name;
    unsigned long best_match_rec_num{0};
    boost::posix_time::ptime best_match_ts;
    boost::posix_time::ptime tri_ts;

    typedef multimap<string, pair<unsigned long, boost::posix_time::ptime>>::iterator HashIterator;
    pair<HashIterator, HashIterator> possible_hash_matches;

    traced_assert(!first_track_ts_.is_not_a_date_time());  // has to be set

    emit statusSignal("Creating Associations");

    for (auto& ut_it : finished_tracks_)  // utn -> UAT, for each unique target
    {
        logdbg << "utn " << ut_it.first;

        for (auto& assoc_it : ut_it.second.rec_nums_tris_)  // rec_num -> (tri, tod), for each TRIs compound string
        {
            if (!assoc_it.second.first.size())  // empty tri, ignored update
                continue;

            tri_splits = String::split(assoc_it.second.first, ';');
            tri_ts = assoc_it.second.second;

            for (auto& tri : tri_splits)  // for each referenced hash
            {
                match_found = false;  // indicates if there was already a (previous) match found
                best_match_dubious = false;  // indicates if the association is dubious
                best_match_dubious_comment = "";

                possible_hash_matches = sensor_hashes_.equal_range(tri);

                for (HashIterator it = possible_hash_matches.first;
                     it != possible_hash_matches.second; ++it)
                {
                    pair<unsigned long, boost::posix_time::ptime>& match = it->second;  // rec_num, tod

                    if (!isPossibleAssociation(tri_ts, match.second))
                        continue;

                    if (match_found)
                    {
                        logdbg << "start"
                                  "found duplicate hash '"
                               << tri
                               << "' in dbcont " << dbcont_man.dbContentWithId(Number::recNumGetDBContId(match.first))
                               << " rec num " << match.first;

                        if (isAssociationHashCollisionInDubiousTime(tri_ts,
                                                                    best_match_ts) &&
                                isAssociationHashCollisionInDubiousTime(tri_ts, match.second))
                        {
                            best_match_dubious = true;
                            best_match_dubious_comment =
                                    tri + " has multiple matches in close time at " +
                                    Time::toString(tri_ts);
                        }
                        else  // not dubious
                        {
                            best_match_dubious = false;
                            best_match_dubious_comment = "";
                        }

                        // store if closer in time
                        if ((tri_ts - match.second).abs() < (tri_ts - best_match_ts).abs())
                        {
                            if (isAssociationInDubiousDistantTime(tri_ts, match.second))
                            {
                                best_match_dubious = true;
                                best_match_dubious_comment =
                                        tri + " in too distant time (" +
                                        Time::toString(tri_ts - match.second) + ") at " +
                                        Time::toString(tri_ts);
                            }
                            else  // not dubious
                            {
                                best_match_dubious = false;
                                best_match_dubious_comment = "";
                            }

                            //                            best_match_dbcontent_name = dbcont_man.dbContentWithId(
                            //                                        Number::recNumGetDBContId(match.first));
                            best_match_rec_num = match.first;  // rec_num
                            best_match_ts = match.second;     // tod
                            match_found = true;
                        }

                        found_hash_duplicates_cnt_++;
                    }
                    else  // store as best match
                    {
                        if (isAssociationInDubiousDistantTime(tri_ts, match.second))
                        {
                            best_match_dubious = true;
                            best_match_dubious_comment =
                                    tri + " in too distant time (" +
                                    Time::toString(tri_ts - match.second) + "s) at " +
                                    Time::toString(tri_ts);
                        }

                        //best_match_dbcontent_name = dbcont_man.dbContentWithId(Number::recNumGetDBContId(match.first));
                        best_match_rec_num = match.first;  // rec_num
                        best_match_ts = match.second;     // tod
                        match_found = true;
                    }
                }

                if (match_found)
                {
                    if (best_match_dubious)
                    {
                        loginf << "utn "
                               << ut_it.first << " match rec_num " << best_match_rec_num
                               << " is dubious because " << best_match_dubious_comment;
                        ++dubious_associations_cnt_;
                    }

                    //                    object_man.object(best_match_dbcontent_name)
                    //                        .addAssociation(best_match_rec_num, ut_it.first, true, assoc_it.first);

                    // add utn to non-tracker rec_num
                    cat062_associations_[best_match_rec_num] = std::vector<unsigned long>();

                    // add non-tracker rec_num to tracker src rec_nums

                    traced_assert(cat062_associations_.count(assoc_it.first));

                    cat062_associations_.at(assoc_it.first).push_back(best_match_rec_num);

                    ++found_hashes_cnt_;
                }
                else
                {
                    logdbg << "utn "
                           << ut_it.first << " has missing hash '" << tri << "' at "
                           << Time::toString(tri_ts);

                    // traced_assert(tri_tod-first_track_tod_ >= 0);

                    if (isTimeAtBeginningOrEnd(tri_ts))
                        ++acceptable_missing_hashes_cnt_;
                    else
                    {
                        loginf << "utn "
                               << ut_it.first << " has missing hash '" << tri << "' at "
                               << Time::toString(tri_ts);

                        missing_hashes_.emplace(tri, make_pair(ut_it.first, assoc_it.first));
                        ++missing_hashes_cnt_;
                    }
                }
            }
        }
    }

    loginf << "done with "
           << found_hashes_cnt_ << " found, " << acceptable_missing_hashes_cnt_
           << " missing at beginning, " << missing_hashes_cnt_ << " missing, "
           << found_hash_duplicates_cnt_ << " duplicates";
}

void CreateARTASAssociationsJob::removePreviousAssociations()
{
    loginf;

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    if (buffers_.count(tracker_dbcontent_name_))
    {
        traced_assert(dbcontent_man.canGetVariable(tracker_dbcontent_name_, DBContent::var_cat062_tri_recnums_));

        string tri_rec_num_var_name =
                dbcontent_man.getVariable(tracker_dbcontent_name_, DBContent::var_cat062_tri_recnums_).name();

        NullableVector<json>& tri_rec_num_vec = buffers_.at(tracker_dbcontent_name_)->get<json>(tri_rec_num_var_name);

        tri_rec_num_vec.setAllNull();
    }
}

bool CreateARTASAssociationsJob::isPossibleAssociation(boost::posix_time::ptime ts_track,
                                                       boost::posix_time::ptime ts_target)
{
    if (ts_target > ts_track)  // target update in the future
        return ts_target - ts_track <= association_time_future_;
    else  // target update in the past (or same time)
        return ts_track - ts_target <= association_time_past_;
}

bool CreateARTASAssociationsJob::isAssociationInDubiousDistantTime(boost::posix_time::ptime ts_track,
                                                                   boost::posix_time::ptime ts_target)
{
    if (ts_target > ts_track)  // target update in the future
        return false;            // only measured in the past
    else                         // target update in the past (or same time)
        return ts_track - ts_target >= associations_dubious_distant_time_;
}

bool CreateARTASAssociationsJob::isAssociationHashCollisionInDubiousTime(boost::posix_time::ptime ts_track,
                                                                         boost::posix_time::ptime ts_target)
{
    if (ts_target > ts_track)  // target update in the future
        return ts_target - ts_track <= association_dubious_close_time_future_;
    else  // target update in the past (or same time)
        return ts_track - ts_target <= association_dubious_close_time_past_;
}

bool CreateARTASAssociationsJob::isTimeAtBeginningOrEnd(boost::posix_time::ptime ts_track)
{
    return ((ts_track - first_track_ts_).abs() <= misses_acceptable_time_) ||
            ((last_track_ts_ - ts_track).abs() <= misses_acceptable_time_);
}

void CreateARTASAssociationsJob::createSensorHashes(DBContent& object)
{
    loginf << "object " << object.name();

    if (!buffers_.count(object.name()))
    {
        logwrn << "no data found";
        return;
    }

    string dbcontent_name = object.name();
    shared_ptr<Buffer> buffer = buffers_.at(dbcontent_name);
    size_t buffer_size = buffer->size();

    using namespace dbContent;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    Variable& rec_num_var = dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_rec_num_);
    Variable& hash_var = dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_artas_hash_);
    Variable& ts_var = dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_);

    traced_assert(buffer->has<unsigned long>(rec_num_var.name()));
    traced_assert(buffer->has<string>(hash_var.name()));
    traced_assert(buffer->has<boost::posix_time::ptime>(ts_var.name()));

    NullableVector<unsigned long>& rec_nums = buffer->get<unsigned long>(rec_num_var.name());
    NullableVector<string>& hashes = buffer->get<string>(hash_var.name());
    NullableVector<boost::posix_time::ptime>& ts_vec = buffer->get<boost::posix_time::ptime>(ts_var.name());

    for (size_t cnt = 0; cnt < buffer_size; ++cnt)
    {
        traced_assert(!rec_nums.isNull(cnt));
        traced_assert(!hashes.isNull(cnt));

        if (ts_vec.isNull(cnt))
        {
            logwrn << "rec_num "
                   << rec_nums.get(cnt) << " of dbcont " << object.name() << " has no time, skipping";
            continue;
        }

        traced_assert(!ts_vec.isNull(cnt));

        // hash -> rec_num, tod
        // map <string, multimap<string, pair<int, float>>> sensor_hashes_;

        sensor_hashes_.emplace(hashes.get(cnt), make_pair(rec_nums.get(cnt), ts_vec.get(cnt)));
    }
}

std::map<std::string, std::pair<unsigned int, unsigned int> > CreateARTASAssociationsJob::associationCounts() const
{
    return association_counts_;
}

void CreateARTASAssociationsJob::setSaveQuestionAnswer(bool value)
{
    loginf << "value " << value;

    save_question_answer_ = value;
    save_question_answered_ = true;
}
