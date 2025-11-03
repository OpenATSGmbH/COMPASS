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

#include "job.h"

#include "boost/date_time/posix_time/ptime.hpp"
//#include "boost/date_time/posix_time/posix_time_duration.hpp"

class CreateARTASAssociationsTask;
class DBInterface;
class Buffer;
class DBContent;

struct UniqueARTASTrack
{
    int utn;
    int track_num;
    std::map<unsigned long, std::pair<std::string, boost::posix_time::ptime>> rec_nums_tris_;  // rec_num -> (tri, timestamp)
    // std::vector<int> track_nums_;
    boost::posix_time::ptime first_ts_;
    boost::posix_time::ptime last_ts_;
};

class CreateARTASAssociationsJob : public Job
{
    Q_OBJECT

signals:
    void statusSignal(QString status);
    void saveAssociationsQuestionSignal(QString question);

public:
    CreateARTASAssociationsJob(CreateARTASAssociationsTask& task, DBInterface& db_interface,
                               std::map<std::string, std::shared_ptr<Buffer>> buffers);

    virtual ~CreateARTASAssociationsJob();

    void setSaveQuestionAnswer(bool value);

    size_t missingHashesAtBeginning() const;
    size_t missingHashes() const;
    size_t foundHashes() const;
    size_t foundHashDuplicates() const;
    size_t dubiousAssociations() const;

    std::map<std::string, std::pair<unsigned int, unsigned int> > associationCounts() const;

protected:
    CreateARTASAssociationsTask& task_;
    DBInterface& db_interface_;
    std::map<std::string, std::shared_ptr<Buffer>> buffers_;

    boost::posix_time::time_duration misses_acceptable_time_;  // time delta at beginning/end of recording where misses are acceptable

    boost::posix_time::time_duration association_time_past_;  // time_delta for which associations are considered into past time
    boost::posix_time::time_duration association_time_future_;  // time_delta for which associations are considered into future time

    boost::posix_time::time_duration associations_dubious_distant_time_;
    // time delta of tou where association is dubious bc too distant in time
    boost::posix_time::time_duration association_dubious_close_time_past_;
    // time delta of tou where association is dubious when multible hashes exist
    boost::posix_time::time_duration association_dubious_close_time_future_;
    // time delta of tou where association is dubious when multible hashes exist

    const std::string tracker_dbcontent_name_{"CAT062"};
    const std::string associations_src_name_{"ARTAS"};
    std::map<int, UniqueARTASTrack> finished_tracks_;  // utn -> unique track

    // hash -> rec_num, timestamp
    std::multimap<std::string, std::pair<unsigned long, boost::posix_time::ptime>> sensor_hashes_;

//    std::map<std::string,
//        std::map<unsigned long,
//            std::tuple<unsigned int, std::vector<std::pair<std::string, unsigned long>>>>> associations_;
    // dbcontent -> rec_num -> <utn, src rec_nums (dbcontent, rec_num)>

    std::map<unsigned long, std::vector<unsigned long>> cat062_associations_;
    // cat062 rec_num -> < src rec_nums>

    boost::posix_time::ptime first_track_ts_;
    boost::posix_time::ptime last_track_ts_;

    size_t ignored_track_updates_cnt_{0};
    size_t acceptable_missing_hashes_cnt_{0};
    size_t missing_hashes_cnt_{0};
    std::multimap<std::string, std::pair<int, unsigned long>> missing_hashes_;  // hash -> (utn, rec_num)
    size_t found_hashes_cnt_{0};                                      // dbcont name -> cnt

    size_t dubious_associations_cnt_{0};  // counter for all dubious
    size_t found_hash_duplicates_cnt_{0};

    volatile bool save_question_answered_{false};
    volatile bool save_question_answer_{false};

    void createUniqueARTASTracks();
    void createARTASAssociations();
    void createSensorAssociations();
    void saveAssociations();

    void createSensorHashes(DBContent& object);

    std::map<unsigned int, unsigned int> track_rec_num_utns_;  // track rec num -> utn

    std::map<std::string, std::pair<unsigned int,unsigned int>> association_counts_; // dbcontent -> total, assoc cnt

    void removePreviousAssociations();

    bool isPossibleAssociation(boost::posix_time::ptime ts_track, boost::posix_time::ptime ts_target);
    bool isAssociationInDubiousDistantTime(boost::posix_time::ptime ts_track, boost::posix_time::ptime ts_target);
    bool isAssociationHashCollisionInDubiousTime(boost::posix_time::ptime ts_track, boost::posix_time::ptime ts_target);
    bool isTimeAtBeginningOrEnd(boost::posix_time::ptime ts_track);

    void run_impl() override;
};

