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

#include "asterixtimestampcalculator.h"
#include "compass.h"
#include "buffer.h"
#include "dbcontentmanager.h"
#include "dbcontent.h"
#include "metavariable.h"
#include "util/stringconv.h"
#include "util/timeconv.h"
#include "logger.h"

using namespace std;
using namespace Utils;

const float tod_24h = 24 * 60 * 60;
const float close_to_midgnight_offset_s = 300; // seconds
const float not_close_to_midgnight_offset_s = 600; // seconds

ASTERIXTimestampCalculator::ASTERIXTimestampCalculator()
{
}

void ASTERIXTimestampCalculator::setBuffers(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    traced_assert(!buffers_.size());
    traced_assert(!processing_);

    processing_ = true;

    buffers_ = std::move(buffers);
}

void ASTERIXTimestampCalculator::calculate(
               std::string source_name,
               boost::posix_time::ptime date, bool reset_date_between_files,
               bool override_tod_active, float override_tod_offset,
               bool ignore_time_jumps, bool do_timestamp_checks)
{
    logdbg;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    traced_assert(processing_);

    traced_assert(source_name.size());

    if (prev_source_name_ != source_name && source_name != "File ''") // new file
    {
        loginf << "current data source name changed, '"
               << prev_source_name_ << "' to '" << source_name << "'";

        if (reset_date_between_files)
        {
            loginf << "resetting date";
            resetDateInfo();
        }

        if (prev_source_name_.size()) // not for empty, first one
            logLastTimestamp();

        clearTimeStats();

        prev_source_name_ = source_name;

        COMPASS::instance().logInfo("ASTERIX Import") << "decoding " << source_name;
    }

    if (!current_date_set_) // init if first time
    {
        current_date_ = date;
        previous_date_ = current_date_;

        current_date_set_ = true;
    }

    doADSBTimeProcessing();

    if (override_tod_active)
        doTodOverride(override_tod_offset);

    if (do_timestamp_checks) // out of sync issue during 24h replay
        doFutureTimestampsCheck();

    doTimeStampCalculation(ignore_time_jumps);

    boost::posix_time::time_duration time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
    logdbg << "done after "
           << String::timeStringFromDouble(time_diff.total_milliseconds() / 1000.0, true);
}

std::map<std::string, std::shared_ptr<Buffer>> ASTERIXTimestampCalculator::buffers()
{
    logdbg;
    traced_assert(processing_);

    return std::move(buffers_);
}

void ASTERIXTimestampCalculator::doADSBTimeProcessing()
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    string dbcontent_name = "CAT021";

    if (!buffers_.count(dbcontent_name))
        return;

    shared_ptr<Buffer> buffer = buffers_.at(dbcontent_name);
    unsigned int buffer_size = buffer->size();

    if (!buffer_size)
        return;

    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_time_of_day_));

    traced_assert(dbcont_man.canGetVariable(dbcontent_name, DBContent::meta_var_time_of_day_));

    dbContent::Variable& tod_var = dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_time_of_day_);

    traced_assert(tod_var.dataType() == PropertyDataType::FLOAT);

    string tod_var_name = tod_var.name();

    if (!buffer->has<float>(tod_var_name))
        buffer->addProperty(tod_var_name, PropertyDataType::FLOAT); // add if needed

    NullableVector<float>& tod_vec = buffer->get<float>(tod_var_name);

    // check if other sources for tod exist

    // "ToA Position" 071.Time of Applicability for Position
    // "ToMR Position" 0.73 Time of Message Reception for Position
    // "ToRT" 077.Time of Report Transmission
    // "Time of Day Deprecated" 030.Time of Day

    NullableVector<float>* toa_position_vec {nullptr};
    NullableVector<float>* tomr_position_vec {nullptr};
    NullableVector<float>* tort_vec {nullptr};
    NullableVector<float>* tod_dep_vec {nullptr};

    if (buffer->has<float>(DBContent::var_cat021_toa_position_.name()))
        toa_position_vec = &buffer->get<float>(DBContent::var_cat021_toa_position_.name());

    if (buffer->has<float>(DBContent::var_cat021_tomr_position_.name()))
        tomr_position_vec = &buffer->get<float>(DBContent::var_cat021_tomr_position_.name());

    if (buffer->has<float>(DBContent::var_cat021_tort_.name()))
        tort_vec = &buffer->get<float>(DBContent::var_cat021_tort_.name());

    if (buffer->has<float>(DBContent::var_cat021_tod_dep_.name()))
        tod_dep_vec = &buffer->get<float>(DBContent::var_cat021_tod_dep_.name());

    for (unsigned int index=0; index < buffer_size; index++)
    {
        if (!tod_vec.isNull(index)) // no need to copy, should not happen
            continue;

        if (toa_position_vec && !toa_position_vec->isNull(index))
        {
            tod_vec.set(index, toa_position_vec->get(index));
            continue;
        }

        if (tomr_position_vec && !tomr_position_vec->isNull(index))
        {
            tod_vec.set(index, tomr_position_vec->get(index));
            continue;
        }

        if (tort_vec && !tort_vec->isNull(index))
        {
            tod_vec.set(index, tort_vec->get(index));
            continue;
        }

        if (tod_dep_vec && !tod_dep_vec->isNull(index))
        {
            tod_vec.set(index, tod_dep_vec->get(index));
            continue;
        }
    }
}

void ASTERIXTimestampCalculator::doTodOverride(float override_tod_offset)
{
    loginf << "offset "
           << String::doubleToStringPrecision(override_tod_offset, 3);

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    unsigned int buffer_size;

    for (auto& buf_it : buffers_)
    {
        buffer_size = buf_it.second->size();

        traced_assert(dbcont_man.metaVariable(DBContent::meta_var_time_of_day_.name()).existsIn(buf_it.first));

        dbContent::Variable& tod_var =
            dbcont_man.metaVariable(DBContent::meta_var_time_of_day_.name()).getFor(buf_it.first);

        Property tod_prop {tod_var.name(), tod_var.dataType()};

        traced_assert(buf_it.second->hasProperty(tod_prop));

        NullableVector<float>& tod_vec = buf_it.second->get<float>(tod_var.name());

        for (unsigned int index=0; index < buffer_size; ++index)
        {
            if (!tod_vec.isNull(index))
            {
                float& tod_ref = tod_vec.getRef(index);

                tod_ref += override_tod_offset;

                // check for out-of-bounds because of midnight-jump
                while (tod_ref < 0.0f)
                    tod_ref += tod_24h;
                while (tod_ref > tod_24h)
                    tod_ref -= tod_24h;

                traced_assert(tod_ref >= 0.0f);
                traced_assert(tod_ref <= tod_24h);
            }
        }
    }
}

const double TMAX_FUTURE_OFFSET = 3*60.0;
const double T24H_OFFSET = 5*60.0;

void ASTERIXTimestampCalculator::doFutureTimestampsCheck()
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    unsigned int buffer_size;

    using namespace boost::posix_time;

    auto p_time = microsec_clock::universal_time (); // UTC

    double current_time_utc = p_time.time_of_day().total_milliseconds() / 1000.0;
    double tod_utc_max = (p_time + Time::partialSeconds(TMAX_FUTURE_OFFSET)).time_of_day().total_milliseconds() / 1000.0;

    bool in_vicinity_of_24h_time = current_time_utc <= T24H_OFFSET || current_time_utc >= (tod_24h - T24H_OFFSET);

    unsigned int cnt=0;

    for (auto& buf_it : buffers_)
        cnt += buf_it.second->size();

    logdbg << "maximum time is "
           << String::timeStringFromDouble(tod_utc_max) << " 24h vicinity " << in_vicinity_of_24h_time
           << " buf size " << cnt;

    for (auto& buf_it : buffers_)
    {
        buffer_size = buf_it.second->size();

        traced_assert(dbcont_man.metaVariable(DBContent::meta_var_time_of_day_.name()).existsIn(buf_it.first));

        dbContent::Variable& tod_var = dbcont_man.metaVariable(DBContent::meta_var_time_of_day_.name()).getFor(buf_it.first);

        Property tod_prop {tod_var.name(), tod_var.dataType()};

        traced_assert(buf_it.second->hasProperty(tod_prop));

        NullableVector<float>& tod_vec = buf_it.second->get<float>(tod_var.name());

        std::tuple<bool,float,float> min_max_tod = tod_vec.minMaxValues();

        if (get<0>(min_max_tod))
            logdbg << "start" << buf_it.first
                   << " min tod " << String::timeStringFromDouble(get<1>(min_max_tod))
                   << " max " << String::timeStringFromDouble(get<2>(min_max_tod));

        for (unsigned int index=0; index < buffer_size; ++index)
        {
            if (!tod_vec.isNull(index))
            {
                if (in_vicinity_of_24h_time)
                {
                    // not at end of day and bigger than max
                    if (tod_vec.get(index) < (tod_24h - T24H_OFFSET) && tod_vec.get(index) > tod_utc_max)
                    {
                        logwrn << "vic doing " << buf_it.first
                               << " cutoff tod index " << index
                               << " tod " << String::timeStringFromDouble(tod_vec.get(index));

                        buf_it.second->cutToSize(index);
                        break;
                    }
                }
                else
                {
                    if (tod_vec.get(index) > tod_utc_max)
                    {
                        logwrn << "doing " << buf_it.first
                               << " cutoff tod index " << index
                               << " tod " << String::timeStringFromDouble(tod_vec.get(index));

                        buf_it.second->cutToSize(index);
                        break;
                    }
                }
            }
        }
    }

    // remove empty buffers

    std::map<std::string, std::shared_ptr<Buffer>> tmp_data = buffers_;

    for (auto& buf_it : tmp_data)
        if (!buf_it.second->size())
            buffers_.erase(buf_it.first);

    cnt=0;

    for (auto& buf_it : buffers_)
        cnt += buf_it.second->size();

    logdbg << "buf size " << cnt;
}

void ASTERIXTimestampCalculator::doTimeStampCalculation(bool ignore_time_jumps)
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    unsigned int buffer_size;

    for (auto& buf_it : buffers_)
    {
        buffer_size = buf_it.second->size();

        // tod
        traced_assert(dbcont_man.metaVariable(DBContent::meta_var_time_of_day_.name()).existsIn(buf_it.first));
        dbContent::Variable& tod_var =
            dbcont_man.metaVariable(DBContent::meta_var_time_of_day_.name()).getFor(buf_it.first);

        Property tod_prop {tod_var.name(), tod_var.dataType()};
        traced_assert(buf_it.second->hasProperty(tod_prop));

        NullableVector<float>& tod_vec = buf_it.second->get<float>(tod_var.name());

        // timestamp
        traced_assert(dbcont_man.metaVariable(DBContent::meta_var_timestamp_.name()).existsIn(buf_it.first));
        dbContent::Variable& timestamp_var =
            dbcont_man.metaVariable(DBContent::meta_var_timestamp_.name()).getFor(buf_it.first);

        Property timestamp_prop {timestamp_var.name(), timestamp_var.dataType()};
        traced_assert(!buf_it.second->hasProperty(timestamp_prop));
        buf_it.second->addProperty(timestamp_prop);

        NullableVector<boost::posix_time::ptime>& timestamp_vec =
            buf_it.second->get<boost::posix_time::ptime>(timestamp_var.name());

        float tod;
        boost::posix_time::ptime timestamp;

        bool in_vicinity_of_24h_time = false; // within 5min of 24hrs
        bool outside_vicinity_of_24h_time = false; // outside of 10min to 24hrs

        for (unsigned int index=0; index < buffer_size; ++index)
        {
            if (!tod_vec.isNull(index))
            {
                tod = tod_vec.get(index);

                if (tod < 0 || tod > tod_24h)
                {
                    logwrn << "impossible tod "
                           << String::timeStringFromDouble(tod);
                    continue;
                }

                had_late_time_ |= (tod >= (tod_24h - close_to_midgnight_offset_s)); // close before midnight

                // within close time midnight
                in_vicinity_of_24h_time = tod <= close_to_midgnight_offset_s
                                          || tod >= (tod_24h - close_to_midgnight_offset_s);

                if (tod > tod_24h/2) // late
                    outside_vicinity_of_24h_time = tod <= tod_24h - not_close_to_midgnight_offset_s;
                else // early
                    outside_vicinity_of_24h_time = tod >= not_close_to_midgnight_offset_s;

                if (in_vicinity_of_24h_time)
                    logdbg << "tod "
                           << String::timeStringFromDouble(tod)
                           << " in 24h vicinity, had_late_time_ " << had_late_time_
                           << " did_recent_time_jump " << did_recent_time_jump_;

                if (did_recent_time_jump_ && outside_vicinity_of_24h_time) // clear if set and 10min outside again
                {
                    loginf << "clearing did_recent_time_jump_";
                    did_recent_time_jump_ = false;
                }

                // check if timejump and assign timestamp to correct day
                if (!ignore_time_jumps && in_vicinity_of_24h_time)
                {
                    // check if timejump (if not yet done)
                    if (!did_recent_time_jump_ && had_late_time_
                        && tod <= close_to_midgnight_offset_s) // not yet handled timejump
                    {
                        current_date_ += boost::posix_time::seconds((unsigned int) tod_24h);
                        previous_date_ = current_date_ - boost::posix_time::seconds((unsigned int) tod_24h);

                        loginf << "tod "
                               << String::timeStringFromDouble(tod)
                               << " detected time-jump from previous " << Time::toDateString(previous_date_)
                               << " to current " << Time::toDateString(current_date_);

                        COMPASS::instance().logInfo("ASTERIX Import")
                            << "ToD " << String::timeStringFromDouble(tod)
                            << " detected time-jump from previous " << Time::toDateString(previous_date_)
                            << " to current " << Time::toDateString(current_date_);

                        did_recent_time_jump_ = true;
                    }

                    if (tod <= close_to_midgnight_offset_s) // early time of current day
                        timestamp = current_date_ + boost::posix_time::millisec((unsigned int) (tod * 1000));
                    else // late time of previous day
                        timestamp = previous_date_ + boost::posix_time::millisec((unsigned int) (tod * 1000));
                }
                else // normal timestamp
                {
                    timestamp = current_date_ + boost::posix_time::millisec((unsigned int) (tod * 1000));
                }

                if (in_vicinity_of_24h_time)
                    logdbg << "tod "
                           << String::timeStringFromDouble(tod)
                           << " timestamp " << Time::toString(timestamp);

                if (outside_vicinity_of_24h_time)
                {
                    did_recent_time_jump_ = false;
                    had_late_time_ = false;
                }

                logdbg << "tod "
                       << String::timeStringFromDouble(tod)
                       << " ts " << Time::toString(timestamp);

                traced_assert(!timestamp.is_not_a_date_time());
                timestamp_vec.set(index, timestamp);
                traced_assert(!timestamp_vec.isNull(index));
                traced_assert(timestamp_vec.get(index) == timestamp);

                // set first and last timestamp

                if (first_time_)
                {
                    tod_first_ = tod;
                    timestamp_first_ = timestamp;

                    first_time_ = false;

                    COMPASS::instance().logInfo("ASTERIX Import")
                        << "first ToD " << String::timeStringFromDouble(tod_first_)
                        << " timestamp " << Time::toString(timestamp_first_);
                }

                if (fabs(tod - last_reported_tod_) > 3600) // timejump in reporting
                    last_reported_tod_ = -3600;

                if (tod - last_reported_tod_ >= 3600) // 1h
                {
                    loginf << "processing tod "
                           << String::timeStringFromDouble(tod);
                    last_reported_tod_ = tod;
                }

                tod_last_ = tod;
                timestamp_last_ = timestamp;
            }
        }
    }
}

void ASTERIXTimestampCalculator::setProcessingDone()
{
    logdbg;

    traced_assert(!buffers_.size());
    traced_assert(processing_);

    processing_ = false;
}

void ASTERIXTimestampCalculator::reset()
{
    traced_assert(!processing_);

    prev_source_name_ = "";

    resetDateInfo();
    clearTimeStats();
}

void ASTERIXTimestampCalculator::resetDateInfo()
{
    current_date_set_ = false;
    current_date_ = {};
    previous_date_ = {};

    did_recent_time_jump_ = false;
    had_late_time_ = false;
}

void ASTERIXTimestampCalculator::clearTimeStats()
{
    logdbg;

    first_time_ = true;
    timestamp_first_ = {};
    tod_first_ = 0;
    timestamp_last_ = {};
    tod_last_ = 0;

    last_reported_tod_ = -3600;
}

void ASTERIXTimestampCalculator::logLastTimestamp()
{
    if (!first_time_ && !timestamp_last_.is_not_a_date_time())
    {
        COMPASS::instance().logInfo("ASTERIX Import")
        << "last ToD " << String::timeStringFromDouble(tod_last_)
        << " timestamp " << Time::toString(timestamp_last_);
    }
}
