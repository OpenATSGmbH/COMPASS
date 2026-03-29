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

#include "asterixdecodejob.h"
#include "asteriximporttask.h"
#include "logger.h"
#include "asteriximportsource.h"

#include <QThread>

#include <memory>

using namespace nlohmann;
using namespace Utils;
using namespace std;

/**
*/
ASTERIXDecodeJob::ASTERIXDecodeJob(ASTERIXImportTask& task,
                                   ASTERIXPostProcess& post_process)
:   Job          ("ASTERIXDecodeJob"),
    task_        (task), 
    settings_    (task.settings()),
    decoder_     (task.decoder()),
    post_process_(post_process)
{
    logdbg;

    traced_assert(decoder_);
}

/**
*/
ASTERIXDecodeJob::~ASTERIXDecodeJob()
{
    traced_assert(done_);
}

/**
*/
void ASTERIXDecodeJob::run_impl()
{
    loginf;

    start_time_ = boost::posix_time::microsec_clock::local_time();
    started_    = true;
    done_       = false;

    traced_assert(decoder_);
    decoder_->start(this);

    if (!obsolete_)
    {
        loginf << "waiting for last data to be fetched...";

        //wait until data is fetched
        while (!extracted_data_.empty())
            QThread::msleep(1);

        traced_assert(extracted_data_.size() == 0);
    }

    done_ = true;

    loginf << "done";
}

/**
*/
void ASTERIXDecodeJob::setObsolete()
{
    logdbg;

    Job::setObsolete();

    traced_assert(decoder_);
    decoder_->stop();
}

/**
*/
void ASTERIXDecodeJob::fileJasterixCallback(std::unique_ptr<nlohmann::json> data, 
                                            unsigned int line_id, 
                                            size_t num_frames,
                                            size_t num_records, 
                                            size_t num_errors)
{
    logdbg << "running on cpu " << sched_getcpu();

    if (obsolete_)
        return;

    if (decoder_ && decoder_->error())
    {
        loginf << "error state";
        return;
    }

    //loginf << "data '" << data->dump(2) << "'";
    logdbg << "line_id " << line_id << " num_records " << num_records;

    if (num_records == 0)
    {
        loginf << "omitting zero data in '"
               << data->dump(2) << "'";
        return;
    }

    traced_assert(data);
    traced_assert(data->is_object());

    num_frames_  = num_frames;
    num_records_ = num_records;
    num_errors_  = num_errors;

    if (num_errors_)
        logwrn << "num errors " << num_errors_;

    // flat format: top-level keys are category numbers, values are objects with array columns
    for (auto it = data->begin(); it != data->end(); ++it)
    {
        unsigned int category = 0;
        try
        {
            category = std::stoul(it.key());
        }
        catch (...)
        {
            continue; // skip non-numeric keys
        }

        // count records from the longest array in this category
        size_t cat_records = 0;
        if (it.value().is_object())
        {
            for (auto arr_it = it.value().begin(); arr_it != it.value().end(); ++arr_it)
            {
                if (arr_it.value().is_array() && arr_it.value().size() > cat_records)
                    cat_records = arr_it.value().size();
            }
        }

        if (cat_records == 0)
            continue;

        // post-process: adds category, ds_id, line_id arrays + category-specific corrections
        post_process_.postProcessFlat(category, line_id, it.value(), cat_records);

        category_counts_[category] += cat_records;
    }

    while (!obsolete_ && extracted_data_.size())  // block decoder until extracted records have been moved out
        QThread::msleep(1);

    {
        boost::mutex::scoped_lock locker(extracted_data_mutex_);

        extracted_data_.emplace_back(std::move(data));
    }

    ++signal_count_;

    logdbg << "emitting signal " << signal_count_;

    emit decodedASTERIXSignal();

    logdbg << "done " << signal_count_;
}

/**
*/
void ASTERIXDecodeJob::netJasterixCallback(std::unique_ptr<nlohmann::json> data, 
                                           unsigned int line_id, 
                                           size_t num_frames,
                                           size_t num_records, 
                                           size_t num_errors)
{
    if (obsolete_)
        return;

    if (decoder_ && decoder_->error())
    {
        loginf << "errors state";
        return;
    }

    loginf << "line_id " << line_id << " num_records " << num_records;

    num_frames_  = num_frames;
    num_records_ = num_records;
    num_errors_  = num_errors;

    if (num_errors_)
        logwrn << "num errors " << num_errors_;

    // flat format: top-level keys are category numbers
    bool has_data = false;

    for (auto it = data->begin(); it != data->end(); ++it)
    {
        unsigned int category = 0;
        try
        {
            category = std::stoul(it.key());
        }
        catch (...)
        {
            continue;
        }

        size_t cat_records = 0;
        if (it.value().is_object())
        {
            for (auto arr_it = it.value().begin(); arr_it != it.value().end(); ++arr_it)
            {
                if (arr_it.value().is_array() && arr_it.value().size() > cat_records)
                    cat_records = arr_it.value().size();
            }
        }

        if (cat_records == 0)
            continue;

        post_process_.postProcessFlat(category, line_id, it.value(), cat_records);

        category_counts_[category] += cat_records;
        has_data = true;
    }

    if (has_data)
    {
        boost::mutex::scoped_lock locker(extracted_data_mutex_);
        extracted_data_.emplace_back(std::move(data));
    }
}

/**
*/
size_t ASTERIXDecodeJob::numFrames() const 
{ 
    return num_frames_; 
}

/**
*/
size_t ASTERIXDecodeJob::numRecords() const 
{ 
    return num_records_; 
}

/**
*/
bool ASTERIXDecodeJob::error() const 
{ 
    return decoder_ && decoder_->error(); 
}

/**
*/
std::map<unsigned int, size_t> ASTERIXDecodeJob::categoryCounts() const
{
    return category_counts_;
}

/**
*/
std::vector<std::unique_ptr<nlohmann::json>> ASTERIXDecodeJob::extractedData()
{
    logdbg << "signal cnt " << signal_count_;

    boost::mutex::scoped_lock locker(extracted_data_mutex_);

    return std::move(extracted_data_);
}

/**
*/
bool ASTERIXDecodeJob::hasStatusInfo()
{
    return decoder_ && decoder_->hasStatusInfo();
}

/**
*/
std::string ASTERIXDecodeJob::statusInfoString()
{
    traced_assert(hasStatusInfo());
    return decoder_->statusInfoString();
}

/**
*/
float ASTERIXDecodeJob::statusInfoProgress() // percent
{
    traced_assert(hasStatusInfo());
    return decoder_->statusInfoProgress();
}

std::string ASTERIXDecodeJob::currentDataSourceName()
{
    traced_assert(decoder_);
    return decoder_->currentDataSourceName();
}

/**
*/
void ASTERIXDecodeJob::forceBlockingDataProcessing()
{
    logdbg << "emitting signal";

    if (obsolete_)
        extracted_data_.clear();
    else
        emit decodedASTERIXSignal();

    while (!obsolete_ && extracted_data_.size())  // block decoder until extracted records have been moved out
        QThread::msleep(1);
}

/**
*/
size_t ASTERIXDecodeJob::numErrors() const 
{ 
    return num_errors_; 
}

/**
*/
std::string ASTERIXDecodeJob::errorMessage() const 
{ 
    return (decoder_ ? decoder_->errorMessage() : ""); 
}


