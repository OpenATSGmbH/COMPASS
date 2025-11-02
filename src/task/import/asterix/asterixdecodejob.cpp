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
#include "json_tools.h"
#include "logger.h"
//#include "asterixfiledecoder.h"
//#include "asterixnetworkdecoder.h"
//#include "asterixpcapdecoder.h"
#include "asteriximportsource.h"

#include <QThread>

//#include <chrono>
//#include <thread>
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

    unsigned int category{0};

    auto count_lambda = [this, &category] (nlohmann::json& record) 
    {
        countRecord(category, record);
    };

    auto process_lambda = [this, line_id, &category] (nlohmann::json& record) 
    {
        record["line_id"] = line_id;
        post_process_.postProcess(category, record);
    };

    if (settings_.activeFileFraming() == "")
    {
        traced_assert(data->contains("data_blocks"));
        traced_assert(data->at("data_blocks").is_array());

        std::vector<std::string> keys{"content", "records"};

        for (json& data_block : data->at("data_blocks"))
        {
            if (!data_block.contains("category"))
            {
                logwrn << "data block without asterix category";
                continue;
            }

            category = data_block.at("category");

            traced_assert(data_block.contains("content"));
            traced_assert(data_block.at("content").is_object());

            if (category == 1)
                checkCAT001SacSics(data_block);

            logdbg << "applying JSON function without framing";
            JSON::applyFunctionToValues(data_block, keys, keys.begin(), process_lambda, false);
            JSON::applyFunctionToValues(data_block, keys, keys.begin(), count_lambda  , false);
        }
    }
    else
    {
        traced_assert(data->contains("frames"));
        traced_assert(data->at("frames").is_array());

        std::vector<std::string> keys{"content", "records"};

        for (json& frame : data->at("frames"))
        {
            if (!frame.contains("content"))  // frame with errors
                continue;

            traced_assert(frame.at("content").is_object());

            if (!frame.at("content").contains("data_blocks"))  // frame with errors
                continue;

            traced_assert(frame.at("content").at("data_blocks").is_array());

            for (json& data_block : frame.at("content").at("data_blocks"))
            {
                if (!data_block.contains("category"))  // data block with errors
                {
                    logwrn << "data block without asterix "
                              "category";
                    continue;
                }

                category = data_block.at("category");

                if (category == 1)
                    checkCAT001SacSics(data_block);

                JSON::applyFunctionToValues(data_block, keys, keys.begin(), process_lambda, false);
                JSON::applyFunctionToValues(data_block, keys, keys.begin(), count_lambda  , false);
            }
        }
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

    //loginf << "data '" << data->dump(2) << "'";
    loginf << "line_id " << line_id << " num_records " << num_records;

    num_frames_  = num_frames;
    num_records_ = num_records;
    num_errors_  = num_errors;

    if (num_errors_)
        logwrn << "num errors " << num_errors_;

    unsigned int category{0};

    auto count_lambda = [this, &category](nlohmann::json& record) 
    {
        countRecord(category, record);
    };

    auto process_lambda = [this, line_id, &category](nlohmann::json& record) 
    {
        record["line_id"] = line_id;
        post_process_.postProcess(category, record);
    };

    traced_assert(data->contains("data_blocks"));
    traced_assert(data->at("data_blocks").is_array());

    std::vector<std::string> keys{"content", "records"};

    for (json& data_block : data->at("data_blocks"))
    {
        if (!data_block.contains("category"))
        {
            logwrn << "data block without asterix category";
            continue;
        }

        category = data_block.at("category");

        traced_assert(data_block.contains("content"));
        traced_assert(data_block.at("content").is_object());

        if (category == 1)
            checkCAT001SacSics(data_block);

        logdbg << "applying JSON function without framing";
        JSON::applyFunctionToValues(data_block, keys, keys.begin(), process_lambda, false);
        JSON::applyFunctionToValues(data_block, keys, keys.begin(), count_lambda  , false);
    }

    if (data->at("data_blocks").size())
    {
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
void ASTERIXDecodeJob::countRecord(unsigned int category, nlohmann::json& record)
{
    logdbg << "cat " << category << " record '" << record.dump(4)
           << "'";

    category_counts_[category] += 1;
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

/**
 * equivalent function in JSONParseJob
 */
void ASTERIXDecodeJob::checkCAT001SacSics(nlohmann::json& data_block)
{
    if (!data_block.contains("content"))
    {
        logdbg << "no content in data block";
        return;
    }

    nlohmann::json& content = data_block.at("content");

    if (!content.contains("records"))
    {
        logdbg << "no records in content";
        return;
    }

    nlohmann::json& records = content.at("records");

    bool found_any_sac_sic = false;

    unsigned int sac = 0;
    unsigned int sic = 0;

    // check if any SAC/SIC info can be found
    for (nlohmann::json& record : records)
    {
        if (record.count("error") && record.at("error") == true)
            return; // skip target reports marked with errors

        if (!found_any_sac_sic)
        {
            if (record.contains("010"))  // found, set as transferable values
            {
                sac = record.at("010").at("SAC");
                sic = record.at("010").at("SIC");
                found_any_sac_sic = true;
            }
            else  // not found, can not set values
                logwrn << "record without any SAC/SIC found";
        }
        else
        {
            if (record.contains("010"))  // found, check values
            {
                if (record.at("010").at("SAC") != sac || record.at("010").at("SIC") != sic)
                    logwrn << "record with differing "
                              "SAC/SICs found";
            }
            else  // not found, set values
            {
                record["010"]["SAC"] = sac;
                record["010"]["SIC"] = sic;
            }
        }
    }
}
