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
#include "asterixdecoderbase.h"

#include "json_fwd.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>

class ASTERIXImportTask;
class ASTERIXImportTaskSettings;
class ASTERIXPostProcess;
class ASTERIXImportSource;

/**
 * is a job, creates specifc reader/decoder, does post-processing
 */ 
class ASTERIXDecodeJob : public Job
{
    Q_OBJECT

signals:
    void decodedASTERIXSignal();

public:
    ASTERIXDecodeJob(ASTERIXImportTask& task, 
                     ASTERIXPostProcess& post_process);
    virtual ~ASTERIXDecodeJob();

    virtual void setObsolete() override;

    size_t numFrames() const;
    size_t numRecords() const;
    size_t numErrors() const;

    bool hasData() { return extracted_data_.size();}

    bool error() const;
    std::string errorMessage() const;

    std::map<unsigned int, size_t> categoryCounts() const;

    std::vector<std::unique_ptr<nlohmann::json>> extractedData();

    bool hasStatusInfo();
    std::string statusInfoString();
    float statusInfoProgress(); // percent
    std::string currentDataSourceName(); // used to distinguish file switches

    void forceBlockingDataProcessing();

protected:
    void run_impl() override;

private:
    void fileJasterixCallback(std::unique_ptr<nlohmann::json> data, 
                              unsigned int line_id, 
                              size_t num_frames,
                              size_t num_records, 
                              size_t numErrors);
    void netJasterixCallback(std::unique_ptr<nlohmann::json> data, 
                             unsigned int line_id, 
                             size_t num_frames,
                             size_t num_records, 
                             size_t numErrors);

    void countRecord(unsigned int category, nlohmann::json& record);

    // checks that SAC/SIC are set in all records in same data block
    void checkCAT001SacSics(nlohmann::json& data_block);

    friend class ASTERIXFileDecoder;
    friend class ASTERIXNetworkDecoder;
    friend class ASTERIXPCAPDecoder; 
    friend class ASTERIXJSONDecoder;

    ASTERIXImportTask& task_;
    const ASTERIXImportTaskSettings& settings_;
    ASTERIXDecoderBase* decoder_ = nullptr;

    ASTERIXPostProcess& post_process_;

    boost::posix_time::ptime start_time_;

    size_t num_frames_   {0};
    size_t num_records_  {0};
    size_t num_errors_   {0};
    size_t num_warnings_ {0};

    std::vector<std::unique_ptr<nlohmann::json>> extracted_data_;
    boost::mutex extracted_data_mutex_;

    std::map<unsigned int, size_t> category_counts_;

    unsigned int signal_count_{0};
};
