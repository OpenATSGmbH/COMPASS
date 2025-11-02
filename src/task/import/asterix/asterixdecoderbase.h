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

#include "asteriximportsource.h"
#include "json_fwd.hpp"

#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>

class ASTERIXImportTask;
class ASTERIXDecodeJob;
class ASTERIXImportTaskSettings;

class AsyncTaskProgressWrapper;

/**
 * Base class for an ASTERIX decoder.
 */
class ASTERIXDecoderBase
{
public:
    ASTERIXDecoderBase(ASTERIXImportSource& source,
                       const ASTERIXImportTaskSettings* settings = nullptr);
    virtual ~ASTERIXDecoderBase();

    virtual std::string name() const { return ""; }

    bool canRun() const;
    bool canDecode(bool force_recompute, AsyncTaskProgressWrapper* progress = nullptr) const;
    
    void start(ASTERIXDecodeJob* job);
    void stop();

    size_t numErrors() const;
    bool error() const;
    std::string errorMessage() const;

    virtual bool hasStatusInfo() const { return false; };
    virtual std::string statusInfoString() const { return ""; }
    virtual float statusInfoProgress() const { return 0; } // percent
    virtual std::string currentDataSourceName() const { return ""; }
    virtual std::vector<std::string> errors() const;

    virtual boost::optional<std::string> requiredASTERIXFraming() const { return {}; }

    nlohmann::json stateAsJSON() const;

    static std::unique_ptr<ASTERIXDecoderBase> createDecoder(ASTERIXImportSource& source,
                                                             const ASTERIXImportTaskSettings* settings = nullptr);
protected:
    ASTERIXDecodeJob* job() { return job_; }
    const ASTERIXDecodeJob* job() const { return job_; }
    ASTERIXImportTask& task() { return *task_; }
    const ASTERIXImportTask& task() const { return *task_; }
    const ASTERIXImportTaskSettings& settings() const { return *settings_; }

    bool isRunning() const { return running_; }

    const boost::posix_time::ptime& startTime() const { return start_time_; }
    float elapsedSeconds() const;

    void logError(const std::string& err = "");

    virtual bool canRun_impl() const = 0;
    virtual bool canDecode_impl() const = 0;
    virtual void checkDecoding_impl(bool force_recompute, AsyncTaskProgressWrapper* progress) const {};

    virtual void start_impl() = 0;
    virtual void stop_impl() = 0;

    ASTERIXImportSource& source_;
    
private:
    void checkDecoding(bool force_recompute, AsyncTaskProgressWrapper* progress) const;

    ASTERIXDecodeJob*                job_      = nullptr;
    ASTERIXImportTask*               task_     = nullptr;
    const ASTERIXImportTaskSettings* settings_ = nullptr;

    bool running_ = false;

    boost::posix_time::ptime start_time_;

    size_t      num_errors_ = 0;
    bool        error_      = false;
    std::string error_message_;
};
