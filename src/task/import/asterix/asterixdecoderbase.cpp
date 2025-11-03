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

#include "asterixdecoderbase.h"

#include "asterixfiledecoder.h"
#include "asterixpcapdecoder.h"
#include "asterixjsondecoder.h"
#include "asterixnetworkdecoder.h"
#include "asteriximporttask.h"

#include "asynctask.h"

#include "compass.h"
#include "taskmanager.h"

/**
 * @param source Import source to retrieve data from.
 * @param settings If set, external settings will be applied, otherwise settings will be retrieved from the import task.
*/
ASTERIXDecoderBase::ASTERIXDecoderBase(ASTERIXImportSource& source,
                                       const ASTERIXImportTaskSettings* settings)
:   source_(source)
{
    task_ = &COMPASS::instance().taskManager().asterixImporterTask();
    traced_assert(task_);

    settings_ = settings ? settings : &task_->settings();
    traced_assert(settings_);
}

/**
*/
ASTERIXDecoderBase::~ASTERIXDecoderBase() = default;

/**
 * Create a specific decoder for the given source type.
 */
std::unique_ptr<ASTERIXDecoderBase> ASTERIXDecoderBase::createDecoder(ASTERIXImportSource& source,
                                                                      const ASTERIXImportTaskSettings* settings)
{
    if (source.sourceType() == ASTERIXImportSource::SourceType::FileASTERIX)
        return std::unique_ptr<ASTERIXDecoderBase>(new ASTERIXFileDecoder(source, settings));
    else if (source.sourceType() == ASTERIXImportSource::SourceType::FilePCAP)
        return std::unique_ptr<ASTERIXDecoderBase>(new ASTERIXPCAPDecoder(source, settings));
    else if (source.sourceType() == ASTERIXImportSource::SourceType::FileJSON)
        return std::unique_ptr<ASTERIXDecoderBase>(new ASTERIXJSONDecoder(source, settings));
    else if (source.sourceType() == ASTERIXImportSource::SourceType::NetASTERIX)
        return std::unique_ptr<ASTERIXDecoderBase>(new ASTERIXNetworkDecoder(source, settings));

    return {};
}

/**
 * Preliminary check if the decoder can even run.
*/
bool ASTERIXDecoderBase::canRun() const
{
    return canRun_impl();
}

/**
Checks if the decoder can decode the currently set import source.
@param force_recompute If true the information needed to decide if decode is valid or not is recomputed
(e.g. by decoding a small portion of the source data).
*/
bool ASTERIXDecoderBase::canDecode(bool force_recompute, AsyncTaskProgressWrapper* progress) const
{
    //cannot run => cannot decode
    if (!canRun())
        return false;

    //refresh decoding info
    checkDecoding(force_recompute, progress);

    //decide if decoding is possible
    return canDecode_impl();
}

/**
Refreshes internal information about decoding the currently set import source.
@param force_recompute If true this information is always recomputed, otherwise
cached values might be used.
*/
void ASTERIXDecoderBase::checkDecoding(bool force_recompute, AsyncTaskProgressWrapper* progress) const
{
    checkDecoding_impl(force_recompute, progress);
}

/**
 * Returns the number of errors logged since starting the decoder.
*/
size_t ASTERIXDecoderBase::numErrors() const
{
    return num_errors_;
}

/**
 * Checks if an error has been logged.
*/
bool ASTERIXDecoderBase::error() const
{
    return error_;
}

/**
 * Returns the currently set error message.
*/
std::string ASTERIXDecoderBase::errorMessage() const
{
    return error_message_;
}

/**
 */
std::vector<std::string> ASTERIXDecoderBase::errors() const
{
    std::vector<std::string> errors;

    if (error())
        errors.push_back("General decoder error: " + (errorMessage().empty() ? "Unknown error" : errorMessage()));

    return errors;
}

/**
 * Stores the given error and increments the error count.
*/
void ASTERIXDecoderBase::logError(const std::string& err)
{
    if (!err.empty())
        error_message_ = err;

    ++num_errors_;
    error_ = true;
}

/**
 * Starts decoding with the given job.
*/
void ASTERIXDecoderBase::start(ASTERIXDecodeJob* job)
{
    traced_assert(!running_);

    //need a job
    traced_assert(job);
    job_ = job;

    //remember start time
    start_time_ = boost::posix_time::microsec_clock::local_time();

    running_ = true;

    //reset errors
    error_         = false;
    num_errors_    = 0;
    error_message_ = "";

    //invoke derived
    start_impl();
}

/**
 * Stops decoding.
*/
void ASTERIXDecoderBase::stop()
{
    if (running_)
    {
        running_ = false;

        //invoke derived
        stop_impl();
    }

    job_ = nullptr;
}

/**
 * Elapsed seconds since starting the decoder.
*/
float ASTERIXDecoderBase::elapsedSeconds() const
{
    return (float)(boost::posix_time::microsec_clock::local_time() - startTime()).total_milliseconds() / 1000.0;
}

/**
*/
nlohmann::json ASTERIXDecoderBase::stateAsJSON() const
{
    nlohmann::json info;

    info[ "source"        ] = source_.toJSON();

    info[ "error"         ] = error_;
    info[ "error_message" ] = error_message_;
    info[ "num_errors"    ] = num_errors_;
    
    return info;
}
