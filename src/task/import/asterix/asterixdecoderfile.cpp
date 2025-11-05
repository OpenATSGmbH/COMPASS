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

#include "asterixdecoderfile.h"

#include "asynctask.h"

#include "compass.h"
#include "logger.h"
#include "files.h"
#include "stringconv.h"

#include <boost/filesystem/path.hpp>

const std::vector<std::string> ASTERIXDecoderFile::SupportedArchives = { ".zip", ".gz", ".tgz", ".tar" };

/**
 * @param source_type File source the decoder is able to decode.
 * @param source Import source to retrieve data from.
 * @param settings If set, external settings will be applied, otherwise settings will be retrieved from the import task.
*/
ASTERIXDecoderFile::ASTERIXDecoderFile(ASTERIXImportSource::SourceType source_type,
                                       ASTERIXImportSource& source,
                                       const ASTERIXImportTaskSettings* settings)
:   ASTERIXDecoderBase(source, settings)
,   source_type_      (source_type)
{
    traced_assert(source_.isFileType() && source_.sourceType() == fileSourceType());
}

/**
*/
ASTERIXDecoderFile::~ASTERIXDecoderFile() = default;

/**
*/
void ASTERIXDecoderFile::start_impl()
{
    total_file_size_          = source_.totalFileSizeInBytes(true);
    total_records_read_       = 0;
    current_file_bytes_read_  = 0;
    current_chunk_bytes_read_ = 0;
    done_file_size_           = 0;
    
    current_file_idx_         = -1;

    while (isRunning() && nextFile())
    {
        processCurrentFile();

        if (!isRunning())
            break;
    }
}

/**
*/
bool ASTERIXDecoderFile::nextFile()
{
    if (atEnd())
        return false;

    ++current_file_idx_;

    return !atEnd();
}

/**
*/
bool ASTERIXDecoderFile::atEnd() const
{
    return (current_file_idx_ >= (int)source_.files().size());
}

/**
*/
void ASTERIXDecoderFile::processCurrentFile()
{
    if (!isRunning())
        return;

    traced_assert(!atEnd());

    auto& current_file = source_.file_infos_.at(current_file_idx_);

    //skip unused
    if (!current_file.used)
        return;

    //COMPASS::instance().logInfo("ASTERIX Import") << "decoding '" << current_file.filename << "'";
    // done in ASTERIXTimestampCalculator

    try
    {
        processFile(current_file);

        if (!isRunning())
            return;

        //another file done
        done_file_size_          += current_file.sizeInBytes(true);
        current_file_bytes_read_  = 0;
        current_chunk_bytes_read_ = 0;

        //flag file as processed
        current_file.processed = true;
    }
    catch(const std::exception& e)
    {
        COMPASS::instance().logError("ASTERIX Import") << "file '" << current_file.filename
                                       << "' decode error '" << e.what() << "'";

        logerr << "decode error '" << e.what() << "'";
        logError(e.what());
    }
    catch(...)
    {
        COMPASS::instance().logError("ASTERIX Import") << "file '" << current_file.filename
                                       << "' unknown decode error";

        logerr << "unknown decode error";
        logError("Unknown decode error");
    }
}

/**
*/
bool ASTERIXDecoderFile::canRun_impl() const
{
    //no files to decode?
    if (source_.files().empty())
        return false;

    return true;
}

/**
 */
bool ASTERIXDecoderFile::canDecode_impl() const
{
    //check on all files
    size_t num_used = 0;
    for (const auto& fi : source_.file_infos_)
    {
        //used and cannot decode? => fail
        if (fi.used && !fi.canDecode())
            return false;

        if (fi.used)
            ++num_used;
    }

    //no used file no decoding
    if (num_used == 0)
        return false;

    return true;
}

/**
 */
void ASTERIXDecoderFile::checkDecoding_impl(bool force_recompute, AsyncTaskProgressWrapper* progress) const
{
    //run file check on all files
    if (progress)
        progress->setMessage("Running preliminary file check...", false, true);

    for (auto& fi : source_.file_infos_)
    {
        checkFile(fi, force_recompute, false);
    }

    //run decode check on all files
    if (progress) 
    {
        progress->setMessage("Checking file decoding...", false, true);

        int num_steps = 0;
        for (const auto& fi : source_.file_infos_)
        {
            if (fi.decoding_tested && !force_recompute)
                continue;
            if (fi.hasError())
                continue;

            num_steps += (fi.hasSections() ? (int)fi.sections.size() : 1);
        }

        //only set steps if we have more than 1 thing to do, otherwise use busy progress
        if (num_steps > 1)
            progress->setSteps(0, num_steps, true, true);
    }

    for (auto& fi : source_.file_infos_)
    {
        checkDecoding(fi, force_recompute, progress);
    }
}

/**
 * Checks if a file can be decoded.
*/
void ASTERIXDecoderFile::checkFile(ASTERIXImportFileInfo& file_info,
                                   bool force_recompute,
                                   bool check_decoding,
                                   AsyncTaskProgressWrapper* progress) const
{
    if (file_info.decoding_tested && !force_recompute)
        return;

    //reset information
    file_info.reset();

    file_info.decoding_tested = true;

    //check file validity first
    if (file_info.filename.empty() || !Utils::Files::fileExists(file_info.filename))
    {
        file_info.error.errtype = ASTERIXImportFileError::ErrorType::Invalid;
        file_info.error.errinfo = "File does not exist";
        return;
    }

    std::string error;
    bool file_ok = false;

    //check file (open file, initially parse file, collect file sections, etc)
    try
    {
        file_ok = checkFile(file_info, error);
    }
    catch(const std::exception& e)
    {
        file_info.error.errinfo = "File check interrupt: " + std::string(e.what());
        file_ok = false;
    }
    catch(...)
    {
        file_info.error.errinfo = "File check interrupt: Unknown error";
        file_ok = false;
    }
    
    if (!file_ok)
    {
        file_info.error.errtype = ASTERIXImportFileError::ErrorType::CouldNotParse;
        file_info.error.errinfo = error;

        loginf << "error: " << file_info.error.errinfo;

        return;
    }

    if (check_decoding)
        checkDecoding(file_info, force_recompute, progress);
}

/**
 * Checks if a file can be decoded.
*/
void ASTERIXDecoderFile::checkDecoding(ASTERIXImportFileInfo& file_info,
                                       bool force_recompute,
                                       AsyncTaskProgressWrapper* progress) const
{
    //had an error during file check? => do not check decoding
    if (file_info.hasError())
        return;
    
    if (file_info.decoding_tested && !force_recompute)
        return;

    //if file has subsections check decoding for each of them...
    if (file_info.hasSections())
    {
        for (size_t i = 0; i < file_info.sections.size(); ++i)
        {
            if (progress) 
            {
                std::string msg = "Checking decoding of file '" + boost::filesystem::path(file_info.filename).filename().string() + "' section " + std::to_string(i + 1);
                progress->setMessage(msg, false, true);
            }

            bool section_ok = checkDecoding(file_info, (int)i, file_info.sections[ i ].contentinfo, file_info.sections[ i ].error);
            
            //set section to unused?
            if (!section_ok)
                file_info.sections[ i ].used = false;

            if (progress) 
                progress->increment(1, true);
        }
        return;
    }

    if (progress) 
    {
        std::string msg = "Checking decoding of file '" + boost::filesystem::path(file_info.filename).filename().string() + "'";
        progress->setMessage(msg, false, true);
    }

    //... otherwise check decoding on complete file
    bool file_dec_ok = checkDecoding(file_info, -1, file_info.contentinfo, file_info.error);

    //set file to unused?
    if (!file_dec_ok)
        file_info.used = false;

    if (progress) 
        progress->increment(1, true);
}

/**
 * Checks if either a file or one of its sections can be decoded and
 * stores related error information.
*/
bool ASTERIXDecoderFile::checkDecoding(ASTERIXImportFileInfo& file_info, 
                                       int section_idx, 
                                       std::string& contentinfo,
                                       ASTERIXImportFileError& error) const
{
    bool ok = false;
    std::string err_msg;
            
    try
    {
        //invoke derived
        ok = checkDecoding(file_info, section_idx, contentinfo, err_msg);
    }
    catch(const std::exception& e)
    {
        err_msg = "Decoder interrupt: " + std::string(e.what());
        ok = false;
    }
    catch(...)
    {
        err_msg = "Decoder interrupt: Unknown error";
        ok = false;
    }
    
    if (!ok)
    {
        error.errtype = ASTERIXImportFileError::ErrorType::DecodingFailed;
        error.errinfo = err_msg;
    }

    return ok;
}

/**
*/
std::string ASTERIXDecoderFile::getCurrentFilename() const
{
    if (current_file_idx_ < (int)source_.file_infos_.size())
        return source_.file_infos_[ current_file_idx_ ].filename;
    return "";
}

/**
*/
std::string ASTERIXDecoderFile::statusInfoString() const
{
    // std::string text;

    // const auto& file_infos = source_.files();

    // for (const auto& file_info : file_infos)
    // {
    //     //skip unused
    //     if (!file_info.used)
    //         continue;

    //     if (file_info.filename == getCurrentFilename())
    //         text += "<p align=\"left\"><b>" + file_info.filename + "</b>";
    //     else
    //         text += "<p align=\"left\">"+file_info.filename + "";
    // }

    // text += "<br><p align=\"left\">Records/s: " + std::to_string((unsigned int) getRecordsPerSecond());
    // text += "<p align=\"right\">Remaining: "+ Utils::String::timeStringFromDouble(getRemainingTime() + 1.0, false);

    // return text;

    std::ostringstream html;
    // Start table and header row
    html << "<table border=\"0\" width=\"100%\">"
         << "<tr>"
            "<th align=\"left\">Filename</th>"
            "<th align=\"right\">Size (MB)</th>"
            "<th align=\"center\">Status</th>"
         << "</tr>";

    const auto& file_infos = source_.files();
    for (const auto& file_info : file_infos)
    {
        // Skip unused files
        if (!file_info.used)
            continue;

        // Filename cell, bold if current
        std::string filename_cell = (file_info.filename == getCurrentFilename())
                                   ? "<b>" + file_info.filename + "</b>"
                                   : file_info.filename;

        // Size in megabytes
        double mb = file_info.sizeInBytes(/*used_only=*/true) / (1024.0 * 1024.0);

        std::ostringstream size_fmt;
        size_fmt << std::fixed << std::setprecision(2) << mb;

        // Decoded‐status cell
        std::string status_cell;
        
        if (file_info.filename == getCurrentFilename())
            status_cell = "Decoding";
        else if (file_info.fileProcessed())
            status_cell = "Done";

        if (file_info.hasError())
            status_cell += "<br> <b><font color=\"red\">(errors detected)</font></b>";

        // One row per file
        html << "<tr>"
                "<td align=\"left\">"   << filename_cell    << "</td>"
                            "<td align=\"right\">"  << size_fmt.str() << "</td>"
                                 "<td align=\"center\">" << status_cell  << "</td>"
             << "</tr>";
    }

    // Two empty spacer rows
    html << "<tr><td colspan=\"3\">&nbsp;</td></tr>"
         << "<tr><td colspan=\"3\">&nbsp;</td></tr>";

    // Elapsed / Remaining row
    html << "<tr>"
            "<td colspan=\"2\" align=\"left\">Elapsed:  "
         << Utils::String::timeStringFromDouble(elapsedSeconds(), false)
         << "</td>"
            "<td align=\"right\">Remaining: "
         << Utils::String::timeStringFromDouble(getRemainingTime(), false)
         << "</td>"
         << "</tr>";

    html << "<tr><td colspan=\"3\">&nbsp;</td></tr>";

    // Records/sec row
    html << "<tr>"
            "<td colspan=\"3\" align=\"right\">"
            "Records/s: " << static_cast<unsigned int>(getRecordsPerSecond())
         << "</td>"
         << "</tr>"

         // Close table
         << "</table>";

    return html.str();
}

/**
*/
std::vector<std::string> ASTERIXDecoderFile::errors() const
{
    auto errors = ASTERIXDecoderBase::errors();

    const auto& file_infos = source_.files();
    for (const auto& file_info : file_infos)
    {
        // Skip unused files
        if (!file_info.used)
            continue;

        auto fn = boost::filesystem::path(file_info.filename).filename().string();

        if (file_info.hasError())
            errors.push_back("File '" + fn + "'" + (file_info.error.errinfo.empty() ? " obtains errors" : ": " + file_info.error.errinfo));
        
        for (const auto& s : file_info.sections)
            if (s.used && s.error.hasError())
                errors.push_back("File '" + fn + "' Section '" + s.description + "': " + s.error.errinfo);
    }

    return errors;
}

/**
*/
size_t ASTERIXDecoderFile::currentlyReadBytes() const
{
    return (done_file_size_ + current_file_bytes_read_ + current_chunk_bytes_read_);
}

/**
*/
float ASTERIXDecoderFile::statusInfoProgress() const
{
    return 100.0 * (float)currentlyReadBytes() / (float)total_file_size_;
}

std::string ASTERIXDecoderFile::currentDataSourceName() const
{
    return "File '"+getCurrentFilename()+"'";
}

/**
*/
float ASTERIXDecoderFile::getRecordsPerSecond() const
{
    return (float) total_records_read_ / elapsedSeconds();
}

/**
*/
float ASTERIXDecoderFile::getRemainingTime() const
{
    float  elapsed_secs    = elapsedSeconds();
    size_t remaining_bytes = total_file_size_ - currentlyReadBytes();
    float  bytes_per_s     = (float)currentlyReadBytes() / elapsed_secs;

    return (float)remaining_bytes / bytes_per_s;
}

/**
*/
void ASTERIXDecoderFile::addRecordsRead(size_t n)
{
    total_records_read_ += n;
}

/**
*/
void ASTERIXDecoderFile::addChunkBytesRead(size_t n)
{
    current_chunk_bytes_read_ += n;
}

/**
*/
void ASTERIXDecoderFile::setChunkBytesRead(size_t n)
{
    current_chunk_bytes_read_ = n;
}

/**
*/
void ASTERIXDecoderFile::addFileBytesRead(size_t n)
{
    current_file_bytes_read_ += n;
}

/**
*/
void ASTERIXDecoderFile::setFileBytesRead(size_t n)
{
    current_file_bytes_read_ = n;
}

/**
*/
void ASTERIXDecoderFile::chunkFinished()
{
    current_file_bytes_read_ += current_chunk_bytes_read_;
    current_chunk_bytes_read_ = 0;
}

/**
*/
bool ASTERIXDecoderFile::isSupportedArchive(const ASTERIXImportFileInfo& file_info)
{
    for (const auto& ext : SupportedArchives)
        if (Utils::String::hasEnding(file_info.filename, ext))
            return true;

    return false;
}
