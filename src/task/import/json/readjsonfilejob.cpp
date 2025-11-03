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

#include "readjsonfilejob.h"
#include "traced_assert.h"

#include <archive.h>
#include <archive_entry.h>

#include <QThread>
#include <regex>

#include "logger.h"
#include "stringconv.h"

using namespace Utils;

ReadJSONFileJob::ReadJSONFileJob(const std::string& file_name, unsigned int num_objects)
    : Job("ReadJSONFileJob"), file_name_(file_name), num_objects_(num_objects)
{
    archive_ = String::hasEnding(file_name_, ".zip") || String::hasEnding(file_name_, ".gz") ||
               String::hasEnding(file_name_, ".tgz") || String::hasEnding(file_name_, ".tar");

    loginf << "filename " << file_name_ << "' archive " << archive_;
}
ReadJSONFileJob::~ReadJSONFileJob()
{
    if (archive_)
        closeArchive();
    else
        file_stream_.close();
}

void ReadJSONFileJob::run_impl()
{
    logdbg;
    started_ = true;

    traced_assert(!done_);
    traced_assert(!file_read_done_);
    traced_assert(!objects_.size());
    traced_assert(!bytes_read_tmp_);

    if (!init_performed_)
    {
        performInit();
        traced_assert(init_performed_);
    }

    while (!file_read_done_)  //&& objects_.size() < num_objects_
    {
        while (pause_)
            QThread::msleep(1);

        bytes_read_tmp_ = 0;
        readFilePart();

        if(objects_.size())
        {
            emit readJSONFilePartSignal();

            while (objects_.size())
                QThread::msleep(1);
        }
    }

    // cleanCommas ();

    done_ = true;

    logdbg << "done";
    return;
}

void ReadJSONFileJob::performInit()
{
    traced_assert(!init_performed_);

    if (archive_)
    {
        // if gz but not tar.gz or tgz
        bool raw =
            String::hasEnding(file_name_, ".gz") && !String::hasEnding(file_name_, ".tar.gz");

        loginf << "importing " << file_name_ << " raw " << raw;

        openArchive(raw);

        while (archive_read_next_header(a, &entry) == ARCHIVE_OK)
        {
            loginf << "got " << archive_entry_pathname(entry)
                   << " size " << archive_entry_size(entry);
            bytes_to_read_ += archive_entry_size(entry);
        }

        closeArchive();
        openArchive(raw);  // slightly dirty

        loginf << "archive size " << bytes_to_read_;
    }
    else
    {
        file_stream_.open(file_name_, std::ios::ate);
        bytes_to_read_ = file_stream_.tellg();
        loginf << "non-archive size " << bytes_to_read_;
        file_stream_.seekg(0);
    }

    init_performed_ = true;
}

void ReadJSONFileJob::readFilePart()
{
    loginf << "begin";

    if (archive_)
    {
        logdbg << "archive";

        const void* buff;
        size_t size;

        int r;
        bool closed_bracked = false;

        while (1)
        {
            if (entry_done_)
            {
                logdbg << "reading next archive entry";

                r = archive_read_next_header(a, &entry);

                if (r != ARCHIVE_OK)
                {
                    logdbg << "reading not ok '" << r << "'";

                    if (r == ARCHIVE_FAILED || r == ARCHIVE_FATAL || r == ARCHIVE_RETRY)
                        throw std::runtime_error("ReadJSONFileJob: readFilePart: header error: " +
                                                 std::string(archive_error_string(a)));
                    else if (r == ARCHIVE_WARN)
                        logwrn << "header error: "
                               << std::string(archive_error_string(a));
                    else if (r == ARCHIVE_EOF)
                    {
                        logdbg << "end of archive";
                        break;
                    }
                    else
                        throw std::runtime_error(
                            "ReadJSONFileJob: readFilePart: unknown header error: " +
                            std::string(archive_error_string(a)));
                }

                logdbg << "reading ok";
            }

            loginf << "parsing archive file: "
                   << archive_entry_pathname(entry) << " size " << archive_entry_size(entry);

            for (;;)
            {
                r = archive_read_data_block(a, &buff, &size, &offset);

                if (r == ARCHIVE_EOF)
                {
                    entry_done_ = true;
                    break;
                }

                if (r != ARCHIVE_OK)
                {
                    if (r == ARCHIVE_FAILED || r == ARCHIVE_FATAL || r == ARCHIVE_RETRY)
                        throw std::runtime_error(
                            "ReadJSONFileJob: readFilePart: data block error: " +
                            std::string(archive_error_string(a)));
                    else if (r == ARCHIVE_WARN)
                        logwrn << "header error: "
                               << std::string(archive_error_string(a));
                    else
                        throw std::runtime_error(
                            "ReadJSONFileJob: readFilePart: unknown data block error: " +
                            std::string(archive_error_string(a)));
                }

                std::string str(reinterpret_cast<char const*>(buff), size);

                for (char c : str)
                {
                    closed_bracked = false;

                    if (c == '{')
                        ++open_count_;
                    else if (c == '}')
                    {
                        --open_count_;
                        closed_bracked = true;
                    }

                    if (open_count_ || closed_bracked)  // only add if enclosed by brackets
                        tmp_stream_ << c;

                    ++bytes_read_;
                    ++bytes_read_tmp_;

                    if (c == '\n')  // next lines after objects
                        continue;

                    if (closed_bracked && open_count_ == 0)
                    {
                        objects_.push_back(tmp_stream_.str());
                        tmp_stream_.str("");
                    }
                }

                if (objects_.size() > num_objects_ || (objects_.size() && bytes_read_tmp_ > 1e7))
                // parsed buffer, reached obj limit
                {
                    //|| (objects_.size() && bytes_read_tmp_ > 1e7)
                    loginf << "reached limit, returning";
                    entry_done_ = false;
                    return;
                }
                else  // parsed buffer, continue
                {
                    entry_done_ = true;
                }
            }

            if (entry_done_)  // will read next entry
            {
                traced_assert(open_count_ == 0);  // nothing left open
                traced_assert(tmp_stream_.str().size() == 0 || tmp_stream_.str() == "\n");
                loginf << "entry done";
                return;
            }
        }

        loginf << "archive done";

        traced_assert(open_count_ == 0);  // nothing left open
        traced_assert(tmp_stream_.str().size() == 0 || tmp_stream_.str() == "\n");

        file_read_done_ = true;
    }
    else
    {
        char c;
        bool closed_bracked = false;

        while (file_stream_.get(c) &&
               objects_.size() < num_objects_)  // loop getting single characters
        {
            closed_bracked = false;

            if (c == '{')
                ++open_count_;
            else if (c == '}')
            {
                --open_count_;
                closed_bracked = true;
            }

            if (open_count_ || closed_bracked)  // only add if enclosed by brackets
                tmp_stream_ << c;

            ++bytes_read_;

            if (c == '\n')  // next lines after objects
                continue;

            if (closed_bracked && open_count_ == 0)
            {
                // loginf << "obj '" << tmp_stream_.str() << "'";
                objects_.push_back(tmp_stream_.str());
                tmp_stream_.str("");
            }
        }

        if (objects_.size() != num_objects_)
            file_read_done_ = true;

        loginf << "parsed " << objects_.size() << " done "
               << file_read_done_;

        traced_assert(open_count_ == 0);  // nothing left open
        traced_assert(tmp_stream_.str().size() == 0 || tmp_stream_.str() == "\n");
    }

    loginf << "emitting signal";

    loginf << "done";
}

void ReadJSONFileJob::pause() { pause_ = true; }

void ReadJSONFileJob::unpause() { pause_ = false; }

// void ReadJSONFileJob::resetDone ()
//{
//    traced_assert(!file_read_done_);
//    traced_assert(!objects_.size());
//    done_ = false; // yet another part
//    bytes_read_tmp_ = 0;
//}

// bool ReadJSONFileJob::fileReadDone() const
//{
//    return file_read_done_;
//}

std::vector<std::string> ReadJSONFileJob::objects() { return std::move(objects_); }

size_t ReadJSONFileJob::bytesRead() const { return bytes_read_; }

size_t ReadJSONFileJob::bytesToRead() const { return bytes_to_read_; }

void ReadJSONFileJob::openArchive(bool raw)
{
    int r;

    a = archive_read_new();

    if (raw)
    {
        archive_read_support_filter_gzip(a);
        archive_read_support_filter_bzip2(a);
        archive_read_support_format_raw(a);
    }
    else
    {
        archive_read_support_filter_all(a);
        archive_read_support_format_all(a);
    }
    r = archive_read_open_filename(a, file_name_.c_str(), 10240);  // Note 1

    if (r != ARCHIVE_OK)
        logerr << "archive open error: "
               << std::string(archive_error_string(a));
}
void ReadJSONFileJob::closeArchive()
{
    int r = archive_read_close(a);
    if (r != ARCHIVE_OK)
        logerr << "archive read close error: "
               << std::string(archive_error_string(a));
    else
    {
        r = archive_read_free(a);

        if (r != ARCHIVE_OK)
            logerr << "archive read free error: "
                   << std::string(archive_error_string(a));
    }

    if (r != ARCHIVE_OK)
        throw std::runtime_error("ReadJSONFileJob: closeArchive: archive error: " +
                                 std::string(archive_error_string(a)));
}

float ReadJSONFileJob::getStatusPercent()
{
    if (bytes_to_read_ == 0)
        return 0.0;
    else
        return 100.0 * static_cast<double>(bytes_read_) / static_cast<double>(bytes_to_read_);
}

void ReadJSONFileJob::cleanCommas()
{
    loginf << "start" << objects_.size() << " objects";

    std::regex commas_between_brackets("\\[(,|\n)+\\]");
    std::regex multiple_commas(",\\n*,+");
    std::regex stupid_commas_at_bracket_begin("\\[\\n*,+");
    std::regex stupid_commas_at_bracket_end(",+\\n*\\]");

    for (auto& str_it : objects_)
    {
        str_it = std::regex_replace(str_it, commas_between_brackets, {"[]"});
        str_it = std::regex_replace(str_it, multiple_commas, {","});
        str_it = std::regex_replace(str_it, stupid_commas_at_bracket_begin, {"["});
        str_it = std::regex_replace(str_it, stupid_commas_at_bracket_end, {"]"});

        // \[(,|\n)+\]   commas between brackets
        // /,(?!["{}[\]])/g  multiple commas
    }
}
