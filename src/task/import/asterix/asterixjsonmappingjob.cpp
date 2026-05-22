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

#include "asterixjsonmappingjob.h"
#include "asterixjsonparser.h"
#include "buffer.h"
#include "logger.h"

#include <exception>

#include <QThread>

using namespace std;
using namespace Utils;
using namespace nlohmann;

ASTERIXJSONMappingJob::ASTERIXJSONMappingJob(std::vector<std::unique_ptr<nlohmann::json>> data,
                                             const std::string& source_name,
                                             const std::map<unsigned int, std::unique_ptr<ASTERIXJSONParser>>& parsers)
    : Job("ASTERIXJSONMappingJob"),
    data_(std::move(data)),
    source_name_(source_name),
    parsers_(parsers)
{
    logdbg;
}

ASTERIXJSONMappingJob::~ASTERIXJSONMappingJob()
{
    logdbg;
}

void ASTERIXJSONMappingJob::run_impl()
{
    logdbg << "run on thread " << QThread::currentThreadId() << " on cpu " << sched_getcpu();

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    started_ = true;

    logdbg << "ASTERIXJSONMappingJob: run_impl: num parsers " << parsers_.size()
           << " num data slices " << data_.size();

    string dbcontent_name;

    for (auto& parser_it : parsers_)
    {
        dbcontent_name = parser_it.second->dbContentName();
        logdbg << "ASTERIXJSONMappingJob: run_impl: parser cat " << parser_it.first
               << " dbcontent '" << dbcontent_name << "'";

        if (!buffers_.count(dbcontent_name))
            buffers_[dbcontent_name] = parser_it.second->getNewBuffer();
        else
            parser_it.second->appendVariablesToBuffer(*buffers_.at(dbcontent_name));
    }

    // flat format: top-level keys are category numbers, values are objects with array columns
    for (auto& data_slice : data_)
    {
        if (!data_slice || !data_slice->is_object())
            continue;

        if (this->obsolete_)
            break;

        for (auto it = data_slice->begin(); it != data_slice->end(); ++it)
        {
            if (this->obsolete_)
                break;

            unsigned int category = 0;
            try
            {
                category = std::stoul(it.key());
            }
            catch (...)
            {
                continue; // skip non-numeric keys (e.g. metadata)
            }

            if (!parsers_.count(category))
                continue;

            const unique_ptr<ASTERIXJSONParser>& parser = parsers_.at(category);
            string dbcontent_name = parser->dbContentName();

            std::shared_ptr<Buffer>& buffer = buffers_.at(dbcontent_name);
            traced_assert(buffer);

            logdbg << "ASTERIXJSONMappingJob: cat " << category
                   << " dbcontent " << dbcontent_name;

            try
            {
                size_t mapped = parser->parseFlatJSON(it.value(), *buffer);

                if (mapped > 0)
                {
                    category_mapped_counts_[category].first += mapped;
                    num_mapped_ += mapped;
                }
            }
            catch (exception& e)
            {
                logerr << "ASTERIXJSONMappingJob: caught exception '"
                       << e.what() << "' for cat " << category;
                ++num_errors_;
            }
        }
    }

    logdbg << "ASTERIXJSONMappingJob: run_impl:"
           << " mapped " << num_mapped_ << " not_mapped " << num_not_mapped_ << " errors " << num_errors_;

    std::map<std::string, std::shared_ptr<Buffer>> not_empty_buffers;

    logdbg << "counting buffer sizes";
    for (auto& buf_it : buffers_)
    {
        if (buf_it.second && buf_it.second->size())
        {
            num_created_ += buf_it.second->size();
            not_empty_buffers[buf_it.first] = buf_it.second;
        }
    }
    buffers_ = not_empty_buffers;  // cleaner

    data_.clear();

    done_ = true;

    auto t_diff = boost::posix_time::microsec_clock::local_time() - start_time;
    float num_secs =  t_diff.total_milliseconds() ? t_diff.total_milliseconds() / 1000.0 : 10E-6;

    logdbg << "done: took "
           << String::timeStringFromDouble(num_secs, true)
           << " full " << String::timeStringFromDouble(num_secs, true)
           << " " << ((float) num_created_+num_not_mapped_) / num_secs << " rec/s";

    logdbg << "done: mapped " << num_created_ << " skipped "
           << num_not_mapped_;
}

std::string ASTERIXJSONMappingJob::sourceName() const
{
    return source_name_;
}

size_t ASTERIXJSONMappingJob::numMapped() const { return num_mapped_; }

size_t ASTERIXJSONMappingJob::numNotMapped() const { return num_not_mapped_; }

size_t ASTERIXJSONMappingJob::numCreated() const { return num_created_; }

std::map<unsigned int, std::pair<size_t, size_t>> ASTERIXJSONMappingJob::categoryMappedCounts() const
{
    return category_mapped_counts_;
}

size_t ASTERIXJSONMappingJob::numErrors() const
{
    return num_errors_;
}
