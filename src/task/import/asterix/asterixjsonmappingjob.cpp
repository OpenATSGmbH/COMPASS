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
//#include "dbcontent/dbcontent.h"
#include "json_tools.h"
//#include "jsonobjectparser.h"
#include "logger.h"

#include <exception>

#include <QThread>

using namespace std;
using namespace Utils;
using namespace nlohmann;

ASTERIXJSONMappingJob::ASTERIXJSONMappingJob(std::vector<std::unique_ptr<nlohmann::json>> data,
                                             const std::string& source_name,
                                             const std::vector<std::string>& data_record_keys,
                                             const std::map<unsigned int, std::unique_ptr<ASTERIXJSONParser>>& parsers)
    : Job("ASTERIXJSONMappingJob"),
    data_(std::move(data)),
    source_name_(source_name),
    data_record_keys_(data_record_keys),
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

    string dbcontent_name;

    for (auto& parser_it : parsers_)
    {
        dbcontent_name = parser_it.second->dbContentName();

        if (!buffers_.count(dbcontent_name))
            buffers_[dbcontent_name] = parser_it.second->getNewBuffer();
        else
            parser_it.second->appendVariablesToBuffer(*buffers_.at(dbcontent_name));
    }

    auto process_lambda = [this](nlohmann::json& record) 
    {
        //loginf << "UGA '" << record.dump(4) << "'";

        if (this->obsolete_)
            return;

        unsigned int category{0};

        if (!record.contains("category"))
        {
            logerr << "record without category '" << record.dump(4) << "', skipping";
            return;
        }

        traced_assert(record.contains("category"));

        category = record.at("category");

        bool parsed{false};
        bool parsed_any{false};

        if (!parsers_.count(category))
            return;

        const unique_ptr<ASTERIXJSONParser>& parser = parsers_.at(category);

        string dbcontent_name = parser->dbContentName();

        logdbg << "mapping json: cat " << category;

        std::shared_ptr<Buffer>& buffer = buffers_.at(dbcontent_name);
        traced_assert(buffer);

        try
        {
            logdbg << "obj " << dbcontent_name << " parsing JSON";

            parsed = parser->parseJSON(record, *buffer);

            logdbg << "obj " << dbcontent_name << " done";

            parsed_any |= parsed;
        }
        catch (exception& e)
        {
            logerr << "caught exception '" << e.what() << "' in \n'"
                       << record.dump(4) << "' parser dbcont " << dbcontent_name;

            ++num_errors_;

            return;
        }

        if (parsed_any)
        {
            category_mapped_counts_[category].first += 1;
            ++num_mapped_;
        }
        else
        {
            category_mapped_counts_[category].second += 1;
            ++num_not_mapped_;
        }
    };

    for (auto& data_slice : data_)
    {
        if (data_slice)
        {
            logdbg << "applying JSON function";
            JSON::applyFunctionToValues(*data_slice.get(), data_record_keys_, data_record_keys_.begin(),
                                        process_lambda, false);
        }
    }

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
