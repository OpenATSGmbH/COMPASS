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

#include "jsonmappingjob.h"

#include "buffer.h"
//#include "dbcontent/dbcontent.h"
#include "json_tools.h"
#include "jsonobjectparser.h"
#include "asterixjsonparser.h"
#include "logger.h"

#include <exception>

using namespace std;
using namespace Utils;
using namespace nlohmann;

JSONMappingJob::JSONMappingJob(std::unique_ptr<nlohmann::json> data,
                               const std::vector<std::string>& data_record_keys,
                               unsigned int line_id,
                               const std::map<std::string, std::unique_ptr<JSONObjectParser>>& parsers)
    : Job("JSONMappingJob"),
      data_(std::move(data)),
      data_record_keys_(data_record_keys), line_id_(line_id),
      json_parsers_(&parsers), asterix_parsers_(nullptr)
{
    logdbg << "start";
}

JSONMappingJob::JSONMappingJob(std::unique_ptr<nlohmann::json> data,
               const std::vector<std::string>& data_record_keys,
                               unsigned int line_id,
               const std::map<unsigned int, std::unique_ptr<ASTERIXJSONParser>>& parsers)
    : Job("JSONMappingJob"),
      data_(std::move(data)),
      data_record_keys_(data_record_keys), line_id_(line_id),
      json_parsers_(nullptr), asterix_parsers_(&parsers)
{
    logdbg << "start";

}

JSONMappingJob::~JSONMappingJob()
{
    loginf;
    traced_assert(done_);
}

void JSONMappingJob::run_impl()
{
    logdbg << "start";

    started_ = true;

    if (json_parsers_)
        parseJSON();
    else
    {
        traced_assert(asterix_parsers_);
        parseASTERIX();
    }

    if (obsolete_)
    {
        done_ = true;
        return;
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

    done_ = true;
    data_ = nullptr;

    logdbg << "done: mapped " << num_created_ << " skipped "
           << num_not_mapped_;
}

size_t JSONMappingJob::numMapped() const { return num_mapped_; }

size_t JSONMappingJob::numNotMapped() const { return num_not_mapped_; }

size_t JSONMappingJob::numCreated() const { return num_created_; }

std::map<unsigned int, std::pair<size_t, size_t>> JSONMappingJob::categoryMappedCounts() const
{
    return category_mapped_counts_;
}

size_t JSONMappingJob::numErrors() const
{
    return num_errors_;
}

void JSONMappingJob::parseJSON()
{
    traced_assert(!asterix_parsers_);

    for (auto& parser_it : *json_parsers_)
    {
        if (!parser_it.second->active())
            continue;

        if (!buffers_.count(parser_it.second->dbContentName()))
            buffers_[parser_it.second->dbContentName()] = parser_it.second->getNewBuffer();
        else
            parser_it.second->appendVariablesToBuffer(
                *buffers_.at(parser_it.second->dbContentName()));
    }

    auto process_lambda = [this](nlohmann::json& record) {
        //loginf << "UGA '" << record.dump(4) << "'";

        unsigned int category{0};
        bool has_cat = record.contains("category");

        record["line_id"] = line_id_;

        if (has_cat)
            category = record.at("category");

        bool parsed{false};
        bool parsed_any{false};

        for (auto& map_it : *json_parsers_)
        {
            if (!map_it.second->active())
                continue;

            logdbg << "mapping json: obj " << map_it.second->dbContentName();
            std::shared_ptr<Buffer>& buffer = buffers_.at(map_it.second->dbContentName());
            traced_assert(buffer);
            try
            {
                logdbg << "obj " << map_it.second->dbContentName() << " parsing JSON";

                parsed = map_it.second->parseJSON(record, *buffer);

                logdbg << "obj " << map_it.second->dbContentName() << " done";

                parsed_any |= parsed;
            }
            catch (exception& e)
            {
                logerr << "caught exception '" << e.what() << "' in \n'"
                       << record.dump(4) << "' parser " << map_it.second->dbContentName();

                ++num_errors_;

                continue;
            }
        }

        if (parsed_any)
        {
            if (has_cat)
                category_mapped_counts_[category].first += 1;
            ++num_mapped_;
        }
        else
        {
            if (has_cat)
                category_mapped_counts_[category].second += 1;
            ++num_not_mapped_;
        }
    };

    if (obsolete_)
    {
        done_ = true;
        return;
    }
    traced_assert(data_);
    logdbg << "applying JSON function";

    JSON::applyFunctionToValues(*data_.get(), data_record_keys_, data_record_keys_.begin(),
                                process_lambda, false);
}

void JSONMappingJob::parseASTERIX()
{
    traced_assert(!json_parsers_);

    for (auto& parser_it : *asterix_parsers_)
    {
        if (!buffers_.count(parser_it.second->dbContentName()))
            buffers_[parser_it.second->dbContentName()] = parser_it.second->getNewBuffer();
        else
            parser_it.second->appendVariablesToBuffer(
                *buffers_.at(parser_it.second->dbContentName()));
    }

    auto process_lambda = [this](nlohmann::json& record) {
        //loginf << "UGA '" << record.dump(4) << "'";

        unsigned int category{0};
        traced_assert(record.contains("category"));

        category = record.at("category");

        record["line_id"] = line_id_;

        bool parsed{false};
        bool parsed_any{false};

        if (!asterix_parsers_->count(category))
            return;

        const unique_ptr<ASTERIXJSONParser>& parser = asterix_parsers_->at(category);

        string dbcontent_name = parser->dbContentName();

        logdbg << "mapping json: cat " << category;

        std::shared_ptr<Buffer>& buffer = buffers_.at(dbcontent_name);
        traced_assert(buffer);

        try
        {
            logdbg << "obj " << dbcontent_name << " parsing JSON";

            parsed = parser->parseJSON(record, *buffer);

//            if (parsed)
//            {
//                logdbg << "obj " << parser.dbObject().name() << " transforming buffer";
//                //parser.transformBuffer(*buffer, buffer->size() - 1);
//            }

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

    if (obsolete_)
    {
        done_ = true;
        return;
    }

    traced_assert(data_);
    logdbg << "applying JSON function";

    JSON::applyFunctionToValues(*data_.get(), data_record_keys_, data_record_keys_.begin(),
                                process_lambda, false);
}
