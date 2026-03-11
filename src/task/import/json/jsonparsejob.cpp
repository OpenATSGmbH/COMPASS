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

#include "jsonparsejob.h"
#include "json.hpp"
#include "asterixpostprocess.h"
#include "logger.h"
#include "traced_assert.h"

using namespace nlohmann;

JSONParseJob::JSONParseJob(std::vector<std::string> objects, const std::string& current_schema,
                           unsigned int line_id, ASTERIXPostProcess& post_process)
    : Job("JSONParseJob"), objects_(std::move(objects)), current_schema_(current_schema),
      line_id_(line_id), post_process_(post_process)
{
}

JSONParseJob::~JSONParseJob() {}

void JSONParseJob::run_impl()
{
    loginf << "start with " << objects_.size() << " objects schema '" << current_schema_ << "'";

    started_ = true;
    traced_assert(!json_objects_);
    json_objects_.reset(new json());

    if (current_schema_ == "jASTERIX")
    {
        traced_assert(objects_.size() == 1);

        try
        {
            *json_objects_ = json::parse(objects_.at(0));

            // flat format: top-level keys are category number strings
            for (auto it = json_objects_->begin(); it != json_objects_->end(); ++it)
            {
                if (!it.value().is_object())
                    continue;

                unsigned int category = 0;
                try
                {
                    category = std::stoul(it.key());
                }
                catch (...)
                {
                    continue; // skip non-numeric keys
                }

                // determine num_records from longest array
                size_t num_records = 0;
                for (auto arr_it = it.value().begin(); arr_it != it.value().end(); ++arr_it)
                {
                    if (arr_it.value().is_array() && arr_it.value().size() > num_records)
                        num_records = arr_it.value().size();
                }

                if (num_records > 0)
                    post_process_.postProcessFlat(category, line_id_, it.value(), num_records);
            }
        }
        catch (nlohmann::detail::parse_error& e)
        {
            logwrn << "parse error " << e.what() << " in '" << objects_.at(0) << "'";
            ++parse_errors_;
        }
        ++objects_parsed_;
    }
    else
    {
        (*json_objects_)["data"] = json::array();

        json& records = json_objects_->at("data");

        for (auto& str_it : objects_)
        {
            try
            {
                records.push_back(json::parse(str_it));
            }
            catch (nlohmann::detail::parse_error& e)
            {
                logwrn << "parse error " << e.what() << " in '" << str_it << "'";
                ++parse_errors_;
                continue;
            }
            ++objects_parsed_;
        }
    }

    loginf << "done with " << objects_parsed_ << " objects, errors "
           << parse_errors_;

    done_ = true;
}

size_t JSONParseJob::objectsParsed() const { return objects_parsed_; }

size_t JSONParseJob::parseErrors() const { return parse_errors_; }

std::unique_ptr<nlohmann::json> JSONParseJob::jsonObjects() { return std::move(json_objects_); }
