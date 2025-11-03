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
#include "json_tools.h"
#include "json.hpp"
#include "asterixpostprocess.h"
#include "logger.h"
#include "traced_assert.h"

using namespace nlohmann;
using namespace Utils;

JSONParseJob::JSONParseJob(std::vector<std::string> objects, const std::string& current_schema,
                           ASTERIXPostProcess& post_process)
    : Job("JSONParseJob"), objects_(std::move(objects)), current_schema_(current_schema),
      post_process_(post_process)
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

        unsigned int category{0};

        auto process_lambda = [this, &category](nlohmann::json& record) {
            post_process_.postProcess(category, record);
        };

        try
        {
            *json_objects_ = json::parse(objects_.at(0));

            if (json_objects_->contains("data_blocks")) // no framing
            {
                logdbg << "data blocks found";

                traced_assert(json_objects_->at("data_blocks").is_array());

                std::vector<std::string> keys{"content", "records"};

                for (json& data_block : json_objects_->at("data_blocks"))
                {
                    if (!data_block.contains("category"))
                    {
                        logwrn << "data block without asterix category";
                        continue;
                    }

                    category = data_block.at("category");

                    if (category == 1)
                        checkCAT001SacSics(data_block);

                    logdbg << "applying JSON function without framing";
                    JSON::applyFunctionToValues(data_block, keys, keys.begin(), process_lambda, false);
                }
            }
            else // framed
            {
                logdbg << "no data blocks found, framed";

                traced_assert(json_objects_->contains("frames"));
                traced_assert(json_objects_->at("frames").is_array());

                std::vector<std::string> keys{"content", "records"};

                for (json& frame : json_objects_->at("frames"))
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
                    }
                }
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

// equivalent function in ASTERIXDecodeJob
void JSONParseJob::checkCAT001SacSics(nlohmann::json& data_block)
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
