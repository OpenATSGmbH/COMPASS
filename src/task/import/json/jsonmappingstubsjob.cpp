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

#include "jsonmappingstubsjob.h"

//#include "buffer.h"
//#include "dbcontent/dbcontent.h"
#include "json_tools.h"
#include "jsonobjectparser.h"

using namespace Utils;

JSONMappingStubsJob::JSONMappingStubsJob(std::unique_ptr<nlohmann::json> data,
                                         const std::vector<std::string>& data_record_keys,
                                         std::map<std::string, JSONObjectParser>& parsers)
    : Job("JSONMappingStubsJob"),
      data_(std::move(data)),
      data_record_keys_(data_record_keys),
      parsers_(parsers)
{
}

JSONMappingStubsJob::~JSONMappingStubsJob() {}

void JSONMappingStubsJob::run_impl()
{
    logdbg;

    started_ = true;

    auto process_lambda = [this](nlohmann::json& record) {
        for (auto& map_it : parsers_)
        {
            logdbg << "mapping json: obj "
                   << map_it.second.dbContentName();
            map_it.second.createMappingStubs(record);
        }
    };

    logdbg << "mapping json";

    traced_assert(data_);
    loginf << "applying JSON function";
    JSON::applyFunctionToValues(*data_.get(), data_record_keys_, data_record_keys_.begin(),
                                process_lambda, false);

    done_ = true;
    logdbg << "done";
}
