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

#include "jsonparsingschema.h"
#include "jsonimporttask.h"

using namespace std;

JSONParsingSchema::JSONParsingSchema(nlohmann::json& config, JSONImportTask* parent)
    : Configurable(config, parent)
{
    registerParameter("name", &name_, std::string());

    traced_assert(name_.size());

    createSubConfigurables();
}

void JSONParsingSchema::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_id = Configuration::getClassName(child_json);
    const auto& instance_id = Configuration::getInstanceName(child_json);

    if (class_id == "JSONObjectParser")
    {
        std::string name;

        if (child_json.contains("parameters"))
        {
            auto& params = child_json["parameters"];
            if (params.contains("name"))
                name = params["name"].get<std::string>();

            if (!name.size() && params.contains("dbcontent_name"))  // name not set hack
                name = params["dbcontent_name"].get<std::string>();

            if (!name.size() && params.contains("db_content_name"))  // current param name
                name = params["db_content_name"].get<std::string>();
        }

        traced_assert(name.size());
        traced_assert(parsers_.find(name) == parsers_.end());

        logdbg << "generating schema " << instance_id
               << " with name " << name;

        auto parser = std::make_unique<JSONObjectParser>(child_json, this);
        parsers_[name] = std::move(parser);
    }
    else
        throw std::runtime_error("JSONParsingSchema: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

std::string JSONParsingSchema::name() const { return name_; }

void JSONParsingSchema::name(const std::string& name) { name_ = name; }

JSONObjectParser& JSONParsingSchema::parser(const std::string& name)
{
    traced_assert(hasObjectParser(name));
    return *parsers_.at(name);
}

void JSONParsingSchema::removeParser(const std::string& name)
{
    traced_assert(hasObjectParser(name));
    parsers_.erase(name);
}

void JSONParsingSchema::updateMappings()
{
    for (auto& p_it : parsers_)
        p_it.second->updateMappings();
}
