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

using namespace std;

JSONParsingSchema::JSONParsingSchema(const std::string& class_id, const std::string& instance_id,
                                     Configurable* parent)
    : Configurable(class_id, instance_id, parent)
{
    registerParameter("name", &name_, std::string());

    traced_assert(name_.size());

    createSubConfigurables();
}

void JSONParsingSchema::generateSubConfigurable(const std::string& class_id,
                                                const std::string& instance_id)
{
    if (class_id == "JSONObjectParser")
    {
        const Configuration& sub_config = getSubConfiguration(class_id, instance_id);

        std::string name;

        if (sub_config.hasParameterConfigValue("name"))
            name = sub_config.getParameterConfigValue<std::string>("name");

        if (!name.size() &&
            sub_config.hasParameterConfigValue("dbcontent_name"))  // name not set hack
            name = sub_config.getParameterConfigValue<std::string>("dbcontent_name");

        traced_assert(name.size());
        traced_assert(parsers_.find(name) == parsers_.end());

        logdbg << "generating schema " << instance_id
               << " with name " << name;

//        parsers_.emplace(
//            std::piecewise_construct,
//            std::forward_as_tuple(name),                          // args for key
//            std::forward_as_tuple(class_id, instance_id, this));  // args for mapped value

        parsers_[name].reset(new JSONObjectParser (class_id, instance_id, this));
    }
    else
        throw std::runtime_error("JSONImporterTask: generateSubConfigurable: unknown class_id " +
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
