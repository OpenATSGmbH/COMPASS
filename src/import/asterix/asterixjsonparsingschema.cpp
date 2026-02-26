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

#include "asterixjsonparsingschema.h"
#include "asteriximporttask.h"
#include "traced_assert.h"

ASTERIXJSONParsingSchema::ASTERIXJSONParsingSchema(nlohmann::json& config, ASTERIXImportTask& task)
    : Configurable(config, &task), task_(task)
{
    registerParameter<std::string>("name", &name_, "");

    traced_assert(name_.size());

    createSubConfigurables();
}

//ASTERIXJSONParsingSchema& ASTERIXJSONParsingSchema::operator=(ASTERIXJSONParsingSchema&& other)
//{
//    name_ = other.name_;

//    parsers_ = std::move(other.parsers_);

//    other.configuration().updateParameterPointer("name", &name_);

//    //    widget_ = std::move(other.widget_);
//    //    if (widget_)
//    //        widget_->setParser(*this);
//    //    other.widget_ = nullptr;

//    return static_cast<ASTERIXJSONParsingSchema&>(Configurable::operator=(std::move(other)));
//}

void ASTERIXJSONParsingSchema::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_id = Configuration::getClassName(child_json);
    const auto& instance_id = Configuration::getInstanceName(child_json);

    if (class_id == "ASTERIXJSONParser")
    {
        unsigned int category{0};

        if (child_json.contains("parameters") && child_json["parameters"].contains("category"))
            category = child_json["parameters"]["category"].get<unsigned int>();

        traced_assert(parsers_.find(category) == parsers_.end());

        logdbg << "generating schema " << instance_id
               << " for cat  " << category;

        auto parser = std::make_unique<ASTERIXJSONParser>(child_json, task_, task_.compass(), this);
        parsers_[category] = std::move(parser);
    }
    else
    {
        throw std::runtime_error("ASTERIXJSONParsingSchema: generateSubConfigurable: unknown class_id " +
                                 class_id);
    }
}

std::string ASTERIXJSONParsingSchema::name() const { return name_; }

void ASTERIXJSONParsingSchema::name(const std::string& name) { name_ = name; }

ASTERIXJSONParser& ASTERIXJSONParsingSchema::parser(unsigned int category)
{
    traced_assert(hasObjectParser(category));
    return *parsers_.at(category).get();
}

void ASTERIXJSONParsingSchema::removeParser(unsigned int category)
{
    traced_assert(hasObjectParser(category));
    parsers_.erase(category);
}

//void ASTERIXJSONParsingSchema::updateMappings()
//{
//    for (auto& p_it : parsers_)
//        p_it.second->updateMappings();
//}

