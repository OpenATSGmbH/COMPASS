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

#include "jsonobjectparser.h"

#include "compass.h"
#include "buffer.h"
#include "configuration.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "stringconv.h"
//#include "unit.h"
//#include "unitmanager.h"
#include "util/json_tools.h"

#include <algorithm>

using namespace std;
using namespace nlohmann;
using namespace Utils;

JSONObjectParser::JSONObjectParser(const std::string& class_id, const std::string& instance_id,
                                   Configurable* parent)
    : Configurable(class_id, instance_id, parent)
{
    registerParameter("name", &name_, std::string());
    registerParameter("active", &active_, true);
    registerParameter("db_content_name", &db_content_name_, std::string());

    registerParameter("json_container_key", &json_container_key_, std::string());
    registerParameter("json_key", &json_key_, std::string("*"));
    registerParameter("json_value", &json_value_, std::string());

    registerParameter("override_data_source", &override_data_source_, false);
    registerParameter("data_source_variable_name", &data_source_variable_name_, std::string());

    traced_assert(db_content_name_.size());

    if (!name_.size())
        name_ = db_content_name_;

    traced_assert(name_.size());

    createSubConfigurables();

    json_values_vector_ = String::split(json_value_, ',');

    traced_assert(!override_data_source_); // TODO reimplement
}

void JSONObjectParser::generateSubConfigurable(const std::string& class_id,
                                               const std::string& instance_id)
{
    if (class_id == "JSONDataMapping")
    {
        data_mappings_.emplace_back(new JSONDataMapping(class_id, instance_id, *this));
    }
    else
        throw std::runtime_error("JSONObjectParser: generateSubConfigurable: unknown class_id " + class_id);
}

DBContent& JSONObjectParser::dbContent() const
{
    traced_assert(dbcontent_);
    return *dbcontent_;
}

std::string JSONObjectParser::JSONKey() const { return json_key_; }

void JSONObjectParser::JSONKey(const std::string& json_key)
{
    loginf << "start" << json_key;

    json_key_ = json_key;
}

std::string JSONObjectParser::JSONValue() const { return json_value_; }

void JSONObjectParser::JSONValue(const std::string& json_value)
{
    loginf << "start" << json_value;

    json_value_ = json_value;
    json_values_vector_ = String::split(json_value_, ',');
}

std::string JSONObjectParser::JSONContainerKey() const { return json_container_key_; }

void JSONObjectParser::JSONContainerKey(const std::string& key)
{
    loginf << "start" << key;

    json_container_key_ = key;
}

void JSONObjectParser::initialize()
{
    traced_assert(!dbcontent_);

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (!dbcont_man.existsDBContent(db_content_name_))
        logwrn << "dbcontbject '" << db_content_name_
               << "' does not exist";
    else
        dbcontent_ = &dbcont_man.dbContent(db_content_name_);

    traced_assert(dbcontent_);

    if (!initialized_)
    {
        for (auto& mapping : data_mappings_)
        {
            if (! mapping->active())
            {
                traced_assert(! mapping->mandatory());
                continue;
            }

             mapping->initializeIfRequired();

            list_.addProperty( mapping->variable().name(),  mapping->variable().dataType());
            var_list_.add( mapping->variable());
        }

        traced_assert(!override_data_source_); // TODO reimplement

        if (override_data_source_)
        {
            traced_assert(data_source_variable_name_.size());
            traced_assert(dbcontent_->hasVariable(data_source_variable_name_));

            list_.addProperty(data_source_variable_name_, PropertyDataType::INT);
            var_list_.add(dbcontent_->variable(data_source_variable_name_));
        }

        not_parse_all_ = (json_key_ != "*") && (json_value_ != "*");

        initialized_ = true;
    }
}

std::shared_ptr<Buffer> JSONObjectParser::getNewBuffer() const
{
    traced_assert(initialized_);
    traced_assert(dbcontent_);
    return std::make_shared<Buffer>(list_, dbcontent_->name());
}

void JSONObjectParser::appendVariablesToBuffer(Buffer& buffer) const
{
    traced_assert(initialized_);
    traced_assert(dbcontent_);

    for (auto& p_it : list_.properties())
    {
        if (!buffer.properties().hasProperty(p_it.name()))
        {
            buffer.addProperty(p_it.name(), p_it.dataType());
        }
    }
}

bool JSONObjectParser::parseJSON(nlohmann::json& j, Buffer& buffer) const
{
    traced_assert(initialized_);

    size_t row_cnt = buffer.size();
    size_t skipped_cnt = 0;

    bool parsed_any = false;

    if (json_container_key_.size())
    {
        bool parsed = false;

        if (j.contains(json_container_key_))
        {
            json& ac_list = j.at(json_container_key_);
            traced_assert(ac_list.is_array());

            // loginf << "found target report array in '" << json_container_key_  << "', parsing";

            for (auto tr_it = ac_list.begin(); tr_it != ac_list.end(); ++tr_it)
            {
                // logdbg << "new target report";

                json& tr = tr_it.value();
                traced_assert(tr.is_object());

                parsed = parseTargetReport(tr, buffer, row_cnt);

                if (parsed)
                    ++row_cnt;
                else
                    ++skipped_cnt;

                parsed_any |= parsed;
            }
        }
        else  // parsed stays false
            loginf << "found target report array but '"
                   << json_container_key_ << "' not found";
    }
    else
    {
        logdbg << "found single target report";
        traced_assert(j.is_object());

        parsed_any = parseTargetReport(j, buffer, row_cnt);
    }

    return parsed_any;
}

void JSONObjectParser::createMappingStubs(nlohmann::json& j)
{
    traced_assert(initialized_);

    if (json_container_key_.size())
    {
        if (j.contains(json_container_key_))
        {
            json& ac_list = j.at(json_container_key_);
            traced_assert(ac_list.is_array());

            // loginf << "found target report array in '" << json_container_key_  << "', parsing";

            for (auto tr_it = ac_list.begin(); tr_it != ac_list.end(); ++tr_it)
            {
                // logdbg << "new target report";

                json& tr = tr_it.value();
                traced_assert(tr.is_object());

                createMappingsFromTargetReport(tr);
            }
        }
        else  // parsed stays false
            loginf << "found target report array but '"
                   << json_container_key_ << "' not found";
    }
    else
    {
        logdbg << "found single target report";
        traced_assert(j.is_object());

        createMappingsFromTargetReport(j);
    }

    return;
}

bool JSONObjectParser::parseTargetReport(const nlohmann::json& tr, Buffer& buffer,
                                         size_t row_cnt) const
{
    // check key match
    if (not_parse_all_)
    {
        if (tr.contains(json_key_))
        {
            if (std::find(json_values_vector_.begin(), json_values_vector_.end(),
                          Utils::JSON::toString(tr.at(json_key_))) == json_values_vector_.end())
            {
                logdbg << "skipping because of wrong key "
                       << tr.at(json_key_) << " value " << Utils::JSON::toString(tr.at(json_key_));
                return false;
            }
            else
                logdbg << "parsing with correct key and value";
        }
        else
        {
            logdbg << "skipping because of missing key '"
                   << json_key_ << "'";
            return false;
        }
    }

    PropertyDataType data_type;
    std::string current_var_name;

    bool mandatory_missing{false};

    for (const auto& map_it : data_mappings_)
    {
        if (!map_it->active())
        {
            traced_assert(!map_it->mandatory());
            continue;
        }

        // logdbg << "setting data mapping key " << data_it.jsonKey();

        try
        {

            data_type = map_it->variable().dataType();
            current_var_name = map_it->variable().name();

            switch (data_type)
            {
            case PropertyDataType::BOOL:
            {
                logdbg << "bool " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<bool>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<bool>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::CHAR:
            {
                logdbg << "char " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<char>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<char>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::UCHAR:
            {
                logdbg << "uchar " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<unsigned char>(current_var_name));
                mandatory_missing = map_it->findAndSetValue(
                            tr, buffer.get<unsigned char>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::INT:
            {
                logdbg << "int " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<int>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<int>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::UINT:
            {
                logdbg << "uint " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<unsigned int>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<unsigned int>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::LONGINT:
            {
                logdbg << "long " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<long int>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<long int>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::ULONGINT:
            {
                logdbg << "ulong " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<unsigned long>(current_var_name));
                mandatory_missing = map_it->findAndSetValue(
                            tr, buffer.get<unsigned long>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::FLOAT:
            {
                logdbg << "float " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<float>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<float>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::DOUBLE:
            {
                logdbg << "double " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<double>(current_var_name));
                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<double>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::STRING:
            {
                logdbg << "string " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<std::string>(current_var_name));

                mandatory_missing =
                        map_it->findAndSetValue(tr, buffer.get<std::string>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::JSON:
            {
                logdbg << "json " << current_var_name
                       << " format '" << map_it->jsonValueFormat() << "'";
                traced_assert(buffer.has<nlohmann::json>(current_var_name));

//                mandatory_missing =
//                        map_it->findAndSetValue(tr, buffer.get<nlohmann::json>(current_var_name), row_cnt);
                 map_it->findAndSetValues(tr, buffer.get<json>(current_var_name), row_cnt);

                break;
            }
            case PropertyDataType::TIMESTAMP: // not possible for timestamp
            default:
                logerr << "impossible for property type "
                       << Property::asString(data_type);
                throw std::runtime_error(
                            "JsonMapping: parseTargetReport: impossible property type " +
                            Property::asString(data_type));
            }

        }
        catch (exception& e)
        {
            logerr << "caught exception '" << e.what() << "' in \n'"
                   << tr.dump(4) << "' mapping " << map_it->jsonKey();
            throw e;
        }

        if (mandatory_missing)
        {
            // TODO make configurable
            logdbg << "'" << name_ << "': mandatory variable '"
                   << current_var_name << "' missing in: \n"
                   << tr.dump(4);
            break;
        }
    }

    if (mandatory_missing)
    {
        // cleanup
        if (buffer.size() > row_cnt)
            buffer.cutToSize(row_cnt);
    }

    return !mandatory_missing;
}

void JSONObjectParser::createMappingsFromTargetReport(const nlohmann::json& tr)
{
    // check key match
    if (not_parse_all_)
    {
        if (tr.contains(json_key_))
        {
            if (std::find(json_values_vector_.begin(), json_values_vector_.end(),
                          Utils::JSON::toString(tr.at(json_key_))) == json_values_vector_.end())
            {
                logdbg << "skipping because of "
                          "wrong key "
                       << tr.at(json_key_) << " value " << Utils::JSON::toString(tr.at(json_key_));
                return;
            }
            else
                logdbg << "parsing with correct "
                          "key and value";
        }
        else
        {
            logdbg << "skipping because of "
                      "missing key '"
                   << json_key_ << "'";
            return;
        }
    }

    checkIfKeysExistsInMappings("", tr);
}

void JSONObjectParser::checkIfKeysExistsInMappings(const std::string& location,
                                                   const nlohmann::json& j, bool is_in_array)
{
    if (j.is_array())  // do map arrays
    {
        if (!j.size())  // do not map if empty
            return;

        bool j_array_contains_only_primitives = true;  // only

        for (auto& j_it : j.get<json::array_t>())  // iterate over array
        {
            if (j_it.is_object())  // only parse sub-objects
            {
                j_array_contains_only_primitives = false;
                checkIfKeysExistsInMappings(location, j_it, true);
            }
            else if (j_it.is_array())
                j_array_contains_only_primitives = false;
        }

        if (!j_array_contains_only_primitives)
            return;  // if objects inside, only parse objects
    }

    if (j.is_object())
    {
        for (auto& j_it : j.get<json::object_t>())
        {
            if (location.size())
                checkIfKeysExistsInMappings(location + "." + j_it.first, j_it.second, is_in_array);
            else
                checkIfKeysExistsInMappings(j_it.first, j_it.second, is_in_array);
        }
        return;
    }

    // loginf << "checking key '" << location << "'";

    bool found = false;

    for (auto& map_it : data_mappings_)
    {
        if (map_it->jsonKey() == location)
        {
            found = true;

            if (!map_it->comment().size())
            {
                std::stringstream ss;

                ss << "Type " << j.type_name() << ", value " << j.dump();
                map_it->comment(ss.str());
            }
            break;
        }
    }

    if (!found)
    {
        loginf << "creating new mapping for dbcont "
               << db_content_name_ << "'" << location << "' type " << j.type_name() << " value "
               << j.dump() << " in array " << is_in_array;

        auto new_cfg = Configuration::create("JSONDataMapping");
        new_cfg->addParameter<std::string>("json_key", location);
        new_cfg->addParameter<std::string>("dbcontent_name", db_content_name_);

        if (is_in_array)
            new_cfg->addParameter<bool>("in_array", true);

        std::stringstream ss;
        ss << "Type " << j.type_name() << ", value " << j.dump();
        new_cfg->addParameter<std::string>("comment", ss.str());

        generateSubConfigurableFromConfig(std::move(new_cfg));
    }
}

bool JSONObjectParser::hasMapping(unsigned int index) const
{
    return index < data_mappings_.size();
}
void JSONObjectParser::removeMapping(unsigned int index)
{
    traced_assert(hasMapping(index));

    unique_ptr<JSONDataMapping>& mapping = data_mappings_.at(index);

    loginf << "index " << index << " key " <<  mapping->jsonKey()
           << " instance " <<  mapping->instanceId();

    logdbg << "size " << data_mappings_.size();

    if ( mapping->active() &&  mapping->initialized())
    {
        if (list_.hasProperty( mapping->variable().name()))
            list_.removeProperty( mapping->variable().name());
        if ( mapping->hasVariable() && var_list_.hasVariable( mapping->variable()))
            var_list_.removeVariable( mapping->variable());
    }

    logdbg << "removing";
    data_mappings_.erase(data_mappings_.begin() + index);

    logdbg << "size " << data_mappings_.size();
}


const dbContent::VariableSet& JSONObjectParser::variableList() const { return var_list_; }

bool JSONObjectParser::overrideDataSource() const { return override_data_source_; }

void JSONObjectParser::overrideDataSource(bool override)
{
    loginf << "start" << override;

    traced_assert(!override); // TODO reimplement
    override_data_source_ = override;
}

std::string JSONObjectParser::dataSourceVariableName() const { return data_source_variable_name_; }

void JSONObjectParser::dataSourceVariableName(const std::string& name)
{
    loginf << "start" << name;

    data_source_variable_name_ = name;
}

JSONObjectParserWidget* JSONObjectParser::widget()
{
    if (!widget_)
    {
        widget_.reset(new JSONObjectParserWidget(*this));
        traced_assert(widget_);
    }

    return widget_.get();  // needed for qt integration, not pretty
}

std::string JSONObjectParser::dbContentName() const { return db_content_name_; }

void JSONObjectParser::setMappingActive(JSONDataMapping& mapping, bool active)
{
    if (!mapping.initialized())
         mapping.initializeIfRequired();

    if (active)
    {
        list_.addProperty(mapping.variable().name(), mapping.variable().dataType());
        var_list_.add(mapping.variable());
    }
    else if (mapping.hasVariable())  // remove if was added
    {
        if (list_.hasProperty(mapping.variable().name()))
            list_.removeProperty(mapping.variable().name());
        if (var_list_.hasVariable(mapping.variable()))
            var_list_.removeVariable(mapping.variable());
    }

     mapping.active(active);
}

void JSONObjectParser::updateMappings()
{
    if (widget_)
        widget_->updateMappingsGrid();
}

std::string JSONObjectParser::name() const { return name_; }

void JSONObjectParser::name(const std::string& name) { name_ = name; }

bool JSONObjectParser::active() const
{
    return active_;
}

void JSONObjectParser::active(bool value)
{
    loginf << "name " << name_ << " active " << value;

    active_ = value;

    if (widget_)
        widget_->updateActive();
}
