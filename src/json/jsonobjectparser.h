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

#pragma once

#include "configurable.h"
//#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
//#include "format.h"
#include "jsondatamapping.h"
#include "jsonobjectparserwidget.h"
#include "propertylist.h"
//#include "stringconv.h"

#include <memory>
#include <string>

class Buffer;
class DBContent;

namespace dbContent {

class Variable;

}


class JSONObjectParser : public Configurable
{
    using MappingIterator = std::vector<std::unique_ptr<JSONDataMapping>>::iterator;

  public:
    JSONObjectParser(const std::string& class_id, const std::string& instance_id,
                     Configurable* parent);
    JSONObjectParser() = default;

    DBContent& dbContent() const;

    std::string JSONKey() const;
    void JSONKey(const std::string& json_key);

    std::string JSONValue() const;
    void JSONValue(const std::string& json_value);

    std::string JSONContainerKey() const;
    void JSONContainerKey(const std::string& key);

    MappingIterator begin() { return data_mappings_.begin(); }
    MappingIterator end() { return data_mappings_.end(); }
    bool hasMapping(unsigned int index) const;
    void removeMapping(unsigned int index);

    // returs true on successful parse
    bool parseJSON(nlohmann::json& j, Buffer& buffer) const;
    void createMappingStubs(nlohmann::json& j);

    const dbContent::VariableSet& variableList() const;

    bool overrideDataSource() const;
    void overrideDataSource(bool override);

    std::string dataSourceVariableName() const;
    void dataSourceVariableName(const std::string& name);

    bool initialized() const { return initialized_; }
    void initialize();

    std::shared_ptr<Buffer> getNewBuffer() const;
    void appendVariablesToBuffer(Buffer& buffer) const;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    JSONObjectParserWidget* widget();

    std::string dbContentName() const;

    void setMappingActive(JSONDataMapping& mapping, bool active);

    void updateMappings();

    std::string name() const;
    void name(const std::string& name);

    bool active() const;
    void active(bool value);

private:
    std::string name_;
    bool active_ {true};

    std::string db_content_name_;
    DBContent* dbcontent_{nullptr};

    std::string json_container_key_;  // location of container with target report data
    std::string json_key_;            // * for all
    std::string json_value_;

    std::vector<std::string> json_values_vector_;

    dbContent::VariableSet var_list_;

    bool override_data_source_{false};
    std::string data_source_variable_name_;

    bool initialized_{false};

    bool not_parse_all_{false};

    PropertyList list_;

    std::unique_ptr<JSONObjectParserWidget> widget_;

    std::vector<std::unique_ptr<JSONDataMapping>> data_mappings_;

    // returns true on successful parse
    bool parseTargetReport(const nlohmann::json& tr, Buffer& buffer, size_t row_cnt) const;
    void createMappingsFromTargetReport(const nlohmann::json& tr);

    void checkIfKeysExistsInMappings(const std::string& location, const nlohmann::json& tr,
                                     bool is_in_array = false);

  protected:
    virtual void checkSubConfigurables() {}
};
