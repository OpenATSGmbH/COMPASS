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
#include "asterixjsonparser.h"

#include <jasterix/iteminfo.h>

#include <vector>
#include <memory>

class ASTERIXImportTask;

class ASTERIXJSONParsingSchema : public Configurable
{
    using ASTERIXJSONParserIterator = std::map<unsigned int, std::unique_ptr<ASTERIXJSONParser>>::iterator;

public:
    /// @brief Constructor backed by a json reference
    ASTERIXJSONParsingSchema(nlohmann::json& config, ASTERIXImportTask& task);
    /// @brief Move constructor
//    ASTERIXJSONParsingSchema& operator=(ASTERIXJSONParsingSchema&& other);

    ASTERIXJSONParserIterator begin() { return parsers_.begin(); }
    ASTERIXJSONParserIterator end() { return parsers_.end(); }

    std::map<unsigned int, std::unique_ptr<ASTERIXJSONParser>>& parsers() { return parsers_; }
    bool hasObjectParser(unsigned int category) { return parsers_.count(category) > 0; }
    ASTERIXJSONParser& parser(unsigned int);
    void removeParser(unsigned int);

    void generateSubConfigurable(nlohmann::json& child_json) override;

    std::string name() const;
    void name(const std::string& name);

    //void updateMappings();

private:
    ASTERIXImportTask& task_;

    std::string name_;
    std::map<unsigned int, std::unique_ptr<ASTERIXJSONParser>> parsers_;

  protected:
    void checkSubConfigurables() override {}

};
