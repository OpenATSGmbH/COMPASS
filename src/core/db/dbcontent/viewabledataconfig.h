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

#include "json.hpp"

#include <string>
#include <utility>
#include <vector>

class ViewableDataConfig
{
public:
  ViewableDataConfig(const nlohmann::json::object_t& data)
  {
      data_ = data;
  }

  ViewableDataConfig(const std::string& json_str)
  {
      data_ = nlohmann::json::parse(json_str);
  }

  const nlohmann::json& data() const { return data_; }

  void reportError(const std::string& component_name, const std::string& error)
  {
      errors_.emplace_back(component_name, error);
  }

  bool hasErrors() const { return !errors_.empty(); }

  const std::vector<std::pair<std::string, std::string>>& errors() const { return errors_; }

protected:
    nlohmann::json data_;
    std::vector<std::pair<std::string, std::string>> errors_;
};

