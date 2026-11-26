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

#include "json_fwd.hpp"

#include <functional>

namespace Utils
{
namespace JSON
{

extern std::string toString(const nlohmann::json& j);

extern bool canFindKey(const nlohmann::json& j, const std::vector<std::string>& keys);
extern const nlohmann::json& findKey(const nlohmann::json& j, const std::vector<std::string>& keys);
extern const nlohmann::json& findParentKey(const nlohmann::json& j,
                                           const std::vector<std::string>& keys);

extern void applyFunctionToValues(nlohmann::json& j, const std::vector<std::string>& keys,
                                  std::vector<std::string>::const_iterator current_key_it,
                                  std::function<void(nlohmann::json&)> function, bool required);

}  // namespace JSON
}  // namespace Utils
