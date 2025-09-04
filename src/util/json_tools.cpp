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

#include "json_tools.h"
#include "traced_assert.h"

#include "json.hpp"

namespace Utils
{
namespace JSON
{

std::string toString(const nlohmann::json& j)
{
    if (j.type() == nlohmann::json::value_t::string)
    {
        return j.get<std::string>();
    }

    return j.dump();
}

bool canFindKey(const nlohmann::json& j, const std::vector<std::string>& keys)
{
    if (!keys.size())
        return false;

    std::vector<std::string>::const_iterator last_key = keys.end() - 1;

    const nlohmann::json* val_ptr = &j;

    for (auto sub_it = keys.cbegin(); sub_it != keys.cend(); ++sub_it)
    {
        if (val_ptr->contains(*sub_it))
        {
            if (sub_it == last_key)  // last found
                return true;

            if (val_ptr->at(*sub_it).is_object())  // not last, step in
                val_ptr = &val_ptr->at(*sub_it);
            else  // not last key, and not object
                return false;
        }
        else  // not found
            return false;
    }

    return false;
}

const nlohmann::json& findKey(const nlohmann::json& j, const std::vector<std::string>& keys)
{
    traced_assert(keys.size());
    std::vector<std::string>::const_iterator last_key = keys.end() - 1;

    const nlohmann::json* val_ptr = &j;

    for (auto sub_it = keys.cbegin(); sub_it != keys.cend(); ++sub_it)
    {
        if (val_ptr->contains(*sub_it))
        {
            if (sub_it == last_key)  // last found
            {
                val_ptr = &val_ptr->at(*sub_it);
                return *val_ptr;
            }

            if (val_ptr->at(*sub_it).is_object())  // not last, step in
                val_ptr = &val_ptr->at(*sub_it);
            else  // not last key, and not object
                throw std::runtime_error("Utils: JSON: findKey: key '" + *sub_it + "' not found");
        }
        else  // not found
            throw std::runtime_error("Utils: JSON: findKey: key '" + *sub_it + "' not found");
    }

    throw std::runtime_error("Utils: JSON: findKey: keys empty");
}

const nlohmann::json& findParentKey(const nlohmann::json& j, const std::vector<std::string>& keys)
{
    const nlohmann::json* val_ptr = &j;

    traced_assert(keys.size() > 1);
    std::vector<std::string>::const_iterator second_to_last_key = keys.end() - 2;

    for (auto sub_it = keys.begin(); sub_it != keys.end(); ++sub_it)
    {
        if (val_ptr->contains(*sub_it))
        {
            if (sub_it == second_to_last_key)  // second to last found
            {
                val_ptr = &val_ptr->at(*sub_it);
                break;
            }

            if (val_ptr->at(*sub_it).is_object())  // not second to last, step in
                val_ptr = &val_ptr->at(*sub_it);
            else  // not last key, and not object
                throw std::runtime_error("Utils: JSON: findParentKey: key '" + *sub_it +
                                         "' not found");
        }
        else  // not found
            throw std::runtime_error("Utils: JSON: findParentKey: key '" + *sub_it + "' not found");
    }

    traced_assert(val_ptr);
    return *val_ptr;
}

void applyFunctionToValues(nlohmann::json& j, const std::vector<std::string>& keys,
                           std::vector<std::string>::const_iterator current_key_it,
                           std::function<void(nlohmann::json&)> function, bool required)
{
    //            loginf << "current_key '" << *current_key_it
    //            << "' data '"
    //                   << j.dump(4) << "'";

    if (current_key_it == keys.end())  // no more keys
    {
        // loginf << "applying to '" << j.dump(4) << "'";
        function(j);
        return;
    }

    // not last key

    if (j.contains(*current_key_it))
    {
        // loginf;
        nlohmann::json& value = j.at(*current_key_it);

        if (value.is_object())
        {
            // loginf << "recursing into object";
            applyFunctionToValues(value, keys, current_key_it + 1, function, required);
        }
        else if (value.is_array())
        {
            // loginf << "recursing into array";

            for (auto& value_it : value)
                applyFunctionToValues(value_it, keys, current_key_it + 1, function, required);
        }
        else
        {
            if (required)
                throw std::runtime_error("Utils: String: applyFunctionToValues: key '" +
                                         *current_key_it + "' stopped at unsupported value type");
            else
                return;
        }
    }
    else
    {
        // loginf << "not contained";

        if (required)
            throw std::runtime_error("Utils: String: applyFunctionToValues: key '" +
                                     *current_key_it + "' not found");
        else
            return;
    }
}
}  // namespace JSON
}  // namespace Utils
