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

#include "buffer_utils.h"
#include "buffer.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;

namespace
{

template <typename T>
std::string getTypedValue(Buffer& buffer, const std::string& property_name,
                          unsigned int buffer_index,
                          dbContent::Variable& variable,
                          bool use_presentation, bool has_repr, bool& is_null)
{
    if (!buffer.has<T>(property_name))
    {
        is_null = true;
        return "";
    }

    auto& vec = buffer.get<T>(property_name);

    if (vec.isNull(buffer_index))
    {
        is_null = true;
        return "";
    }

    std::string raw = vec.getAsString(buffer_index);

    if (use_presentation && has_repr)
        return variable.getRepresentationStringFromValue(raw);

    return raw;
}

} // anonymous namespace

namespace buffer_utils
{

void transformVariables(Buffer& buffer, dbContent::VariableSet& list, bool dbcol2dbcontvar)
{
    logdbg << "dbcont '" << buffer.dbContentName() << "' dbcol2dbcontvar " << dbcol2dbcontvar;

    const vector<dbContent::Variable*>& variables = list.getSet();
    string variable_name;
    string name_in_db;

    string current_var_name;
    string transformed_var_name;

    for (auto var_it : variables)
    {
        logdbg << "variable " << var_it->name() << " db column " << name_in_db;

        variable_name = var_it->name();
        name_in_db = var_it->dbColumnOrExpression();

        PropertyDataType data_type = var_it->dataType();

        if (dbcol2dbcontvar)
        {
            if (!buffer.properties().hasProperty(name_in_db))
            {
                continue;
            }

            traced_assert(buffer.properties().hasProperty(name_in_db));
            traced_assert(buffer.properties().get(name_in_db).dataType() == var_it->dataType());

            current_var_name = name_in_db;
            transformed_var_name = variable_name;
        }
        else
        {
            if (!buffer.properties().hasProperty(var_it->name()))
            {
                logerr << "variable '" << variable_name << "' not found";
                continue;
            }

            traced_assert(buffer.properties().hasProperty(variable_name));
            traced_assert(buffer.properties().get(variable_name).dataType() == var_it->dataType());

            current_var_name = variable_name;
            transformed_var_name = name_in_db;
        }

        // rename to reflect dbcont variable
        if (current_var_name != transformed_var_name)
        {
            logdbg << "renaming variable " << current_var_name
                   << " to variable name " << transformed_var_name;

            switch (data_type)
            {
                case PropertyDataType::BOOL:
                    buffer.rename<bool>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::CHAR:
                    buffer.rename<char>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::UCHAR:
                    buffer.rename<unsigned char>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::INT:
                    buffer.rename<int>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::UINT:
                    buffer.rename<unsigned int>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::LONGINT:
                    buffer.rename<long int>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::ULONGINT:
                    buffer.rename<unsigned long int>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::FLOAT:
                    buffer.rename<float>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::DOUBLE:
                    buffer.rename<double>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::STRING:
                    buffer.rename<string>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::JSON:
                    buffer.rename<nlohmann::json>(current_var_name, transformed_var_name);
                    break;
                case PropertyDataType::TIMESTAMP:
                    buffer.rename<boost::posix_time::ptime>(current_var_name, transformed_var_name);
                    break;
                default:
                    logerr << "unknown property type "
                           << Property::asString(data_type);
                    throw runtime_error("buffer_utils::transformVariables: unknown property type " +
                                             Property::asString(data_type));
            }
        }
    }
}

std::string getValueString(dbContent::Variable& variable,
                           Buffer& buffer,
                           unsigned int buffer_index,
                           bool use_presentation,
                           bool& is_null)
{
    PropertyDataType data_type = variable.dataType();
    std::string property_name = variable.name();
    is_null = false;

    switch (data_type)
    {
        case PropertyDataType::BOOL:
            return getTypedValue<bool>(buffer, property_name, buffer_index,
                                      variable, use_presentation, true, is_null);
        case PropertyDataType::CHAR:
            return getTypedValue<char>(buffer, property_name, buffer_index,
                                      variable, use_presentation, true, is_null);
        case PropertyDataType::UCHAR:
            return getTypedValue<unsigned char>(buffer, property_name, buffer_index,
                                               variable, use_presentation, true, is_null);
        case PropertyDataType::INT:
            return getTypedValue<int>(buffer, property_name, buffer_index,
                                     variable, use_presentation, true, is_null);
        case PropertyDataType::UINT:
            return getTypedValue<unsigned int>(buffer, property_name, buffer_index,
                                              variable, use_presentation, true, is_null);
        case PropertyDataType::LONGINT:
            return getTypedValue<long int>(buffer, property_name, buffer_index,
                                          variable, use_presentation, true, is_null);
        case PropertyDataType::ULONGINT:
            return getTypedValue<unsigned long int>(buffer, property_name, buffer_index,
                                                   variable, use_presentation, true, is_null);
        case PropertyDataType::FLOAT:
            return getTypedValue<float>(buffer, property_name, buffer_index,
                                       variable, use_presentation, true, is_null);
        case PropertyDataType::DOUBLE:
            return getTypedValue<double>(buffer, property_name, buffer_index,
                                        variable, use_presentation, true, is_null);
        case PropertyDataType::STRING:
            return getTypedValue<std::string>(buffer, property_name, buffer_index,
                                             variable, use_presentation, false, is_null);
        case PropertyDataType::JSON:
            return getTypedValue<nlohmann::json>(buffer, property_name, buffer_index,
                                                variable, use_presentation, false, is_null);
        case PropertyDataType::TIMESTAMP:
            return getTypedValue<boost::posix_time::ptime>(buffer, property_name, buffer_index,
                                                          variable, use_presentation, false, is_null);
        default:
            throw std::domain_error("buffer_utils::getValueString: unknown property data type");
    }
}

} // namespace buffer_utils
