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

#include "jsondatamapping.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "jsondatamappingwidget.h"
#include "jsonobjectparser.h"
#include "unit.h"
#include "unitmanager.h"

#include "util/json_tools.h"

using namespace Utils;
using namespace nlohmann;

// Legacy constructor removed - use json-backed constructor below
// JSONDataMapping::JSONDataMapping(... Configurable& parent) { ... }

//JSONDataMapping& JSONDataMapping::operator=(JSONDataMapping&& other)
//{
//    logdbg2 << "JSONDataMapping: operator=: this " << this << " other " << &other;

//    active_ = other.active_;
//    json_key_ = other.json_key_;
//    db_content_name_ = other.db_content_name_;
//    dbcontent_variable_name_ = other.dbcontent_variable_name_;
//    variable_ = other.variable_;

//    mandatory_ = other.mandatory_;
//    comment_ = other.comment_;
//    // json_value_format_ = other.json_value_format_;
//    format_data_type_ = other.format_data_type_;
//    json_value_format_ = std::move(other.json_value_format_);

//    dimension_ = other.dimension_;
//    unit_ = other.unit_;

//    in_array_ = other.in_array_;
//    append_value_ = other.append_value_;

//    has_sub_keys_ = other.has_sub_keys_;
//    sub_keys_ = std::move(other.sub_keys_);
//    num_sub_keys_ = other.num_sub_keys_;

//    if (sub_keys_.size())
//        last_key_ = sub_keys_.end() - 1;
//    if (sub_keys_.size() > 1)
//        second_to_last_key_ = sub_keys_.end() - 2;

//    other.configuration().updateParameterPointer("active", &active_);
//    other.configuration().updateParameterPointer("json_key", &json_key_);
//    other.configuration().updateParameterPointer("dbcontent_name", &db_content_name_);
//    other.configuration().updateParameterPointer("dbcontvariable_name", &dbcontent_variable_name_);
//    other.configuration().updateParameterPointer("mandatory", &mandatory_);
//    other.configuration().updateParameterPointer("comment", &comment_);
//    other.configuration().updateParameterPointer("format_data_type", &format_data_type_);
//    other.configuration().updateParameterPointer("json_value_format", &json_value_format_);
//    other.configuration().updateParameterPointer("dimension", &dimension_);
//    other.configuration().updateParameterPointer("unit", &unit_);
//    other.configuration().updateParameterPointer("in_array", &in_array_);
//    other.configuration().updateParameterPointer("append_value", &append_value_);

//    widget_ = std::move(other.widget_);
//    if (widget_)
//        widget_->setMapping(*this);
//    other.widget_ = nullptr;

//    return static_cast<JSONDataMapping&>(Configurable::operator=(std::move(other)));
//}

JSONDataMapping::JSONDataMapping(nlohmann::json& config, Configurable* parent, COMPASS& compass)
    : Configurable(config, parent), compass_(compass)
{
    logdbg2 << "this " << this << " (json-backed)";

    registerParameter("active", &active_, false);
    registerParameter("json_key", &json_key_, std::string());

    registerParameter("db_content_name", &db_content_name_, std::string());
    registerParameter("db_content_variable_name", &dbcontent_variable_name_, std::string());

    registerParameter("comment", &comment_, std::string());

    registerParameter("mandatory", &mandatory_, false);

    registerParameter("format_data_type", &format_data_type_, std::string());
    registerParameter("json_value_format", (std::string*)&json_value_format_, std::string());

    registerParameter("dimension", &dimension_, std::string());
    registerParameter("unit", &unit_, std::string());

    registerParameter("in_array", &in_array_, false);
    registerParameter("append_value", &append_value_, false);

    sub_keys_ = Utils::String::split(json_key_, '.');
    has_sub_keys_ = sub_keys_.size() > 1;
    num_sub_keys_ = sub_keys_.size();

    if (sub_keys_.size())
        last_key_ = sub_keys_.end() - 1;
    if (sub_keys_.size() > 1)
        second_to_last_key_ = sub_keys_.end() - 2;

    createSubConfigurables();
}

JSONDataMapping::~JSONDataMapping(){}

void JSONDataMapping::initializeIfRequired()
{
    if (!initialized_)
        initialize();

    //assert(initialized_); can fail
}

std::string& JSONDataMapping::formatDataTypeRef() { return format_data_type_; }

bool JSONDataMapping::initialized() const { return initialized_; }

std::string JSONDataMapping::comment() const { return comment_; }

void JSONDataMapping::comment(const std::string& comment) { comment_ = comment; }

bool JSONDataMapping::appendValue() const { return append_value_; }

void JSONDataMapping::appendValue(bool append_value) { append_value_ = append_value; }

bool JSONDataMapping::inArray() const { return in_array_; }

void JSONDataMapping::inArray(bool in_array) { in_array_ = in_array; }


void JSONDataMapping::check()
{
    DBContentManager& dbcont_man = compass_.dbContentManager();

    if (db_content_name_.size() && !dbcont_man.existsDBContent(db_content_name_))
    {
        logerr << "'" << db_content_name_ << "' does not exist";
        traced_assert(false);
    }

    DBContent& dbcontent = dbcont_man.dbContent(db_content_name_);

    if (dbcontent_variable_name_.size() && !dbcontent.hasVariable(dbcontent_variable_name_))
        dbcontent_variable_name_ = "";

    if (active_ && !canBeActive())
    {
        active_ = false;
    }
}

dbContent::Variable& JSONDataMapping::variable() const
{
    traced_assert(initialized_);
    traced_assert(variable_);
    return *variable_;
}

bool JSONDataMapping::mandatory() const { return mandatory_; }

void JSONDataMapping::mandatory(bool mandatory)
{
    logdbg2 << "start" << mandatory;
    mandatory_ = mandatory;
}

Format JSONDataMapping::jsonValueFormat() const
{
    return json_value_format_;
}

Format& JSONDataMapping::jsonValueFormatRef()
{
    return json_value_format_;
}

// void JSONDataMapping::jsonValueFormat(const Format &json_value_format)
//{
//    json_value_format_ = json_value_format;
//}

std::string JSONDataMapping::dbObjectName() const { return db_content_name_; }

void JSONDataMapping::dbcontVariableName(const std::string& name)
{
    loginf << "start" << name;

    dbcontent_variable_name_ = name;
    initialized_ = false;

    if (!dbcontent_variable_name_.size())
        active_ = false;

    initialize();
}

std::string JSONDataMapping::dbcontVariableName() const { return dbcontent_variable_name_; }

std::string JSONDataMapping::dimensionUnitStr()
{
    if (dimension_.size())
        return dimension_ + ":" + unit_;
    else
        return "";
}

const std::string& JSONDataMapping::jsonKey() const { return json_key_; }

void JSONDataMapping::jsonKey(const std::string& json_key)
{
    loginf << "start" << json_key;

    json_key_ = json_key;

    sub_keys_ = Utils::String::split(json_key_, '.');
    has_sub_keys_ = sub_keys_.size() > 1;
    num_sub_keys_ = sub_keys_.size();

    if (sub_keys_.size())
        last_key_ = sub_keys_.end() - 1;
    if (sub_keys_.size() > 1)
        second_to_last_key_ = sub_keys_.end() - 2;

    if (!json_key_.size())
        active_ = false;
}

// JSONDataMappingWidget* JSONDataMapping::widget ()
//{
//    if (!widget_)
//    {
//        widget_.reset(new JSONDataMappingWidget (*this));
//        traced_assert(widget_);
//    }

//    return widget_.get(); // needed for qt integration, not pretty
//}

bool JSONDataMapping::active() const { return active_; }

void JSONDataMapping::active(bool active)
{
    loginf << "start" << active;
    active_ = active;
}

bool JSONDataMapping::canBeActive() const
{
    return json_key_.size() && dbcontent_variable_name_.size();
}

void JSONDataMapping::initialize()
{
    logdbg2 << "start";

    traced_assert(!initialized_);

    DBContentManager& dbcont_man = compass_.dbContentManager();

    if (db_content_name_.size() && !dbcont_man.existsDBContent(db_content_name_))
        logwrn << "dbcontbject '" << db_content_name_
               << "' does not exist";

    if (db_content_name_.size() && dbcont_man.existsDBContent(db_content_name_) &&
            dbcontent_variable_name_.size() && !dbcont_man.dbContent(db_content_name_).hasVariable(dbcontent_variable_name_))
        logwrn << "dbcontbject " << db_content_name_ << " variable '"
               << dbcontent_variable_name_ << "' does not exist";

    if (db_content_name_.size() && dbcont_man.existsDBContent(db_content_name_) &&
            dbcontent_variable_name_.size() && dbcont_man.dbContent(db_content_name_).hasVariable(dbcontent_variable_name_))
        variable_ = &dbcont_man.dbContent(db_content_name_).variable(dbcontent_variable_name_);


    if (append_value_)
    {
        if (sub_keys_.size() < 2)
            logwrn << "append set but olny " << sub_keys_.size()
                   << " sub keys";

        if (!variable_)
            logwrn << "append set not variable set";
    }

    initialized_ = true; // set for access to variables

    // check for dimension factor
    has_factor_ = false;

    if (variable_)
    {

        if (dimension() != variable().dimension())
        {
            logwrn << "variable " << variable().name()
                   << " has differing dimensions " << dimension() << " "
               << variable().dimension();

            initialized_ = false;
        }
        else if (unit() != variable().unit())  // do unit conversion stuff
        {
            if (json_value_format_ != "")
            {
                logwrn << "dimension unit conversion required but format used";
                initialized_ = false;
            }
            else
            {

                const Dimension& dimension =
                        compass_.unitManager().dimension(variable().dimension());

                if (!dimension.hasUnit(unit()))
                    logerr << "dimension '" << this->dimension()
                           << "' has unknown unit '" << unit() << "'";

                if (!dimension.hasUnit(variable().unit()))
                    logerr << "dimension '"
                   << variable().dimension() << "' has unknown unit '"
                   << variable().unit() << "'";

                has_factor_ = true;
                factor_ = dimension.getFactor(unit(), variable().unit());
            }
        }
    }
    // already initialized, or removed on error
}

template <typename T>
bool JSONDataMapping::findAndSetValue(const json& j, NullableVector<T>& array_list,
                                      size_t row_cnt, bool debug) const
{
    if (in_array_)
    {
        const json* val_ptr = findParentKey(j);

        if (val_ptr == nullptr || *val_ptr == nullptr)
        {
            if (mandatory_)
                return true;

            return false;
        }
        else
        {
            if (!val_ptr->is_array())
            {
                logerr << "key " << json_key_ << " not in array '"
                       << val_ptr->dump(4) << "'";
                return true;
            }

            try
            {
                const std::string& last_key = sub_keys_.back();

                for (auto& j_it : val_ptr->get<json::array_t>())  // iterate over array
                {
                    if (j_it.contains(last_key))
                    {
                        if (append_value_)
                            appendValue(&j_it.at(last_key), array_list, row_cnt, debug);
                        else
                            setValue(&j_it.at(last_key), array_list, row_cnt, debug);
                    }
                }

                return false;  // everything ok
            }
            catch (json::exception& e)
            {
                logerr << "key " << json_key_
                       << " json exception " << e.what() << " property "
                       << array_list.propertyID();
                array_list.setNull(row_cnt);
                return true;  // last entry might be wrong
            }
        }
    }
    else
    {
        const json* val_ptr = findKey(j);

        if (val_ptr == nullptr || *val_ptr == nullptr)
        {
            if (mandatory_)
                return true;

            return false;
        }
        else
        {
            try
            {
                logdbg2 << "row_cnt " << row_cnt
                       << " key " << json_key_ << " value '" << val_ptr->dump() << "'";

                if (append_value_)
                    appendValue(val_ptr, array_list, row_cnt, debug);
                else
                    setValue(val_ptr, array_list, row_cnt, debug);

                return false;  // everything ok
            }
            catch (json::exception& e)
            {
                logerr << "key " << json_key_
                       << " json exception " << e.what() << " property "
                       << array_list.propertyID();
                array_list.setNull(row_cnt);
                return true;  // last entry might be wrong
            }
        }
    }
}

// template void foo::do<int>(const int&);

template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<bool>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<char>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<unsigned char>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<int>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<unsigned int>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<long int>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<unsigned long int>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<float>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<double>& array_list,
size_t row_cnt, bool debug) const;
template bool JSONDataMapping::findAndSetValue(const json& j,
NullableVector<std::string>& array_list,
size_t row_cnt, bool debug) const;
//template bool JSONDataMapping::findAndSetValue(const json& j,
//NullableVector<json>& array_list,
//size_t row_cnt, bool debug) const;

bool JSONDataMapping::findAndSetValues(const json& j, NullableVector<json>& array_list,
                     size_t row_cnt, bool debug) const
{
    traced_assert(!in_array_);
    traced_assert(!append_value_);

    std::vector<json> values = findKeys(j);

    if (!values.size())
    {
        if (mandatory_)
            return true;

        return false;
    }
    else
    {
        try
        {
//            for (const json& val_ptr : val_ptrs)
//            {
//                traced_assert(val_ptr);
//                loginf << "row_cnt " << row_cnt
//                       << " key " << json_key_ << " value '" << val_ptr.dump() << "'";

//                pushBackValue(val_ptr, array_list, row_cnt, debug);
//            }
            array_list.set(row_cnt, values);
            return false;  // everything ok
        }
        catch (json::exception& e)
        {
            logerr << "key " << json_key_
                   << " json exception " << e.what() << " property "
                   << array_list.propertyID();
            array_list.setNull(row_cnt);
            return true;  // last entry might be wrong
        }
    }
}

template <typename T>
bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data, NullableVector<T>& array_list,
                                         size_t start_row, size_t num_records) const
{
    if (!flat_cat_data.contains(json_key_))
    {
        if (mandatory_)
            return true;
        return false;
    }

    const json& arr = flat_cat_data.at(json_key_);

    if (!arr.is_array() || arr.empty())
    {
        if (mandatory_)
            return true;
        return false;
    }

    size_t arr_size = arr.size();

    if (arr_size > num_records)
    {
        logwrn << "JSONDataMapping: setFlatArrayValues: key '" << json_key_
               << "' array size " << arr_size << " > num_records " << num_records;
    }

    for (size_t i = 0; i < arr_size && i < num_records; ++i)
    {
        const json& val = arr[i];

        if (val.is_null())
            continue; // leave as null

        size_t row = start_row + i;

        try
        {
            if (in_array_)
            {
                // repetitive item leaf: cell is an array of scalars (struct-of-arrays,
                // aligned by repetition index) - mirror the nested-path semantics
                if (!val.is_array())
                {
                    logerr << "JSONDataMapping: setFlatArrayValues: key " << json_key_
                           << " row " << row << " cell not an array '" << val.dump() << "'";
                    continue;
                }

                for (const json& elem : val)
                {
                    if (append_value_)
                        appendValue(&elem, array_list, row);
                    else
                        setValue(&elem, array_list, row);
                }
            }
            else
                setValue(&val, array_list, row);
        }
        catch (json::exception& e)
        {
            logerr << "JSONDataMapping: setFlatArrayValues: key " << json_key_
                   << " row " << row << " json exception " << e.what();
            array_list.setNull(row);
        }
    }

    return false;
}

template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<bool>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<char>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<unsigned char>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<int>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<unsigned int>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<long int>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<unsigned long int>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<float>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<double>& array_list, size_t start_row, size_t num_records) const;
template bool JSONDataMapping::setFlatArrayValues(const json& flat_cat_data,
NullableVector<std::string>& array_list, size_t start_row, size_t num_records) const;

bool JSONDataMapping::setFlatArrayJsonValues(const json& flat_cat_data,
                                             NullableVector<json>& array_list,
                                             size_t start_row, size_t num_records) const
{
    if (!flat_cat_data.contains(json_key_))
    {
        // repetitive container fallback: jASTERIX flat format emits per-leaf columns
        // (e.g. '250.Mode S MB Data.MB Data', '250.Mode S MB Data.BDS1') with cells
        // holding arrays of scalars aligned by repetition index. A mapping keyed at
        // the container ('250.Mode S MB Data') zips them back into the array-of-objects
        // form the nested decode produces.
        const std::string prefix = json_key_ + ".";
        std::vector<std::pair<std::string, const json*>> leaf_columns;  // leaf name, column

        for (auto it = flat_cat_data.begin(); it != flat_cat_data.end(); ++it)
            if (it.key().compare(0, prefix.size(), prefix) == 0 && it.value().is_array())
                leaf_columns.emplace_back(it.key().substr(prefix.size()), &it.value());

        if (leaf_columns.empty())
        {
            if (mandatory_)
                return true;
            return false;
        }

        for (size_t i = 0; i < num_records; ++i)
        {
            json zipped = json::array();

            for (const auto& leaf_it : leaf_columns)
            {
                const json& column = *leaf_it.second;

                if (i >= column.size())
                    continue;

                const json& cell = column[i];

                if (!cell.is_array())
                    continue;

                for (size_t rep_cnt = 0; rep_cnt < cell.size(); ++rep_cnt)
                    zipped[rep_cnt][leaf_it.first] = cell[rep_cnt];
            }

            if (zipped.empty())
                continue;

            size_t row = start_row + i;

            try
            {
                std::vector<json> values;
                for (auto& elem : zipped)
                    values.push_back(std::move(elem));
                array_list.set(row, values);
            }
            catch (json::exception& e)
            {
                logerr << "JSONDataMapping: setFlatArrayJsonValues: container key " << json_key_
                       << " row " << row << " json exception " << e.what();
                array_list.setNull(row);
            }
        }

        return false;
    }

    const json& arr = flat_cat_data.at(json_key_);

    if (!arr.is_array() || arr.empty())
    {
        if (mandatory_)
            return true;
        return false;
    }

    size_t arr_size = arr.size();

    for (size_t i = 0; i < arr_size && i < num_records; ++i)
    {
        const json& val = arr[i];

        if (val.is_null())
            continue;

        size_t row = start_row + i;

        try
        {
            if (val.is_array())
            {
                std::vector<json> values;
                for (const auto& elem : val)
                    values.push_back(elem);
                array_list.set(row, values);
            }
            else
            {
                std::vector<json> values;
                values.push_back(val);
                array_list.set(row, values);
            }
        }
        catch (json::exception& e)
        {
            logerr << "JSONDataMapping: setFlatArrayJsonValues: key " << json_key_
                   << " row " << row << " json exception " << e.what();
            array_list.setNull(row);
        }
    }

    return false;
}

const json* JSONDataMapping::findKey(const json& j) const
{
    const json* val_ptr = &j;

    if (has_sub_keys_)
    {
        for (auto sub_it = sub_keys_.begin(); sub_it != sub_keys_.end(); ++sub_it)
        {
            if (val_ptr->contains(*sub_it))
            {
                if (sub_it == last_key_)  // last found
                {
                    val_ptr = &val_ptr->at(*sub_it);
                    break;
                }

                if (val_ptr->at(*sub_it).is_object())  // not last, step in
                    val_ptr = &val_ptr->at(*sub_it);
                else  // not last key, and not object
                {
                    val_ptr = nullptr;
                    break;
                }
            }
            else  // not found
            {
                val_ptr = nullptr;
                break;
            }
        }
    }
    else
    {
        if (val_ptr->contains(json_key_))
            val_ptr = &val_ptr->at(json_key_);
        else
            val_ptr = nullptr;
    }
    return val_ptr;
}

const std::vector<json>JSONDataMapping::findKeys(const json& j) const
{
    std::vector<json> rets;

    if (has_sub_keys_)
        addKeys(j, rets, 0);
    else
    {
        if (j.contains(json_key_))
            rets.push_back(j.at(json_key_));
    }

    logdbg2 << "UGA rets " << rets.size();

    return rets;
}

void JSONDataMapping::addKeys(const json& j, std::vector<json>& rets ,
                                    unsigned int key_cnt) const
{
    traced_assert(key_cnt < sub_keys_.size());

    if (j.contains(sub_keys_.at(key_cnt)))
    {
        const json& value = j.at(sub_keys_.at(key_cnt));

        if (key_cnt == sub_keys_.size()-1) // last found
        {
            logdbg2 << "last value '" << value.dump() << "'";
            rets.push_back(value);
            return;
        }
        else // not last
        {
            if (value.is_object())
            {
                //loginf << "stepping into object";

                addKeys(value, rets, key_cnt+1);

            }
            else if (value.is_array())
            {
                //loginf << "stepping into array";

                for (const json& it : value.get<json::array_t>())
                    addKeys(it, rets, key_cnt+1);
            }
            else
            {
                //loginf << "unkown value type '" << value.dump() << "'";
                traced_assert(false); // not gonna happen
            }
        }
    }

}

const json* JSONDataMapping::findParentKey(const json& j) const
{
    const json* val_ptr = &j;

    if (has_sub_keys_)
    {
        for (auto sub_it = sub_keys_.begin(); sub_it != sub_keys_.end(); ++sub_it)
        {
            if (val_ptr->contains(*sub_it))
            {
                if (sub_it == second_to_last_key_)  // second to last found
                {
                    val_ptr = &val_ptr->at(*sub_it);
                    break;
                }

                if (val_ptr->at(*sub_it).is_object())  // not second to last, step in
                    val_ptr = &val_ptr->at(*sub_it);
                else  // not last key, and not object
                {
                    val_ptr = nullptr;
                    break;
                }
            }
            else  // not found
            {
                val_ptr = nullptr;
                break;
            }
        }
    }  // else path means j is already parent

    return val_ptr;
}

template <typename T>
void JSONDataMapping::setValue(const json* val_ptr, NullableVector<T>& array_list,
                               size_t row_cnt, bool debug) const
{
    traced_assert(val_ptr);

    logdbg2 << "key " << json_key_ << " json " << val_ptr->type_name()
           << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    if (debug)
        loginf << "key " << json_key_ << " json " << val_ptr->type_name()
               << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    if (json_value_format_ == "")
    {
        if (has_factor_)
        {
            double tmp = *val_ptr;
            array_list.set(row_cnt, tmp * factor_);
        }
        else
            array_list.set(row_cnt, *val_ptr);
    }
    else
        array_list.setFromFormat(row_cnt, json_value_format_, JSON::toString(*val_ptr));

    logdbg2 << "key " << json_key_ << " json " << *val_ptr << " buffer "
           << array_list.get(row_cnt);
}

template <typename T>
void JSONDataMapping::appendValue(const json* val_ptr, NullableVector<T>& array_list,
                                  size_t row_cnt, bool debug) const
{
    traced_assert(val_ptr);

    logdbg2 << "key " << json_key_ << " json " << val_ptr->type_name()
           << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    if (debug)
        loginf << "key " << json_key_ << " json " << val_ptr->type_name()
               << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    if (json_value_format_ == "")
    {
        if (has_factor_)
        {
            double tmp = *val_ptr;
            array_list.append(row_cnt, tmp * factor_);
        }
        else
            array_list.append(row_cnt, *val_ptr);
    }
    else
        array_list.appendFromFormat(row_cnt, json_value_format_, JSON::toString(*val_ptr));

    logdbg2 << "key " << json_key_ << " json " << *val_ptr
           << " buffer " << array_list.get(row_cnt);
}

void JSONDataMapping::setValue(const json* val_ptr, NullableVector<bool>& array_list,
                               size_t row_cnt, bool debug) const
{
    traced_assert(val_ptr);

    if (debug)
        loginf << "key " << json_key_ << " json " << val_ptr->type_name()
               << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    bool tmp_bool;

    if (val_ptr->is_number())
    {
        unsigned int tmp = *val_ptr;
        if (tmp > 1)
            logerr << "key " << json_key_ << " json " << val_ptr->type_name()
                   << " value '" << tmp << "' format '" << json_value_format_ << "'";

        tmp_bool = static_cast<bool>(tmp);
    }
    else
    {
        tmp_bool = *val_ptr;  // works for bool, throws for rest
    }

    if (json_value_format_ == "")
        array_list.set(row_cnt, tmp_bool);
    else if (json_value_format_ == "invert")
        array_list.set(row_cnt, !tmp_bool);
    else
        array_list.setFromFormat(row_cnt, json_value_format_, JSON::toString(tmp_bool));

    logdbg2 << "(bool): json " << tmp_bool << " buffer "
           << array_list.get(row_cnt);
}

void JSONDataMapping::appendValue(const json* val_ptr, NullableVector<bool>& array_list,
                                  size_t row_cnt, bool debug) const
{
    traced_assert(val_ptr);

    if (debug)
        loginf << "key " << json_key_ << " json " << val_ptr->type_name()
               << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    bool tmp_bool;

    if (val_ptr->is_number())
    {
        unsigned int tmp = *val_ptr;
        traced_assert(tmp == 0 || tmp == 1);
        tmp_bool = static_cast<bool>(tmp);
    }
    else
        tmp_bool = *val_ptr;  // works for bool, throws for rest

    if (json_value_format_ == "")
        array_list.append(row_cnt, tmp_bool);
    else
        array_list.appendFromFormat(row_cnt, json_value_format_, JSON::toString(tmp_bool));

    logdbg2 << "(bool): json " << tmp_bool << " buffer "
           << array_list.get(row_cnt);
}

void JSONDataMapping::setValue(const json* val_ptr, NullableVector<char>& array_list,
                               size_t row_cnt, bool debug) const
{
    traced_assert(val_ptr);

    if (debug)
        loginf << "key " << json_key_ << " json " << val_ptr->type_name()
               << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    if (json_value_format_ == "")
        array_list.set(row_cnt, static_cast<int>(*val_ptr));
    else
        array_list.setFromFormat(row_cnt, json_value_format_,
                                 JSON::toString(static_cast<int>(*val_ptr)));

    logdbg2 << "(char): json " << static_cast<int>(*val_ptr) << " buffer "
           << array_list.get(row_cnt);
}

void JSONDataMapping::appendValue(const json* val_ptr, NullableVector<char>& array_list,
                                  size_t row_cnt, bool debug) const
{
    traced_assert(val_ptr);

    if (debug)
        loginf << "key " << json_key_ << " json " << val_ptr->type_name()
               << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    if (json_value_format_ == "")
        array_list.append(row_cnt, static_cast<int>(*val_ptr));
    else
        array_list.appendFromFormat(row_cnt, json_value_format_,
                                    JSON::toString(static_cast<int>(*val_ptr)));

    logdbg2 << "(char): json " << static_cast<int>(*val_ptr)
           << " buffer " << array_list.get(row_cnt);
}

template void JSONDataMapping::setValue(const json* val_ptr,
NullableVector<unsigned char>& array_list,
size_t row_cnt, bool debug) const;
template void JSONDataMapping::appendValue(const json* val_ptr,
NullableVector<unsigned char>& array_list,
size_t row_cnt, bool debug) const;

template void JSONDataMapping::setValue(const json* val_ptr,
NullableVector<int>& array_list, size_t row_cnt, bool debug) const;
template void JSONDataMapping::appendValue(const json* val_ptr,
NullableVector<int>& array_list, size_t row_cnt, bool debug) const;

template void JSONDataMapping::setValue(const json* val_ptr,
NullableVector<unsigned int>& array_list,
size_t row_cnt, bool debug) const;
template void JSONDataMapping::appendValue(const json* val_ptr,
NullableVector<unsigned int>& array_list,
size_t row_cnt, bool debug) const;

template void JSONDataMapping::setValue(const json* val_ptr,
NullableVector<long int>& array_list, size_t row_cnt, bool debug) const;
template void JSONDataMapping::appendValue(const json* val_ptr,
NullableVector<long int>& array_list,
size_t row_cnt, bool debug) const;

template void JSONDataMapping::setValue(const json* val_ptr,
NullableVector<unsigned long int>& array_list,
size_t row_cnt, bool debug) const;
template void JSONDataMapping::appendValue(const json* val_ptr,
NullableVector<unsigned long int>& array_list,
size_t row_cnt, bool debug) const;

template void JSONDataMapping::setValue(const json* val_ptr,
NullableVector<float>& array_list, size_t row_cnt, bool debug) const;
template void JSONDataMapping::appendValue(const json* val_ptr,
NullableVector<float>& array_list, size_t row_cnt, bool debug) const;

template void JSONDataMapping::setValue(const json* val_ptr,
NullableVector<double>& array_list, size_t row_cnt, bool debug) const;
template void JSONDataMapping::appendValue(const json* val_ptr,
NullableVector<double>& array_list,
size_t row_cnt, bool debug) const;

void JSONDataMapping::setValue(const json* val_ptr,
                               NullableVector<std::string>& array_list, size_t row_cnt, bool debug) const
{
    traced_assert(val_ptr);

    if (debug)
        loginf << "key " << json_key_ << " json " << val_ptr->type_name()
               << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    if (json_value_format_ == "")
        array_list.set(row_cnt, Utils::JSON::toString(*val_ptr));
    else
    {
        array_list.setFromFormat(row_cnt, json_value_format_, Utils::JSON::toString(*val_ptr), debug);
    }

    logdbg2 << "(string): json " << Utils::JSON::toString(*val_ptr)
           << " buffer " << array_list.get(row_cnt);
}

void JSONDataMapping::appendValue(const json* val_ptr,
                                  NullableVector<std::string>& array_list, size_t row_cnt, bool debug) const
{
    traced_assert(val_ptr);

    if (debug)
        loginf << "key " << json_key_ << " json " << val_ptr->type_name()
               << " '" << val_ptr->dump() << "' format '" << json_value_format_ << "'";

    if (json_value_format_ == "")
        array_list.append(row_cnt, Utils::JSON::toString(*val_ptr));
    else
        array_list.appendFromFormat(row_cnt, json_value_format_, Utils::JSON::toString(*val_ptr));

    logdbg2 << "(string): json " << Utils::JSON::toString(*val_ptr)
           << " buffer " << array_list.get(row_cnt);
}


void JSONDataMapping::pushBackValue(const nlohmann::json& val_ref, NullableVector<nlohmann::json>& array_list,
                   size_t row_cnt, bool debug) const
{
    if (debug)
        loginf << "key " << json_key_ << " json " << val_ref.type_name()
               << " '" << val_ref.dump() << "' format '" << json_value_format_ << "'";

    traced_assert(json_value_format_ == "");

    if (array_list.isNull(row_cnt))
    {
        std::vector<unsigned int> list;
        list.push_back(val_ref);
        array_list.set(row_cnt, list);
    }
    else
        array_list.pushBack(row_cnt, val_ref);
}
