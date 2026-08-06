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

#include "dbfiltercondition.h"
#include "dbfilter.h"
#include "idbvariableresolver.h"
#include "stringconv.h"
#include "global.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QWidget>
#include <QApplication>
#include <QStyle>

#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "traced_assert.h"
#include <sstream>

using namespace Utils;
using namespace std;

DBFilterCondition::DBFilterCondition(nlohmann::json& config, DBFilter* parent)
    : Configurable(config, parent),
      var_resolver_(parent->variableResolver()),
      filter_parent_(parent)
{
    registerParameter("operator", &operator_, std::string(">"));
    registerParameter("absolute_value", &absolute_value_, false);

    registerParameter("variable_dbcontent_name", &variable_dbcontent_name_, std::string());
    registerParameter("variable_name", &variable_name_, std::string());

    registerParameter("display_instance_name", &display_instance_name_, false);

    registerParameter("reset_value", &reset_value_, std::string(""));
    registerParameter("value", &value_, std::string());
    registerParameter("value2", &value2_, std::string());
    registerParameter("include_null", &include_null_, false);

    logdbg << "start" << instanceName() << " value " << value_
           << " usable " << usable_ << " invalid " << value_invalid_;

    if (display_instance_name_)
        base_label_text_ = instanceName() + " " + operator_;
    else
        base_label_text_ = variable_name_ + " " + operator_;

    label_ = new QLabel();
    label_->setText(tr((label_prefix_ + base_label_text_).c_str()));

    edit_ = new QLineEdit(tr(value_.c_str()));
    edit_->setMaxLength(16*1024*1024);
    connect(edit_, SIGNAL(textChanged(QString)), this, SLOT(valueChanged()));

    if (!usable_)
    {
        label_->setDisabled(true);
        edit_->setDisabled(true);
    }
}

DBFilterCondition::~DBFilterCondition() = default;

void DBFilterCondition::invert()
{
    // TODO
    // op_and_=!op_and_;
}

/**
 * Returns if variable_ exists in DBContent of type
 */
bool DBFilterCondition::filters(const std::string& dbcontent_name)
{
    traced_assert(usable_);

    return hasVariable(dbcontent_name);
}

FilterClause DBFilterCondition::getClause(const std::string& dbcontent_name)
{
    traced_assert(usable_);
    traced_assert(hasVariable(dbcontent_name));

    return sqlFor(var_resolver_, dbcontent_name, variable_name_, variable_dbcontent_name_,
                  operator_, value_, value2_, include_null_, absolute_value_);
}

FilterClause DBFilterCondition::sqlFor(
    IDBVariableResolver& resolver, const std::string& dbcontent_name,
    const std::string& variable_name, const std::string& variable_dbcontent_name,
    const std::string& op, const std::string& value, const std::string& value2,
    bool include_null, bool absolute_value, bool value_is_representation)
{
    FilterClause clause;

    // variable not resolvable for this content -> no constraint (mirrors the config path's
    // cond->filters() skip), so callers may pass meta variables blindly per content
    if (!variableResolvable(resolver, dbcontent_name, variable_name, variable_dbcontent_name))
        return clause;

    std::string variable_prefix;
    std::string variable_suffix;

    if (absolute_value)
    {
        variable_prefix = "ABS(";
        variable_suffix = ")";
    }

    std::string db_expression = resolver.variableDBExpression(
        dbcontent_name, variable_name, variable_dbcontent_name);

    std::string db_item_str;

    if (db_expression.size())
        db_item_str = resolver.variableDBColumnName(
            dbcontent_name, variable_name, variable_dbcontent_name);
    else
        db_item_str = resolver.variableDBTableName(
                          dbcontent_name, variable_name, variable_dbcontent_name)
                      + "."
                      + resolver.variableDBColumnName(
                          dbcontent_name, variable_name, variable_dbcontent_name);

    string val_str;
    bool null_contained;

    tie(val_str, null_contained) = transformValue(
        resolver, dbcontent_name, variable_name, variable_dbcontent_name, op, value,
        value_is_representation);

    std::string col_expr = variable_prefix + db_item_str + variable_suffix;

    bool needs_null_or = null_contained || include_null;
    bool is_between = (op == filter_op::between);
    bool has_main_condition = false;

    std::stringstream ss;

    if (needs_null_or)
        ss << "(";

    if (is_between)
    {
        std::string val2_str;
        bool null2;
        tie(val2_str, null2) = transformValue(
            resolver, dbcontent_name, variable_name, variable_dbcontent_name, op, value2,
            value_is_representation);
        ss << col_expr << " " << filter_op::between << " " << val_str
           << " " << filter_op::logic_and << " " << val2_str;
        has_main_condition = true;
    }
    else if (null_contained && val_str.size())
    {
        ss << col_expr << " " << op << val_str;
        has_main_condition = true;
    }
    else if (!null_contained)
    {
        ss << col_expr << " " << op << val_str;
        has_main_condition = true;
    }

    if (needs_null_or)
    {
        if (has_main_condition)
            ss << " " << filter_op::logic_or << " ";
        ss << col_expr << " " << filter_op::is_null << ")";
    }

    clause.sql = ss.str();

    // a DB-expression (computed column) needs its source variable read; a plain column is
    // read directly, so only expressions contribute a required var (matches the old path)
    if (db_expression.size())
        resolver.addVariableToReadSet(dbcontent_name, variable_name, variable_dbcontent_name,
                                      clause.required_vars);

    return clause;
}

std::string DBFilterCondition::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first,
                                                   const std::string& logic_op)
{
    logdbg << "dbcont_name " << dbcontent_name << " first " << first;
    traced_assert(usable_);

    std::stringstream ss;

    std::string variable_prefix;
    std::string variable_suffix;

    if (absolute_value_)
    {
        variable_prefix = "ABS(" + variable_prefix;
        variable_suffix = variable_suffix + ")";
    }

    traced_assert(hasVariable(dbcontent_name));

    std::string db_expression = var_resolver_.variableDBExpression(
        dbcontent_name, variable_name_, variable_dbcontent_name_);

    std::string db_item_str;

    if (db_expression.size())
        db_item_str = var_resolver_.variableDBColumnName(
            dbcontent_name, variable_name_, variable_dbcontent_name_);
    else
        db_item_str = var_resolver_.variableDBTableName(
                          dbcontent_name, variable_name_, variable_dbcontent_name_)
                      + "."
                      + var_resolver_.variableDBColumnName(
                          dbcontent_name, variable_name_, variable_dbcontent_name_);

    if (!first)
    {
        ss << " " << logic_op << " ";
    }
    first = false;

    string val_str;
    bool null_contained;

    tie(val_str, null_contained) = getTransformedValue(value_, dbcontent_name);

    std::string col_expr = variable_prefix + db_item_str + variable_suffix;

    bool needs_null_or = null_contained || include_null_;
    bool is_between = (operator_ == filter_op::between);
    bool has_main_condition = false;

    if (needs_null_or)
        ss << "(";

    if (is_between)
    {
        std::string val2_str;
        bool null2;
        tie(val2_str, null2) = getTransformedValue(value2_, dbcontent_name);
        ss << col_expr << " " << filter_op::between << " " << val_str
           << " " << filter_op::logic_and << " " << val2_str;
        has_main_condition = true;
    }
    else if (null_contained && val_str.size())
    {
        ss << col_expr << " " << operator_ << val_str;
        has_main_condition = true;
    }
    else if (!null_contained)
    {
        ss << col_expr << " " << operator_ << val_str;
        has_main_condition = true;
    }

    if (needs_null_or)
    {
        if (has_main_condition)
            ss << " " << filter_op::logic_or << " ";
        ss << col_expr << " " << filter_op::is_null << ")";
    }

    if (ss.str().size())
        loginf << instanceName() << ": '" << ss.str()
               << "'";

    if (db_expression.size()
        && !var_resolver_.readSetHasVariable(dbcontent_name, variable_name_,
                                             variable_dbcontent_name_, read_set))
    {
        loginf << "db expression, adding var " << variable_name_ << " to read set";
        var_resolver_.addVariableToReadSet(dbcontent_name, variable_name_,
                                           variable_dbcontent_name_, read_set);
    }

    return ss.str();
}

void DBFilterCondition::valueChanged()
{
    logdbg;
    traced_assert(usable_);
    traced_assert(edit_);

    std::string new_value = edit_->text().toStdString();

    value_invalid_ = checkValueInvalid(new_value);

    if (!value_invalid_ && value_ != new_value)
    {
        value_ = new_value;
    }

    loginf << "value '" << value_ << "' invalid "
           << value_invalid_;

    if (value_invalid_)
    {
        edit_->setStyleSheet(LINE_EDIT_INVALID_STYLE);
    }
    else
    {
        edit_->setStyleSheet("");
    }
}

std::string DBFilterCondition::getVariableName() const
{
    return variable_name_;
}

void DBFilterCondition::setVariableName(const std::string& variable_name)
{
    loginf << "name '" << variable_name << "'";

    if (variable_name != variable_name_)
    {
        variable_name_ = variable_name;

        reset();
    }
}

bool DBFilterCondition::hasVariable(const std::string& dbcontent_name)
{
    return variableResolvable(var_resolver_, dbcontent_name, variable_name_,
                              variable_dbcontent_name_);
}

bool DBFilterCondition::variableResolvable(
    IDBVariableResolver& resolver, const std::string& dbcontent_name,
    const std::string& variable_name, const std::string& variable_dbcontent_name)
{
    if (variable_dbcontent_name == META_OBJECT_NAME)
    {
        if (!resolver.existsMetaVariable(variable_name))
            return false;

        return resolver.metaVariableExistsIn(variable_name, dbcontent_name);
    }
    else
    {
        if (dbcontent_name != variable_dbcontent_name)
            return false;

        if (!resolver.existsDBContent(variable_dbcontent_name))
            return false;

        return resolver.dbContentHasVariable(variable_dbcontent_name, variable_name);
    }
}


void DBFilterCondition::update()
{
    if (display_instance_name_)
        base_label_text_ = instanceName() + " " + operator_;
    else
        base_label_text_ = variable_name_ + " " + operator_;

    label_->setText(tr((label_prefix_ + base_label_text_).c_str()));
    edit_->setText(tr(value_.c_str()));
}

void DBFilterCondition::setLabelPrefix(const std::string& prefix)
{
    label_prefix_ = prefix;
    label_->setText(tr((label_prefix_ + base_label_text_).c_str()));
}

void DBFilterCondition::setValue(const std::string& value)
{
    logdbg << "len " << value.size();

    value_ = value;

    update();
}

void DBFilterCondition::reset()
{
    traced_assert(usable_);

    std::string value;

    value = reset_value_;

    value_ = value;
    value_invalid_ = checkValueInvalid(value_);

    loginf << "value '" << value_ << " invalid " << value_invalid_;

    update();
}

bool DBFilterCondition::getDisplayInstanceId() const
{
    return display_instance_name_;
}

bool DBFilterCondition::checkValueInvalid(const std::string& new_value)
{
    traced_assert(usable_);

    if (new_value.size() == 0)
    {
        loginf << "no value, returning invalid";
        return true;
    }

    // collect dbcontent names for all concrete variables
    std::vector<std::string> dbcontent_names;

    if (variable_dbcontent_name_ == META_OBJECT_NAME)
    {
        traced_assert(var_resolver_.existsMetaVariable(variable_name_));
        dbcontent_names = var_resolver_.metaVariableDBContentNames(variable_name_);
    }
    else
    {
        traced_assert(hasVariable(variable_dbcontent_name_));
        dbcontent_names.push_back(variable_dbcontent_name_);
    }

    bool invalid = true;

    try
    {
        for (const auto& dbc_name : dbcontent_names)
        {
            std::string transformed_value;
            bool null_contained;
            tie(transformed_value, null_contained) = getTransformedValue(new_value, dbc_name);
            logdbg << "transformed value " << transformed_value
                   << " null " << null_contained;
        }
        invalid = false;
    }
    catch (std::exception& e)
    {
        logdbg << "exception thrown: " << e.what();
    }
    catch (...)
    {
        logdbg << "exception thrown";
    }

    return invalid;
}

std::pair<std::string, bool> DBFilterCondition::getTransformedValue(
    const std::string& untransformed_value, const std::string& dbcontent_name)
{
    return transformValue(var_resolver_, dbcontent_name, variable_name_, variable_dbcontent_name_,
                          operator_, untransformed_value);
}

std::pair<std::string, bool> DBFilterCondition::transformValue(
    IDBVariableResolver& resolver, const std::string& dbcontent_name,
    const std::string& variable_name, const std::string& variable_dbcontent_name,
    const std::string& op, const std::string& untransformed_value,
    bool value_is_representation)
{
    std::vector<std::string> value_strings;
    std::vector<std::string> transformed_value_strings;

    if (op == filter_op::in || op == filter_op::not_in)
    {
        value_strings = String::split(untransformed_value, ',');
    }
    else
    {
        value_strings.push_back(untransformed_value);
    }

    bool null_set = find(value_strings.begin(), value_strings.end(),
                         filter_op::null_value) != value_strings.end();

    if (null_set) // remove null value
        value_strings.erase(find(value_strings.begin(), value_strings.end(), filter_op::null_value));

    std::string value_str;

    for (auto value_it : value_strings)
    {
        value_str = String::trim(value_it);

        if (value_str.empty())
            continue;

        if (value_is_representation && resolver.variableHasNonStandardRepresentation(
                dbcontent_name, variable_name, variable_dbcontent_name))
            value_str = resolver.variableValueFromRepresentation(
                dbcontent_name, variable_name, variable_dbcontent_name, value_str);

        if (resolver.variableDataType(
                dbcontent_name, variable_name, variable_dbcontent_name) == PropertyDataType::STRING)
        {
            boost::replace_all(value_str, "'", "''");
            transformed_value_strings.push_back("'" + value_str + "'");
        }
        else
            transformed_value_strings.push_back(value_str);
    }

    if (transformed_value_strings.size()) // can be empty if only NULL
    {
        if (op != filter_op::in && op != filter_op::not_in)
        {
            traced_assert(transformed_value_strings.size() == 1);
            value_str = transformed_value_strings.at(0);
        }
        else
            value_str = "(" + boost::algorithm::join(transformed_value_strings, ",") + ")";
    }

    return {value_str, null_set};
}

