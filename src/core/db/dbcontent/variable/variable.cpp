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

#include "dbcontent/variable/variable.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "db_context_manager.h"
#include "dbcontent/variable/variablewidget.h"
#include "stringconv.h"
#include "global.h"

#include <json.hpp>

#include <algorithm>


using namespace Utils;
using namespace std;

namespace dbContent
{

std::map<Variable::Representation, std::string> Variable::representation_2_string_{
    {Variable::Representation::STANDARD, "STANDARD"},
    {Variable::Representation::SECONDS_TO_TIME, "SECONDS_TO_TIME"},
    {Variable::Representation::DEC_TO_OCTAL, "DEC_TO_OCTAL"},
    {Variable::Representation::DEC_TO_HEX, "DEC_TO_HEX"},
    {Variable::Representation::FEET_TO_FLIGHTLEVEL, "FEET_TO_FLIGHTLEVEL"},
    {Variable::Representation::DATA_SRC_NAME, "DATA_SRC_NAME"},
    {Variable::Representation::CLIMB_DESCENT, "CLIMB_DESCENT"},
    {Variable::Representation::FLOAT_PREC0, "FLOAT_PREC0"},
    {Variable::Representation::FLOAT_PREC1, "FLOAT_PREC1"},
    {Variable::Representation::FLOAT_PREC2, "FLOAT_PREC2"},
    {Variable::Representation::FLOAT_PREC4, "FLOAT_PREC4"},
    {Variable::Representation::LINE_NAME, "LINE_NAME"},
    {Variable::Representation::MLAT_RUS, "MLAT_RUS"},
    {Variable::Representation::SELECTION_SOURCE, "SELECTION_SOURCE"}};

std::map<std::string, Variable::Representation> Variable::string_2_representation_{
    {"STANDARD", Variable::Representation::STANDARD},
    {"SECONDS_TO_TIME", Variable::Representation::SECONDS_TO_TIME},
    {"DEC_TO_OCTAL", Variable::Representation::DEC_TO_OCTAL},
    {"DEC_TO_HEX", Variable::Representation::DEC_TO_HEX},
    {"FEET_TO_FLIGHTLEVEL", Variable::Representation::FEET_TO_FLIGHTLEVEL},
    {"DATA_SRC_NAME", Variable::Representation::DATA_SRC_NAME},
    {"CLIMB_DESCENT", Variable::Representation::CLIMB_DESCENT},
    {"FLOAT_PREC0", Variable::Representation::FLOAT_PREC0},
    {"FLOAT_PREC1", Variable::Representation::FLOAT_PREC1},
    {"FLOAT_PREC2", Variable::Representation::FLOAT_PREC2},
    {"FLOAT_PREC4", Variable::Representation::FLOAT_PREC4},
    {"LINE_NAME", Variable::Representation::LINE_NAME},
    {"MLAT_RUS", Variable::Representation::MLAT_RUS},
    {"SELECTION_SOURCE", Variable::Representation::SELECTION_SOURCE}};

Variable::Representation Variable::stringToRepresentation(
    const std::string& representation_str)
{
    traced_assert(string_2_representation_.count(representation_str) == 1);
    return string_2_representation_.at(representation_str);
}

std::string Variable::representationToString(Representation representation)
{
    traced_assert(representation_2_string_.count(representation) == 1);
    return representation_2_string_.at(representation);
}

//#include <boost/algorithm/string.hpp>

Variable::Variable(nlohmann::json& config, DBContent* parent,
                   const std::string& dbcontent_name)
    : Property(), Configurable(config, parent),
      dbcontent_(parent), dbcontent_name_(dbcontent_name)
{
    registerParameter("name", &name_, std::string());
    registerParameter("short_name", &short_name_, std::string());
    registerParameter("description", &description_, std::string());
    registerParameter("group", &group_, std::string());

    registerParameter("origin", &origin_, std::string());
    registerParameter("data_item", &data_item_, std::string());
    registerParameter("inputs", &inputs_, nlohmann::json::array());
    registerParameter("calculation", &calculation_, nlohmann::json::object());
    registerParameter("selection", &selection_, nlohmann::json::array());
    registerParameter("source_variable", &source_variable_, std::string());

    registerParameter("db_column_name", &db_column_name_, std::string());
    registerParameter("is_key", &is_key_, false);

    registerParameter("data_type_str", &data_type_str_, std::string());
    registerParameter("representation_str", &representation_str_, std::string());
    registerParameter("dimension", &dimension_, std::string());
    registerParameter("unit", &unit_, std::string());

    traced_assert_msg(name_.size(), "Name required");

    traced_assert_msg(db_column_name_.size(), "DB column name required");

    traced_assert_msg(data_type_str_.size(), "Data type required");
    data_type_ = Property::asDataType(data_type_str_);

    if (representation_str_.size() == 0)
        representation_str_ = representationToString(Representation::STANDARD);

    representation_ = stringToRepresentation(representation_str_);

    traced_assert_msg(origin_.empty() || origin_ == OriginAsterix || origin_ == OriginCalculated
                      || origin_ == OriginInternal || origin_ == OriginSelection,
                      ("Unknown origin '" + origin_ + "' of variable '" + name_ + "'").c_str());
    traced_assert_msg(inputs_.is_array(), ("Inputs of variable '" + name_ + "' must be a list").c_str());
    traced_assert_msg(calculation_.is_object(), ("Calculation of variable '" + name_ + "' must be an object").c_str());
    traced_assert_msg(selection_.is_array(), ("Selection of variable '" + name_ + "' must be a list").c_str());
    traced_assert_msg(origin_ != OriginSelection || selection_.size(),
                      ("Selection variable '" + name_ + "' has no candidates").c_str());

    createSubConfigurables();
}

const std::string Variable::OriginAsterix    = "asterix";
const std::string Variable::OriginCalculated = "calculated";
const std::string Variable::OriginInternal   = "internal";
const std::string Variable::OriginSelection  = "selection";
const std::string Variable::OriginReport     = "report";

const std::string& Variable::origin() const
{
    if (origin_.size())
        return origin_;

    if (isReportVariable())
        return OriginReport;

    if (dbcontent_)
        return dbcontent_->origin();

    return origin_;
}

void Variable::origin(const std::string& origin)
{
    traced_assert(origin.empty() || origin == OriginAsterix || origin == OriginCalculated
                  || origin == OriginInternal || origin == OriginSelection);
    origin_ = origin;
}

std::string Variable::dataItemPart() const
{
    return dataItemPartOf(data_item_);
}

std::string Variable::dataItemPartOf(const std::string& data_item)
{
    // "I021/071.Time of Applicability for Position" -> "I021/071", "I021/REF.SGV.GSS" -> "I021/REF.SGV"

    if (data_item.empty())
        return "";

    size_t slash = data_item.find('/');

    if (slash == std::string::npos)
        return data_item;

    size_t first_dot = data_item.find('.', slash);

    if (first_dot == std::string::npos)
        return data_item;

    std::string first_segment = data_item.substr(slash + 1, first_dot - slash - 1);

    if (first_segment == "REF" || first_segment == "SPF")
    {
        size_t second_dot = data_item.find('.', first_dot + 1);

        if (second_dot == std::string::npos)
            return data_item;

        return data_item.substr(0, second_dot);
    }

    return data_item.substr(0, first_dot);
}

std::vector<std::string> Variable::inputs() const
{
    std::vector<std::string> inputs;

    for (const auto& input : inputs_)
        if (input.is_string())
            inputs.push_back(input.get<std::string>());

    return inputs;
}

void Variable::inputs(const std::vector<std::string>& inputs)
{
    inputs_ = inputs;
}

bool Variable::hasCalculation() const
{
    return calculation_.is_object() && calculation_.contains("op");
}

unsigned int Variable::numSelectionCandidates() const
{
    return selection_.size();
}

unsigned int Variable::selectionCandidateIndex(const std::string& calculation) const
{
    for (unsigned int cnt = 0; cnt < selection_.size(); ++cnt)
    {
        const auto& candidate = selection_.at(cnt);

        if (candidate.is_object() && candidate.contains("calculation")
            && candidate.at("calculation") == calculation)
            return cnt + 1;
    }

    return 0;
}

std::string Variable::selectionCandidateLabel(unsigned int index) const
{
    traced_assert(index >= 1 && index <= selection_.size());

    const auto& candidate = selection_.at(index - 1);

    if (!candidate.is_object())
        return "";

    if (candidate.contains("label") && candidate.at("label").is_string())
        return candidate.at("label");

    if (candidate.contains("variable") && candidate.at("variable").is_string())
    {
        std::string var_name = candidate.at("variable");

        if (dbcontent_ && dbcontent_->hasVariable(var_name))
        {
            const Variable& var = dbcontent_->variable(var_name);

            if (var.dataItem().size())
                return var.dataItemPart();
        }

        return var_name;
    }

    if (candidate.contains("calculation") && candidate.at("calculation").is_string())
        return candidate.at("calculation");

    return "";
}

std::vector<std::string> Variable::selectionCandidateVariables() const
{
    std::vector<std::string> names;

    if (!isSelection())
        return names;

    for (const auto& candidate : selection_)
    {
        if (candidate.is_object() && candidate.contains("variable") && candidate.at("variable").is_string())
            names.push_back(candidate.at("variable"));
    }

    return names;
}

std::string Variable::displayName(unsigned int max_chars) const
{
    std::vector<std::string> forms;

    std::string item = isAsterix() ? dataItemPart() : "";

    if (item.size())
    {
        forms.push_back(item + " " + name_);

        if (hasShortName())
            forms.push_back(item + " " + short_name_);
    }

    forms.push_back(name_);

    if (hasShortName())
        forms.push_back(short_name_);

    if (!max_chars)
        return forms.front();

    const std::string* longest_fitting {nullptr};
    const std::string* shortest = &forms.front();

    for (const auto& form : forms)
    {
        if (form.size() <= max_chars && (!longest_fitting || form.size() > longest_fitting->size()))
            longest_fitting = &form;

        if (form.size() < shortest->size())
            shortest = &form;
    }

    return longest_fitting ? *longest_fitting : *shortest;
}

std::string Variable::selectionSourceLabel(const std::string& index_str) const
{
    const Variable* selection = selectionOf();

    if (!selection)
        return index_str;

    unsigned int index {0};

    try
    {
        index = std::stoul(index_str);
    }
    catch (const std::exception&)
    {
        return index_str;
    }

    if (index < 1 || index > selection->numSelectionCandidates())
        return index_str;

    return selection->selectionCandidateLabel(index);
}

std::string Variable::selectionSourceIndex(const std::string& label) const
{
    const Variable* selection = selectionOf();

    if (!selection)
        return label;

    for (unsigned int index = 1; index <= selection->numSelectionCandidates(); ++index)
        if (selection->selectionCandidateLabel(index) == label)
            return std::to_string(index);

    return label;
}

const Variable* Variable::selectionOf() const
{
    if (!dbcontent_)
        return nullptr;

    for (const auto& var_it : dbcontent_->variables())
    {
        const Variable& var = *var_it.second;

        if (var.isSelection() && var.sourceVariableName() == name_)
            return &var;
    }

    return nullptr;
}

std::string Variable::originInfo() const
{
    const std::string& orig = origin();

    if (orig == OriginAsterix)
        return orig + (data_item_.size() ? " " + data_item_ : "");

    if (orig == OriginCalculated)
    {
        std::string text = orig;
        auto input_names = inputs();

        for (unsigned int cnt = 0; cnt < input_names.size(); ++cnt)
            text += (cnt ? ", " : " from ") + input_names.at(cnt);

        return text;
    }

    if (orig == OriginSelection)
    {
        std::string text = orig;

        for (unsigned int cnt = 1; cnt <= selection_.size(); ++cnt)
            text += (cnt > 1 ? ", " : " ") + selectionCandidateLabel(cnt);

        return text;
    }

    return orig;
}

//Variable& Variable::operator=(Variable&& other)
////: Configurable(std::move(other))
//{
//    loginf << "Variable: move operator: moving";

//    data_type_ = other.data_type_;
//    data_type_str_ = other.data_type_str_;

//    name_ = other.name_;
//    other.name_ = "";

//    dbcontent_ = other.dbcontent_;
//    other.dbcontent_ = nullptr;

//    representation_str_ = other.representation_str_;
//    other.representation_str_ = "";

//    representation_ = other.representation_;
//    other.representation_ = Representation::STANDARD;

//    description_ = other.description_;
//    other.description_ = "";

//    db_column_name_ = other.db_column_name_;
//    other.db_column_name_ = "";

////    min_max_set_ = other.min_max_set_;
////    other.min_max_set_ = false;

////    min_ = other.min_;
////    other.min_ = "";

////    max_ = other.max_;
////    other.max_ = "";

//    dimension_ = other.dimension_;
//    other.dimension_ = "";

//    unit_ = other.unit_;
//    other.unit_ = "";

//    widget_ = other.widget_;
//    if (widget_)
//        widget_->setVariable(*this);
//    other.widget_ = nullptr;

//    other.configuration().updateParameterPointer("name", &name_);
//    other.configuration().updateParameterPointer("description", &description_);
//    other.configuration().updateParameterPointer("variable_identifier", &db_column_name_);
//    other.configuration().updateParameterPointer("data_type_str", &data_type_str_);
//    other.configuration().updateParameterPointer("representation_str", &representation_str_);
//    other.configuration().updateParameterPointer("dimension", &dimension_);
//    other.configuration().updateParameterPointer("unit", &unit_);

//    // return *this;
//    return static_cast<Variable&>(Configurable::operator=(std::move(other)));
//}

Variable::~Variable()
{
    if (widget_)
    {
        delete widget_;
        widget_ = nullptr;
    }
}

bool Variable::operator==(const Variable& var)
{
    if (dbContentName() != var.dbContentName())
        return false;
    if (data_type_ != var.data_type_)
        return false;
    if (name_.compare(var.name_) != 0)
        return false;

    return true;
}

std::string Variable::str() const
{
    return dbcontent_name_ + ": " + name_ + " (" + data_type_str_ + ")";
}

void Variable::print() const
{
    loginf << str();
}

DBContent& Variable::object() const
{
    traced_assert(dbcontent_);
    return *dbcontent_;
}

const std::string& Variable::dbContentName() const
{
    traced_assert(dbcontent_);
    return dbcontent_->name();
}

void Variable::name(const std::string& name)
{
    loginf << "old " << name_ << " new " << name;
    name_ = name;
}

//const std::string& Variable::metaTable() const
//{
//    traced_assert(dbcontent_);
//    return dbcontent_->currentMetaTable();
//}

std::string Variable::info() const
{
    std::ostringstream ss;

    ss << "Name: '" << name_ << "'" << endl;

    if (hasShortName())
        ss << "Short Name: " << shortName() << endl;

    ss << "Description: " << description() << endl;
    ss << "Origin: " << originInfo() << endl;
    ss << "Data Type: " << dataTypeString() << endl;
    ss << "Unit: " << dimensionUnitStr() << endl;

//    QTextEdit* description_edit_ {nullptr};
//    VariableDataTypeComboBox* type_combo_ {nullptr};
//    UnitSelectionWidget* unit_sel_ {nullptr};
//    StringRepresentationComboBox* representation_box_ {nullptr};
//    QLineEdit* db_column_edit_ {nullptr};

    return ss.str();
}

std::string Variable::dbColumnName() const
{
    return db_column_name_;
}

void Variable::dbColumnName(const std::string& value)
{
    db_column_name_ = value;
}

std::string Variable::dbTableName() const
{
    //a Report Variable is read through a join subquery, its alias is what qualifies the column
    //in a SELECT list or a WHERE clause
    if (isReportVariable())
        return report_join_alias_;

    traced_assert(dbcontent_);
    return dbcontent_->dbTableName();
}

/**
 * Runtime variable for one column of a Report Table. It is not a child of a Configurable, so it
 * is never written to db_content_*.json, and it carries the join information.
 */
std::unique_ptr<Variable> Variable::createReportVariable(const std::string& name,
                                                         const std::string& dbcontent_name,
                                                         const std::string& column_alias,
                                                         const std::string& join_alias,
                                                         const std::string& report_table_name,
                                                         const std::string& report_column_name,
                                                         PropertyDataType data_type,
                                                         const std::string& description,
                                                         const std::string& dimension,
                                                         const std::string& unit,
                                                         const std::string& representation_str)
{
    nlohmann::json config = nlohmann::json::object();

    Configuration::setClassName(config, "Variable");
    Configuration::setInstanceName(config, "ReportVariable_" + report_table_name + "_" + report_column_name);

    auto& params = config[ Configuration::ParameterSection ];

    params[ "name"           ] = name;
    params[ "db_column_name" ] = column_alias;
    params[ "data_type_str"  ] = Property::asString(data_type);
    params[ "description"    ] = description;
    params[ "dimension"      ] = dimension;
    params[ "unit"           ] = unit;

    if (!representation_str.empty())
        params[ "representation_str" ] = representation_str;

    std::unique_ptr<Variable> variable(new Variable(config, nullptr, dbcontent_name));

    variable->report_table_name_  = report_table_name;
    variable->report_column_name_ = report_column_name;
    variable->report_join_alias_  = join_alias;

    return variable;
}

// std::string Variable::dbColumnIdentifier() const
// {
//     return dbTableName()+":"+dbColumnName();
// }

//void Variable::setMinMax()
//{
//    traced_assert(!min_max_set_);

//    traced_assert(dbcontent_);
//    logdbg << "variable " << dbcontent_->name() << " " << name_ << ": setMinMax";

//    if (!dbObject().existsInDB() || !dbObject().count())
//    {
//        min_ = NULL_STRING;
//        max_ = NULL_STRING;
//    }
//    else
//    {
//        std::pair<std::string, std::string> min_max =
//            COMPASS::instance().interface().getMinMaxString(*this);

//        min_ = min_max.first;
//        max_ = min_max.second;
//    }

//    min_max_set_ = true;

//    logdbg << "min " << min_ << " max " << max_;
//}

//std::string Variable::getMinString()
//{
//    if (!min_max_set_)
//        setMinMax();  // already unit transformed

//    traced_assert(min_max_set_);

//    logdbg << "object " << dbContentName() << " name " << name()
//           << " returning " << min_;
//    return min_;
//}

//std::string Variable::getMaxString()
//{
//    if (!min_max_set_)
//        setMinMax();  // is already unit transformed

//    traced_assert(min_max_set_);

//    logdbg << "object " << dbContentName() << " name " << name()
//           << " returning " << max_;
//    return max_;
//}

//std::string Variable::getMinStringRepresentation()
//{
//    if (representation_ == Representation::STANDARD)
//        return getMinString();
//    else
//        return getRepresentationStringFromValue(getMinString());
//}

//std::string Variable::getMaxStringRepresentation()
//{
//    if (representation_ == Representation::STANDARD)
//        return getMaxString();
//    else
//        return getRepresentationStringFromValue(getMaxString());
//}

std::string Variable::dimensionUnitStr() const
{
    if (dimension_.size())
        return dimension_ + ":" + unit_;
    else
        return "";
}

VariableWidget* Variable::widget()
{
    if (!widget_)
    {
        widget_ = new VariableWidget(*this);
        traced_assert(widget_);
    }

    return widget_;
}

Variable::Representation Variable::representation() const { return representation_; }

const std::string& Variable::representationString() const { return representation_str_; }

void Variable::representation(const Variable::Representation& representation)
{
    representation_str_ = representationToString(representation);
    representation_ = representation;
}

std::string Variable::getAsJSONRepresentationString(const nlohmann::json& value,
                                                    const context::DBContextManager* ctx_man,
                                                    unsigned int ds_id) const
{
    if (representation_ == Representation::MLAT_RUS)
    {
        if (!value.is_array() || value.empty())
            return value.dump();

        std::string out = "[";
        bool first = true;
        for (const auto& j_idx : value)
        {
            if (!j_idx.is_number_integer())
                continue;

            int idx = j_idx.get<int>();
            if (!first) out += ", ";
            first = false;

            out += ctx_man ? ctx_man->remoteUnitName(ds_id, idx) : std::to_string(idx);
        }
        out += "]";
        return out;
    }

    return value.dump();
}

std::string Variable::getRepresentationStringFromValue(const std::string& value_str) const
{
    logdbg << "value " << value_str << " data_type "
           << Property::asString(data_type_) << " representation "
           << representationToString(representation_);

    if (value_str == NULL_STRING)
        return value_str;

    if (representation_ == Variable::Representation::STANDARD)
        return value_str;

    switch (data_type_)
    {
        case PropertyDataType::BOOL:
        {
            bool value = std::stoul(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::CHAR:
        {
            char value = std::stoi(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::UCHAR:
        {
            unsigned char value = std::stoul(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::INT:
        {
            int value = std::stoi(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::UINT:
        {
            unsigned int value = std::stoul(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::LONGINT:
        {
            long value = std::stol(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::ULONGINT:
        {
            unsigned long value = std::stoul(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::FLOAT:
        {
            float value = std::stof(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::DOUBLE:
        {
            double value = std::stod(value_str);
            return getAsSpecialRepresentationString(value);
        }
        case PropertyDataType::STRING:
            throw std::invalid_argument(
                "Variable: getRepresentationStringFromValue: representation of string variable"
                " impossible");
        case PropertyDataType::JSON:
            throw std::invalid_argument(
                "Variable: getRepresentationStringFromValue: representation of json variable"
                " impossible");
    case PropertyDataType::TIMESTAMP:
        throw std::invalid_argument(
            "Variable: getRepresentationStringFromValue: representation of timestamp variable"
            " impossible");
        default:
            logerr << ": unknown property type "
                   << Property::asString(data_type_);
            throw std::runtime_error(
                "Variable: getRepresentationStringFromValue:: unknown property type " +
                Property::asString(data_type_));
    }
}

std::string Variable::getValueStringFromRepresentation(
    const std::string& representation_str) const
{
    if (representation_str == NULL_STRING)
        return representation_str;

    traced_assert(representation_ != Variable::Representation::STANDARD);

    if (representation_ == Variable::Representation::SECONDS_TO_TIME)
    {
        return String::getValueString(Utils::String::timeFromString(representation_str));
    }
    else if (representation_ == Variable::Representation::DEC_TO_OCTAL)
    {
        return String::getValueString(Utils::String::intFromOctalString(representation_str));
    }
    else if (representation_ == Variable::Representation::DEC_TO_HEX)
    {
        return String::getValueString(Utils::String::intFromHexString(representation_str));
    }
    else if (representation_ == Variable::Representation::FEET_TO_FLIGHTLEVEL)
    {
        return String::getValueString(std::stod(representation_str) * 100.0);
    }
    else if (representation_ == Variable::Representation::DATA_SRC_NAME)
    {
        auto& ctx_man = dbcontent_->dbContentManager().dbContextManager();

        if (ctx_man.hasDataSource(representation_str))
            return std::to_string(ctx_man.getDataSourceId(representation_str));

        // not found, return original

        return representation_str;
    }
    else if (representation_ == Variable::Representation::SELECTION_SOURCE)
    {
        return selectionSourceIndex(representation_str);
    }
    else if (representation_ == Variable::Representation::CLIMB_DESCENT)
    {
        if (representation_str == "LVL")
            return "0";
        else if (representation_str == "CLB")
            return "1";
        else if (representation_str == "DSC")
            return "2";
        else
            return "3";
    }
    else if (representation_ == Variable::Representation::FLOAT_PREC0)
    {
        return representation_str;
    }
    else if (representation_ == Variable::Representation::FLOAT_PREC1)
    {
        return representation_str;
    }
    else if (representation_ == Variable::Representation::FLOAT_PREC2)
    {
        return representation_str;
    }
    else if (representation_ == Variable::Representation::FLOAT_PREC4)
    {
        return representation_str;
    }
    else if (representation_ == Variable::Representation::LINE_NAME)
    {
        return std::to_string(Utils::String::lineFromStr(representation_str));
    }
    else
    {
        throw std::runtime_error(
            "Utils: String: getValueStringFromRepresentation: unknown representation " +
            std::to_string((int)representation_));
    }
}

std::string Variable::multiplyString(const std::string& value_str, double factor) const
{
    logdbg << "value " << value_str << " factor " << factor
           << " data_type " << Property::asString(data_type_);

    if (value_str == NULL_STRING)
        return value_str;

    std::string return_string;

    switch (data_type_)
    {
        case PropertyDataType::BOOL:
        {
            bool value = std::stoul(value_str);
            value = value && factor;
            return_string = std::to_string(value);
            break;
        }
        case PropertyDataType::CHAR:
        {
            char value = std::stoi(value_str);
            value *= factor;
            return_string = std::to_string(value);
            break;
        }
        case PropertyDataType::UCHAR:
        {
            unsigned char value = std::stoul(value_str);
            value *= factor;
            return_string = std::to_string(value);
            break;
        }
        case PropertyDataType::INT:
        {
            int value = std::stoi(value_str);
            value *= factor;
            return_string = std::to_string(value);
            break;
        }
        case PropertyDataType::UINT:
        {
            unsigned int value = std::stoul(value_str);
            value *= factor;
            return_string = std::to_string(value);
            break;
        }
        case PropertyDataType::LONGINT:
        {
            long value = std::stol(value_str);
            value *= factor;
            return_string = std::to_string(value);
            break;
        }
        case PropertyDataType::ULONGINT:
        {
            unsigned long value = std::stoul(value_str);
            value *= factor;
            return_string = std::to_string(value);
            break;
        }
        case PropertyDataType::FLOAT:
        {
            float value = std::stof(value_str);
            value *= factor;
            return_string = String::getValueString(value);
            break;
        }
        case PropertyDataType::DOUBLE:
        {
            double value = std::stod(value_str);
            value *= factor;
            return_string = String::getValueString(value);
            break;
        }
        case PropertyDataType::STRING:
            throw std::invalid_argument(
                "Variable: multiplyString: multiplication of string variable impossible");
        case PropertyDataType::JSON:
            throw std::invalid_argument(
                "Variable: multiplyString: multiplication of json variable impossible");
        case PropertyDataType::TIMESTAMP:
            throw std::invalid_argument(
                "Variable: multiplyString: multiplication of timestamp variable impossible");
        default:
            logerr << ": unknown property type "
                   << Property::asString(data_type_);
            throw std::runtime_error("Variable: multiplyString:: unknown property type " +
                                     Property::asString(data_type_));
    }

    logdbg << "return value " << return_string;

    return return_string;
}

const std::string& Variable::getLargerValueString(const std::string& value_a_str,
                                                     const std::string& value_b_str) const
{
    logdbg << "value a " << value_a_str << " b " << value_b_str
           << " data_type " << Property::asString(data_type_);

    if (value_a_str == NULL_STRING || value_b_str == NULL_STRING)
    {
        if (value_a_str != NULL_STRING)
            return value_a_str;
        if (value_b_str != NULL_STRING)
            return value_b_str;
        return NULL_STRING;
    }

    switch (data_type_)
    {
        case PropertyDataType::BOOL:
        case PropertyDataType::UCHAR:
        case PropertyDataType::UINT:
        case PropertyDataType::ULONGINT:
        {
            if (std::stoul(value_a_str) > std::stoul(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::CHAR:
        case PropertyDataType::INT:
        {
            if (std::stoi(value_a_str) > std::stoi(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::LONGINT:
        {
            if (std::stol(value_a_str) > std::stol(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::FLOAT:
        {
            if (std::stof(value_a_str) > std::stof(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::DOUBLE:
        {
            if (std::stod(value_a_str) > std::stod(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::STRING:
            throw std::invalid_argument(
                "Variable: getLargerValueString: operation on string variable impossible");
        case PropertyDataType::JSON:
            throw std::invalid_argument(
                "Variable: getLargerValueString: operation on json variable impossible");
        case PropertyDataType::TIMESTAMP:
            throw std::invalid_argument(
                "Variable: getLargerValueString: operation on timestamp variable impossible");
        default:
            logerr << ": unknown property type "
                   << Property::asString(data_type_);
            throw std::runtime_error("Variable: getLargerValueString:: unknown property type " +
                                     Property::asString(data_type_));
    }
}

const std::string& Variable::getSmallerValueString(const std::string& value_a_str,
                                                      const std::string& value_b_str) const
{
    logdbg << "value a " << value_a_str << " b " << value_b_str
           << " data_type " << Property::asString(data_type_);

    if (value_a_str == NULL_STRING || value_b_str == NULL_STRING)
    {
        if (value_a_str != NULL_STRING)
            return value_a_str;
        if (value_b_str != NULL_STRING)
            return value_b_str;
        return NULL_STRING;
    }

    switch (data_type_)
    {
        case PropertyDataType::BOOL:
        case PropertyDataType::UCHAR:
        case PropertyDataType::UINT:
        case PropertyDataType::ULONGINT:
        {
            if (std::stoul(value_a_str) < std::stoul(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::CHAR:
        case PropertyDataType::INT:
        {
            if (std::stoi(value_a_str) < std::stoi(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::LONGINT:
        {
            if (std::stol(value_a_str) < std::stol(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::FLOAT:
        {
            if (std::stof(value_a_str) < std::stof(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::DOUBLE:
        {
            if (std::stod(value_a_str) < std::stod(value_b_str))
                return value_a_str;
            else
                return value_b_str;
        }
        case PropertyDataType::STRING:
            throw std::invalid_argument(
                "Variable: getSmallerValueString: operation on string variable impossible");
        case PropertyDataType::JSON:
            throw std::invalid_argument(
                "Variable: getSmallerValueString: operation on json variable impossible");
        case PropertyDataType::TIMESTAMP:
            throw std::invalid_argument(
                "Variable: getSmallerValueString: operation on timestamp variable impossible");
        default:
            logerr << ": unknown property type "
                   << Property::asString(data_type_);
            throw std::runtime_error("Variable: getSmallerValueString:: unknown property type " +
                                     Property::asString(data_type_));
    }
}

bool Variable::hasShortName() const
{
    return short_name_.size();
}

std::string Variable::shortName() const
{
    return short_name_;
}

void Variable::shortName(const std::string& short_name)
{
    short_name_ = short_name;
}

bool Variable::hasDBContent() const
{
    //a Report Variable exists exactly as long as its Report Table does, and the registry only
    //offers the tables of the current database
    if (isReportVariable())
        return true;

    traced_assert(dbcontent_);

    if (db_column_name_.size())
        return dbcontent_->dbContentManager().dbInterface().hasContentIn(dbTableName(), db_column_name_);

    return false;
}

void Variable::setHasDBContent()
{
    dbcontent_->dbContentManager().dbInterface().setContentIn(dbTableName(), db_column_name_);
}

bool Variable::isKey() const
{
    return is_key_;
}

void Variable::isKey(bool value)
{
    is_key_ = value;
}

std::string Variable::getDataSourcesAsString(const std::string& value) const
{
    traced_assert(dbcontent_);

    auto& ctx_man = dbcontent_->dbContentManager().dbContextManager();

    unsigned int ds_id = stoi(value);

    if (ctx_man.hasDataSource(ds_id))
    {
        const auto* ds = ctx_man.dataSource(ds_id);

        if (ds->hasShortName())
            return ds->shortName();
        else
            return ds->name();
    }

    // has no datasources, return original

    return value;
}

}

