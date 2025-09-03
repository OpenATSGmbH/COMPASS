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
#include "property.h"
#include "stringconv.h"
#include "logger.h"
#include "traced_assert.h"

#include <QObject>

#include <string>
#include <vector>

class DBTableColumn;
class DBContent;

namespace dbContent
{

class VariableWidget;

class Variable : public QObject, public Property, public Configurable
{
    Q_OBJECT
  public:
    enum class Representation
    {  // TODO rework to m3a/ta
        STANDARD,
        SECONDS_TO_TIME,
        DEC_TO_OCTAL,
        DEC_TO_HEX,
        FEET_TO_FLIGHTLEVEL,
        DATA_SRC_NAME,
        CLIMB_DESCENT,
        FLOAT_PREC0,
        FLOAT_PREC1,
        FLOAT_PREC2,
        FLOAT_PREC4,
        LINE_NAME
    };

    static Representation stringToRepresentation(const std::string& representation_str);
    static std::string representationToString(Representation representation);
    static const std::map<Representation, std::string>& Representations()
    {
        return representation_2_string_;
    }

    Variable(const std::string& class_id, const std::string& instance_id, DBContent* parent);
    virtual ~Variable();

    bool operator==(const Variable& var);

    std::string str() const;
    void print() const;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    const std::string& name() const { return name_; }
    void name(const std::string& name);

    DBContent& object() const;
    const std::string& dbContentName() const;

    const std::string& description() const { return description_; }
    void description(const std::string& description) { description_ = description; }

    std::string info() const;

    std::string dbColumnName() const;
    void dbColumnName(const std::string& value);

    std::string dbTableName() const;
    //std::string dbColumnIdentifier() const;

    const std::string& dbExpression() const { return db_expression_; }
    std::vector<std::string> dbExpressionVariables() const;

    const std::string& dbColumnOrExpression() const;

    bool isKey() const;
    void isKey(bool value);

    bool hasDimension() { return dimension_.size() > 0; }
    const std::string& dimensionConst() const { return dimension_; }  // TODO should be const
    std::string& dimension() { return dimension_; }                   // TODO should be const
    const std::string& unitConst() const { return unit_; }
    std::string& unit() { return unit_; }
    std::string dimensionUnitStr() const;

    DBContent& dbObject() const
    {
        traced_assert(dbcontent_);
        return *dbcontent_;
    }

//    std::string getMinString();
//    std::string getMaxString();
//    std::string getMinStringRepresentation();
//    std::string getMaxStringRepresentation();

    VariableWidget* widget();

    Representation representation() const;
    Representation& representationRef() { return representation_; } // have to take care of representation_str
    const std::string& representationString() const;
    std::string& representationStringRef() { return representation_str_; }
    void representation(const Representation& representation);

    template <typename T>
    std::string getAsSpecialRepresentationString(T value) const
    {
        traced_assert(representation_ != Variable::Representation::STANDARD);

        std::ostringstream out;
        try
        {
            if (representation_ == Variable::Representation::SECONDS_TO_TIME)
            {
                return Utils::String::timeStringFromDouble(value);
            }
            else if (representation_ == Variable::Representation::DEC_TO_OCTAL)
            {
                out << std::oct << std::setfill('0') << std::setw(4) << value;
            }
            else if (representation_ == Variable::Representation::DEC_TO_HEX)
            {
                out << std::uppercase << std::hex << std::setfill('0') << std::setw(6) << value;
            }
            else if (representation_ == Variable::Representation::FEET_TO_FLIGHTLEVEL)
            {
                out << value / 100.0;
            }
            else if (representation_ == Variable::Representation::DATA_SRC_NAME)
            {
                return getDataSourcesAsString(std::to_string(value));
            }
            else if (representation_ == Variable::Representation::CLIMB_DESCENT)
            {
                int numeric_value = static_cast<int>(value);
                if (numeric_value == 0)
                    return "LVL";
                else if (numeric_value == 1)
                    return "CLB";
                else if (numeric_value == 2)
                    return "DSC";
                else
                    return "UDF";
            }
            else if (representation_ == Variable::Representation::FLOAT_PREC0)
            {
                out << std::fixed << std::setprecision(0)<< value;
            }
            else if (representation_ == Variable::Representation::FLOAT_PREC1)
            {
                out << std::fixed << std::setprecision(1)<< value;
            }
            else if (representation_ == Variable::Representation::FLOAT_PREC2)
            {
                out << std::fixed << std::setprecision(2)<< value;
            }
            else if (representation_ == Variable::Representation::FLOAT_PREC4)
            {
                out << std::fixed << std::setprecision(4)<< value;
            }
            else if (representation_ == Variable::Representation::LINE_NAME)
            {
                return Utils::String::lineStrFrom(value);
            }
            else
            {
                throw std::runtime_error(
                    "Variable: getAsSpecialRepresentationString: unknown representation " +
                    std::to_string((int)representation_));
            }
        }
        catch (std::exception& e)
        {
            logerr << "exception thrown: "
                   << e.what();
        }
        catch (...)
        {
            logerr << "exception thrown";
            ;
        }

        return out.str();
    }

    std::string getRepresentationStringFromValue(const std::string& value_str) const;
    std::string getValueStringFromRepresentation(const std::string& representation_str) const;

    std::string multiplyString(const std::string& value_str, double factor) const;
    const std::string& getLargerValueString(const std::string& value_a_str,
                                            const std::string& value_b_str) const;
    const std::string& getSmallerValueString(const std::string& value_a_str,
                                             const std::string& value_b_str) const;

    bool hasShortName() const;
    std::string shortName() const;
    void shortName(const std::string& short_name);

    bool hasDBContent() const;
    void setHasDBContent();

private:
    static std::map<Representation, std::string> representation_2_string_;
    static std::map<std::string, Representation> string_2_representation_;

    DBContent* dbcontent_{nullptr};

    std::string short_name_;
    /// Value representation type, based on enum STRING_REPRESENTATION
    std::string representation_str_;
    Representation representation_;

    std::string description_;
    
    std::string db_column_name_;
    bool is_key_ {false};

    std::string db_expression_;
    nlohmann::json db_expression_variables_;

    std::string dimension_;
    std::string unit_;

//    bool min_max_set_{false};
//    /// Minimum as string
//    std::string min_;
//    /// Maximum as string
//    std::string max_;

    VariableWidget* widget_{nullptr};

    std::string getDataSourcesAsString(const std::string& value) const;

  protected:
    virtual void checkSubConfigurables();
    //void setMinMax();
};

}

Q_DECLARE_METATYPE(dbContent::Variable*)
