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
#include "representation.h"
#include "stringconv.h"
#include "logger.h"
#include "traced_assert.h"

#include <QObject>

#include <json_fwd.hpp>

#include <memory>
#include <string>
#include <vector>

class DBTableColumn;
class DBContent;

namespace context { class DBContextManager; }

namespace dbContent
{

class VariableWidget;

class Variable : public QObject, public Property, public Configurable
{
    Q_OBJECT
  public:
    // the enum itself lives in representation.h, so that the context-free part
    // of the formatting can be used without an application instance
    using Representation = dbContent::Representation;

    static Representation stringToRepresentation(const std::string& representation_str);
    static std::string representationToString(Representation representation);
    static const std::map<Representation, std::string>& Representations()
    {
        return representation_2_string_;
    }

    Variable(nlohmann::json& config, DBContent* parent,
             const std::string& dbcontent_name);
    virtual ~Variable();

    bool operator==(const Variable& var);

    std::string str() const;
    void print() const;

    const std::string& name() const { return name_; }
    void name(const std::string& name);

    DBContent& object() const;
    const std::string& dbContentName() const;

    const std::string& description() const { return description_; }
    void description(const std::string& description) { description_ = description; }

    /// origin of the value: asterix, calculated, internal, selection or report, see readme_dbcontent.md.
    /// Falls back to the origin of the DBContent when the variable has none, RefTraj
    const std::string& origin() const;
    void origin(const std::string& origin);
    bool isAsterix() const { return origin() == OriginAsterix; }
    bool isCalculated() const { return origin() == OriginCalculated; }
    bool isInternal() const { return origin() == OriginInternal; }
    bool isSelection() const { return origin() == OriginSelection; }

    /// ASTERIX item of an asterix variable in EUROCONTROL notation, "I021/071.Time of Applicability for Position"
    const std::string& dataItem() const { return data_item_; }
    void dataItem(const std::string& data_item) { data_item_ = data_item; }
    /// item part of the data item, "I021/071", for REF and SPF the first two segments, "I021/REF.SGV"
    std::string dataItemPart() const;
    static std::string dataItemPartOf(const std::string& data_item);

    /// names of the variables of the same DBContent a calculated variable is computed from
    std::vector<std::string> inputs() const;
    void inputs(const std::vector<std::string>& inputs);

    /// operator of a calculated variable, {"op": "max"}, empty object when none, see readme_calc_operators.md
    const nlohmann::json& calculation() const { return calculation_; }
    bool hasCalculation() const;

    /// ordered candidate list of a selection variable, see readme_dbcontent.md.
    /// Each candidate is a JSON object with "variable" or "calculation" plus "label",
    /// optional "require_true" and "require_false" lists of boolean variable names
    const nlohmann::json& selection() const { return selection_; }
    void selection(const nlohmann::json& selection) { selection_ = selection; }
    unsigned int numSelectionCandidates() const;
    /// 1-based index of the candidate running the named calculation, 0 when there is none
    unsigned int selectionCandidateIndex(const std::string& calculation) const;
    /// label of the 1-based candidate: the data item part of a variable candidate or the explicit label
    std::string selectionCandidateLabel(unsigned int index) const;
    /// names of the variable candidates in their order, calculation candidates left out,
    /// empty when this is no selection. A filter matches every candidate, not the chosen value only
    std::vector<std::string> selectionCandidateVariables() const;

    /// name of the companion variable holding the 1-based candidate index per record, selection only
    const std::string& sourceVariableName() const { return source_variable_; }
    void sourceVariableName(const std::string& name) { source_variable_ = name; }

    /// one-line summary of the origin for tooltips and dialogs,
    /// "asterix I021/071.Time of Applicability for Position", "calculated from Vx, Vy", ...
    std::string originInfo() const;

    /// name shown in the GUI: the longest of item plus name, item plus short name, name and
    /// short name that fits into max_chars, the shortest form when none fits, 0 means no limit.
    /// Variables without a data item have the forms name and short name only. Never truncates
    std::string displayName(unsigned int max_chars = 0) const;

    /// the selection variable this variable is the source variable of, nullptr when none
    const Variable* selectionOf() const;
    /// SELECTION_SOURCE representation: the candidate label of a 1-based index string, the
    /// index itself when this is no source variable or the index is out of range
    std::string selectionSourceLabel(const std::string& index_str) const;
    /// the reverse: the 1-based index string of a candidate label, the label when unknown
    std::string selectionSourceIndex(const std::string& label) const;

    static const std::string OriginAsterix;
    static const std::string OriginCalculated;
    static const std::string OriginInternal;
    static const std::string OriginSelection;
    static const std::string OriginReport;

    const std::string& group() const { return group_; }
    void group(const std::string& group) { group_ = group; }

    std::string info() const;

    std::string dbColumnName() const;
    void dbColumnName(const std::string& value);

    std::string dbTableName() const;
    //std::string dbColumnIdentifier() const;

    bool isKey() const;
    void isKey(bool value);

    bool hasDimension() { return dimension_.size() > 0; }
    const std::string& dimensionConst() const { return dimension_; }  // TODO should be const
    std::string& dimension() { return dimension_; }                   // TODO should be const
    const std::string& unitConst() const { return unit_; }
    std::string& unit() { return unit_; }
    std::string dimensionUnitStr() const;

    DBContent& dbContent() const
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

        try
        {
            //everything that can be resolved from the value alone
            std::string str;
            if (dbContent::representationString(str, representation_, value))
                return str;

            //the rest needs the application context
            if (representation_ == Variable::Representation::DATA_SRC_NAME)
                return getDataSourcesAsString(std::to_string(value));

            if (representation_ == Variable::Representation::SELECTION_SOURCE)
                return selectionSourceLabel(std::to_string(value));

            throw std::runtime_error(
                "Variable: getAsSpecialRepresentationString: unknown representation " +
                std::to_string((int)representation_));
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

        return "";
    }

    /// Formats a JSON-typed variable's value according to representation_.
    /// For STANDARD (and any non-JSON-aware mode) returns value.dump().
    /// For MLAT_RUS expects value to be an array of integer RU indices and
    /// resolves each through ctx_man.remoteUnitName(ds_id, idx); returns a
    /// "[name1, name2, ...]" string. ctx_man may be null and ds_id 0 - in
    /// which case the index is rendered as a number.
    std::string getAsJSONRepresentationString(const nlohmann::json& value,
                                              const context::DBContextManager* ctx_man,
                                              unsigned int ds_id) const;

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

    /*report variable related, see readme_dynamic_dbcontent.md Section 4.3*/

    /// a Report Variable is a column of a Report Table joined onto the host data content
    bool isReportVariable() const { return !report_table_name_.empty(); }
    /// the DuckDB table holding the column
    const std::string& reportTableName() const { return report_table_name_; }
    /// the column inside that table
    const std::string& reportColumnName() const { return report_column_name_; }
    /// the alias of the join subquery, dbTableName() returns it so a WHERE clause qualifies right
    const std::string& reportJoinAlias() const { return report_join_alias_; }

    /// creates a runtime variable for a Report Table column, outside the configuration tree
    static std::unique_ptr<Variable> createReportVariable(const std::string& name,
                                                          const std::string& dbcontent_name,
                                                          const std::string& column_alias,
                                                          const std::string& join_alias,
                                                          const std::string& report_table_name,
                                                          const std::string& report_column_name,
                                                          PropertyDataType data_type,
                                                          const std::string& description,
                                                          const std::string& dimension,
                                                          const std::string& unit,
                                                          const std::string& representation_str);

private:
    static std::map<Representation, std::string> representation_2_string_;
    static std::map<std::string, Representation> string_2_representation_;

    DBContent* dbcontent_{nullptr};
    std::string dbcontent_name_;

    std::string short_name_;
    /// Value representation type, based on enum STRING_REPRESENTATION
    std::string representation_str_;
    Representation representation_;

    std::string description_;
    std::string group_;

    std::string origin_;
    std::string data_item_;
    nlohmann::json inputs_;
    nlohmann::json calculation_;
    nlohmann::json selection_;
    std::string source_variable_;

    std::string db_column_name_;
    bool is_key_ {false};

    //set on a runtime Report Variable only, empty on a configured variable
    std::string report_table_name_;
    std::string report_column_name_;
    std::string report_join_alias_;

    std::string dimension_;
    std::string unit_;

//    bool min_max_set_{false};
//    /// Minimum as string
//    std::string min_;
//    /// Maximum as string
//    std::string max_;

    VariableWidget* widget_{nullptr};

    std::string getDataSourcesAsString(const std::string& value) const;

    //void setMinMax();
};

}

Q_DECLARE_METATYPE(dbContent::Variable*)
