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

#include <string>
#include <set>

#include "property.h"

#include <boost/optional.hpp>

namespace dbContent 
{
    class Variable;
    class MetaVariable;
    class VariableSet;
    class VariableSelectionWidget;
}

class VariableView;

/**
*/
class ViewVariable
{
public:
    struct Settings
    {
        std::string data_var_dbcont;
        std::string data_var_name;

        std::string display_name;
        std::string var_name;

        std::set<PropertyDataType> valid_data_types;
        bool show_meta_vars  = true;
        bool show_empty_vars = false;
        bool allow_empty_var = false;
    };

    ViewVariable(const std::string& id_str, int idx, VariableView* view = nullptr);
    virtual ~ViewVariable();

    const std::string& id() const;
    std::string description() const;

    bool hasVariable() const;
    bool isMetaVariable() const;
    bool isEmpty() const;

    boost::optional<PropertyDataType> dataType() const;

    void set(const std::string& dbcont, const std::string& name, bool notify_changes);

    dbContent::Variable& variable();
    const dbContent::Variable& variable() const;
    dbContent::Variable* variablePtr();
    void setVariable(dbContent::Variable& var, bool notify_changes);

    dbContent::MetaVariable& metaVariable();
    const dbContent::MetaVariable& metaVariable() const;
    dbContent::MetaVariable* metaVariablePtr();
    void setMetaVariable(dbContent::MetaVariable& var, bool notify_changes);

    void setEmpty(bool notify_changes);

    dbContent::Variable* getFor(const std::string& dbcontent_name);
    const dbContent::Variable* getFor(const std::string& dbcontent_name) const;

    const std::string& variableDBContent() const;
    const std::string& variableName() const;

    void addToSet(dbContent::VariableSet& set, const std::string& dbcontent_name);

    void setVariable(const dbContent::VariableSelectionWidget& selection, bool notify_changes);
    void configureWidget(dbContent::VariableSelectionWidget& selection);
    void updateWidget(dbContent::VariableSelectionWidget& selection);

    std::string regParamDBCont() const;
    std::string regParamName() const;

    const Settings& settings() const { return settings_; }
    Settings& settings() { return settings_; }

    static const std::string ParamDataVar;
    static const std::string ParamDataVarDBCont;
    static const std::string ParamDataVarName;

private:
    Settings settings_;

    std::string   id_;
    int           idx_;
    VariableView* view_ = nullptr;
};
