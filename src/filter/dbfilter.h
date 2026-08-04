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
#include "appmode.h"
#include "dbfilterwidget.h"
#include "filterclause.h"

#include "json_fwd.hpp"

#include <string>
#include <vector>
#include <memory>

class IDBVariableResolver;
class DBFilterCondition;
class FilterManager;
class Buffer;

namespace dbContent {

class Variable;
class VariableSet;

}

class DBFilter : public Configurable
{
  public:
    // DBFilter(const std::string& class_name, const std::string& instance_name, Configurable* parent,
    //          bool is_generic = true);
    DBFilter(nlohmann::json& config, bool is_generic,
             FilterManager* parent, IDBVariableResolver& var_resolver);
    virtual ~DBFilter();

    FilterManager& filterManager() { traced_assert(filter_manager_); return *filter_manager_; }
    IDBVariableResolver& variableResolver() { return var_resolver_; }

    void setActive(bool active);
    bool getActive();

    // returns if the DBContent is filtered by this filter
    virtual bool filters(const std::string& dbcont_name);

    bool getVisible();
    void setVisible(bool visible);

    const std::string& getName() const { return name_; }
    void setName(const std::string& name);

    bool isCustom() { return is_custom_; }

    const std::string& conditionLogic() const { return condition_logic_; }
    void conditionLogic(const std::string& logic);

    /// where condition string for a DBContent
    virtual std::string getConditionString(const std::string& dbcontent_name,
      dbContent::VariableSet& read_set, bool& first);

    /// self-contained WHERE fragment for a DBContent (no leading AND/OR join - the combiner
    /// joins filters), with referenced vars as required_vars. Replaces getConditionString.
    virtual FilterClause getClause(const std::string& dbcontent_name);

    bool onlyHasSubFilter() { return conditions_.size() > 0; }

    // resets the filter (sub-filters and conditions) to their inital values.
    virtual void reset();

    virtual void generateSubConfigurable(nlohmann::json& child_json) override;

    const std::vector<std::unique_ptr<DBFilterCondition>>& getConditions() const { return conditions_; }
    unsigned int getNumConditions() { return conditions_.size(); }
    void clearConditions();
    void deleteCondition(DBFilterCondition* condition);

    DBFilterWidget* widget();

    bool unusable() const { return unusable_; }

    virtual void saveViewPointConditions (nlohmann::json& filters);
    virtual void loadViewPointConditions (const nlohmann::json& filters);

    void updateToAppMode (AppMode app_mode);

    virtual bool activeInLiveMode();
    virtual std::vector<unsigned int> filterBuffer(const std::string& dbcontent_name, std::shared_ptr<Buffer> buffer);

    bool widgetVisible() const;
    void widgetVisible(bool widget_expanded);

protected:
    FilterManager* filter_manager_{nullptr};
    IDBVariableResolver& var_resolver_;
    std::string name_;
    bool is_custom_; // indicates if created by user and can be deleted

    bool active_ {true}; // active flag, if false no conditions are used

    bool visible_ {true}; // widget expanded flag
    bool unusable_{false};  // if a conditions is unusable
    bool disabled_ {false}; // if disabled due to other reasons (e.g. AppMode)

    bool widget_visible_ {true};

    std::string condition_logic_{"AND"}; // "AND" or "OR" - how conditions are joined

    std::vector<std::unique_ptr<DBFilterCondition>> conditions_;

    // widget with configuration elements.
    std::unique_ptr<DBFilterWidget> widget_{nullptr};

    virtual DBFilterWidget* createWidget();
};
