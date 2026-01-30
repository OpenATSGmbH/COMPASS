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

#include <tuple>
#include "dbfilter.h"

class MLATRUFilter : public DBFilter
{
public:
    MLATRUFilter(const std::string& class_id, const std::string& instance_id,
                 Configurable* parent);
    virtual ~MLATRUFilter();

    virtual std::string getConditionString(const std::string& dbcontent_name, 
      dbContent::VariableSet& read_set, bool& first) override;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    virtual bool filters(const std::string& dbcontent_name) override;
    virtual void reset() override;

    virtual void saveViewPointConditions (nlohmann::json& filters) override;
    virtual void loadViewPointConditions (const nlohmann::json& filters) override;

    std::string rus() const;
    void rus(const std::string& rus_str);

    bool checkRUs(const std::string& rus_str);

    bool matchAll() const;
    void matchAll(bool match_all);

protected:
    std::string db_column_name_;

    std::string rus_str_;
    bool match_all_;

    virtual void checkSubConfigurables() override;
    virtual DBFilterWidget* createWidget() override;
};
