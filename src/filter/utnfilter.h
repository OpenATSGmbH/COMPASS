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

#include "dbfilter.h"

class UTNFilter : public DBFilter
{
public:
    UTNFilter(const std::string& class_id, const std::string& instance_id,
              Configurable* parent);
    virtual ~UTNFilter();

    virtual std::string getConditionString(const std::string& dbcontent_name, bool& first) override;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    virtual bool filters(const std::string& dbcontent_name) override;
    virtual void reset() override;

    virtual void saveViewPointConditions (nlohmann::json& filters) override;
    virtual void loadViewPointConditions (const nlohmann::json& filters) override;

    std::string utns() const;
    void utns(const std::string& utns);

protected:
    std::string utns_str_;
    std::vector<unsigned int> values_;
    bool null_wanted_ {false};  // indicates NULL in values

    virtual void checkSubConfigurables() override;
    virtual DBFilterWidget* createWidget() override;

    bool updateUTNSFromStr(const std::string& values_str); // returns success
};
