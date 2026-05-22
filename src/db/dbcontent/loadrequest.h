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

#include "dbcontent/variable/variableset.h"

#include <functional>
#include <set>
#include <string>

struct LoadRequest
{
    // {"*"} = all eligible (loadable && loadingWanted); empty = nothing
    std::set<std::string> dbcontents_;

    bool apply_datasrc_filters_ = true;   // DBContext ds/line filters
    bool apply_view_filters_    = true;   // FilterManager

    // optional, per-content WHERE fragment (AND-ed after datasrc/view filters)
    std::function<std::string(const std::string&)> custom_filter_clause_;
    // optional read-set override per content; default = mgr.getReadSet(name)
    std::function<dbContent::VariableSet(const std::string&)> read_set_;

    bool show_status_           = true;
    bool cancellable_           = true;
    bool measure_db_performance_ = false;

    static LoadRequest standard()
    {
        LoadRequest r;
        r.dbcontents_ = {"*"};
        return r;
    }

    static LoadRequest withFilter(std::string clause)
    {
        LoadRequest r;
        r.dbcontents_ = {"*"};
        std::string cl = std::move(clause);
        r.custom_filter_clause_ = [cl](const std::string&) { return cl; };
        return r;
    }

    static LoadRequest forContent(std::string name,
                                  dbContent::VariableSet rs,
                                  std::string clause = "")
    {
        LoadRequest r;
        r.dbcontents_ = {name};
        r.apply_datasrc_filters_ = false;
        r.apply_view_filters_    = false;
        if (!clause.empty())
        {
            std::string cl = std::move(clause);
            r.custom_filter_clause_ = [cl](const std::string&) { return cl; };
        }
        dbContent::VariableSet rs_copy = std::move(rs);
        r.read_set_ = [rs_copy](const std::string&) { return rs_copy; };
        return r;
    }
};
