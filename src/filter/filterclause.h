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

#include <string>

/**
 * SQL comparison operators used when building filter conditions - the single source for
 * these tokens (mirrors the operator combo box), so consumers don't sprinkle string literals.
 */
namespace filter_op
{
    inline const std::string equal         = "=";
    inline const std::string not_equal     = "!=";
    inline const std::string greater       = ">";
    inline const std::string greater_equal = ">=";
    inline const std::string less          = "<";
    inline const std::string less_equal    = "<=";
    inline const std::string between       = "BETWEEN";
    inline const std::string in            = "IN";
    inline const std::string not_in        = "NOT IN";
    inline const std::string like          = "LIKE";
    inline const std::string not_like      = "NOT LIKE";
    inline const std::string is            = "IS";
    inline const std::string is_not        = "IS NOT";

    // logical / null tokens used when rendering and combining conditions
    inline const std::string logic_and     = "AND";
    inline const std::string logic_or      = "OR";
    inline const std::string is_null       = "IS NULL";
    inline const std::string null_value    = "NULL";   // user-typed value sentinel for SQL NULL
}

/**
 * A rendered filter constraint: the SQL WHERE fragment plus the variables it references.
 * required_vars is an explicit output (unioned into the load's read set by the caller),
 * replacing the read-set mutation side effect of the old getConditionString path.
 */
struct FilterClause
{
    std::string            sql;           // WHERE fragment ("" = no constraint)
    dbContent::VariableSet required_vars; // variables the clause references

    bool empty() const { return sql.empty(); }
};

/**
 * Joins the non-empty clauses with the operator, unioning their required_vars. `wrap`
 * parenthesises the result when more than one part was joined (needed for OR groups so a
 * surrounding AND binds correctly). Empty parts are skipped.
 */
inline FilterClause combineClauses(const std::vector<FilterClause>& parts,
                                   const std::string& op, bool wrap)
{
    FilterClause out;

    std::string joined;
    unsigned int cnt = 0;
    for (const auto& p : parts)
    {
        if (p.sql.empty())
            continue;
        if (cnt++)
            joined += " " + op + " ";
        joined += p.sql;
        out.required_vars.add(p.required_vars);
    }

    if (cnt)
        out.sql = (wrap && cnt > 1) ? ("(" + joined + ")") : joined;

    return out;
}

// @TODO: unwrapped - a part containing a top-level OR would mis-bind. All current producers
// pass leaf clauses or combineOr results; the one raw-SQL clause (RT get_data) is never combined.
inline FilterClause combineAnd(const std::vector<FilterClause>& parts) { return combineClauses(parts, filter_op::logic_and, false); }
inline FilterClause combineOr (const std::vector<FilterClause>& parts) { return combineClauses(parts, filter_op::logic_or,  true);  }

// strips the leading join space the legacy getConditionString path prepends (used where a
// filter's getClause delegates to getConditionString instead of a clean re-render)
inline std::string trimLeadingSpace(const std::string& s)
{
    size_t i = s.find_first_not_of(' ');
    return i == std::string::npos ? std::string() : s.substr(i);
}
