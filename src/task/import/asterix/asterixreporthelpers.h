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

#include "json_fwd.hpp"

#include <cstddef>
#include <map>
#include <string>

namespace ResultReport { class Section; }
namespace jASTERIX     { class jASTERIX; }

namespace ASTERIXReportHelpers
{

/// Non-owning view of a single (item) row's stats for the renderer.
struct ItemStatsView
{
    std::size_t           count = 0;
    const nlohmann::json* min   = nullptr;   // null pointer = not provided
    const nlohmann::json* max   = nullptr;
};

/// Non-owning view of a single category's data for the renderer.
struct CategoryView
{
    std::size_t total_count = 0;
    std::map<std::string, ItemStatsView> items;
};

/// Renders a "Note" text plus one CAT<NNN> table per category into the given
/// DS section. Tables have columns Item, Count, Min, Max, Description.
/// Items defined in the active edition (via `jasterix`) but not seen in the
/// data are listed with count 0 and are highlighted red. Count is formatted
/// "<n>" or "<n> (<pct.1f> %)" relative to the per-(DS, CAT) total.
///
/// `jasterix` may be null - in that case only seen items are listed (no
/// edition-driven count-0 rows, no descriptions).
void renderDataItemTablesForDS(ResultReport::Section& ds_section,
                               const std::map<unsigned int, CategoryView>& categories,
                               jASTERIX::jASTERIX* jasterix);

/// "<count>" or "<count> (<pct.1f> %)". Exposed for callers that need the same
/// formatting outside the table renderer.
std::string formatCountWithPercent(std::size_t count, std::size_t total);

}  // namespace ASTERIXReportHelpers
