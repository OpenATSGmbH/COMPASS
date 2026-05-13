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

#include "json.hpp"

#include <map>
#include <string>
#include <vector>

class ASTERIXImportSource;
struct ASTERIXImportFileInfo;

namespace context { class DBContextManager; }

/**
 * Pure aggregation of jASTERIX probe results from an ASTERIXImportSource.
 *
 * Walks the per-file analysis_info JSON populated by ASTERIXFileDecoder /
 * ASTERIXPCAPDecoder during testFileDecoding(), and joins with the active
 * DBContext to classify each (sac, sic) as context-only, probe-only, or both.
 */
class ASTERIXImportProbeAggregator
{
public:
    /// Per-data-item statistics aggregated across all files/sections of one (ds, cat).
    /// min/max are kept as raw JSON values because jASTERIX emits them in the
    /// item's native type (int, double, string, ...). null when the probe did
    /// not provide min/max for this item (e.g. compound items with only a count).
    struct ItemStats
    {
        std::size_t    count = 0;
        nlohmann::json min;     // null if not provided
        nlohmann::json max;     // null if not provided
    };

    /// Per-category probe data for one data source.
    struct CategoryProbe
    {
        std::size_t                       total_count = 0;   // category-level "count"
        std::map<std::string, ItemStats>  items;             // dotted item name -> stats
    };

    /// Probe data for one data source (or for the "unknown SAC/SIC" bucket).
    struct DSProbe
    {
        bool          unknown_sac_sic = false;   // true for the aggregator's "unknown" bucket
        unsigned int  sac = 0;
        unsigned int  sic = 0;
        unsigned int  ds_id = 0;                 // dsIdFrom(sac, sic); 0 for unknown bucket
        std::map<unsigned int, CategoryProbe> categories;
    };

    /// Result of one aggregation run.
    struct Result
    {
        bool probe_available = false;            // false until at least one file has been probed

        std::map<unsigned int, DSProbe> probe_by_dsid;   // ds_id -> probe data
        DSProbe                          unknown;        // unknown_sac_sic == true if populated

        std::vector<unsigned int> context_only_ds_ids;   // in context, no probe data
        std::vector<unsigned int> probe_only_ds_ids;     // in probe, not in context
        std::vector<unsigned int> both_ds_ids;           // in both
    };

    /// Aggregate probe info from `source` and join with the active context in `ctx_man`.
    /// Safe to call when no probe has run yet (returns probe_available == false).
    static Result aggregate(const ASTERIXImportSource& source,
                            const context::DBContextManager& ctx_man);

    /// Aggregate probe info for a single file (and its used sections, if any).
    /// Returns a Result containing the per-(ds_id, cat) probe data for that file
    /// only. Does not populate context_only/probe_only/both lists - those are
    /// only meaningful at source level.
    static Result aggregateFile(const ASTERIXImportFileInfo& file_info);

    /// True when `key` should be displayed as a data item.
    /// Drops jASTERIX bookkeeping ("index", "length") and FX extension bits
    /// ("FX", "*.FX") which are decoder internals, not real data items.
    static bool isDisplayableDataItem(const std::string& key);

    /// Best-effort guess of the data source type from the set of categories
    /// observed for it. Returns one of "Radar", "MLAT", "ADSB", "Tracker", or
    /// "Other". Falls back to "Other" when categories from different groups
    /// are mixed or when no category matches a known group.
    static std::string inferDsType(const std::map<unsigned int, CategoryProbe>& cats);
};
