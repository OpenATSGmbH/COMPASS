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

#include "asterixreporthelpers.h"
#include "asteriximportprobeaggregator.h"

#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"
#include "reportdefs.h"
#include "stringconv.h"

#include "json.hpp"

#include <jasterix/category.h>
#include <jasterix/jasterix.h>

#include <set>

namespace ASTERIXReportHelpers
{

std::string formatCountWithPercent(std::size_t count, std::size_t total)
{
    if (total == 0)
        return std::to_string(count);

    const double pct = 100.0 * static_cast<double>(count) / static_cast<double>(total);
    return std::to_string(count) + " (" + Utils::String::doubleToStringPrecision(pct, 1) + " %)";
}

void renderDataItemTablesForDS(ResultReport::Section& ds_section,
                               const std::map<unsigned int, CategoryView>& categories,
                               jASTERIX::jASTERIX* jasterix)
{
    if (!ds_section.hasText("Note"))
    {
        auto& note = ds_section.addText("Note");
        note.addText("The information below comes from probing - only the beginning of "
                     "the ASTERIX file is read. Counts, min and max values are "
                     "sample-side estimates and do not reflect the full file. Items "
                     "defined in the active edition but not seen during probing are "
                     "listed with count 0 (highlighted red).");
    }

    for (const auto& [cat, cat_view] : categories)
    {
        const std::string table_name = "CAT" + Utils::String::categoryString(cat);

        if (!ds_section.hasTable(table_name))
            ds_section.addTable(table_name, 5,
                                {"Item", "Count", "Min", "Max", "Description"},
                                true);
        auto& it_t = ds_section.getTable(table_name);

        // Union of seen items + edition-defined items.
        jASTERIX::CategoryItemInfo edition_items;
        if (jasterix && jasterix->hasCategory(cat))
        {
            auto cat_def = jasterix->category(cat);
            if (cat_def)
                edition_items = cat_def->itemInfo();
        }

        std::set<std::string> all_keys;
        for (const auto& kv : cat_view.items)
            all_keys.insert(kv.first);
        for (const auto& kv : edition_items)
            if (ASTERIXImportProbeAggregator::isDisplayableDataItem(kv.first))
                all_keys.insert(kv.first);

        for (const auto& key : all_keys)
        {
            std::string description;
            auto info_it = edition_items.find(key);
            if (info_it != edition_items.end())
                description = info_it->second.description_;

            std::size_t count = 0;
            std::string min_s;
            std::string max_s;
            auto stats_it = cat_view.items.find(key);
            if (stats_it != cat_view.items.end())
            {
                count = stats_it->second.count;
                if (stats_it->second.min && !stats_it->second.min->is_null())
                    min_s = stats_it->second.min->dump();
                if (stats_it->second.max && !stats_it->second.max->is_null())
                    max_s = stats_it->second.max->dump();
            }

            const unsigned int row_style =
                count == 0 ? ResultReport::CellStyleTextColorRed : 0u;

            it_t.addRow({key,
                         formatCountWithPercent(count, cat_view.total_count),
                         min_s,
                         max_s,
                         description},
                        ResultReport::SectionContentViewable(),
                        std::string(),
                        std::string(),
                        QVariant(),
                        row_style);
        }
    }
}

}  // namespace ASTERIXReportHelpers
