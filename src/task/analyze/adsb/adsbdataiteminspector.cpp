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

#include "adsbdataiteminspector.h"
#include "analyzedatasourcetask.h"
#include "asterixreporthelpers.h"
#include "asteriximporttask.h"

#include "compass.h"
#include "db_context_manager.h"
#include "data_source.h"
#include "logger.h"
#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"
#include "stringconv.h"
#include "taskmanager.h"

#include "json.hpp"

#include <jasterix/jasterix.h>

ADSBDataItemInspectorSettings::ADSBDataItemInspectorSettings(nlohmann::json& config_json,
                                                             Configurable* parent)
    : InspectorSettingsBase(config_json, parent)
{
    registerParameter("included_cats", &included_cats_, nlohmann::json::object());
}

bool ADSBDataItemInspectorSettings::catIncluded(unsigned int cat) const
{
    auto key = std::to_string(cat);
    if (included_cats_.contains(key))
        return included_cats_.at(key).get<bool>();
    return true;
}

void ADSBDataItemInspectorSettings::setCatIncluded(unsigned int cat, bool value)
{
    included_cats_[std::to_string(cat)] = value;
}

ADSBDataItemInspector::ADSBDataItemInspector(AnalyzeDataSourceTask& task,
                                             ADSBDataItemInspectorSettings& settings)
    : DataSourceInspectorBase(task, settings)
{
}

void ADSBDataItemInspector::writeReport(ResultReport::Section& root)
{
    auto& section = root.addSubSection(name());

    {
        auto& intro = section.addText("About");
        intro.addText(
            "Which CAT021 data items the selected sources actually send, "
            "counted cumulatively across all imports into the open database.");
        intro.addList({
            "Summary table: per source, the categories present and total record count.",
            "Per source: one table per category, each item with its record count and observed min / max.",
            "Items defined in the edition but never sent are listed with count 0 (red).",
            "Example: I021/210 MOPS Version, count 152340, min 1, max 2."});
    }

    auto& compass = task_.compass();
    auto& ctx     = compass.dbContextManager();
    const auto& info_map = ctx.asterixInfo();

    auto& settings = static_cast<ADSBDataItemInspectorSettings&>(settings_);

    auto selected = task_.selectedDataSourceIDs();

    if (selected.empty())
    {
        auto& note = section.addText("Note");
        note.addText("No data sources selected; nothing to report.");
        return;
    }

    // Borrow the active jASTERIX from the import task so the rendered tables
    // can include edition-defined-but-unseen items (count 0, red).
    std::shared_ptr<jASTERIX::jASTERIX> jasterix;
    try
    {
        jasterix = compass.taskManager().asterixImporterTask().jASTERIX();
    }
    catch (...) { /* helper handles a null pointer gracefully */ }

    auto& summary = section.addTable("Summary",
                                     3,
                                     {"Data Source", "CATs", "Total records"},
                                     false);

    for (auto ds_id : selected)
    {
        const auto* ds = ctx.dataSource(ds_id);
        std::string ds_label;
        if (ds)
            ds_label = ds->name() + " (" + std::to_string(ds->sac())
                       + "/" + std::to_string(ds->sic()) + ")";
        else
            ds_label = std::to_string(ds_id);

        auto info_it = info_map.find(ds_id);
        if (info_it == info_map.end())
        {
            summary.addRow({ds_label, "-", "no asterix info"});
            continue;
        }

        // Summary row.
        std::string cats_str;
        std::size_t total = 0;
        for (const auto& cat_kv : info_it->second)
        {
            unsigned int cat = cat_kv.first;
            if (!settings.catIncluded(cat))
                continue;
            if (!cats_str.empty()) cats_str += ", ";
            cats_str += "CAT" + Utils::String::categoryString(cat);
            total += cat_kv.second.total_count;
        }
        summary.addRow({ds_label, cats_str, std::to_string(total)});

        // Per-DS section, identical layout to the ASTERIX Import report.
        auto& ds_section = section.addSubSection(ds_label);

        std::map<unsigned int, ASTERIXReportHelpers::CategoryView> view_cats;
        for (const auto& [cat, cat_stats] : info_it->second)
        {
            if (!settings.catIncluded(cat))
                continue;
            auto& cv = view_cats[cat];
            cv.total_count = cat_stats.total_count;
            for (const auto& [name, stats] : cat_stats.items)
            {
                cv.items[name] = { stats.count, &stats.min, &stats.max };
            }
        }

        ASTERIXReportHelpers::renderDataItemTablesForDS(
            ds_section, view_cats, jasterix.get());
    }

    loginf << "ADS-B data-item analysis for " << selected.size() << " data sources written";
}
