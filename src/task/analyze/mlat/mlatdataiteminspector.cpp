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

#include "mlatdataiteminspector.h"
#include "analysedatasourcetask.h"

#include "compass.h"
#include "db_context_manager.h"
#include "data_source.h"
#include "logger.h"
#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"

#include "json.hpp"

#include <set>
#include <sstream>

MLATDataItemInspectorSettings::MLATDataItemInspectorSettings(nlohmann::json& config_json,
                                                             Configurable* parent)
    : InspectorSettingsBase(config_json, parent)
{
    registerParameter("include_cat019", &include_cat019_, true);
}

MLATDataItemInspector::MLATDataItemInspector(AnalyseDataSourceTask& task,
                                             MLATDataItemInspectorSettings& settings)
    : DataSourceInspectorBase(task, settings)
{
}

namespace
{
std::string jsonScalarToString(const nlohmann::json& j)
{
    if (j.is_null())            return "";
    if (j.is_string())          return j.get<std::string>();
    if (j.is_boolean())         return j.get<bool>() ? "true" : "false";
    if (j.is_number_integer())  return std::to_string(j.get<long long>());
    if (j.is_number_unsigned()) return std::to_string(j.get<unsigned long long>());
    if (j.is_number_float())
    {
        std::ostringstream os;
        os << j.get<double>();
        return os.str();
    }
    return j.dump();
}
}

void MLATDataItemInspector::writeReport(ResultReport::Section& root)
{
    auto& section = root.addSubSection(name());

    auto& compass = task_.compass();
    auto& ctx     = compass.dbContextManager();
    const auto& info_map = ctx.asterixInfo();

    auto& settings = static_cast<MLATDataItemInspectorSettings&>(settings_);

    std::set<unsigned int> mlat_cats   = {20, 10};
    std::set<unsigned int> in_scope_cats = mlat_cats;
    if (settings.include_cat019_)
        in_scope_cats.insert(19);

    auto selected = task_.selectedDataSourceIDs();

    if (selected.empty())
    {
        auto& note = section.addText("Note");
        note.addText("No data sources selected; nothing to report.");
        return;
    }

    auto& summary = section.addTable("Summary",
                                     3,
                                     {"Data Source", "CATs", "Total records"},
                                     false);

    for (auto ds_id : selected)
    {
        const auto* ds = ctx.dataSource(ds_id);
        std::string ds_label = ds ? (ds->name() + " (" + std::to_string(ds_id) + ")")
                                  : std::to_string(ds_id);

        auto info_it = info_map.find(ds_id);
        if (info_it == info_map.end())
        {
            summary.addRow({ds_label, "-", "no asterix info"});
            continue;
        }

        std::string cats_str;
        std::size_t total = 0;
        for (const auto& cat_kv : info_it->second)
        {
            unsigned int cat = cat_kv.first;
            if (!in_scope_cats.count(cat))
                continue;
            if (!cats_str.empty()) cats_str += ", ";
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%03u", cat);
            cats_str += buf;
            total += cat_kv.second.total_count;
        }
        summary.addRow({ds_label, cats_str, std::to_string(total)});

        auto& ds_section = section.addSubSection(ds_label);

        for (const auto& cat_kv : info_it->second)
        {
            unsigned int cat = cat_kv.first;
            if (!in_scope_cats.count(cat))
                continue;

            char cat_buf[16];
            std::snprintf(cat_buf, sizeof(cat_buf), "CAT%03u", cat);
            std::string cat_heading = cat_buf;

            auto& cat_section = ds_section.addSubSection(cat_heading);

            auto& info_t = cat_section.addTable(
                cat_heading + " Records",
                2,
                {"Property", "Value"},
                false);
            info_t.addRow({"Total records", std::to_string(cat_kv.second.total_count)});

            auto& items_t = cat_section.addTable(
                cat_heading + " Items",
                4,
                {"Item", "Count", "Min", "Max"},
                true,
                0,
                Qt::AscendingOrder);

            for (const auto& it_kv : cat_kv.second.items)
            {
                items_t.addRow({
                    it_kv.first,
                    std::to_string(it_kv.second.count),
                    jsonScalarToString(it_kv.second.min),
                    jsonScalarToString(it_kv.second.max)
                });
            }
        }
    }

    loginf << "data-item analysis for " << selected.size() << " data sources written";
}
