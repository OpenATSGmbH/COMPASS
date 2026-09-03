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

#include "smrdataiteminspector.h"
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

#include <QVariant>

#include <algorithm>
#include <set>
#include <vector>

SMRDataItemInspectorSettings::SMRDataItemInspectorSettings(nlohmann::json& config_json,
                                                           Configurable* parent)
    : InspectorSettingsBase(config_json, parent)
{
    registerParameter("included_cats", &included_cats_, nlohmann::json::object());
}

bool SMRDataItemInspectorSettings::catIncluded(unsigned int cat) const
{
    auto key = std::to_string(cat);
    if (included_cats_.contains(key))
        return included_cats_.at(key).get<bool>();
    return true;
}

void SMRDataItemInspectorSettings::setCatIncluded(unsigned int cat, bool value)
{
    included_cats_[std::to_string(cat)] = value;
}

SMRDataItemInspector::SMRDataItemInspector(AnalyzeDataSourceTask& task,
                                           SMRDataItemInspectorSettings& settings)
    : DataSourceInspectorBase(task, settings)
{
}

namespace
{
constexpr unsigned int kCAT010 = 10;

// One target report field ED-116 section 3.4.10 requires. `item_numbers` are
// the CAT010 item numbers that satisfy the requirement (any of them counts).
// `per_target_report` items are counted against the Target Report Descriptor
// count, which every target report carries; the others against all records.
// Every item gets its own row. `alternative_group` links the rows the standard
// accepts as alternatives to each other: one member with data satisfies the
// requirement, so an empty sibling is reported as not sent, not as missing.
constexpr int kNoAlternative       = -1;
constexpr int kPositionAlternative = 1;

struct MandatoryItem
{
    const char*              label;
    std::vector<std::string> item_numbers;
    bool                     per_target_report;
    bool                     status_message_only;
    int                      alternative_group;
};

const std::vector<MandatoryItem>& mandatoryItems()
{
    static const std::vector<MandatoryItem> items = {
        { "I010/000 Message Type",                        {"000"}, false, false, kNoAlternative },
        { "I010/010 Data Source Identifier",              {"010"}, false, false, kNoAlternative },
        { "I010/020 Target Report Descriptor",            {"020"}, true,  false, kNoAlternative },
        { "I010/140 Time of Day",                         {"140"}, false, false, kNoAlternative },
        { "I010/040 Measured Position in Polar Coordinates",
                                                          {"040"}, true,  false, kPositionAlternative },
        { "I010/041 Position in WGS-84 Coordinates",      {"041"}, true,  false, kPositionAlternative },
        { "I010/042 Position in Cartesian Coordinates",   {"042"}, true,  false, kPositionAlternative },
        { "I010/270 Target Size and Orientation",         {"270"}, true,  false, kNoAlternative },
        { "I010/550 System Status",                       {"550"}, false, true,  kNoAlternative },
    };
    return items;
}

// True if the flattened item key (e.g. "010.SAC", "140") belongs to the item number.
bool keyInItem(const std::string& key, const std::string& item_number)
{
    if (key == item_number)
        return true;
    return key.size() > item_number.size()
           && key.compare(0, item_number.size(), item_number) == 0
           && key[item_number.size()] == '.';
}

// Highest count over all keys of the given item numbers (sub-fields of one
// item share the record count, so the maximum is the item's record count).
std::size_t itemCount(const std::map<std::string, context::DBContextManager::AsterixItemStats>& items,
                      const std::vector<std::string>& item_numbers)
{
    std::size_t count = 0;
    for (const auto& kv : items)
        for (const auto& num : item_numbers)
            if (keyInItem(kv.first, num))
                count = std::max(count, kv.second.count);
    return count;
}
}  // namespace

void SMRDataItemInspector::writeReport(ResultReport::Section& root)
{
    auto& section = root.addSubSection(name());

    {
        auto& intro = section.addText("About");
        intro.addText(
            "Which CAT010 data items the selected SMR sources actually send, "
            "counted cumulatively across all imports into the open database, and "
            "whether the target report fields EUROCAE ED-116 section 3.4.10 requires "
            "are populated.");
        intro.addList({
            "Summary table: per source, the categories present and total record count.",
            "Mandatory Items table: per source, each required field with its record "
            "count, its share and a status. Target report fields are counted against "
            "the Target Report Descriptor count, which every target report carries. "
            "Message Type and Data Source Identifier are counted against all records. "
            "System Status is sent in status messages and is shown as a count only.",
            "Per source: one table per category, each item with its record count and "
            "observed min / max. Items defined in the edition but never sent are listed "
            "with count 0 (red)."});
    }

    auto& compass = task_.compass();
    auto& ctx     = compass.dbContextManager();
    const auto& info_map = ctx.asterixInfo();

    auto& settings = static_cast<SMRDataItemInspectorSettings&>(settings_);

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

    auto dsLabel = [&](unsigned int ds_id) {
        const auto* ds = ctx.dataSource(ds_id);
        if (ds)
            return ds->name() + " (" + std::to_string(ds->sac())
                   + "/" + std::to_string(ds->sic()) + ")";
        return std::to_string(ds_id);
    };

    auto& summary = section.addTable("Summary",
                                     3,
                                     {"Data Source", "CATs", "Total records"},
                                     false);

    auto& mandatory = section.addTable("Mandatory Items",
                                       5,
                                       {"Data Source", "Item", "Count", "Share", "Status"},
                                       false);

    for (auto ds_id : selected)
    {
        const std::string ds_label = dsLabel(ds_id);

        auto info_it = info_map.find(ds_id);
        if (info_it == info_map.end())
        {
            summary.addRow({ds_label, "-", "no asterix info"});
            mandatory.addRow({ds_label, "-", "-", "-", "no asterix info"});
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

        // Mandatory items (ED-116 3.4.10), CAT010 only.
        auto cat010_it = info_it->second.find(kCAT010);
        if (cat010_it == info_it->second.end() || !settings.catIncluded(kCAT010))
        {
            mandatory.addRow({ds_label, "-", "-", "-", "no CAT010 data"});
        }
        else
        {
            const auto& cat_stats = cat010_it->second;
            const std::size_t total_records = cat_stats.total_count;
            const std::size_t descriptor_count = itemCount(cat_stats.items, {"020"});
            const std::size_t target_report_records =
                descriptor_count > 0 ? descriptor_count : total_records;

            // Alternative groups that carry data: an empty member of such a
            // group is not a missing mandatory field.
            std::set<int> covered_groups;
            for (const auto& item : mandatoryItems())
            {
                if (item.alternative_group != kNoAlternative
                    && itemCount(cat_stats.items, item.item_numbers) > 0)
                    covered_groups.insert(item.alternative_group);
            }

            for (const auto& item : mandatoryItems())
            {
                const std::size_t count = itemCount(cat_stats.items, item.item_numbers);
                const bool covered_by_alternative =
                    count == 0 && covered_groups.count(item.alternative_group) > 0;

                std::string share_s = "-";
                std::string status;
                unsigned int row_style = 0;

                if (item.status_message_only)
                {
                    status = count > 0 ? "present" : "missing";
                }
                else
                {
                    const std::size_t denom = item.per_target_report ? target_report_records
                                                                     : total_records;
                    double share = denom > 0 ? 100.0 * static_cast<double>(count)
                                                   / static_cast<double>(denom)
                                             : 0.0;
                    share_s = Utils::String::doubleToStringPrecision(share, 1) + " %";
                    if (covered_by_alternative)
                        status = "not sent";
                    else if (count == 0)
                        status = "missing";
                    else if (share >= 99.0)
                        status = "present";
                    else
                        status = "partial";
                }

                if (count == 0 && !covered_by_alternative)
                    row_style = ResultReport::CellStyleTextColorRed;

                mandatory.addRow({ds_label, item.label, std::to_string(count), share_s, status},
                                 ResultReport::SectionContentViewable(),
                                 std::string(),
                                 std::string(),
                                 QVariant(),
                                 row_style);
            }
        }

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

    loginf << "SMR data-item analysis for " << selected.size() << " data sources written";
}
