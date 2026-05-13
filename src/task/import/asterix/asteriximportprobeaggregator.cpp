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

#include "asteriximportprobeaggregator.h"

#include "asteriximportsource.h"
#include "data_source.h"
#include "db_context.h"
#include "db_context_manager.h"
#include "number.h"

#include <set>

using nlohmann::json;

namespace
{

/// Compare two JSON values for "less than" in a min/max sense. Only meaningful
/// for matching numeric/string types - otherwise the existing value wins.
bool jsonLess(const json& a, const json& b)
{
    if (a.is_number() && b.is_number())
        return a.get<double>() < b.get<double>();
    if (a.is_string() && b.is_string())
        return a.get<std::string>() < b.get<std::string>();
    return false;
}

void mergeMin(json& dst, const json& src)
{
    if (src.is_null())
        return;
    if (dst.is_null() || jsonLess(src, dst))
        dst = src;
}

void mergeMax(json& dst, const json& src)
{
    if (src.is_null())
        return;
    if (dst.is_null() || jsonLess(dst, src))
        dst = src;
}

void mergeCategoryFromJSON(ASTERIXImportProbeAggregator::CategoryProbe& target,
                           const json& cat_obj)
{
    if (!cat_obj.is_object())
        return;

    for (auto it = cat_obj.begin(); it != cat_obj.end(); ++it)
    {
        const std::string& key = it.key();
        const json&        val = it.value();

        // category-level "count" is a number, the per-item entries are objects
        if (key == "count")
        {
            if (val.is_number_unsigned())
                target.total_count += val.get<std::size_t>();
            else if (val.is_number_integer())
                target.total_count += static_cast<std::size_t>(val.get<long long>());
            continue;
        }

        if (!val.is_object())
            continue;

        if (!ASTERIXImportProbeAggregator::isDisplayableDataItem(key))
            continue;

        auto& stats = target.items[key];

        if (val.contains("count"))
        {
            const auto& c = val.at("count");
            if (c.is_number_unsigned())
                stats.count += c.get<std::size_t>();
            else if (c.is_number_integer())
                stats.count += static_cast<std::size_t>(c.get<long long>());
        }
        if (val.contains("min"))
            mergeMin(stats.min, val.at("min"));
        if (val.contains("max"))
            mergeMax(stats.max, val.at("max"));
    }
}

/// Parse "<sac>/<sic>" key. Returns {true, sac, sic} on success.
bool parseSacSicKey(const std::string& key, unsigned int& sac, unsigned int& sic)
{
    auto slash = key.find('/');
    if (slash == std::string::npos)
        return false;
    try
    {
        sac = static_cast<unsigned int>(std::stoul(key.substr(0, slash)));
        sic = static_cast<unsigned int>(std::stoul(key.substr(slash + 1)));
        return true;
    }
    catch (...)
    {
        return false;
    }
}

/// Merge one analysis_info JSON object (as produced by jASTERIX::analyzeFile/Data)
/// into the running aggregation result.
void mergeAnalysisJSON(const json& analysis,
                       ASTERIXImportProbeAggregator::Result& result)
{
    if (!analysis.is_object())
        return;

    for (auto it = analysis.begin(); it != analysis.end(); ++it)
    {
        const std::string& key = it.key();

        // skip top-level scalar metadata
        if (key == "num_errors" || key == "num_records" || key == "num_frames")
            continue;

        const json& sensor_obj = it.value();
        if (!sensor_obj.is_object())
            continue;

        ASTERIXImportProbeAggregator::DSProbe* target = nullptr;

        if (key == "unknown")
        {
            result.unknown.unknown_sac_sic = true;
            target = &result.unknown;
        }
        else
        {
            unsigned int sac = 0;
            unsigned int sic = 0;
            if (!parseSacSicKey(key, sac, sic))
                continue;

            unsigned int ds_id = Utils::Number::dsIdFrom(sac, sic);

            auto& probe = result.probe_by_dsid[ds_id];
            probe.unknown_sac_sic = false;
            probe.sac   = sac;
            probe.sic   = sic;
            probe.ds_id = ds_id;
            target      = &probe;
        }

        // categories: keys are decimal category numbers ("48", "62", ...)
        for (auto cat_it = sensor_obj.begin(); cat_it != sensor_obj.end(); ++cat_it)
        {
            const std::string& cat_key = cat_it.key();
            unsigned int cat = 0;
            try
            {
                cat = static_cast<unsigned int>(std::stoul(cat_key));
            }
            catch (...)
            {
                continue;
            }

            mergeCategoryFromJSON(target->categories[cat], cat_it.value());
        }
    }
}

} // anonymous namespace

bool ASTERIXImportProbeAggregator::isDisplayableDataItem(const std::string& key)
{
    // jASTERIX bookkeeping fields, not real data items
    if (key == "index" || key == "length")
        return false;

    // FX extension indicators (e.g. "FX", "020.FX", "...FX")
    if (key == "FX")
        return false;
    if (key.size() >= 3 && key.compare(key.size() - 3, 3, ".FX") == 0)
        return false;

    return true;
}

std::string ASTERIXImportProbeAggregator::inferDsType(
    const std::map<unsigned int, CategoryProbe>& cats)
{
    auto category_to_type = [](unsigned int cat) -> std::string {
        switch (cat)
        {
            case 1:  case 2:  case 34: case 48: return "Radar";
            case 19: case 20:                   return "MLAT";
            case 21: case 23:                   return "ADSB";
            case 62: case 63: case 65:          return "Tracker";
            default:                            return "";
        }
    };

    std::set<std::string> types;
    for (const auto& kv : cats)
    {
        std::string t = category_to_type(kv.first);
        if (!t.empty())
            types.insert(t);
    }

    if (types.size() == 1)
        return *types.begin();
    return "Other";
}

ASTERIXImportProbeAggregator::Result
ASTERIXImportProbeAggregator::aggregateFile(const ASTERIXImportFileInfo& file_info)
{
    Result result;

    if (!file_info.used)
        return result;

    if (file_info.hasSections())
    {
        for (const auto& section : file_info.sections)
        {
            if (!section.used)
                continue;
            if (!section.error.analysis_info.is_object())
                continue;
            mergeAnalysisJSON(section.error.analysis_info, result);
            result.probe_available = true;
        }
    }
    else if (file_info.error.analysis_info.is_object())
    {
        mergeAnalysisJSON(file_info.error.analysis_info, result);
        result.probe_available = true;
    }

    return result;
}

ASTERIXImportProbeAggregator::Result
ASTERIXImportProbeAggregator::aggregate(const ASTERIXImportSource& source,
                                        const context::DBContextManager& ctx_man)
{
    Result result;

    // walk all used files / sections, accumulate analysis_info
    for (const auto& file_info : source.files())
    {
        if (!file_info.used)
            continue;

        if (file_info.hasSections())
        {
            for (const auto& section : file_info.sections)
            {
                if (!section.used)
                    continue;
                if (!section.error.analysis_info.is_object())
                    continue;
                mergeAnalysisJSON(section.error.analysis_info, result);
                result.probe_available = true;
            }
        }
        else if (file_info.error.analysis_info.is_object())
        {
            mergeAnalysisJSON(file_info.error.analysis_info, result);
            result.probe_available = true;
        }
    }

    // join with active context
    std::set<unsigned int> context_ds_ids;
    if (ctx_man.hasActiveContext())
    {
        for (const auto& [ds_id, ds] : ctx_man.activeContext().dataSources())
            context_ds_ids.insert(ds.id());
    }

    std::set<unsigned int> probe_ds_ids;
    for (const auto& kv : result.probe_by_dsid)
        probe_ds_ids.insert(kv.first);

    for (auto id : context_ds_ids)
    {
        if (probe_ds_ids.count(id))
            result.both_ds_ids.push_back(id);
        else
            result.context_only_ds_ids.push_back(id);
    }
    for (auto id : probe_ds_ids)
    {
        if (!context_ds_ids.count(id))
            result.probe_only_ds_ids.push_back(id);
    }

    return result;
}
