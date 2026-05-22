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

#include "db_context_diff.h"
#include "db_context.h"
#include "data_source.h"
#include "fft.h"
#include "asterix_decoding_config.h"
#include "sector.h"

#include <json.hpp>

#include <map>
#include <sstream>

using namespace std;
using namespace nlohmann;

namespace context
{

namespace
{

/// Recursively find field-level differences between two JSON objects.
void diffJSON(const json& a, const json& b, const string& prefix, vector<FieldDiff>& out)
{
    if (a == b)
        return;

    // if both are objects, recurse into keys
    if (a.is_object() && b.is_object())
    {
        // keys in a
        for (auto it = a.begin(); it != a.end(); ++it)
        {
            string path = prefix.empty() ? it.key() : prefix + "." + it.key();
            if (b.contains(it.key()))
                diffJSON(it.value(), b.at(it.key()), path, out);
            else
                out.push_back({path, it.value(), nullptr});
        }
        // keys only in b
        for (auto it = b.begin(); it != b.end(); ++it)
        {
            if (!a.contains(it.key()))
            {
                string path = prefix.empty() ? it.key() : prefix + "." + it.key();
                out.push_back({path, nullptr, it.value()});
            }
        }
    }
    else
    {
        // leaf difference
        out.push_back({prefix.empty() ? "<root>" : prefix, a, b});
    }
}

/// Generic diff for a section: items keyed by a string key, serialized to JSON.
/// Optional display_fn provides a human-readable key for the summary.
template<typename T, typename KeyFunc, typename ToJSONFunc, typename DisplayFunc = nullptr_t>
vector<ItemDiff> diffSection(const vector<T>& items_a,
                             const vector<T>& items_b,
                             KeyFunc key_fn,
                             ToJSONFunc to_json_fn,
                             DisplayFunc display_fn = nullptr)
{
    vector<ItemDiff> diffs;

    auto display_key = [&](const T& item)
    {
        if constexpr (!std::is_same_v<DisplayFunc, nullptr_t>)
            return display_fn(item);
        else
            return key_fn(item);
    };

    // build maps
    map<string, size_t> map_a, map_b;
    for (size_t i = 0; i < items_a.size(); ++i)
        map_a[key_fn(items_a[i])] = i;
    for (size_t i = 0; i < items_b.size(); ++i)
        map_b[key_fn(items_b[i])] = i;

    // removed (in a, not in b)
    for (const auto& [key, idx] : map_a)
    {
        if (map_b.find(key) == map_b.end())
        {
            ItemDiff d;
            d.type = ItemDiff::Removed;
            d.key = key;
            d.display_key = display_key(items_a[idx]);
            d.item_a = to_json_fn(items_a[idx]);
            diffs.push_back(d);
        }
    }

    // added (in b, not in a)
    for (const auto& [key, idx] : map_b)
    {
        if (map_a.find(key) == map_a.end())
        {
            ItemDiff d;
            d.type = ItemDiff::Added;
            d.key = key;
            d.display_key = display_key(items_b[idx]);
            d.item_b = to_json_fn(items_b[idx]);
            diffs.push_back(d);
        }
    }

    // modified (in both, but different)
    for (const auto& [key, idx_a] : map_a)
    {
        auto it = map_b.find(key);
        if (it != map_b.end())
        {
            json ja = to_json_fn(items_a[idx_a]);
            json jb = to_json_fn(items_b[it->second]);

            if (ja != jb)
            {
                ItemDiff d;
                d.type = ItemDiff::Modified;
                d.key = key;
                d.display_key = display_key(items_a[idx_a]);
                d.item_a = ja;
                d.item_b = jb;
                diffJSON(ja, jb, "", d.fields);
                diffs.push_back(d);
            }
        }
    }

    return diffs;
}

} // anonymous namespace

bool DBContextDiff::hasDifferences() const
{
    return hasSensorDifferences()
        || hasFFTDifferences()
        || hasASTERIXDifferences()
        || hasSectorDifferences()
        || hasColorDifferences();
}

string DBContextDiff::summary() const
{
    // In the diff, A = Configuration, B = Database.
    // "Added" = only in Database, "Removed" = only in Configuration.

    ostringstream oss;

    auto summarize = [&](const string& section_name, const vector<ItemDiff>& diffs)
    {
        if (diffs.empty())
            return;

        oss << section_name << ":\n";

        for (const auto& d : diffs)
        {
            const string& name = d.display_key.empty() ? d.key : d.display_key;

            switch (d.type)
            {
            case ItemDiff::Added:
                oss << "  + " << name << " (only in Database)\n";
                break;
            case ItemDiff::Removed:
                oss << "  - " << name << " (only in Configuration)\n";
                break;
            case ItemDiff::Modified:
                oss << "  ~ " << name << " (modified)\n";
                break;
            }
        }
    };

    summarize("Sensors", sensor_diffs);
    summarize("FFTs", fft_diffs);
    summarize("ASTERIX Decoding", asterix_diffs);
    summarize("Sectors", sector_diffs);

    if (!color_diffs.empty())
    {
        oss << "Colors:\n";
        for (const auto& f : color_diffs)
            oss << "  ~ " << f.path << "\n";
    }

    return oss.str();
}

DBContextDiff DBContextDiff::compute(const DBContext& a, const DBContext& b)
{
    DBContextDiff result;

    // sensors - keyed by sac/sic, display includes name. dataSources() is a
    // map<ds_id, DataSource>; diffSection needs random-access containers, so
    // copy values into vectors here (one-shot, used only for diff display).
    auto map_to_vec = [](const std::map<unsigned int, DataSource>& m)
    {
        std::vector<DataSource> v;
        v.reserve(m.size());
        for (const auto& [ds_id, ds] : m)
            v.push_back(ds);
        return v;
    };
    auto a_ds_vec = map_to_vec(a.dataSources());
    auto b_ds_vec = map_to_vec(b.dataSources());

    result.sensor_diffs = diffSection(
        a_ds_vec, b_ds_vec,
        [](const DataSource& ds) { return to_string(ds.sac()) + "/" + to_string(ds.sic()); },
        [](const DataSource& ds) { return ds.toJSON(); },
        [](const DataSource& ds) { return ds.name() + " " + to_string(ds.sac()) + "/" + to_string(ds.sic()); }
    );

    // ffts - keyed by name
    result.fft_diffs = diffSection(
        a.ffts(), b.ffts(),
        [](const FFT& fft) { return fft.name(); },
        [](const FFT& fft) { return fft.toJSON(); }
    );

    // asterix decoding - keyed by category
    result.asterix_diffs = diffSection(
        a.asterixDecoding(), b.asterixDecoding(),
        [](const ASTERIXDecodingConfig& c) { return to_string(c.category()); },
        [](const ASTERIXDecodingConfig& c) { return c.toJSON(); }
    );

    // sectors - keyed by id, display includes name
    result.sector_diffs = diffSection(
        a.sectors(), b.sectors(),
        [](const shared_ptr<Sector>& s) { return to_string(s->id()); },
        [](const shared_ptr<Sector>& s) { return s->jsonData(); },
        [](const shared_ptr<Sector>& s) { return s->name(); }
    );

    // colors - preference + per-key entries in both palettes
    diffJSON(a.colors().toJSON(), b.colors().toJSON(), "colors", result.color_diffs);

    return result;
}

} // namespace context
