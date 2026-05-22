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

#include <json.hpp>

#include <string>
#include <vector>

namespace context
{

class DBContext;

/**
 * Represents a single field-level difference between two items.
 */
struct FieldDiff
{
    std::string path;         // e.g. "name", "info.latitude"
    nlohmann::json value_a;   // value in context A
    nlohmann::json value_b;   // value in context B
};

/**
 * Represents a difference for a single item (sensor, FFT, etc.)
 */
struct ItemDiff
{
    enum Type { Added, Removed, Modified };

    Type type;
    std::string key;                   // identifying key (e.g. "12/1" for sac/sic, "FFT_01" for name)
    std::string display_key;           // human-readable key (e.g. "Radar_A 12/1"); empty = use key
    std::vector<FieldDiff> fields;     // only for Modified
    nlohmann::json item_a;             // full item from context A (null for Added)
    nlohmann::json item_b;             // full item from context B (null for Removed)
};

/**
 * Result of comparing two DBContext instances.
 * Each section has its own list of item-level diffs.
 */
struct DBContextDiff
{
    std::vector<ItemDiff> sensor_diffs;
    std::vector<ItemDiff> fft_diffs;
    std::vector<ItemDiff> asterix_diffs;
    std::vector<ItemDiff> sector_diffs;
    std::vector<FieldDiff> color_diffs;

    bool hasDifferences() const;
    bool hasSensorDifferences() const { return !sensor_diffs.empty(); }
    bool hasFFTDifferences() const { return !fft_diffs.empty(); }
    bool hasASTERIXDifferences() const { return !asterix_diffs.empty(); }
    bool hasSectorDifferences() const { return !sector_diffs.empty(); }
    bool hasColorDifferences() const { return !color_diffs.empty(); }

    std::string summary() const;

    /// Compute the diff between two contexts.
    static DBContextDiff compute(const DBContext& a, const DBContext& b);
};

} // namespace context
