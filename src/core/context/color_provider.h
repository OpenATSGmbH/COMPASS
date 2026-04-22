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

#include <QColor>

#include <array>
#include <map>
#include <string>
#include <vector>

namespace context
{

/**
 * Color generation / resolution helper for the DBContext color system.
 *
 * Default palettes:
 *  - DSType:    Radar=green, ADSB=blue, MLAT=red, Tracker=white, RefTraj=orange, Other=purple
 *  - DBContent: mirrors the Geographic View "Color By DBContent" palette
 *
 * Base color generation:
 *  - HSV sampling, S~0.6, V in [0.7,0.85] (light) or [0.25,0.45] (dark)
 *  - max-hue-distance selection: generate N candidates, pick one whose minimum
 *    hue distance to existing base colors is largest
 *
 * Line color generation:
 *  - four shades derived from the base with lightness offsets +/-{8,16,24,32}%.
 *  - dark base -> lighter shades, light base -> darker shades.
 */
class ColorProvider
{
public:
    enum class Mode
    {
        DSType,
        DBContent,
        DataSource,
        DataSourceLine
    };

    enum class Band
    {
        Light,
        Dark
    };

    /// default DSType palette (Radar=green, ADSB=blue, MLAT=red, ...)
    static const std::map<std::string, QColor>& defaultDSTypeColors();

    /// default DBContent palette (matches Geographic View "Color By DBContent")
    static const std::map<std::string, QColor>& defaultDBContentColors();

    /// resolves a default color for a DSType; falls back to hashed color for unknown types
    static QColor defaultDSTypeColor(const std::string& ds_type);

    /// resolves a default color for a DBContent name; falls back to hashed color
    static QColor defaultDBContentColor(const std::string& dbcontent);

    /// derives four shades from a base color (dark base -> lighter, light base -> darker)
    static std::array<QColor, 4> autoLineColors(const QColor& base);

    /// samples a base color for a new data source. The hue and saturation range
    /// are biased toward the DSType's default color (Radar->greens, MLAT->reds,
    /// ADSB->blues, RefTraj->oranges, Other->purples, Tracker->near-whites),
    /// with max-hue-distance selection against `existing` (typically the base
    /// colors of other data sources of the same DSType).
    ///
    /// ds_type may be empty — in that case the full hue wheel is used.
    static QColor generateBaseColor(const std::vector<QColor>& existing, Band band,
                                    const std::string& ds_type = "");

    /// hash-based hex color for unknown keys (stable across runs for same key)
    static QColor hashedColor(const std::string& key);

    /// returns c with alpha channel set to the given value
    static QColor withAlpha(const QColor& c, int alpha);

    static constexpr int kNumLineShades = 4;
    static constexpr int kNumSamplingCandidates = 32;

    /// saturation used by generateBaseColor
    static constexpr double kBaseSaturation = 0.6;

    /// value ranges for generateBaseColor
    static constexpr double kLightBandMin = 0.70;
    static constexpr double kLightBandMax = 0.85;
    static constexpr double kDarkBandMin = 0.25;
    static constexpr double kDarkBandMax = 0.45;

    /// lightness offsets used by autoLineColors (fractions of 1.0)
    static constexpr double kLineOffsetStep = 0.08;
};

} // namespace context
