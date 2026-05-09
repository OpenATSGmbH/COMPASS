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

#include "color_provider.h"

#include "compass.h"
#include "data_source.h"
#include "db_context.h"
#include "db_context_manager.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>

namespace context
{

namespace
{

double clamp01(double v)
{
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

/// Returns the angular distance between two hues in [0, 360) degrees.
double hueDistance(double a, double b)
{
    double d = std::fabs(a - b);
    if (d > 180.0) d = 360.0 - d;
    return d;
}

/// Returns a thread-local RNG seeded once. Deterministic is acceptable here -
/// the HSV sampler does not require cryptographic randomness.
std::mt19937& rng()
{
    static thread_local std::mt19937 gen{std::random_device{}()};
    return gen;
}

} // anonymous

// ============================================================
// Default palettes
// ============================================================

const std::map<std::string, QColor>& ColorProvider::defaultDSTypeColors()
{
    static const std::map<std::string, QColor> palette = {
        {"Radar",   QColor(Qt::green)},
        {"ADSB",    QColor(Qt::blue)},
        {"MLAT",    QColor(Qt::red)},
        {"Tracker", QColor(QStringLiteral("#DDDDDD"))}, // light grey (avoid pure white)
        {"RefTraj", QColor(QStringLiteral("#FFA500"))}, // orange
        {"Other",   QColor(QStringLiteral("#800080"))}, // purple
    };
    return palette;
}

const std::map<std::string, QColor>& ColorProvider::defaultDBContentColors()
{
    // mirror the Geographic View "Color By DBContent" palette, plus a light
    // brown default for CAT010 (not present in the Geographic View palette).
    // (experimental_src/view/geographicview/style/rule/geometrystylerulegenerator.cpp)
    static const std::map<std::string, QColor> palette = {
        {"CAT001",  QColor(QStringLiteral("#00FF00"))},
        {"CAT048",  QColor(QStringLiteral("#00FF00"))},
        {"CAT010",  QColor(QStringLiteral("#C19A6B"))}, // light brown (camel)
        {"CAT020",  QColor(QStringLiteral("#FF0000"))},
        {"CAT021",  QColor(QStringLiteral("#6666FF"))},
        {"RefTraj", QColor(QStringLiteral("#FFA500"))},
        {"CAT062",  QColor(QStringLiteral("#DDDDDD"))},
    };
    return palette;
}

QColor ColorProvider::defaultDSTypeColor(const std::string& ds_type)
{
    const auto& palette = defaultDSTypeColors();
    auto it = palette.find(ds_type);
    if (it != palette.end())
        return it->second;
    return hashedColor(ds_type);
}

QColor ColorProvider::defaultDBContentColor(const std::string& dbcontent)
{
    const auto& palette = defaultDBContentColors();
    auto it = palette.find(dbcontent);
    if (it != palette.end())
        return it->second;
    return hashedColor(dbcontent);
}

// ============================================================
// Hashed fallback color
// ============================================================

QColor ColorProvider::hashedColor(const std::string& key)
{
    std::size_t h = std::hash<std::string>{}(key);
    int r = (h >> 16) & 0xFF;
    int g = (h >> 8)  & 0xFF;
    int b = h         & 0xFF;
    return QColor(r, g, b);
}

QColor ColorProvider::withAlpha(const QColor& c, int alpha)
{
    QColor out = c;
    out.setAlpha(alpha);
    return out;
}

// ============================================================
// Line shade generation
// ============================================================

std::array<QColor, 4> ColorProvider::autoLineColors(const QColor& base)
{
    qreal h, s, l, a;
    base.getHslF(&h, &s, &l, &a);

    // direction: dark base (l < 0.5) -> lighter steps; light base -> darker
    double direction = (l < 0.5) ? +1.0 : -1.0;

    std::array<QColor, kNumLineShades> out;
    for (int i = 0; i < kNumLineShades; ++i)
    {
        double offset = direction * kLineOffsetStep * (i + 1);
        double new_l = clamp01(l + offset);
        QColor shade = QColor::fromHslF(
            std::max(0.0, h),
            clamp01(s),
            new_l,
            clamp01(a));
        shade = shade.toRgb();
        out[i] = shade;
    }
    return out;
}

// ============================================================
// Base color generation
// ============================================================

namespace
{

/// Sampling range for a DSType: hue window centered on `hue_center` (half-width
/// in degrees; >= 180 means unrestricted), and saturation range.
/// Tracker is special: near-white, so saturation is near zero and the value
/// range is overridden to a high band regardless of preference.
struct DSTypeSamplingRange
{
    double hue_center {0.0};
    double hue_half_window {180.0}; // >= 180 => full wheel
    double s_min {0.5};
    double s_max {0.7};
    bool   override_value {false};
    double v_min {0.0};
    double v_max {0.0};
};

DSTypeSamplingRange rangeFor(const std::string& ds_type)
{
    DSTypeSamplingRange r;
    if (ds_type == "Radar")
    {
        // green + some yellow-green/brown extension
        r.hue_center = 120.0; r.hue_half_window = 45.0;
        r.s_min = 0.30; r.s_max = 0.70;
    }
    else if (ds_type == "MLAT")
    {
        r.hue_center = 0.0; r.hue_half_window = 25.0;
        r.s_min = 0.55; r.s_max = 0.80;
    }
    else if (ds_type == "ADSB")
    {
        r.hue_center = 225.0; r.hue_half_window = 25.0;
        r.s_min = 0.50; r.s_max = 0.80;
    }
    else if (ds_type == "RefTraj")
    {
        r.hue_center = 32.0; r.hue_half_window = 15.0;
        r.s_min = 0.60; r.s_max = 0.90;
    }
    else if (ds_type == "Other")
    {
        r.hue_center = 300.0; r.hue_half_window = 25.0;
        r.s_min = 0.50; r.s_max = 0.80;
    }
    else if (ds_type == "Tracker")
    {
        // light grey + very light pastel tints - very low saturation, cap value
        // below 1.0 so generated base colors can't land on pure white
        r.hue_center = 0.0; r.hue_half_window = 180.0;
        r.s_min = 0.00; r.s_max = 0.18;
        r.override_value = true;
        r.v_min = 0.82; r.v_max = 0.92;
    }
    // else: full wheel, default saturation (unknown DSType)
    return r;
}

} // anonymous

QColor ColorProvider::generateBaseColor(const std::vector<QColor>& existing, Band band,
                                        const std::string& ds_type)
{
    auto& gen = rng();

    DSTypeSamplingRange range = rangeFor(ds_type);

    double v_min = range.override_value ? range.v_min
                 : ((band == Band::Light) ? kLightBandMin : kDarkBandMin);
    double v_max = range.override_value ? range.v_max
                 : ((band == Band::Light) ? kLightBandMax : kDarkBandMax);

    std::uniform_real_distribution<double> val_dist(v_min, v_max);
    std::uniform_real_distribution<double> sat_dist(range.s_min, range.s_max);
    std::uniform_real_distribution<double> hue_any_dist(0.0, 360.0);
    std::uniform_real_distribution<double> hue_offset_dist(-range.hue_half_window,
                                                            range.hue_half_window);

    // collect existing hues for max-hue-distance selection
    std::vector<double> existing_hues;
    existing_hues.reserve(existing.size());
    for (const auto& c : existing)
    {
        if (!c.isValid()) continue;
        qreal h, s, v, a;
        c.getHsvF(&h, &s, &v, &a);
        if (h < 0) continue; // achromatic - no defined hue
        existing_hues.push_back(h * 360.0);
    }

    auto sampleHue = [&]()
    {
        if (range.hue_half_window >= 180.0)
            return hue_any_dist(gen);
        double h = range.hue_center + hue_offset_dist(gen);
        h = std::fmod(h, 360.0);
        if (h < 0.0) h += 360.0;
        return h;
    };

    double best_hue = sampleHue();
    double best_sat = sat_dist(gen);
    double best_val = val_dist(gen);
    double best_min_distance = -1.0;

    for (int i = 0; i < kNumSamplingCandidates; ++i)
    {
        double h = sampleHue();
        double s = sat_dist(gen);
        double v = val_dist(gen);

        double min_distance = 180.0;
        for (double eh : existing_hues)
        {
            double d = hueDistance(h, eh);
            if (d < min_distance) min_distance = d;
        }

        if (existing_hues.empty() || min_distance > best_min_distance)
        {
            best_hue = h;
            best_sat = s;
            best_val = v;
            best_min_distance = min_distance;
        }
    }

    return QColor::fromHsvF(clamp01(best_hue / 360.0),
                            clamp01(best_sat),
                            clamp01(best_val)).toRgb();
}

QColor resolveSeriesColor(const std::string& ds_type,
                          const std::string& ds_name,
                          int line_index,
                          const std::string& dbcontent_name,
                          COMPASS& compass,
                          std::function<QColor(const std::string&)> hashed_fallback)
{
    const unsigned int line_id = (line_index >= 0 && line_index < 4) ? (unsigned int)line_index : 0;

    auto& ctx_mgr = compass.dbContextManager();
    const auto mode = (ColorProvider::Mode)compass.colorMode();

    auto derive_line_shade = [line_id](const QColor& base) {
        auto shades = ColorProvider::autoLineColors(base);
        return shades[line_id];
    };

    auto fallback = [&hashed_fallback, &ds_type, &ds_name, &dbcontent_name,
                     derive_line_shade, mode]() -> QColor {
        std::string key;
        switch (mode)
        {
            case ColorProvider::Mode::DSType:    key = ds_type;        break;
            case ColorProvider::Mode::DBContent: key = dbcontent_name; break;
            default:                             key = ds_name;        break;
        }
        if (key.empty())
            key = ds_name;

        QColor base = hashed_fallback ? hashed_fallback(key)
                                      : ColorProvider::hashedColor(key);
        if (mode == ColorProvider::Mode::DataSourceLine)
            return derive_line_shade(base);
        return base;
    };

    if (!ctx_mgr.hasActiveContext())
    {
        // key carries ds_type / dbcontent_name directly - DSType & DBContent palettes still resolvable
        if (mode == ColorProvider::Mode::DSType && !ds_type.empty())
            return ColorProvider::defaultDSTypeColor(ds_type);
        if (mode == ColorProvider::Mode::DBContent && !dbcontent_name.empty())
            return ColorProvider::defaultDBContentColor(dbcontent_name);
        return fallback();
    }

    const auto& ctx = ctx_mgr.activeContext();
    const DataSource* ds = nullptr;
    if (ctx_mgr.hasDataSource(ds_name))
        ds = ctx_mgr.dataSource(ctx_mgr.getDataSourceId(ds_name));

    switch (mode)
    {
        case ColorProvider::Mode::DSType:
        {
            if (ds_type.empty())
                return fallback();
            const auto& palette = ctx.colors().ds_type_colors;
            auto it = palette.find(ds_type);
            if (it != palette.end() && it->second.isValid())
                return it->second;
            return ColorProvider::defaultDSTypeColor(ds_type);
        }
        case ColorProvider::Mode::DBContent:
        {
            if (dbcontent_name.empty())
                return fallback();
            const auto& palette = ctx.colors().dbcontent_colors;
            auto it = palette.find(dbcontent_name);
            if (it != palette.end() && it->second.isValid())
                return it->second;
            return ColorProvider::defaultDBContentColor(dbcontent_name);
        }
        case ColorProvider::Mode::DataSource:
        {
            if (ds && ds->baseColor().isValid())
                return ds->baseColor();
            return fallback();
        }
        case ColorProvider::Mode::DataSourceLine:
        {
            if (ds)
            {
                QColor c = ds->lineColor(line_id);
                if (c.isValid())
                    return c;
                if (ds->baseColor().isValid())
                    return derive_line_shade(ds->baseColor());
            }
            return fallback();
        }
    }
    return fallback();
}

} // namespace context
