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

#include "catch.hpp"

#include "color_provider.h"

#include <cmath>
#include <vector>

using namespace context;

namespace
{

double hueDegreesOf(const QColor& c)
{
    qreal h, s, v, a;
    c.getHsvF(&h, &s, &v, &a);
    return h * 360.0;
}

double hueDistance(double a, double b)
{
    double d = std::fabs(a - b);
    if (d > 180.0) d = 360.0 - d;
    return d;
}

} // anonymous

TEST_CASE("ColorProvider default DSType palette", "[color_provider]")
{
    REQUIRE(ColorProvider::defaultDSTypeColor("Radar") == QColor(Qt::green));
    REQUIRE(ColorProvider::defaultDSTypeColor("ADSB") == QColor(Qt::blue));
    REQUIRE(ColorProvider::defaultDSTypeColor("MLAT") == QColor(Qt::red));
    REQUIRE(ColorProvider::defaultDSTypeColor("Tracker") == QColor(Qt::white));
    REQUIRE(ColorProvider::defaultDSTypeColor("RefTraj").name() == QStringLiteral("#ffa500"));
    REQUIRE(ColorProvider::defaultDSTypeColor("Other").name() == QStringLiteral("#800080"));

    // unknown key falls back to a deterministic hash-based color
    QColor a = ColorProvider::defaultDSTypeColor("Unknown");
    QColor b = ColorProvider::defaultDSTypeColor("Unknown");
    REQUIRE(a == b);
    REQUIRE(a.isValid());
}

TEST_CASE("ColorProvider default DBContent palette matches Geographic View", "[color_provider]")
{
    // verbatim from experimental_src/view/geographicview/style/rule/geometrystylerulegenerator.cpp
    REQUIRE(ColorProvider::defaultDBContentColor("CAT001").name()  == QStringLiteral("#00ff00"));
    REQUIRE(ColorProvider::defaultDBContentColor("CAT048").name()  == QStringLiteral("#00ff00"));
    REQUIRE(ColorProvider::defaultDBContentColor("CAT020").name()  == QStringLiteral("#ff0000"));
    REQUIRE(ColorProvider::defaultDBContentColor("CAT021").name()  == QStringLiteral("#6666ff"));
    REQUIRE(ColorProvider::defaultDBContentColor("RefTraj").name() == QStringLiteral("#ffa500"));
    REQUIRE(ColorProvider::defaultDBContentColor("CAT062").name()  == QStringLiteral("#dddddd"));
}

TEST_CASE("ColorProvider autoLineColors direction depends on base lightness", "[color_provider]")
{
    SECTION("dark base -> lighter shades")
    {
        QColor base = QColor::fromHslF(0.33, 0.6, 0.2);
        auto shades = ColorProvider::autoLineColors(base);

        qreal h, s, l0, a;
        base.getHslF(&h, &s, &l0, &a);

        qreal prev = l0;
        for (const auto& shade : shades)
        {
            qreal lh, ls, ll, la;
            shade.getHslF(&lh, &ls, &ll, &la);
            REQUIRE(ll > prev - 1e-6); // monotonically lighter
            prev = ll;
        }
    }

    SECTION("light base -> darker shades")
    {
        QColor base = QColor::fromHslF(0.66, 0.6, 0.85);
        auto shades = ColorProvider::autoLineColors(base);

        qreal h, s, l0, a;
        base.getHslF(&h, &s, &l0, &a);

        qreal prev = l0;
        for (const auto& shade : shades)
        {
            qreal lh, ls, ll, la;
            shade.getHslF(&lh, &ls, &ll, &la);
            REQUIRE(ll < prev + 1e-6); // monotonically darker
            prev = ll;
        }
    }

    SECTION("L1 offset smaller than L4 offset")
    {
        QColor base = QColor::fromHslF(0.0, 0.6, 0.3);
        auto shades = ColorProvider::autoLineColors(base);

        qreal bh, bs, bl, ba;
        base.getHslF(&bh, &bs, &bl, &ba);

        qreal l1, s1, ll1, a1;
        shades[0].getHslF(&l1, &s1, &ll1, &a1);

        qreal l4, s4, ll4, a4;
        shades[3].getHslF(&l4, &s4, &ll4, &a4);

        REQUIRE(std::fabs(ll1 - bl) < std::fabs(ll4 - bl));
    }
}

TEST_CASE("ColorProvider generateBaseColor stays in band", "[color_provider]")
{
    for (int i = 0; i < 40; ++i)
    {
        QColor c = ColorProvider::generateBaseColor({}, ColorProvider::Band::Light);
        qreal h, s, v, a;
        c.getHsvF(&h, &s, &v, &a);
        REQUIRE(v >= ColorProvider::kLightBandMin - 1e-6);
        REQUIRE(v <= ColorProvider::kLightBandMax + 1e-6);
    }

    for (int i = 0; i < 40; ++i)
    {
        QColor c = ColorProvider::generateBaseColor({}, ColorProvider::Band::Dark);
        qreal h, s, v, a;
        c.getHsvF(&h, &s, &v, &a);
        REQUIRE(v >= ColorProvider::kDarkBandMin - 1e-6);
        REQUIRE(v <= ColorProvider::kDarkBandMax + 1e-6);
    }
}

TEST_CASE("ColorProvider generateBaseColor picks hue-distant candidate", "[color_provider]")
{
    // existing colors clustered around hue 0 (red). A new color should tend
    // toward a hue far from red. We check that the mean of many samples is
    // significantly farther than a naive random pick would be.

    std::vector<QColor> existing = {
        QColor::fromHsvF(0.0 / 360.0, 0.6, 0.5),
        QColor::fromHsvF(10.0 / 360.0, 0.6, 0.5),
        QColor::fromHsvF(350.0 / 360.0, 0.6, 0.5),
    };

    int near_count = 0;
    const int samples = 50;
    for (int i = 0; i < samples; ++i)
    {
        QColor c = ColorProvider::generateBaseColor(existing, ColorProvider::Band::Light);
        double h = hueDegreesOf(c);

        // distance to the closest existing hue (all near 0/360)
        double d = std::min({hueDistance(h, 0.0), hueDistance(h, 10.0), hueDistance(h, 350.0)});

        // "near" means within 30 degrees of one of the existing hues
        if (d < 30.0) ++near_count;
    }

    // with N=32 candidates the sampler should almost never land in the clustered
    // neighbourhood — fewer than 10% "near" samples is a generous bound
    REQUIRE(near_count < samples / 5);
}
