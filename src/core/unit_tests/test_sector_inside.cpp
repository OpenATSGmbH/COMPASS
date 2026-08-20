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
#include "sector.h"
#include "targetposition.h"

#include <ogr_geometry.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

namespace
{
    //sector geometry is expressed as (latitude, longitude), matching Sector::createPolygon
    const double CenterLat = 35.9;
    const double CenterLon = 14.5;
    const double Radius    = 0.9;
    const double Delta     = 0.1;

    /**
     * Generates a closed, non-convex ring around the center. The radius wobble keeps the
     * boundary from being a trivial circle, so the discretized raster is actually exercised.
     */
    std::vector<std::pair<double,double>> makeSectorPoints(int num_points, double radius = Radius)
    {
        std::vector<std::pair<double,double>> points;
        points.reserve(num_points);

        for (int cnt = 0; cnt < num_points; ++cnt)
        {
            double angle  = 2.0 * M_PI * cnt / num_points;
            double radius_cur = radius * (0.85 + 0.15 * std::sin(5.0 * angle));

            points.emplace_back(CenterLat + radius_cur * std::cos(angle),
                                CenterLon + radius_cur * std::sin(angle));
        }

        return points;
    }

    /**
     * Builds the same polygon Sector::createPolygon builds, for use as an independent reference.
     */
    std::unique_ptr<OGRPolygon> makeReferencePolygon(const std::vector<std::pair<double,double>>& points)
    {
        std::unique_ptr<OGRPolygon> polygon (new OGRPolygon());
        OGRLinearRing* ring = new OGRLinearRing();

        for (const auto& point_it : points)
            ring->addPoint(point_it.first, point_it.second);

        if (*points.begin() != *points.rbegin())
            ring->addPoint(points.begin()->first, points.begin()->second);

        polygon->addRingDirectly(ring);

        return polygon;
    }

    /**
     * Exact meaning of 'within delta of the sector': distance to the polygon is at most delta.
     * OGRGeometry::Distance returns 0 for positions inside the polygon.
     */
    double distanceToPolygon(const OGRPolygon& polygon, double latitude, double longitude)
    {
        OGRPoint point (latitude, longitude);

        return polygon.Distance(&point);
    }

    std::vector<std::pair<double,double>> makeProbes(unsigned int count, double margin = 1.4)
    {
        std::mt19937 rng (4242); //fixed seed, the test must be reproducible

        std::uniform_real_distribution<double> lat_dist (CenterLat - Radius * margin,
                                                         CenterLat + Radius * margin);
        std::uniform_real_distribution<double> lon_dist (CenterLon - Radius * margin,
                                                         CenterLon + Radius * margin);

        std::vector<std::pair<double,double>> probes;
        probes.reserve(count);

        for (unsigned int cnt = 0; cnt < count; ++cnt)
            probes.emplace_back(lat_dist(rng), lon_dist(rng));

        return probes;
    }
}

/**
 * Regression test for the fast inside test itself.
 *
 * The pixel classification used to compare a QRgb against Qt::GlobalColor enumerators, which
 * resolves to an integer comparison (Qt::white == 3) and never matches, so every position was
 * reported as Border and the raster never avoided a single geometric check.
 */
TEST_CASE("SectorInsideTest classifies positions", "[sector][inside]")
{
    auto points = makeSectorPoints(64);

    double lat_min = points[ 0 ].first,  lat_max = points[ 0 ].first;
    double lon_min = points[ 0 ].second, lon_max = points[ 0 ].second;

    for (const auto& point_it : points)
    {
        lat_min = std::min(lat_min, point_it.first);
        lat_max = std::max(lat_max, point_it.first);
        lon_min = std::min(lon_min, point_it.second);
        lon_max = std::max(lon_max, point_it.second);
    }

    SectorInsideTest inside_test (points, lat_min, lon_min, lat_max, lon_max);

    REQUIRE(inside_test.isValid());

    SECTION("center is reported as inside")
    {
        REQUIRE(inside_test.isInside(CenterLat, CenterLon) == SectorInsideTest::CheckResult::Inside);
    }

    SECTION("far away position is reported as outside")
    {
        //well inside the raster bounds but clearly outside the sector: the raster must be
        //decisive here, otherwise the fast path is worthless
        REQUIRE(inside_test.isInside(lat_min + 0.001, lon_min + 0.001) == SectorInsideTest::CheckResult::Outside);
    }

    SECTION("not every position is reported as border")
    {
        auto probes = makeProbes(2000);

        unsigned int num_inside = 0, num_outside = 0, num_border = 0, num_checked = 0;

        for (const auto& probe_it : probes)
        {
            //positions outside the raster bounds are reported as border by design, they carry
            //no information about the classification and are rejected by the bounding rect check
            //before the raster is ever consulted
            if (probe_it.first  < lat_min || probe_it.first  > lat_max ||
                probe_it.second < lon_min || probe_it.second > lon_max)
                continue;

            ++num_checked;

            auto res = inside_test.isInside(probe_it.first, probe_it.second);

            if (res == SectorInsideTest::CheckResult::Inside)
                ++num_inside;
            else if (res == SectorInsideTest::CheckResult::Outside)
                ++num_outside;
            else
                ++num_border;
        }

        INFO(num_checked << " positions inside the raster bounds, " << num_inside << " inside, "
             << num_outside << " outside, " << num_border << " border");

        REQUIRE(num_checked > 0);
        REQUIRE(num_inside  > 0);
        REQUIRE(num_outside > 0);

        //the border band is 7 pixels of a 1000 pixel raster, so it must stay a small minority
        REQUIRE(num_border < num_checked / 10);
    }
}

/**
 * Proves that Sector::isInside(lat, lon, delta) matches the exact definition of the check,
 * namely that the distance from the position to the sector polygon is at most delta.
 */
TEST_CASE("Sector delta inside check matches exact distance", "[sector][inside]")
{
    const unsigned int NumProbes = 20000;

    //positions whose distance is this close to delta cannot be decided reliably, because
    //Buffer approximates the rounded corners of the inflated polygon with straight segments
    //(8 per quadrant, worst case sagitta delta * (1 - cos(pi/32)) which is about 0.5% of delta)
    const double AmbiguousBand = Delta * 0.01;

    auto probes = makeProbes(NumProbes);

    for (int num_points : {20, 100, 500})
    {
        auto points    = makeSectorPoints(num_points);
        auto reference = makeReferencePolygon(points);

        Sector sector (1, "test_sector", "test_layer", false, false, QColor(), points);
        sector.createFastInsideTest(Delta);

        //a second sector without the prepared delta shape, which takes the probe ring path
        Sector sector_probe_ring (2, "test_sector_ring", "test_layer", false, false, QColor(), points);

        unsigned int num_checked = 0, num_ambiguous = 0, num_truly_inside = 0;
        unsigned int errors_fast = 0, errors_probe_ring = 0;

        for (const auto& probe_it : probes)
        {
            double distance = distanceToPolygon(*reference, probe_it.first, probe_it.second);

            if (std::fabs(distance - Delta) < AmbiguousBand)
            {
                ++num_ambiguous;
                continue;
            }

            bool expected = distance <= Delta;

            if (expected)
                ++num_truly_inside;

            ++num_checked;

            if (sector.isInside(probe_it.first, probe_it.second, Delta) != expected)
                ++errors_fast;

            if (sector_probe_ring.isInside(probe_it.first, probe_it.second, Delta) != expected)
                ++errors_probe_ring;
        }

        INFO("sector with " << num_points << " points, " << num_checked << " positions checked, "
             << num_ambiguous << " skipped as ambiguous");

        //the test must actually exercise both answers
        REQUIRE(num_truly_inside > 0);
        REQUIRE(num_truly_inside < num_checked);

        //the ambiguous band must stay a small minority, otherwise the comparison is meaningless
        REQUIRE(num_ambiguous < NumProbes / 100);

        //the prepared check is exact outside the ambiguous band
        REQUIRE(errors_fast == 0);

        //and it is never worse than the probe ring it replaces
        REQUIRE(errors_fast <= errors_probe_ring);
    }
}

/**
 * The inflated polygon describes the sector grown by delta. The position based overload must
 * keep reporting the sector itself, so it must not consult a raster built from an inflated shape.
 */
TEST_CASE("Sector position check is unaffected by the delta shape", "[sector][inside]")
{
    auto points    = makeSectorPoints(100);
    auto reference = makeReferencePolygon(points);

    Sector sector (1, "test_sector", "test_layer", false, false, QColor(), points);
    sector.createFastInsideTest(Delta);

    auto probes = makeProbes(20000);

    unsigned int num_in_ring = 0;

    for (const auto& probe_it : probes)
    {
        double distance = distanceToPolygon(*reference, probe_it.first, probe_it.second);

        //positions in the ring between the sector and its inflated version
        if (distance <= 0.0 || distance > Delta * 0.9)
            continue;

        ++num_in_ring;

        dbContent::TargetPosition pos (probe_it.first, probe_it.second, false, false, 0.0f);

        //inside the inflated shape
        REQUIRE(sector.isInside(probe_it.first, probe_it.second, Delta));

        //but outside the sector itself
        REQUIRE(!sector.isInside(pos, false, false, Sector::InsideCheckType::XY));
    }

    REQUIRE(num_in_ring > 0);
}

/**
 * Changing the delta must rebuild the prepared shape rather than answer from a stale one.
 */
TEST_CASE("Sector delta inside check rebuilds on delta change", "[sector][inside]")
{
    auto points    = makeSectorPoints(100);
    auto reference = makeReferencePolygon(points);

    Sector sector (1, "test_sector", "test_layer", false, false, QColor(), points);

    auto probes = makeProbes(5000);

    for (double delta : {0.05, 0.1, 0.2})
    {
        sector.createFastInsideTest(delta);

        const double ambiguous_band = delta * 0.01;

        unsigned int errors = 0, checked = 0;

        for (const auto& probe_it : probes)
        {
            double distance = distanceToPolygon(*reference, probe_it.first, probe_it.second);

            if (std::fabs(distance - delta) < ambiguous_band)
                continue;

            ++checked;

            if (sector.isInside(probe_it.first, probe_it.second, delta) != (distance <= delta))
                ++errors;
        }

        INFO("delta " << delta << ", " << checked << " positions checked");

        REQUIRE(checked > 0);
        REQUIRE(errors == 0);
    }
}
