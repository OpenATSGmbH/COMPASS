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

#include "targetreport3dgrid.h"
#include "grid2d.h"
#include "grid2dlayer.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace
{

// cell size giving 0.01 deg per cell in latitude, and in longitude at reference latitude 0
const double kCellDeg = 0.01;
const double kCellM   = 6371000.0 * M_PI / 180.0 * kCellDeg;
const double kCellFt  = 1000.0;
const double kMargin  = 1e-9;

// center of a cell, so that no bin boundary is hit by rounding
double center(int bin)
{
    return (bin + 0.5) * kCellDeg;
}

std::pair<std::int32_t, std::int32_t> bins(std::uint64_t key)
{
    return TargetReport3DGrid::unpack2(key);
}

const TargetReport3DGrid::Cell& onlyCell(const std::unordered_map<std::uint64_t, TargetReport3DGrid::Cell>& cells)
{
    REQUIRE(cells.size() == 1);
    return cells.begin()->second;
}

} // namespace

TEST_CASE("TargetReport3DGrid pack2 and unpack2", "[target_report_3d_grid]")
{
    const std::vector<std::pair<std::int32_t, std::int32_t>> pairs = {
        {0, 0}, {1, 2}, {-1, -1}, {-5, 7}, {7, -5},
        {std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::max()},
        {std::numeric_limits<std::int32_t>::max(), std::numeric_limits<std::int32_t>::min()}};

    for (const auto& p : pairs)
        REQUIRE(TargetReport3DGrid::unpack2(TargetReport3DGrid::pack2(p.first, p.second)) == p);

    REQUIRE(TargetReport3DGrid::pack2(-1, 0) != TargetReport3DGrid::pack2(0, -1));
    REQUIRE(TargetReport3DGrid::pack2(1, 2) != TargetReport3DGrid::pack2(2, 1));
}

TEST_CASE("TargetReport3DGrid horizontal bins and cell centers", "[target_report_3d_grid]")
{
    TargetReport3DGrid grid(kCellM, kCellFt, 0.0);

    SECTION("bins of positive and negative coordinates")
    {
        REQUIRE(bins(grid.horizontalKey(0.015, 0.025)) == std::make_pair(1, 2));
        REQUIRE(bins(grid.horizontalKey(0.005, 0.005)) == std::make_pair(0, 0));
        REQUIRE(bins(grid.horizontalKey(-0.005, -0.015)) == std::make_pair(-1, -2));
        REQUIRE(bins(grid.horizontalKey(-33.865, 151.205)) == std::make_pair(-3387, 15120));
    }

    SECTION("a cell center is at most half a cell away from its positions")
    {
        for (double lat : {48.123456, -33.865, -0.004, 0.004})
            for (double lon : {16.37, -122.42, -0.001, 179.99})
            {
                auto c = grid.horizontalCellCenter(grid.horizontalKey(lat, lon));
                REQUIRE(std::abs(c.first - lat) <= kCellDeg / 2 + kMargin);
                REQUIRE(std::abs(c.second - lon) <= kCellDeg / 2 + kMargin);
            }
    }

    SECTION("longitude cells widen with the reference latitude")
    {
        TargetReport3DGrid grid60(kCellM, kCellFt, 60.0);

        REQUIRE(bins(grid60.horizontalKey(center(1), 0.025)).second == 1);
        auto c = grid60.horizontalCellCenter(grid60.horizontalKey(0.005, 0.005));
        REQUIRE(c.first == Approx(0.005).margin(kMargin));
        REQUIRE(c.second == Approx(0.01).margin(1e-6));
    }
}

TEST_CASE("TargetReport3DGrid altitude bins", "[target_report_3d_grid]")
{
    // (altitude ft, expected bin): a value on a boundary belongs to the upper cell
    const std::vector<std::pair<double, std::int32_t>> cases = {
        {0.0, 0}, {999.999, 0}, {1000.0, 1}, {2000.0, 2}, {-0.001, -1}, {-1000.0, -1}, {-1000.001, -2}};

    for (const auto& c : cases)
    {
        TargetReport3DGrid g(kCellM, kCellFt, 0.0);
        g.addEUI(center(0), center(0), c.first);

        auto alt_lon = g.projectAltLon();
        REQUIRE(alt_lon.size() == 1);
        REQUIRE(bins(alt_lon.begin()->first).first == c.second);
    }
}

TEST_CASE("TargetReport3DGrid accumulation", "[target_report_3d_grid]")
{
    TargetReport3DGrid grid(kCellM, kCellFt, 0.0);

    SECTION("weighted expected and missed slots, extra reports")
    {
        grid.addEUI(center(2), center(5), 500, 0.5);
        grid.addEUI(center(2), center(5), 500, 0.25);
        grid.addMUI(center(2), center(5), 500, 0.25);
        grid.addExtra(center(2), center(5), 500, 3);

        const auto& c = onlyCell(grid.projectHorizontal());
        REQUIRE(grid.numCells() == 1);
        REQUIRE(c.num_eui == Approx(0.75));
        REQUIRE(c.num_mui == Approx(0.25));
        REQUIRE(c.num_extra == 3);
    }

    SECTION("accuracy samples: invalid distances and std-devs are left out")
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();

        grid.addAccuracySample(center(2), center(5), 500, 10.0, 5.0);   // distance, stddev, ratio 2
        grid.addAccuracySample(center(2), center(5), 500, nan, 5.0);    // stddev only
        grid.addAccuracySample(center(2), center(5), 500, inf, 4.0);    // stddev only
        grid.addAccuracySample(center(2), center(5), 500, 6.0, 0.0);    // distance only
        grid.addAccuracySample(center(2), center(5), 500, 6.0, -1.0);   // distance only
        grid.addAccuracySample(center(2), center(5), 500, 6.0, nan);    // distance only

        const auto& c = onlyCell(grid.projectHorizontal());
        REQUIRE(c.num_samples == 6);
        REQUIRE(c.num_distance == 4);
        REQUIRE(c.sum_distance_m == Approx(28.0));
        REQUIRE(c.num_stddev == 3);
        REQUIRE(c.sum_stddev_m == Approx(14.0));
        REQUIRE(c.num_ratio == 1);
        REQUIRE(c.sum_ratio == Approx(2.0));
    }

    SECTION("offset samples: a non-finite component drops the sample and creates no cell")
    {
        grid.addOffsetSample(center(2), center(5), 500, std::nan(""), 1.0);
        grid.addOffsetSample(center(2), center(5), 500, 1.0, INFINITY);
        REQUIRE(grid.numCells() == 0);

        grid.addOffsetSample(center(2), center(5), 500, 3.0, -2.0);
        grid.addOffsetSample(center(2), center(5), 500, 1.0, 4.0);

        const auto& c = onlyCell(grid.projectHorizontal());
        REQUIRE(c.num_offset == 2);
        REQUIRE(c.sum_radial_m == Approx(4.0));
        REQUIRE(c.sum_tangential_m == Approx(2.0));
        REQUIRE(c.num_samples == 0);
    }
}

TEST_CASE("TargetReport3DGrid projections merge along the dropped axis", "[target_report_3d_grid]")
{
    TargetReport3DGrid grid(kCellM, kCellFt, 0.0);

    grid.addEUI(center(2), center(5), 500, 1.0);   // lat 2, lon 5, alt 0
    grid.addEUI(center(2), center(5), 1500, 2.0);  // lat 2, lon 5, alt 1
    grid.addEUI(center(3), center(5), 500, 4.0);   // lat 3, lon 5, alt 0

    REQUIRE(grid.numCells() == 3);

    auto horizontal = grid.projectHorizontal();
    REQUIRE(horizontal.size() == 2);
    REQUIRE(horizontal.at(TargetReport3DGrid::pack2(2, 5)).num_eui == Approx(3.0));
    REQUIRE(horizontal.at(TargetReport3DGrid::pack2(3, 5)).num_eui == Approx(4.0));

    auto alt_lon = grid.projectAltLon();
    REQUIRE(alt_lon.size() == 2);
    REQUIRE(alt_lon.at(TargetReport3DGrid::pack2(0, 5)).num_eui == Approx(5.0));
    REQUIRE(alt_lon.at(TargetReport3DGrid::pack2(1, 5)).num_eui == Approx(2.0));

    auto alt_lat = grid.projectAltLat();
    REQUIRE(alt_lat.size() == 3);
    REQUIRE(alt_lat.at(TargetReport3DGrid::pack2(0, 2)).num_eui == Approx(1.0));
    REQUIRE(alt_lat.at(TargetReport3DGrid::pack2(1, 2)).num_eui == Approx(2.0));
    REQUIRE(alt_lat.at(TargetReport3DGrid::pack2(0, 3)).num_eui == Approx(4.0));
}

TEST_CASE("TargetReport3DGrid projection layers", "[target_report_3d_grid]")
{
    using Grid = TargetReport3DGrid;

    auto eui_value = [](const Grid::Cell& c) -> std::optional<double>
    {
        if (c.num_eui <= 0.0)
            return std::nullopt;
        return c.num_eui;
    };
    auto extra_weight = [](const Grid::Cell& c) { return c.num_extra; };

    SECTION("empty grid: invalid layer")
    {
        Grid grid(kCellM, kCellFt, 0.0);
        REQUIRE(!grid.projectionLayer(Grid::Projection::Horizontal, eui_value, extra_weight).valid);
    }

    Grid grid(kCellM, kCellFt, 0.0);

    // value cells A (lat 2, lon 5) = 1, B (lat 3, lon 7) = 2, C (lat 2, lon 6) = 4,
    // D (lat 4, lon 5) has no value; weights 1, 2, 3, 4
    grid.addEUI(center(2), center(5), 500, 1.0);
    grid.addExtra(center(2), center(5), 500, 1);
    grid.addEUI(center(3), center(7), 500, 2.0);
    grid.addExtra(center(3), center(7), 500, 2);
    grid.addEUI(center(2), center(6), 1500, 4.0);
    grid.addExtra(center(2), center(6), 1500, 3);
    grid.addExtra(center(4), center(5), 500, 4);

    SECTION("horizontal: bounds over the cells with a value, statistics, cell placement")
    {
        auto res = grid.projectionLayer(Grid::Projection::Horizontal, eui_value, extra_weight, "eui");

        REQUIRE(res.valid);
        REQUIRE(res.layer);
        REQUIRE(res.layer->name == "eui");
        REQUIRE(res.x_axis_label == "Longitude (deg)");
        REQUIRE(res.y_axis_label == "Latitude (deg)");

        REQUIRE(res.x_min == Approx(0.05).margin(kMargin));
        REQUIRE(res.x_max == Approx(0.08).margin(kMargin));
        REQUIRE(res.y_min == Approx(0.02).margin(kMargin));
        REQUIRE(res.y_max == Approx(0.04).margin(kMargin));

        REQUIRE(res.cells_with_value == 3);
        REQUIRE(res.v_min == Approx(1.0));
        REQUIRE(res.v_max == Approx(4.0));
        REQUIRE(res.v_mean == Approx(7.0 / 3.0));
        REQUIRE(res.v_median == Approx(2.0));
        REQUIRE(res.v_p95 == Approx(3.8));

        // sample statistics over all 4 projected cells, also D without a value
        REQUIRE(res.total_samples == 10);
        REQUIRE(res.spc_min == 1);
        REQUIRE(res.spc_max == 4);
        REQUIRE(res.spc_mean == Approx(2.5));

        // row 0 = smallest latitude bin, column 0 = smallest longitude bin
        const auto& data = res.layer->data;
        REQUIRE(data.rows() == 2);
        REQUIRE(data.cols() == 3);
        REQUIRE(data(0, 0) == Approx(1.0));
        REQUIRE(data(1, 2) == Approx(2.0));
        REQUIRE(data(0, 1) == Approx(4.0));
        REQUIRE(data(1, 0) == Grid2D::InvalidValue);
        REQUIRE(data(1, 1) == Grid2D::InvalidValue);
        REQUIRE(data(0, 2) == Grid2D::InvalidValue);
    }

    SECTION("altitude / longitude: x longitude, y altitude")
    {
        auto res = grid.projectionLayer(Grid::Projection::AltLon, eui_value, extra_weight);

        REQUIRE(res.valid);
        REQUIRE(res.x_axis_label == "Longitude (deg)");
        REQUIRE(res.y_axis_label == "Altitude (ft)");
        REQUIRE(res.x_min == Approx(0.05).margin(kMargin));
        REQUIRE(res.x_max == Approx(0.08).margin(kMargin));
        REQUIRE(res.y_min == Approx(0.0));
        REQUIRE(res.y_max == Approx(2000.0));
        REQUIRE(res.layer->data.rows() == 2);
        REQUIRE(res.layer->data.cols() == 3);
        REQUIRE(res.layer->data(1, 1) == Approx(4.0));
    }

    SECTION("altitude / latitude: x latitude, y altitude, cells merged over longitude")
    {
        auto res = grid.projectionLayer(Grid::Projection::AltLat, eui_value, extra_weight);

        REQUIRE(res.valid);
        REQUIRE(res.x_axis_label == "Latitude (deg)");
        REQUIRE(res.y_axis_label == "Altitude (ft)");
        REQUIRE(res.x_min == Approx(0.02).margin(kMargin));
        REQUIRE(res.x_max == Approx(0.04).margin(kMargin));

        // (alt 0, lat 2) = A, (alt 0, lat 3) = B, (alt 1, lat 2) = C, (alt 0, lat 4) = D without value
        REQUIRE(res.cells_with_value == 3);
        REQUIRE(res.layer->data(0, 0) == Approx(1.0));
        REQUIRE(res.layer->data(0, 1) == Approx(2.0));
        REQUIRE(res.layer->data(1, 0) == Approx(4.0));
    }

    SECTION("a non-finite cell value is left out")
    {
        auto nan_for_b = [](const Grid::Cell& c) -> std::optional<double>
        {
            if (c.num_eui <= 0.0)
                return std::nullopt;
            if (c.num_eui == 2.0)
                return std::numeric_limits<double>::quiet_NaN();
            return c.num_eui;
        };

        auto res = grid.projectionLayer(Grid::Projection::Horizontal, nan_for_b, extra_weight);

        REQUIRE(res.cells_with_value == 2);
        REQUIRE(res.v_max == Approx(4.0));
        REQUIRE(res.x_max == Approx(0.07).margin(kMargin));
        REQUIRE(res.total_samples == 10);
    }
}

TEST_CASE("TargetReport3DGrid horizontal layer from inspector values", "[target_report_3d_grid]")
{
    TargetReport3DGrid grid(kCellM, kCellFt, 0.0);

    const std::uint64_t a = grid.horizontalKey(center(-2), center(-3));
    const std::uint64_t b = grid.horizontalKey(center(-1), center(-1));
    const std::uint64_t c = grid.horizontalKey(center(0), center(0));

    std::unordered_map<std::uint64_t, double> values = {
        {a, 0.5}, {b, 0.9}, {c, std::numeric_limits<double>::quiet_NaN()}};
    std::unordered_map<std::uint64_t, std::uint64_t> samples = {{a, 10}, {c, 6}};

    auto res = grid.horizontalLayer(values, samples, "pd");

    REQUIRE(res.valid);
    REQUIRE(res.layer->name == "pd");
    REQUIRE(res.cells_with_value == 2);
    REQUIRE(res.v_min == Approx(0.5));
    REQUIRE(res.v_max == Approx(0.9));

    // bounds over a and b (c has no finite value), negative bins included
    REQUIRE(res.x_min == Approx(-0.03).margin(kMargin));
    REQUIRE(res.x_max == Approx(0.0).margin(kMargin));
    REQUIRE(res.y_min == Approx(-0.02).margin(kMargin));
    REQUIRE(res.y_max == Approx(0.0).margin(kMargin));
    REQUIRE(res.layer->data.rows() == 2);
    REQUIRE(res.layer->data.cols() == 3);
    REQUIRE(res.layer->data(0, 0) == Approx(0.5));
    REQUIRE(res.layer->data(1, 2) == Approx(0.9));

    // samples over all keys, b has none
    REQUIRE(res.total_samples == 16);
    REQUIRE(res.spc_min == 0);
    REQUIRE(res.spc_max == 10);
    REQUIRE(res.spc_mean == Approx(16.0 / 3.0));
}
