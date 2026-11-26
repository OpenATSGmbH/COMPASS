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
#include "mock_variable_resolver.h"
#include "sectorsfilter.h"
#include "sectorinsidecache.h"
#include "buffer/buffer.h"
#include "dbcontent/dbcontent.h"
#include "json.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace dbContent;
using nlohmann::json;

namespace
{
    using Geometry = SectorInsideCache::SectorGeometry;

    Geometry squareSector(unsigned int id, const std::string& name,
                          double lat0, double lat1, double lon0, double lon1, bool exclusion = false)
    {
        Geometry geometry;
        geometry.id        = id;
        geometry.name      = name;
        geometry.exclusion = exclusion;
        geometry.points    = { {lat0, lon0}, {lat0, lon1}, {lat1, lon1}, {lat1, lon0} };
        return geometry;
    }

    std::vector<Geometry> layerOf(const Geometry& geometry)
    {
        return { geometry };
    }
}

TEST_CASE("SectorsFilter construction", "[filter][sectors]")
{
    auto mock = createStandardMock();
    auto cfg  = makeFilterConfig("SectorsFilter", "SectorsFilter0");

    SectorsFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Sectors");
    CHECK_FALSE(filter.getActive());
    CHECK(filter.selectedLayers().empty());
    CHECK_FALSE(filter.useDelta());
    CHECK(filter.deltaDeg() == Approx(0.1));
    CHECK(filter.effectiveDeltaDeg() == 0.0);
    CHECK(filter.activeInLiveMode());
}

TEST_CASE("SectorsFilter configuration and selection", "[filter][sectors]")
{
    auto mock = createStandardMock();
    auto cfg  = makeFilterConfig("SectorsFilter", "SectorsFilter0", {
        {"layers", json::array({"L1", "L2"})},
        {"use_delta", true},
        {"delta_deg", 0.05}
    });

    SectorsFilter filter(cfg, nullptr, mock);

    CHECK(filter.selectedLayers() == std::vector<std::string>{"L1", "L2"});
    CHECK(filter.layerSelected("L1"));
    CHECK_FALSE(filter.layerSelected("L3"));
    CHECK(filter.useDelta());
    CHECK(filter.effectiveDeltaDeg() == Approx(0.05));

    filter.selectLayer("L1", false);
    CHECK(filter.selectedLayers() == std::vector<std::string>{"L2"});

    //selecting twice adds once
    filter.selectLayer("L3", true);
    filter.selectLayer("L3", true);
    CHECK(filter.selectedLayers() == std::vector<std::string>{"L2", "L3"});

    filter.useDelta(false);
    CHECK(filter.effectiveDeltaDeg() == 0.0);
}

TEST_CASE("SectorsFilter filters applicability", "[filter][sectors]")
{
    auto mock = createStandardMock();
    auto cfg  = makeFilterConfig("SectorsFilter", "SectorsFilter0");

    SectorsFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("CAT001"));
    CHECK(filter.filters("CAT020"));
    CHECK(filter.filters("CAT021"));
    CHECK(filter.filters("CAT048"));
    CHECK(filter.filters("CAT062"));
    CHECK(filter.filters("RefTraj"));
    CHECK_FALSE(filter.filters("CAT002"));
}

TEST_CASE("SectorsFilter sqlFor", "[filter][sectors]")
{
    auto mock = createStandardMock();

    const std::string lat = mock.metaGetVariableDBColumn("CAT048", dbcontent_vars::meta_var_latitude_);
    const std::string lon = mock.metaGetVariableDBColumn("CAT048", dbcontent_vars::meta_var_longitude_);
    const std::string alt = mock.metaGetVariableDBColumn("CAT048", dbcontent_vars::meta_var_mc_);

    std::vector<SectorsFilter::Layer> layers = { { "L1", { 47.0, 48.0, 15.0, 16.0 } } };

    SECTION("one layer, plain containment")
    {
        auto clause = SectorsFilter::sqlFor(mock, "CAT048", layers, 0.0);

        CHECK(clause.sql == "(" + lat + " BETWEEN 47 AND 48 AND " + lon + " BETWEEN 15 AND 16 AND "
                            "sector_inside('L1', " + lat + ", " + lon + ", " + alt + ", 0))");
    }

    SECTION("two layers, inflating delta widens the box")
    {
        layers.push_back({ "L2", { 46.5, 47.5, 16.0, 17.0 } });

        auto clause = SectorsFilter::sqlFor(mock, "CAT048", layers, 0.1);

        CHECK(clause.sql == "(" + lat + " BETWEEN 46.4 AND 48.1 AND " + lon + " BETWEEN 14.9 AND 17.1 AND "
                            "(sector_inside('L1', " + lat + ", " + lon + ", " + alt + ", 0.1) OR "
                            "sector_inside('L2', " + lat + ", " + lon + ", " + alt + ", 0.1)))");
    }

    SECTION("deflating delta keeps the box")
    {
        auto clause = SectorsFilter::sqlFor(mock, "CAT048", layers, -0.1);

        CHECK(clause.sql == "(" + lat + " BETWEEN 47 AND 48 AND " + lon + " BETWEEN 15 AND 16 AND "
                            "sector_inside('L1', " + lat + ", " + lon + ", " + alt + ", -0.1))");
    }

    SECTION("layer name quoting")
    {
        layers[0].name = "it's";

        auto clause = SectorsFilter::sqlFor(mock, "CAT048", layers, 0.0);

        CHECK(clause.sql.find("sector_inside('it''s', ") != std::string::npos);
    }

    SECTION("no layers or no position gives no clause")
    {
        CHECK(SectorsFilter::sqlFor(mock, "CAT048", {}, 0.0).empty());
        CHECK(SectorsFilter::sqlFor(mock, "CAT002", layers, 0.0).empty());
    }
}

TEST_CASE("SectorsFilter extentDeltaDeg", "[filter][sectors]")
{
    using Layer  = SectorsFilter::Layer;
    using Bounds = SectorInsideCache::LayerBounds;

    SECTION("no layers gives 0")
    {
        CHECK(SectorsFilter::extentDeltaDeg({}, 0.1, 200.0, 0.2) == 0.0);
    }

    SECTION("fraction of the larger side, the longitude side scaled by the latitude")
    {
        // 0.1 deg of latitude by 0.3 deg of longitude at 60 N, the longitude side is 0.15 deg
        Layer layer{"L", Bounds{59.95, 60.05, 10.0, 10.3}};

        CHECK(SectorsFilter::extentDeltaDeg({layer}, 0.1, 0.0, 1.0) == Approx(0.015).epsilon(1e-6));
    }

    SECTION("combined bounds of several layers")
    {
        Layer a{"A", Bounds{48.0, 48.1, 16.0, 16.1}};
        Layer b{"B", Bounds{48.4, 48.5, 16.0, 16.1}}; // 0.5 deg of latitude together

        CHECK(SectorsFilter::extentDeltaDeg({a, b}, 0.1, 0.0, 1.0) == Approx(0.05).epsilon(1e-6));
    }

    SECTION("minimum in meters, maximum in degrees")
    {
        Layer small{"S", Bounds{48.0, 48.001, 16.0, 16.001}}; // about 110 m
        CHECK(SectorsFilter::extentDeltaDeg({small}, 0.1, 200.0, 0.2)
              == Approx(200.0 / 111320.0).epsilon(1e-6));

        Layer large{"F", Bounds{45.0, 50.0, 9.0, 19.0}}; // 5 deg of latitude
        CHECK(SectorsFilter::extentDeltaDeg({large}, 0.1, 200.0, 0.2) == Approx(0.2).epsilon(1e-9));
    }
}

TEST_CASE("SectorsFilter getClause uses the cache layers", "[filter][sectors]")
{
    auto mock = createStandardMock();
    auto cfg  = makeFilterConfig("SectorsFilter", "SectorsFilter0", {
        {"active", true},
        {"layers", json::array({"L1", "Unknown"})}
    });

    SectorsFilter filter(cfg, nullptr, mock);

    filter.cache().setLayer("L1", layerOf(squareSector(1, "S1", 47.0, 48.0, 15.0, 16.0)));

    CHECK(filter.availableLayers() == std::vector<std::string>{"L1"});

    const std::string lat = mock.metaGetVariableDBColumn("CAT048", dbcontent_vars::meta_var_latitude_);
    const std::string lon = mock.metaGetVariableDBColumn("CAT048", dbcontent_vars::meta_var_longitude_);
    const std::string alt = mock.metaGetVariableDBColumn("CAT048", dbcontent_vars::meta_var_mc_);

    //the unknown layer contributes nothing
    auto clause = filter.getClause("CAT048");
    CHECK(clause.sql == "(" + lat + " BETWEEN 47 AND 48 AND " + lon + " BETWEEN 15 AND 16 AND "
                        "sector_inside('L1', " + lat + ", " + lon + ", " + alt + ", 0))");

    //legacy path
    dbContent::VariableSet read_set;
    bool first = true;
    CHECK(filter.getConditionString("CAT048", read_set, first) == " " + clause.sql);
    CHECK_FALSE(first);
    CHECK(filter.getConditionString("CAT048", read_set, first) == " AND " + clause.sql);

    //delta
    filter.useDelta(true);
    filter.deltaDeg(0.1);
    CHECK(filter.getClause("CAT048").sql.find(", 0.1)") != std::string::npos);

    //a layer with exclusion sectors only has no bounds and gives no clause
    filter.cache().setLayer("L1", layerOf(squareSector(1, "S1", 47.0, 48.0, 15.0, 16.0, true)));
    CHECK(filter.getClause("CAT048").empty());

    //inactive
    filter.cache().setLayer("L1", layerOf(squareSector(1, "S1", 47.0, 48.0, 15.0, 16.0)));
    filter.setActive(false);
    CHECK(filter.getClause("CAT048").empty());

    //no position
    filter.setActive(true);
    CHECK(filter.getClause("CAT002").empty());
}

TEST_CASE("SectorsFilter removes layers not in the context", "[filter][sectors]")
{
    auto mock = createStandardMock();
    auto cfg  = makeFilterConfig("SectorsFilter", "SectorsFilter0", {
        {"layers", json::array({"L1", "Old"})}
    });

    SectorsFilter filter(cfg, nullptr, mock);

    //without a filter manager the cache layers stand in for the context layers
    filter.cache().setLayer("L1", layerOf(squareSector(1, "S1", 47.0, 48.0, 15.0, 16.0)));

    CHECK(filter.removeLayersNotInContext() == 1);
    CHECK(filter.selectedLayers() == std::vector<std::string>{"L1"});
    CHECK(filter.cache().hasLayer("L1"));

    //nothing left to remove
    CHECK(filter.removeLayersNotInContext() == 0);
    CHECK(filter.selectedLayers() == std::vector<std::string>{"L1"});
}

TEST_CASE("SectorsFilter is unusable without sector layers", "[filter][sectors]")
{
    auto mock = createStandardMock();
    auto cfg  = makeFilterConfig("SectorsFilter", "SectorsFilter0", {
        {"active", true},
        {"layers", json::array({"L1"})}
    });

    SectorsFilter filter(cfg, nullptr, mock);

    //no layers anywhere
    CHECK_FALSE(filter.unusable());
    CHECK(filter.updateUsability());
    CHECK(filter.unusable());
    CHECK_FALSE(filter.filters("CAT048"));
    CHECK(filter.getClause("CAT048").empty());

    //layers appear
    filter.cache().setLayer("L1", layerOf(squareSector(1, "S1", 47.0, 48.0, 15.0, 16.0)));

    CHECK(filter.updateUsability());
    CHECK_FALSE(filter.unusable());
    CHECK(filter.filters("CAT048"));
    CHECK_FALSE(filter.getClause("CAT048").empty());

    //unchanged
    CHECK_FALSE(filter.updateUsability());
}

TEST_CASE("SectorsFilter view point conditions", "[filter][sectors]")
{
    auto mock = createStandardMock();
    auto cfg  = makeFilterConfig("SectorsFilter", "SectorsFilter0", {
        {"layers", json::array({"L1"})},
        {"use_delta", true},
        {"delta_deg", 0.05}
    });

    SectorsFilter filter(cfg, nullptr, mock);

    json filters;
    filter.saveViewPointConditions(filters);

    REQUIRE(filters.contains("Sectors"));
    CHECK(filters["Sectors"]["Layers"] == json::array({"L1"}));
    CHECK(filters["Sectors"]["Use Surrounding Delta"].get<bool>() == true);
    CHECK(filters["Sectors"]["Surrounding Delta"].get<double>() == Approx(0.05));

    auto cfg2 = makeFilterConfig("SectorsFilter", "SectorsFilter1");
    SectorsFilter other(cfg2, nullptr, mock);

    //without a filter manager the cache layers stand in for the context layers
    other.cache().setLayer("L1", layerOf(squareSector(1, "S1", 47.0, 48.0, 15.0, 16.0)));

    other.loadViewPointConditions(filters);

    CHECK(other.selectedLayers() == std::vector<std::string>{"L1"});
    CHECK(other.useDelta());
    CHECK(other.deltaDeg() == Approx(0.05));

    //layers not in the context are dropped
    json unknown = filters;
    unknown["Sectors"]["Layers"] = json::array({"L1", "Old"});

    other.loadViewPointConditions(unknown);

    CHECK(other.selectedLayers() == std::vector<std::string>{"L1"});

    //missing keys fall back to defaults, the delta value stays
    json minimal;
    minimal["Sectors"] = json::object();

    other.loadViewPointConditions(minimal);

    CHECK(other.selectedLayers().empty());
    CHECK_FALSE(other.useDelta());
    CHECK(other.deltaDeg() == Approx(0.05));
}

TEST_CASE("SectorsFilter filterBuffer", "[filter][buffer][sectors]")
{
    auto mock = createStandardMock();
    auto cfg  = makeFilterConfig("SectorsFilter", "SectorsFilter0", {
        {"active", true},
        {"layers", json::array({"L1"})}
    });

    SectorsFilter filter(cfg, nullptr, mock);

    filter.cache().setLayer("L1", layerOf(squareSector(1, "S1", 47.0, 48.0, 15.0, 16.0)));

    const std::string lat_name = mock.metaGetVariableName("CAT048", dbcontent_vars::meta_var_latitude_);
    const std::string lon_name = mock.metaGetVariableName("CAT048", dbcontent_vars::meta_var_longitude_);

    PropertyList pl;
    pl.addProperty(lat_name, PropertyDataType::DOUBLE);
    pl.addProperty(lon_name, PropertyDataType::DOUBLE);

    auto buffer = std::make_shared<Buffer>(pl, "CAT048");

    auto& lat = buffer->get<double>(lat_name);
    auto& lon = buffer->get<double>(lon_name);

    lat.set(0, 47.5);  lon.set(0, 15.5);   //inside
    lat.set(1, 49.0);  lon.set(1, 15.5);   //far outside
    lat.set(2, 48.05); lon.set(2, 15.5);   //0.05 deg north of the sector
    lat.setNull(3);    lon.set(3, 15.5);   //no position

    REQUIRE(buffer->size() == 4);

    SECTION("plain containment")
    {
        auto removed = filter.filterBuffer("CAT048", buffer);
        CHECK(removed == std::vector<unsigned int>{1, 2, 3});
    }

    SECTION("surrounding delta keeps the near report")
    {
        filter.useDelta(true);
        filter.deltaDeg(0.1);

        auto removed = filter.filterBuffer("CAT048", buffer);
        CHECK(removed == std::vector<unsigned int>{1, 3});
    }

    SECTION("no selected layer means no constraint")
    {
        filter.selectLayer("L1", false);

        auto removed = filter.filterBuffer("CAT048", buffer);
        CHECK(removed.empty());
    }

    SECTION("content without position is not filtered")
    {
        auto removed = filter.filterBuffer("CAT002", buffer);
        CHECK(removed.empty());
    }
}
