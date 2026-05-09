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

#include "configurable.h"
#include "configuration.h"
#include "dimension.h"
#include "unit.h"
#include "unitmanager.h"
#include "json.hpp"

using nlohmann::json;

namespace
{

/**
 * Builds a minimal config JSON with class_name and instance_name.
 */
static json makeConfig(const std::string& class_name,
                       const std::string& instance_name,
                       json params = json())
{
    json cfg;
    cfg[Configuration::CLASS_NAME_KEY]    = class_name;
    cfg[Configuration::INSTANCE_NAME_KEY] = instance_name;
    if (!params.is_null() && !params.empty())
        cfg["parameters"] = params;
    return cfg;
}

/**
 * Minimal Configurable for use as a parent in path tests.
 */
struct MinCfg : public Configurable
{
    MinCfg(nlohmann::json& cfg, Configurable* parent = nullptr)
        : Configurable(cfg, parent) {}
    void checkSubConfigurables() override {}
};

} // anonymous namespace

// ---------------------------------------------------------------------------
// Dimension (interior: owns Unit children)
// ---------------------------------------------------------------------------

TEST_CASE("Dimension construction", "[interior][dimension]")
{
    SECTION("creates Unit children from sub_configs")
    {
        json cfg = makeConfig("Dimension", "Length");
        cfg["sub_configs"] = {{"Unit", {
            {"Metre",     {{"parameters", {{"definition", "base unit"}, {"factor", 1.0}}}}},
            {"Kilometre", {{"parameters", {{"definition", ""},          {"factor", 0.001}}}}}
        }}};

        Dimension dim(cfg, nullptr);

        REQUIRE(dim.units().size() == 2);
        REQUIRE(dim.hasUnit("Metre"));
        REQUIRE(dim.hasUnit("Kilometre"));
        REQUIRE(dim.units().at("Metre")->factor() == Approx(1.0));
        REQUIRE(dim.units().at("Kilometre")->factor() == Approx(0.001));
    }

    SECTION("child paths derive from parent")
    {
        json cfg = makeConfig("Dimension", "Time");
        cfg["sub_configs"] = {{"Unit", {
            {"Second", {{"parameters", {{"definition", ""}, {"factor", 1.0}}}}}
        }}};

        Dimension dim(cfg, nullptr);

        REQUIRE(dim.getPath() == "Time");
        REQUIRE(dim.units().at("Second")->getPath() == "Time.Second");
    }

    SECTION("getFactor works for json-backed units")
    {
        json cfg = makeConfig("Dimension", "Length");
        cfg["sub_configs"] = {{"Unit", {
            {"Metre", {{"parameters", {{"factor", 1.0}}}}},
            {"Foot",  {{"parameters", {{"factor", 0.3048}}}}}
        }}};

        Dimension dim(cfg, nullptr);

        // 1 Metre = 1/0.3048 Feet ≈ 3.28084
        double factor = dim.getFactor("Metre", "Foot");
        REQUIRE(factor == Approx(0.3048));
    }

    SECTION("empty sub_configs creates no children")
    {
        json cfg = makeConfig("Dimension", "Empty");
        Dimension dim(cfg, nullptr);

        REQUIRE(dim.units().empty());
    }

    SECTION("write-back preserves children in json")
    {
        json cfg = makeConfig("Dimension", "Length");
        cfg["sub_configs"] = {{"Unit", {
            {"Metre", {{"parameters", {{"definition", "base"}, {"factor", 1.0}}}}}
        }}};

        Dimension dim(cfg, nullptr);

        // Write back the child's config
        for (const auto& [name, unit] : dim.units())
            unit->writeBackConfig();

        // Sub-configs live in storage, not backing json - use generateJSON to verify
        json output;
        dim.generateJSON(output, Configuration::JSONExportType::General);

        auto* metre = Configuration::findSubConfigEntry(output, "Unit", "Metre");
        REQUIRE(metre != nullptr);
        REQUIRE((*metre)["parameters"]["factor"].get<double>() == Approx(1.0));
        REQUIRE((*metre)["parameters"]["definition"].get<std::string>() == "base");
    }
}

// ---------------------------------------------------------------------------
// UnitManager (interior: owns Dimension children, which own Unit children)
// ---------------------------------------------------------------------------

// Helper: builds a json config with all 5 required dimensions so checkSubConfigurables is a no-op
static json makeFullUnitManagerConfig()
{
    auto unit = [](double f) { return json{{"parameters", {{"factor", f}}}}; };

    json cfg;
    cfg[Configuration::CLASS_NAME_KEY]    = "UnitManager";
    cfg[Configuration::INSTANCE_NAME_KEY] = "UnitManager0";
    cfg["sub_configs"] = {
        {"Dimension", {
            {"Angle",  {{"sub_configs", {{"Unit", {{"Degree", unit(1.0)}, {"Radian", unit(M_PI / 180.0)}}}}}}},
            {"Length", {{"sub_configs", {{"Unit", {{"Meter", unit(1.0)}, {"Nautical Mile", unit(1.0 / 1852.0)}}}}}}},
            {"Height", {{"sub_configs", {{"Unit", {{"Feet", unit(1.0)}, {"Meter", unit(0.3048)}}}}}}},
            {"Time",   {{"sub_configs", {{"Unit", {{"Second", unit(1.0)}, {"Minute", unit(1.0 / 60.0)}}}}}}},
            {"Speed",  {{"sub_configs", {{"Unit", {{"Knots", unit(1.0)}, {"Meter/Second", unit(0.514444)}}}}}}}
        }}
    };
    return cfg;
}

TEST_CASE("UnitManager construction", "[interior][unitmanager]")
{
    SECTION("creates Dimension and Unit children from nested json")
    {
        json cfg = makeFullUnitManagerConfig();

        UnitManager um(cfg, nullptr);

        REQUIRE(um.dimensions().size() == 5);
        REQUIRE(um.hasDimension("Angle"));
        REQUIRE(um.hasDimension("Length"));
        REQUIRE(um.hasDimension("Height"));
        REQUIRE(um.hasDimension("Time"));
        REQUIRE(um.hasDimension("Speed"));

        const Dimension& angle = um.dimension("Angle");
        REQUIRE(angle.hasUnit("Degree"));
        REQUIRE(angle.hasUnit("Radian"));
        REQUIRE(angle.units().at("Degree")->factor() == Approx(1.0));

        const Dimension& length = um.dimension("Length");
        REQUIRE(length.hasUnit("Meter"));
        REQUIRE(length.hasUnit("Nautical Mile"));
    }

    SECTION("full path chain propagates through 3 levels")
    {
        json cfg = makeFullUnitManagerConfig();

        UnitManager um(cfg, nullptr);

        REQUIRE(um.getPath() == "UnitManager0");

        const Dimension& time_dim = um.dimension("Time");
        REQUIRE(time_dim.getPath() == "UnitManager0.Time");
        REQUIRE(time_dim.units().at("Second")->getPath() == "UnitManager0.Time.Second");
    }


    SECTION("checkSubConfigurables adds defaults for missing dimensions")
    {
        json cfg = makeConfig("UnitManager", "UnitManager0");
        UnitManager um(cfg, nullptr);

        // checkSubConfigurables creates 5 default dimensions via legacy path
        REQUIRE(um.dimensions().size() == 5);
        REQUIRE(um.hasDimension("Angle"));
        REQUIRE(um.hasDimension("Length"));
        REQUIRE(um.hasDimension("Height"));
        REQUIRE(um.hasDimension("Time"));
        REQUIRE(um.hasDimension("Speed"));
    }
}

// ---------------------------------------------------------------------------
// generateJSON correctness
// ---------------------------------------------------------------------------

TEST_CASE("generateJSON produces complete nested output", "[interior][generatejson]")
{
    SECTION("Dimension generateJSON includes Unit sub_configs")
    {
        json cfg = makeConfig("Dimension", "Length");
        cfg["sub_configs"] = {{"Unit", {
            {"Metre",     {{"parameters", {{"definition", "base unit"}, {"factor", 1.0}}}}},
            {"Kilometre", {{"parameters", {{"definition", ""},          {"factor", 0.001}}}}}
        }}};

        Dimension dim(cfg, nullptr);

        // writeBackConfigRecursive ensures sub_config_storage_ is up to date
        dim.writeBackConfigRecursive();

        json output;
        dim.generateJSON(output, Configuration::JSONExportType::General);

        REQUIRE(output.contains("sub_configs"));
        REQUIRE(output["sub_configs"].is_array());
        REQUIRE(output["sub_configs"].size() == 2);

        auto* metre = Configuration::findSubConfigEntry(output, "Unit", "Metre");
        auto* km    = Configuration::findSubConfigEntry(output, "Unit", "Kilometre");
        REQUIRE(metre != nullptr);
        REQUIRE(km != nullptr);
        REQUIRE((*metre)["parameters"]["factor"].get<double>() == Approx(1.0));
        REQUIRE((*km)["parameters"]["factor"].get<double>() == Approx(0.001));
    }

    SECTION("UnitManager generateJSON includes nested Dimension and Unit sub_configs")
    {
        json cfg = makeFullUnitManagerConfig();
        UnitManager um(cfg, nullptr);

        // writeBackConfigRecursive rebuilds the full tree bottom-up
        um.writeBackConfigRecursive();

        json output;
        um.generateJSON(output, Configuration::JSONExportType::General);

        REQUIRE(output.contains("sub_configs"));

        // Find the Angle dimension in the output
        auto* angle = Configuration::findSubConfigEntry(output, "Dimension", "Angle");
        REQUIRE(angle != nullptr);

        // Angle dimension must contain its Unit sub_configs (nested level)
        REQUIRE(angle->contains("sub_configs"));
        auto* degree = Configuration::findSubConfigEntry(*angle, "Unit", "Degree");
        auto* radian = Configuration::findSubConfigEntry(*angle, "Unit", "Radian");
        REQUIRE(degree != nullptr);
        REQUIRE(radian != nullptr);
        REQUIRE((*degree)["parameters"]["factor"].get<double>() == Approx(1.0));
    }

    SECTION("generateJSON without writeBackConfigRecursive produces shallow output")
    {
        json cfg = makeFullUnitManagerConfig();
        UnitManager um(cfg, nullptr);

        // Deliberately skip writeBackConfigRecursive - sub_configs are shallow
        json output;
        um.generateJSON(output, Configuration::JSONExportType::General);

        REQUIRE(output.contains("sub_configs"));

        // Dimension entries exist but their Unit sub_configs may be absent
        // because the storage was consumed during construction
        auto* angle = Configuration::findSubConfigEntry(output, "Dimension", "Angle");
        REQUIRE(angle != nullptr);

        // Without writeBackConfigRecursive, nested sub_configs are not guaranteed
        // (they depend on whether storage was rebuilt). This documents the current behavior.
        // The key point: writeBackConfigRecursive makes them complete.
    }

    SECTION("generateJSON round-trips through a new Configurable")
    {
        // Build a UnitManager, export its JSON, then construct a new one from that JSON
        json cfg = makeFullUnitManagerConfig();
        UnitManager um(cfg, nullptr);

        um.writeBackConfigRecursive();

        json exported;
        um.generateJSON(exported, Configuration::JSONExportType::General);

        // Construct a second UnitManager from the exported JSON
        UnitManager um2(exported, nullptr);

        REQUIRE(um2.dimensions().size() == 5);
        REQUIRE(um2.hasDimension("Angle"));
        REQUIRE(um2.dimension("Angle").hasUnit("Degree"));
        REQUIRE(um2.dimension("Angle").hasUnit("Radian"));
        REQUIRE(um2.dimension("Angle").units().at("Degree")->factor() == Approx(1.0));
    }
}

