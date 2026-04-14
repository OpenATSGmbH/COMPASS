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

#include "unit.h"
#include "dimension.h"
#include "asterixcategoryconfig.h"
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
// Unit
// ---------------------------------------------------------------------------

TEST_CASE("Unit construction", "[leaf][unit]")
{
    SECTION("reads parameters from json")
    {
        json cfg = makeConfig("Unit", "millimetre0",
                              {{"definition", "metres"}, {"factor", 0.001}});
        Unit u(cfg, nullptr);

        REQUIRE(u.factor() == Approx(0.001));
    }

    SECTION("uses defaults for empty json")
    {
        json cfg = makeConfig("Unit", "default0");
        Unit u(cfg, nullptr);

        REQUIRE(u.factor() == Approx(1.0));
    }

    SECTION("write-back updates json")
    {
        json cfg = makeConfig("Unit", "foot0",
                              {{"definition", "feet"}, {"factor", 0.3048}});
        Unit u(cfg, nullptr);

        REQUIRE(u.factor() == Approx(0.3048));
        u.writeBackConfig();

        REQUIRE(cfg.contains("parameters"));
        REQUIRE(cfg["parameters"]["factor"].get<double>() == Approx(0.3048));
    }

    SECTION("getPath derives from parent_path")
    {
        json length_cfg = makeConfig("Dimension", "Length0");
        Dimension length(length_cfg, nullptr);

        json cfg = makeConfig("Unit", "metre0");
        Unit u(cfg, &length);

        REQUIRE(u.getPath() == "Length0.metre0");
    }
}

// ---------------------------------------------------------------------------
// ASTERIXCategoryConfig
// ---------------------------------------------------------------------------

TEST_CASE("ASTERIXCategoryConfig construction", "[leaf][asterix]")
{
    SECTION("reads parameters from json")
    {
        json cfg = makeConfig("ASTERIXCategoryConfig", "cat048_0", {
            {"category", 48u}, {"decode", true},
            {"edition", "1.31"}, {"ref", "1.0"}, {"spf", ""}
        });
        ASTERIXCategoryConfig ac(cfg, nullptr);

        REQUIRE(ac.category() == 48u);
        REQUIRE(ac.decode() == true);
        REQUIRE(ac.edition() == "1.31");
        REQUIRE(ac.ref() == "1.0");
        REQUIRE(ac.spf().empty());
    }

    SECTION("uses defaults for empty json")
    {
        json cfg = makeConfig("ASTERIXCategoryConfig", "cat000_0");
        ASTERIXCategoryConfig ac(cfg, nullptr);

        REQUIRE(ac.category() == 0u);
        REQUIRE(ac.decode() == false);
        REQUIRE(ac.edition().empty());
    }

    SECTION("write-back updates json")
    {
        json cfg = makeConfig("ASTERIXCategoryConfig", "cat062_0", {
            {"category", 62u}, {"decode", false},
            {"edition", "1.18"}, {"ref", ""}, {"spf", ""}
        });
        ASTERIXCategoryConfig ac(cfg, nullptr);

        ac.decode(true);
        ac.writeBackConfig();

        REQUIRE(cfg["parameters"]["category"].get<unsigned int>() == 62u);
        REQUIRE(cfg["parameters"]["decode"].get<bool>() == true);
        REQUIRE(cfg["parameters"]["edition"].get<std::string>() == "1.18");
    }

    SECTION("getPath derives from parent_path")
    {
        json parent_cfg = makeConfig("ImportTask", "ImportTask0");
        MinCfg parent(parent_cfg);

        json cfg = makeConfig("ASTERIXCategoryConfig", "cat048_0");
        ASTERIXCategoryConfig ac(cfg, &parent);

        REQUIRE(ac.getPath() == "ImportTask0.cat048_0");
    }
}

