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
#include "jsonobjectparser.h"
#include "jsondatamapping.h"
#include "jsonparsingschema.h"
#include "fftmanager.h"
#include "configurationfft.h"
#include "datasourcemanager.h"
#include "configurationdatasource.h"
#include "json.hpp"

using nlohmann::json;

namespace
{

/**
 * Builds a minimal config JSON with class_name and instance_name.
 */
static json makeConfig(const std::string& class_id,
                       const std::string& instance_id,
                       json params = json())
{
    json cfg;
    cfg[Configuration::CLASS_NAME_KEY]    = class_id;
    cfg[Configuration::INSTANCE_NAME_KEY] = instance_id;
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

        // Sub-configs live in storage, not backing json — use generateJSON to verify
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
// JSONObjectParser (interior: owns JSONDataMapping children)
// ---------------------------------------------------------------------------

// Helper: builds a minimal mapping entry
static json makeMapping(const std::string& json_key, const std::string& dbcontent_name,
                        const std::string& dbcontent_var = "")
{
    json m = {
        {"parameters", {
            {"json_key", json_key},
            {"dbcontent_name", dbcontent_name},
            {"active", true},
            {"mandatory", false}
        }}
    };
    if (!dbcontent_var.empty())
        m["parameters"]["dbcontent_variable_name"] = dbcontent_var;
    return m;
}

TEST_CASE("JSONObjectParser construction", "[interior][jsonobjectparser]")
{
    SECTION("creates JSONDataMapping children from sub_configs")
    {
        json cfg = makeConfig("JSONObjectParser", "CAT048Parser", {
            {"name", "CAT048"}, {"db_content_name", "CAT048"},
            {"json_key", "*"}, {"json_value", ""}
        });
        cfg["sub_configs"] = {{"JSONDataMapping", {
            {"mapping0", makeMapping("010.SAC", "CAT048")},
            {"mapping1", makeMapping("010.SIC", "CAT048")},
            {"mapping2", makeMapping("140.Time-of-Day", "CAT048")}
        }}};

        JSONObjectParser parser(cfg, nullptr);

        REQUIRE(parser.name() == "CAT048");
        REQUIRE(parser.dbContentName() == "CAT048");
        REQUIRE(parser.hasMapping(0));
        REQUIRE(parser.hasMapping(1));
        REQUIRE(parser.hasMapping(2));
        REQUIRE(!parser.hasMapping(3));
    }

    SECTION("children are json-backed too")
    {
        json cfg = makeConfig("JSONObjectParser", "TestParser", {
            {"name", "TestParser"}, {"db_content_name", "CAT062"}
        });
        cfg["sub_configs"] = {{"JSONDataMapping", {
            {"m0", makeMapping("070.Mode3A", "CAT062")}
        }}};

        JSONObjectParser parser(cfg, nullptr);

        REQUIRE(parser.hasMapping(0));
        // Access the mapping through the iterator
        auto it = parser.begin();
    }

    SECTION("child paths derive from parent")
    {
        json cfg = makeConfig("JSONObjectParser", "TestParser", {
            {"name", "TestParser"}, {"db_content_name", "CAT021"}
        });
        cfg["sub_configs"] = {{"JSONDataMapping", {
            {"posMapping", makeMapping("130.Latitude", "CAT021")}
        }}};

        JSONObjectParser parser(cfg, nullptr);

        REQUIRE(parser.getPath() == "TestParser");
        REQUIRE((*parser.begin())->getPath() == "TestParser.posMapping");
    }

    SECTION("empty sub_configs creates no children")
    {
        json cfg = makeConfig("JSONObjectParser", "EmptyParser", {
            {"name", "EmptyParser"}, {"db_content_name", "CAT048"}
        });

        JSONObjectParser parser(cfg, nullptr);

        REQUIRE(!parser.hasMapping(0));
        REQUIRE(parser.begin() == parser.end());
    }

    SECTION("parameters read from json correctly")
    {
        json cfg = makeConfig("JSONObjectParser", "MyParser", {
            {"name", "MyParser"}, {"active", false},
            {"db_content_name", "CAT062"},
            {"json_container_key", "target_reports"},
            {"json_key", "message_type"}, {"json_value", "target_report"}
        });

        JSONObjectParser parser(cfg, nullptr);

        REQUIRE(parser.name() == "MyParser");
        REQUIRE(!parser.active());
        REQUIRE(parser.dbContentName() == "CAT062");
        REQUIRE(parser.JSONContainerKey() == "target_reports");
        REQUIRE(parser.JSONKey() == "message_type");
        REQUIRE(parser.JSONValue() == "target_report");
    }

    SECTION("write-back preserves children in json")
    {
        json cfg = makeConfig("JSONObjectParser", "WBParser", {
            {"name", "WBParser"}, {"db_content_name", "CAT048"}
        });
        cfg["sub_configs"] = {{"JSONDataMapping", {
            {"m0", makeMapping("010.SAC", "CAT048")}
        }}};

        JSONObjectParser parser(cfg, nullptr);

        // Write back the child's config
        auto it = parser.begin();
        (*it)->writeBackConfig();

        // Sub-configs live in storage, not backing json — use generateJSON to verify
        json output;
        parser.generateJSON(output, Configuration::JSONExportType::General);

        auto* m0 = Configuration::findSubConfigEntry(output, "JSONDataMapping", "m0");
        REQUIRE(m0 != nullptr);
        REQUIRE((*m0)["parameters"]["json_key"].get<std::string>() == "010.SAC");
        REQUIRE((*m0)["parameters"]["dbcontent_name"].get<std::string>() == "CAT048");
    }
}

// ---------------------------------------------------------------------------
// FFTManager (interior: owns ConfigurationFFT children)
// ---------------------------------------------------------------------------

static json makeConfigFFT(const std::string& name, const std::string& info = "")
{
    return {{"parameters", {{"name", name}, {"info", info}}}};
}

TEST_CASE("FFTManager construction", "[interior][fftmanager]")
{
    SECTION("creates ConfigurationFFT children from sub_configs")
    {
        json cfg = makeConfig("FFTManager", "FFTManager0");
        cfg["sub_configs"] = {{"ConfigurationFFT", {
            {"fft0", makeConfigFFT("TestFFT1")},
            {"fft1", makeConfigFFT("TestFFT2", "some info")}
        }}};

        FFTManager mgr(cfg, nullptr);

        REQUIRE(mgr.hasConfigFFT("TestFFT1"));
        REQUIRE(mgr.hasConfigFFT("TestFFT2"));
        REQUIRE(mgr.configFFTs().size() == 2);
    }

    SECTION("children are json-backed too")
    {
        json cfg = makeConfig("FFTManager", "FFTManager0");
        cfg["sub_configs"] = {{"ConfigurationFFT", {
            {"fft0", makeConfigFFT("MyFFT")}
        }}};

        FFTManager mgr(cfg, nullptr);

        REQUIRE(mgr.configFFTs().size() == 1);
    }

    SECTION("child paths derive from parent")
    {
        json cfg = makeConfig("FFTManager", "FFTManager0");
        cfg["sub_configs"] = {{"ConfigurationFFT", {
            {"fft0", makeConfigFFT("PathFFT")}
        }}};

        FFTManager mgr(cfg, nullptr);

        REQUIRE(mgr.getPath() == "FFTManager0");
        REQUIRE(mgr.configFFTs().front()->getPath() == "FFTManager0.fft0");
    }

    SECTION("empty config creates no children")
    {
        json cfg = makeConfig("FFTManager", "FFTManager0");
        FFTManager mgr(cfg, nullptr);

        REQUIRE(mgr.configFFTs().empty());
    }
}

// ---------------------------------------------------------------------------
// DataSourceManager (interior: owns ConfigurationDataSource children)
// ---------------------------------------------------------------------------

static json makeConfigDS(const std::string& ds_type, unsigned int sac, unsigned int sic,
                         const std::string& name)
{
    return {{"parameters", {
        {"ds_type", ds_type},
        {"sac", sac},
        {"sic", sic},
        {"name", name}
    }}};
}

TEST_CASE("DataSourceManager construction", "[interior][datasourcemanager]")
{
    SECTION("creates ConfigurationDataSource children from sub_configs")
    {
        json cfg = makeConfig("DataSourceManager", "DataSourceManager0");
        cfg["sub_configs"] = {{"ConfigurationDataSource", {
            {"ds0", makeConfigDS("Radar", 1, 2, "Radar1")},
            {"ds1", makeConfigDS("MLAT", 3, 4, "MLAT1")}
        }}};

        DataSourceManager mgr(cfg, nullptr);

        REQUIRE(mgr.configDataSources().size() == 2);
    }

    SECTION("children are json-backed too")
    {
        json cfg = makeConfig("DataSourceManager", "DataSourceManager0");
        cfg["sub_configs"] = {{"ConfigurationDataSource", {
            {"ds0", makeConfigDS("Radar", 10, 20, "TestRadar")}
        }}};

        DataSourceManager mgr(cfg, nullptr);

        REQUIRE(mgr.configDataSources().size() == 1);
    }

    SECTION("child paths derive from parent")
    {
        json cfg = makeConfig("DataSourceManager", "DataSourceManager0");
        cfg["sub_configs"] = {{"ConfigurationDataSource", {
            {"ds0", makeConfigDS("Radar", 5, 6, "PathDS")}
        }}};

        DataSourceManager mgr(cfg, nullptr);

        REQUIRE(mgr.getPath() == "DataSourceManager0");
        REQUIRE(mgr.configDataSources().front()->getPath() == "DataSourceManager0.ds0");
    }

    SECTION("empty config creates no children")
    {
        json cfg = makeConfig("DataSourceManager", "DataSourceManager0");
        DataSourceManager mgr(cfg, nullptr);

        REQUIRE(mgr.configDataSources().empty());
    }

    SECTION("parameters read from json correctly")
    {
        json cfg = makeConfig("DataSourceManager", "DataSourceManager0", {
            {"ds_font_size", 14u},
            {"load_widget_show_counts", true}
        });

        DataSourceManager mgr(cfg, nullptr);

        REQUIRE(mgr.config().ds_font_size_ == 14);
        REQUIRE(mgr.config().load_widget_show_counts_ == true);
    }
}

// ---------------------------------------------------------------------------
// JSONParsingSchema (interior: owns JSONObjectParser children)
// ---------------------------------------------------------------------------

// Helper: builds a JSONObjectParser sub-config entry
static json makeJSONObjectParserConfig(const std::string& name, const std::string& db_content_name)
{
    return {
        {"parameters", {
            {"name", name},
            {"db_content_name", db_content_name}
        }}
    };
}

TEST_CASE("JSONParsingSchema construction", "[interior][jsonparsingschema]")
{
    SECTION("creates JSONObjectParser children from sub_configs")
    {
        json cfg = makeConfig("JSONParsingSchema", "TestSchema", {{"name", "TestSchema"}});
        cfg["sub_configs"] = {{"JSONObjectParser", {
            {"parser0", makeJSONObjectParserConfig("CAT048", "CAT048")},
            {"parser1", makeJSONObjectParserConfig("CAT062", "CAT062")}
        }}};

        JSONParsingSchema schema(cfg, nullptr);

        REQUIRE(schema.name() == "TestSchema");
        REQUIRE(schema.parsers().size() == 2);
        REQUIRE(schema.hasObjectParser("CAT048"));
        REQUIRE(schema.hasObjectParser("CAT062"));
    }

    SECTION("children are json-backed too")
    {
        json cfg = makeConfig("JSONParsingSchema", "Schema1", {{"name", "Schema1"}});
        cfg["sub_configs"] = {{"JSONObjectParser", {
            {"p0", makeJSONObjectParserConfig("CAT021", "CAT021")}
        }}};

        JSONParsingSchema schema(cfg, nullptr);

        REQUIRE(schema.parsers().size() == 1);
        auto it = schema.begin();
    }

    SECTION("child paths derive from parent")
    {
        json cfg = makeConfig("JSONParsingSchema", "PathSchema", {{"name", "PathSchema"}});
        cfg["sub_configs"] = {{"JSONObjectParser", {
            {"p0", makeJSONObjectParserConfig("CAT048", "CAT048")}
        }}};

        JSONParsingSchema schema(cfg, nullptr);

        REQUIRE(schema.getPath() == "PathSchema");
        auto it = schema.begin();
        REQUIRE(it->second->getPath() == "PathSchema.p0");
    }

    SECTION("empty sub_configs creates no children")
    {
        json cfg = makeConfig("JSONParsingSchema", "EmptySchema", {{"name", "EmptySchema"}});

        JSONParsingSchema schema(cfg, nullptr);

        REQUIRE(schema.parsers().empty());
    }

    SECTION("dbcontent_name fallback for parser name")
    {
        json cfg = makeConfig("JSONParsingSchema", "FallbackSchema", {{"name", "FallbackSchema"}});
        cfg["sub_configs"] = {{"JSONObjectParser", {
            {"p0", {{"parameters", {
                {"db_content_name", "CAT048"}
            }}}}
        }}};

        JSONParsingSchema schema(cfg, nullptr);

        REQUIRE(schema.parsers().size() == 1);
        REQUIRE(schema.hasObjectParser("CAT048"));
    }
}
