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

#include "data_source.h"
#include "fft.h"
#include "asterix_decoding_config.h"
#include "db_context.h"
#include "db_context_serializer.h"
#include "db_context_diff.h"

#include <json.hpp>

#include <boost/filesystem.hpp>
#include <fstream>

using namespace context;
using namespace nlohmann;
namespace fs = boost::filesystem;

// ============================================================
// DataSource
// ============================================================

TEST_CASE("DataSource JSON round-trip", "[context]")
{
    DataSource ds;
    ds.dsType("Radar");
    ds.sac(12);
    ds.sic(1);
    ds.name("Malta PSR");
    ds.shortName("MLT");

    json info;
    info["position"]["latitude"] = 35.85;
    info["position"]["longitude"] = 14.48;
    ds.info(info);

    json j = ds.toJSON();
    DataSource ds2 = DataSource::fromJSON(j);

    REQUIRE(ds == ds2);
    REQUIRE(ds2.dsType() == "Radar");
    REQUIRE(ds2.sac() == 12);
    REQUIRE(ds2.sic() == 1);
    REQUIRE(ds2.name() == "Malta PSR");
    REQUIRE(ds2.hasShortName());
    REQUIRE(ds2.shortName() == "MLT");
    REQUIRE(ds2.info()["position"]["latitude"] == Approx(35.85));
}

TEST_CASE("DataSource without short name", "[context]")
{
    DataSource ds;
    ds.dsType("ADSB");
    ds.sac(0);
    ds.sic(0);
    ds.name("ADS-B");

    json j = ds.toJSON();
    REQUIRE_FALSE(j.contains("short_name"));

    DataSource ds2 = DataSource::fromJSON(j);
    REQUIRE_FALSE(ds2.hasShortName());
    REQUIRE(ds == ds2);
}

// ============================================================
// FFT
// ============================================================

TEST_CASE("FFT JSON round-trip", "[context]")
{
    FFT fft;
    fft.name("FFT_MALTA_01");
    fft.latitude(35.9);
    fft.longitude(14.5);
    fft.altitude(100.0);

    json j = fft.toJSON();
    FFT fft2 = FFT::fromJSON(j);

    REQUIRE(fft == fft2);
    REQUIRE(fft2.name() == "FFT_MALTA_01");
    REQUIRE(fft2.hasPosition());
    REQUIRE(fft2.latitude() == Approx(35.9));
    REQUIRE(fft2.longitude() == Approx(14.5));
    REQUIRE(fft2.hasAltitude());
    REQUIRE(fft2.altitude() == Approx(100.0));
}

// ============================================================
// ASTERIXDecodingConfig
// ============================================================

TEST_CASE("ASTERIXDecodingConfig JSON round-trip", "[context]")
{
    ASTERIXDecodingConfig cfg(48, "1.31", "1.4", "");

    json j = cfg.toJSON();
    ASTERIXDecodingConfig cfg2 = ASTERIXDecodingConfig::fromJSON(j);

    REQUIRE(cfg == cfg2);
    REQUIRE(cfg2.category() == 48);
    REQUIRE(cfg2.edition() == "1.31");
    REQUIRE(cfg2.ref() == "1.4");
    REQUIRE(cfg2.spf().empty());
}

// ============================================================
// DBContext
// ============================================================

TEST_CASE("DBContext JSON round-trip", "[context]")
{
    DBContext ctx("test_context");
    ctx.description("A test context");

    // add a data source
    DataSource ds;
    ds.dsType("Radar");
    ds.sac(12);
    ds.sic(1);
    ds.name("Malta PSR");
    ctx.dataSources().push_back(ds);

    // add an FFT
    FFT fft;
    fft.name("FFT_01");
    ctx.ffts().push_back(fft);

    // add an ASTERIX config
    ctx.asterixDecoding().push_back(ASTERIXDecodingConfig(48, "1.31"));

    json j = ctx.toJSON();
    DBContext ctx2 = DBContext::fromJSON(j);

    REQUIRE(ctx2.name() == "test_context");
    REQUIRE(ctx2.description() == "A test context");
    REQUIRE(ctx2.dataSources().size() == 1);
    REQUIRE(ctx2.dataSources()[0].sac() == 12);
    REQUIRE(ctx2.ffts().size() == 1);
    REQUIRE(ctx2.ffts()[0].name() == "FFT_01");
    REQUIRE(ctx2.asterixDecoding().size() == 1);
    REQUIRE(ctx2.asterixDecoding()[0].category() == 48);
}

// ============================================================
// DBContextSerializer
// ============================================================

TEST_CASE("DBContextSerializer save and load", "[context]")
{
    // use a temp directory
    fs::path tmp = fs::temp_directory_path() / fs::unique_path("compass_test_%%%%-%%%%");
    fs::create_directories(tmp);

    SECTION("save and reload a context")
    {
        DBContext ctx("malta_site");
        ctx.description("Malta radar site config");

        DataSource ds;
        ds.dsType("Radar");
        ds.sac(12);
        ds.sic(1);
        ds.name("Malta PSR");
        ctx.dataSources().push_back(ds);

        ctx.asterixDecoding().push_back(ASTERIXDecodingConfig(48, "1.31"));

        DBContextSerializer::save(ctx, tmp.string());

        // verify files exist
        REQUIRE(fs::exists(tmp / "malta_site" / "context_meta.json"));
        REQUIRE(fs::exists(tmp / "malta_site" / "data_sources.json"));
        REQUIRE(fs::exists(tmp / "malta_site" / "ffts.json"));
        REQUIRE(fs::exists(tmp / "malta_site" / "asterix_decoding.json"));
        REQUIRE(fs::exists(tmp / "malta_site" / "sectors.json"));

        // reload
        DBContext loaded = DBContextSerializer::load((tmp / "malta_site").string());
        REQUIRE(loaded.name() == "malta_site");
        REQUIRE(loaded.description() == "Malta radar site config");
        REQUIRE(loaded.dataSources().size() == 1);
        REQUIRE(loaded.dataSources()[0].sac() == 12);
        REQUIRE(loaded.asterixDecoding().size() == 1);
    }

    SECTION("list contexts")
    {
        DBContext ctx1("context_a");
        DBContext ctx2("context_b");
        DBContextSerializer::save(ctx1, tmp.string());
        DBContextSerializer::save(ctx2, tmp.string());

        auto names = DBContextSerializer::listContexts(tmp.string());
        REQUIRE(names.size() == 2);

        // sort for deterministic comparison
        std::sort(names.begin(), names.end());
        REQUIRE(names[0] == "context_a");
        REQUIRE(names[1] == "context_b");
    }

    SECTION("delete context")
    {
        DBContext ctx("to_delete");
        DBContextSerializer::save(ctx, tmp.string());
        REQUIRE(DBContextSerializer::contextExists(tmp.string(), "to_delete"));

        DBContextSerializer::deleteContext(tmp.string(), "to_delete");
        REQUIRE_FALSE(DBContextSerializer::contextExists(tmp.string(), "to_delete"));
    }

    SECTION("rename context")
    {
        DBContext ctx("old_name");
        DBContextSerializer::save(ctx, tmp.string());

        DBContextSerializer::renameContext(tmp.string(), "old_name", "new_name");
        REQUIRE_FALSE(DBContextSerializer::contextExists(tmp.string(), "old_name"));
        REQUIRE(DBContextSerializer::contextExists(tmp.string(), "new_name"));

        DBContext loaded = DBContextSerializer::load((tmp / "new_name").string());
        REQUIRE(loaded.name() == "new_name");
    }

    SECTION("version string is present in files")
    {
        DBContext ctx("versioned");
        DBContextSerializer::save(ctx, tmp.string());

        std::ifstream ifs((tmp / "versioned" / "data_sources.json").string());
        json j;
        ifs >> j;
        REQUIRE(j.contains("version"));
        REQUIRE(j["version"] == "1.0");
        REQUIRE(j.contains("content_type"));
        REQUIRE(j["content_type"] == "data_sources");
    }

    // cleanup
    fs::remove_all(tmp);
}

// ============================================================
// DBContextDiff
// ============================================================

TEST_CASE("DBContextDiff identical contexts", "[context]")
{
    DBContext a("ctx");
    a.dataSources().push_back(DataSource());
    a.dataSources()[0].dsType("Radar");
    a.dataSources()[0].sac(1);
    a.dataSources()[0].sic(2);
    a.dataSources()[0].name("Test");

    DBContext b = a; // copy

    auto diff = DBContextDiff::compute(a, b);
    REQUIRE_FALSE(diff.hasDifferences());
}

TEST_CASE("DBContextDiff added and removed sensors", "[context]")
{
    DBContext a("ctx_a");
    DataSource ds1;
    ds1.dsType("Radar");
    ds1.sac(1);
    ds1.sic(1);
    ds1.name("Sensor A");
    a.dataSources().push_back(ds1);

    DBContext b("ctx_b");
    DataSource ds2;
    ds2.dsType("MLAT");
    ds2.sac(2);
    ds2.sic(2);
    ds2.name("Sensor B");
    b.dataSources().push_back(ds2);

    auto diff = DBContextDiff::compute(a, b);
    REQUIRE(diff.hasSensorDifferences());
    REQUIRE(diff.sensor_diffs.size() == 2);

    // one removed (1/1), one added (2/2)
    int added = 0, removed = 0;
    for (const auto& d : diff.sensor_diffs)
    {
        if (d.type == ItemDiff::Added) ++added;
        if (d.type == ItemDiff::Removed) ++removed;
    }
    REQUIRE(added == 1);
    REQUIRE(removed == 1);
}

TEST_CASE("DBContextDiff modified sensor", "[context]")
{
    DBContext a("ctx");
    DataSource ds;
    ds.dsType("Radar");
    ds.sac(1);
    ds.sic(1);
    ds.name("Original Name");
    a.dataSources().push_back(ds);

    DBContext b = a;
    b.dataSources()[0].name("Changed Name");

    auto diff = DBContextDiff::compute(a, b);
    REQUIRE(diff.hasSensorDifferences());
    REQUIRE(diff.sensor_diffs.size() == 1);
    REQUIRE(diff.sensor_diffs[0].type == ItemDiff::Modified);
    REQUIRE(diff.sensor_diffs[0].key == "1/1");
    REQUIRE_FALSE(diff.sensor_diffs[0].fields.empty());
}

TEST_CASE("DBContextDiff summary", "[context]")
{
    DBContext a("ctx");
    DBContext b("ctx");

    // add an FFT to b only
    FFT fft;
    fft.name("FFT_01");
    b.ffts().push_back(fft);

    auto diff = DBContextDiff::compute(a, b);
    REQUIRE(diff.hasFFTDifferences());

    std::string summary = diff.summary();
    REQUIRE(summary.find("FFTs") != std::string::npos);
    REQUIRE(summary.find("1 added") != std::string::npos);
}
