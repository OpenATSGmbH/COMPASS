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

#include "db_context_serializer.h"
#include "db_context.h"
#include "data_source.h"
#include "fft.h"
#include "asterix_decoding_config.h"
#include "sector.h"
#include "logger.h"
#include "traced_assert.h"

#include <json.hpp>

#include <fstream>

#include <boost/filesystem.hpp>

using namespace std;
using namespace nlohmann;
namespace fs = boost::filesystem;

namespace context
{

namespace
{

json readJSONFile(const string& filepath)
{
    ifstream ifs(filepath);
    traced_assert(ifs.is_open());

    json j;
    ifs >> j;
    return j;
}

void writeJSONFile(const string& filepath, const json& j)
{
    ofstream ofs(filepath);
    traced_assert(ofs.is_open());

    ofs << j.dump(4);
}

/// Upgrade a section's JSON if needed. Currently all at version "1.0", no upgrades required.
json upgradeSection(json j, const string& /*section_name*/)
{
    string version = j.value("version", "1.0");

    // future: if (version < "2.0") j = upgradeTo2(j);

    return j;
}

} // anonymous namespace

string DBContextSerializer::contextDir(const string& base_path, const string& name)
{
    return base_path + "/" + name;
}

void DBContextSerializer::save(const DBContext& ctx, const string& base_path)
{
    string dir = contextDir(base_path, ctx.name());

    // create directory if needed
    fs::create_directories(dir);

    // context_meta.json
    {
        json meta;
        meta["version"] = CURRENT_VERSION;
        meta["name"] = ctx.name();
        meta["description"] = ctx.description();
        meta["created"] = ctx.created();
        meta["modified"] = ctx.modified();
        writeJSONFile(dir + "/" + META_FILENAME, meta);
    }

    // data_sources.json
    {
        json j;
        j["version"] = CURRENT_VERSION;
        j["content_type"] = "data_sources";
        json arr = json::array();
        for (const auto& ds : ctx.dataSources())
            arr.push_back(ds.toJSON());
        j["data"] = arr;
        writeJSONFile(dir + "/" + SENSORS_FILENAME, j);
    }

    // ffts.json
    {
        json j;
        j["version"] = CURRENT_VERSION;
        j["content_type"] = "ffts";
        json arr = json::array();
        for (const auto& fft : ctx.ffts())
            arr.push_back(fft.toJSON());
        j["data"] = arr;
        writeJSONFile(dir + "/" + FFTS_FILENAME, j);
    }

    // asterix_decoding.json
    {
        json j;
        j["version"] = CURRENT_VERSION;
        j["content_type"] = "asterix_decoding";
        json arr = json::array();
        for (const auto& cfg : ctx.asterixDecoding())
            arr.push_back(cfg.toJSON());
        j["data"] = arr;
        writeJSONFile(dir + "/" + ASTERIX_DECODING_FILENAME, j);
    }

    // sectors.json
    {
        json j;
        j["version"] = CURRENT_VERSION;
        j["content_type"] = "sectors";
        json arr = json::array();
        for (const auto& sec : ctx.sectors())
        {
            traced_assert(sec);
            arr.push_back(sec->jsonData());
        }
        j["data"] = arr;
        writeJSONFile(dir + "/" + SECTORS_FILENAME, j);
    }

    loginf << "saved context '" << ctx.name() << "' to " << dir;
}

DBContext DBContextSerializer::load(const string& context_dir)
{
    DBContext ctx;

    // context_meta.json
    {
        string path = context_dir + "/" + META_FILENAME;
        traced_assert(fs::exists(path));

        json meta = readJSONFile(path);
        meta = upgradeSection(meta, "meta");

        ctx.name(meta.at("name"));

        if (meta.contains("description"))
            ctx.description(meta.at("description"));

        if (meta.contains("created"))
            ctx.created(meta.at("created"));

        if (meta.contains("modified"))
            ctx.modified(meta.at("modified"));
    }

    // data_sources.json
    {
        string path = context_dir + "/" + SENSORS_FILENAME;
        if (fs::exists(path))
        {
            json j = readJSONFile(path);
            j = upgradeSection(j, "data_sources");

            if (j.contains("data"))
            {
                for (const auto& ds_j : j.at("data"))
                    ctx.dataSources().push_back(DataSource::fromJSON(ds_j));
            }
        }
    }

    // ffts.json
    {
        string path = context_dir + "/" + FFTS_FILENAME;
        if (fs::exists(path))
        {
            json j = readJSONFile(path);
            j = upgradeSection(j, "ffts");

            if (j.contains("data"))
            {
                for (const auto& fft_j : j.at("data"))
                    ctx.ffts().push_back(FFT::fromJSON(fft_j));
            }
        }
    }

    // asterix_decoding.json
    {
        string path = context_dir + "/" + ASTERIX_DECODING_FILENAME;
        if (fs::exists(path))
        {
            json j = readJSONFile(path);
            j = upgradeSection(j, "asterix_decoding");

            if (j.contains("data"))
            {
                for (const auto& cfg_j : j.at("data"))
                    ctx.asterixDecoding().push_back(ASTERIXDecodingConfig::fromJSON(cfg_j));
            }
        }
    }

    // sectors.json
    {
        string path = context_dir + "/" + SECTORS_FILENAME;
        if (fs::exists(path))
        {
            json j = readJSONFile(path);
            j = upgradeSection(j, "sectors");

            if (j.contains("data"))
            {
                for (const auto& sec_j : j.at("data"))
                {
                    unsigned int id = sec_j.at("id");
                    string name = sec_j.at("name");
                    string layer_name = sec_j.at("layer_name");

                    auto sector = make_shared<Sector>(id, name, layer_name, false);
                    sector->readJSON(sec_j);
                    ctx.sectors().push_back(sector);
                }
            }
        }
    }

    loginf << "loaded context '" << ctx.name() << "' from " << context_dir;

    return ctx;
}

vector<string> DBContextSerializer::listContexts(const string& base_path)
{
    vector<string> names;

    if (!fs::exists(base_path))
        return names;

    for (const auto& entry : fs::directory_iterator(base_path))
    {
        if (fs::is_directory(entry.status()))
        {
            string meta_path = entry.path().string() + "/" + META_FILENAME;
            if (fs::exists(meta_path))
                names.push_back(entry.path().filename().string());
        }
    }

    return names;
}

bool DBContextSerializer::contextExists(const string& base_path, const string& name)
{
    string dir = contextDir(base_path, name);
    return fs::exists(dir + "/" + META_FILENAME);
}

void DBContextSerializer::deleteContext(const string& base_path, const string& name)
{
    string dir = contextDir(base_path, name);
    traced_assert(fs::exists(dir));
    fs::remove_all(dir);

    loginf << "deleted context '" << name << "'";
}

void DBContextSerializer::renameContext(const string& base_path,
                                        const string& old_name,
                                        const string& new_name)
{
    string old_dir = contextDir(base_path, old_name);
    string new_dir = contextDir(base_path, new_name);

    traced_assert(fs::exists(old_dir));
    traced_assert(!fs::exists(new_dir));

    fs::rename(old_dir, new_dir);

    // update the meta file to reflect the new name
    string meta_path = new_dir + "/" + META_FILENAME;
    json meta = readJSONFile(meta_path);
    meta["name"] = new_name;
    writeJSONFile(meta_path, meta);

    loginf << "renamed context '" << old_name << "' to '" << new_name << "'";
}

} // namespace context
