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
#include "color_provider.h"
#include "db_context.h"
#include "data_source.h"
#include "fft.h"
#include "asterix_decoding_config.h"
#include "sector.h"
#include "logger.h"
#include "traced_assert.h"

#include <json.hpp>

#include <fstream>
#include <set>
#include <vector>

#include <boost/filesystem.hpp>

#include <archive.h>
#include <archive_entry.h>

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
        j["version"] = DATA_SOURCES_VERSION;
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

    // colors.json
    {
        json j;
        j["version"] = CURRENT_VERSION;
        j["content_type"] = "colors";
        j["data"] = ctx.colors().toJSON();
        writeJSONFile(dir + "/" + COLORS_FILENAME, j);
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

    // colors.json (read before data_sources so the preference is available when
    // auto-generating base colors for legacy data sources without color fields)
    {
        string path = context_dir + "/" + COLORS_FILENAME;
        if (fs::exists(path))
        {
            json j = readJSONFile(path);
            j = upgradeSection(j, "colors");
            if (j.contains("data"))
                ctx.colors(ContextColors::fromJSON(j.at("data")));
        }
    }

    // data_sources.json
    {
        string path = context_dir + "/" + SENSORS_FILENAME;
        if (fs::exists(path))
        {
            json j = readJSONFile(path);
            j = upgradeSection(j, "data_sources");

            ColorProvider::Band band =
                (ctx.colors().preference == ContextColors::Preference::Dark)
                    ? ColorProvider::Band::Dark
                    : ColorProvider::Band::Light;

            if (j.contains("data"))
            {
                std::set<unsigned int> seen_ids;
                // existing base colors grouped by ds_type for per-type hue spacing
                std::map<std::string, std::vector<QColor>> existing_by_type;

                for (const auto& ds_j : j.at("data"))
                {
                    auto ds = DataSource::fromJSON(ds_j);
                    unsigned int ds_id = ds.id();

                    if (seen_ids.count(ds_id))
                    {
                        logerr << "duplicate data source SAC/SIC "
                               << ds.sac() << "/" << ds.sic()
                               << " in context '" << ctx.name() << "', skipping duplicate";
                        continue;
                    }

                    // legacy file: no base_color -> autogenerate one using the context
                    // preference, biased toward the DSType's default hue
                    if (!ds.baseColor().isValid())
                    {
                        const auto& same_type = existing_by_type[ds.dsType()];
                        ds.baseColor(ColorProvider::generateBaseColor(same_type, band, ds.dsType()));
                    }

                    existing_by_type[ds.dsType()].push_back(ds.baseColor());

                    seen_ids.insert(ds_id);
                    ctx.dataSources().push_back(std::move(ds));
                }
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

void DBContextSerializer::exportContextZip(const string& base_path,
                                           const string& name,
                                           const string& zip_filepath)
{
    string ctx_dir = contextDir(base_path, name);
    traced_assert(fs::exists(ctx_dir));

    // collect all files in the context directory
    vector<string> files;
    for (const auto& entry : fs::recursive_directory_iterator(ctx_dir))
    {
        if (fs::is_regular_file(entry.status()))
            files.push_back(entry.path().string());
    }

    struct archive* a = archive_write_new();
    archive_write_set_format_zip(a);
    int r = archive_write_open_filename(a, zip_filepath.c_str());
    traced_assert(r == ARCHIVE_OK);

    for (const auto& file_path : files)
    {
        // archive entry path: <context_name>/<filename>
        string rel_path = name + "/" + fs::path(file_path).filename().string();

        // read file contents
        ifstream ifs(file_path, ios::binary | ios::ate);
        traced_assert(ifs.is_open());
        auto size = ifs.tellg();
        ifs.seekg(0);
        vector<char> buf(size);
        ifs.read(buf.data(), size);

        struct archive_entry* entry = archive_entry_new();
        archive_entry_set_pathname(entry, rel_path.c_str());
        archive_entry_set_size(entry, size);
        archive_entry_set_filetype(entry, AE_IFREG);
        archive_entry_set_perm(entry, 0644);

        archive_write_header(a, entry);
        archive_write_data(a, buf.data(), buf.size());
        archive_entry_free(entry);
    }

    archive_write_close(a);
    archive_write_free(a);

    loginf << "exported context '" << name << "' to " << zip_filepath
           << " (" << files.size() << " files)";
}

string DBContextSerializer::importContextZip(const string& base_path,
                                             const string& zip_filepath)
{
    traced_assert(fs::exists(zip_filepath));

    struct archive* a = archive_read_new();
    archive_read_support_format_zip(a);
    archive_read_support_filter_all(a);

    int r = archive_read_open_filename(a, zip_filepath.c_str(), 10240);
    traced_assert(r == ARCHIVE_OK);

    // determine context name from first entry (should be "<name>/something")
    string context_name;

    // extract all entries
    struct archive_entry* entry;
    while (archive_read_next_header(a, &entry) == ARCHIVE_OK)
    {
        string entry_path = archive_entry_pathname(entry);

        // skip directory entries
        if (archive_entry_filetype(entry) == AE_IFDIR)
            continue;

        // extract context name from first path component
        auto slash_pos = entry_path.find('/');
        if (slash_pos == string::npos)
        {
            logwrn << "skipping entry without directory prefix: " << entry_path;
            archive_read_data_skip(a);
            continue;
        }

        string dir_name = entry_path.substr(0, slash_pos);
        string file_name = entry_path.substr(slash_pos + 1);

        if (context_name.empty())
            context_name = dir_name;

        // write to base_path/<context_name>/<filename>
        string out_dir = base_path + "/" + context_name;
        fs::create_directories(out_dir);

        string out_path = out_dir + "/" + file_name;
        ofstream ofs(out_path, ios::binary);
        traced_assert(ofs.is_open());

        const void* buff;
        size_t size;
        la_int64_t offset;
        while (archive_read_data_block(a, &buff, &size, &offset) == ARCHIVE_OK)
            ofs.write(static_cast<const char*>(buff), size);
    }

    archive_read_close(a);
    archive_read_free(a);

    traced_assert(!context_name.empty());

    loginf << "imported context '" << context_name << "' from " << zip_filepath;

    return context_name;
}

} // namespace context
