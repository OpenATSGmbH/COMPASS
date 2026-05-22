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

#include "configjson.h"
#include "config_paths.h"
#include "configuration.h"
#include "logger.h"
#include "stringconv.h"
#include "traced_assert.h"

#include <filesystem>
#include <fstream>

using namespace nlohmann;
using namespace Utils;

const std::string ConfigJSON::SubConfigFileSection = "sub_config_files";

// Key used in the legacy object-format sub_config_files entries (read-only, never written)
static const std::string SCF_LEGACY_PATH_KEY = "path";

ConfigJSON::ConfigJSON(const std::string& conf_filename)
    : filename_(conf_filename)
    , save_path_(CURRENT_CONF_DIRECTORY + conf_filename)
    , file_backed_(true)
{
    logdbg << "loading '" << save_path_ << "'";

    loadFromPath(save_path_);
}

ConfigJSON::ConfigJSON(nlohmann::json& json)
    : json_ptr_(&json)
{
}

ConfigJSON::ConfigJSON(const std::string& abs_path, FromFileTag)
    : save_path_(abs_path)
    , file_backed_(true)
{
    logdbg << "loading '" << abs_path << "'";

    loadFromPath(abs_path);
}

ConfigJSON ConfigJSON::fromFile(const std::string& abs_path)
{
    return ConfigJSON(abs_path, FromFileTag{});
}

ConfigJSON::ConfigJSON(ConfigJSON&& other) noexcept
    : owned_json_(std::move(other.owned_json_))
    , json_ptr_(other.json_ptr_ == &other.owned_json_ ? &owned_json_ : other.json_ptr_)
    , filename_(std::move(other.filename_))
    , save_path_(std::move(other.save_path_))
    , file_backed_(other.file_backed_)
    , sub_config_files_(std::move(other.sub_config_files_))
{
}

void ConfigJSON::loadFromPath(const std::string& path)
{
    if (!std::filesystem::exists(path))
        throw std::runtime_error("ConfigJSON: loadFromPath: file '" + path + "' does not exist");

    std::ifstream file(path, std::ifstream::in);

    try
    {
        owned_json_ = json::parse(file);
        traced_assert(owned_json_.is_object());
    }
    catch (json::exception& e)
    {
        logerr << "could not parse file '" << path << "'";
        throw;
    }

    json_ptr_ = &owned_json_;

    // Log top-level keys for debugging
    logdbg << "loaded '" << path << "' top-level keys: [";
    for (auto& [key, _] : owned_json_.items())
        logdbg << " " << key;
    logdbg << " ]";

    // Convert old nested sub_configs format to array before resolving file refs
    Configuration::convertSubConfigsFormat(owned_json_);

    resolveSubConfigFiles(owned_json_, sub_config_files_);

    logdbg << "'" << path << "' load complete"
           << " has_sub_configs " << owned_json_.contains(Configuration::SubConfigSection)
           << " has_sub_config_files " << owned_json_.contains(ConfigJSON::SubConfigFileSection);
}

void ConfigJSON::resolveSubConfigFiles(nlohmann::json& json,
                                       std::vector<SubConfigFile>& file_info)
{
    // First, recurse into existing inline sub_configs entries that may have their own sub_config_files
    if (json.contains(Configuration::SubConfigSection) && json[Configuration::SubConfigSection].is_array())
    {
        for (auto& entry : json[Configuration::SubConfigSection])
        {
            if (entry.is_object())
            {
                std::vector<SubConfigFile> nested_info;
                resolveSubConfigFiles(entry, nested_info);

                if (!nested_info.empty())
                {
                    // Track as inline parent with file-backed children (empty filename = inline)
                    SubConfigFile scf;
                    scf.class_name    = Configuration::getClassName(entry);
                    scf.instance_name = Configuration::getInstanceName(entry);
                    scf.filename    = "";
                    scf.children    = std::move(nested_info);
                    file_info.push_back(std::move(scf));
                }
            }
        }
    }

    // Then resolve sub_config_files at this level
    if (!json.contains(ConfigJSON::SubConfigFileSection))
        return;

    auto& scf_array = json[ConfigJSON::SubConfigFileSection];
    traced_assert(scf_array.is_array());

    logdbg << "resolving " << scf_array.size() << " sub_config_file entries";

    for (auto& entry : scf_array)
    {
        // New format: plain path string.  Legacy format: object with a "path" key.
        std::string path;
        if (entry.is_string())
        {
            path = entry.get<std::string>();
        }
        else if (entry.is_object())
        {
            traced_assert(entry.contains(SCF_LEGACY_PATH_KEY));
            path = entry.at(SCF_LEGACY_PATH_KEY).get<std::string>();
        }
        else
        {
            throw std::runtime_error("sub_config_files entry must be a string path or a legacy object");
        }

        traced_assert(!path.empty());

        // Load the referenced file
        std::string file_path = CURRENT_CONF_DIRECTORY + path;
        if (!std::filesystem::exists(file_path))
            throw std::runtime_error("ConfigJSON: file '" + file_path + "' does not exist");

        std::ifstream file(file_path, std::ifstream::in);
        nlohmann::json child_json;

        try
        {
            child_json = nlohmann::json::parse(file);
            traced_assert(child_json.is_object());
        }
        catch (json::exception& e)
        {
            logerr << "could not parse sub-config file '" << file_path << "'";
            throw;
        }

        // Convert old nested sub_configs format in the loaded child
        Configuration::convertSubConfigsFormat(child_json);

        // class_name and instance_name come from the file itself - no override needed
        auto class_name    = Configuration::getClassName(child_json);
        auto instance_name = Configuration::getInstanceName(child_json);

        traced_assert(!class_name.empty() && !instance_name.empty());

        logdbg << "resolving sub_config_file: class '" << class_name
               << "' instance '" << instance_name << "' path '" << path << "'";

        // Build file info entry
        SubConfigFile scf;
        scf.class_name    = class_name;
        scf.instance_name = instance_name;
        scf.filename    = path;

        // Recursively resolve sub_config_files within the loaded child
        resolveSubConfigFiles(child_json, scf.children);

        if (!json.contains(Configuration::SubConfigSection))
            json[Configuration::SubConfigSection] = nlohmann::json::array();

        json[Configuration::SubConfigSection].push_back(std::move(child_json));

        file_info.push_back(std::move(scf));

        logdbg << "resolved sub_config_file: class '" << class_name
               << "' instance '" << instance_name << "' from '" << path << "'";
    }

    // Remove the sub_config_files array (now resolved into sub_configs)
    json.erase(ConfigJSON::SubConfigFileSection);

    // Log final sub_configs count
    if (json.contains(Configuration::SubConfigSection))
        logdbg << "after resolving: sub_configs array has "
               << json[Configuration::SubConfigSection].size() << " entries";
}

void ConfigJSON::save() const
{
    if (!file_backed_ || save_path_.empty())
    {
        logdbg << "save: skipping, file_backed " << file_backed_
               << " save_path '" << save_path_ << "'";
        return;
    }

    loginf << "save: saving to '" << save_path_ << "'"
           << " num sub files " << sub_config_files_.size();

    try
    {
        saveToFile(*json_ptr_, save_path_, sub_config_files_);
    }
    catch (const std::exception& e)
    {
        logerr << "save: failed saving to '" << save_path_ << "': " << e.what();
        throw;
    }
}

void ConfigJSON::saveToFile(const nlohmann::json& json,
                            const std::string& file_path,
                            const std::vector<SubConfigFile>& file_info)
{
    logdbg << "saving to '" << file_path << "'";

    nlohmann::json output;

    // Copy all top-level keys except sub_configs and sub_config_files
    // (we reconstruct those below)
    for (auto& [key, value] : json.items())
    {
        if (key == Configuration::SubConfigSection ||
            key == ConfigJSON::SubConfigFileSection)
            continue;

        output[key] = value;
    }

    // Build a lookup set for file-backed children at this level
    std::map<std::pair<std::string, std::string>, const SubConfigFile*> file_backed;
    for (const auto& scf : file_info)
        file_backed[{scf.class_name, scf.instance_name}] = &scf;

    // Process sub_configs (array format)
    if (json.contains(Configuration::SubConfigSection) && json[Configuration::SubConfigSection].is_array())
    {
        logdbg << "'" << file_path << "' processing "
               << json[Configuration::SubConfigSection].size() << " sub_config entries"
               << " (" << file_backed.size() << " file-backed)";

        for (auto& child_json : json[Configuration::SubConfigSection])
        {
            try
            {
                auto cid = Configuration::getClassName(child_json);
                auto iid = Configuration::getInstanceName(child_json);

                auto key = std::make_pair(cid, iid);
                auto it  = file_backed.find(key);

                if (it != file_backed.end())
                {
                    if (it->second->filename.empty())
                    {
                        // Inline parent with file-backed children: save inline but
                        // reconstruct sub_config_files for its children
                        logdbg << "'" << file_path << "' inline parent with file-backed children: class '"
                               << cid << "' instance '" << iid
                               << "' children " << it->second->children.size();

                        nlohmann::json inline_entry;

                        // Copy all keys except sub_configs
                        for (auto& [k, v] : child_json.items())
                        {
                            if (k == Configuration::SubConfigSection ||
                                k == ConfigJSON::SubConfigFileSection)
                                continue;
                            inline_entry[k] = v;
                        }

                        // Build file-backed lookup for the inline parent's children
                        std::map<std::pair<std::string, std::string>, const SubConfigFile*> child_file_backed;
                        for (const auto& cscf : it->second->children)
                            child_file_backed[{cscf.class_name, cscf.instance_name}] = &cscf;

                        // Process inline parent's sub_configs
                        if (child_json.contains(Configuration::SubConfigSection) &&
                            child_json[Configuration::SubConfigSection].is_array())
                        {
                            for (auto& grandchild : child_json[Configuration::SubConfigSection])
                            {
                                auto gcid = Configuration::getClassName(grandchild);
                                auto giid = Configuration::getInstanceName(grandchild);
                                auto gkey = std::make_pair(gcid, giid);
                                auto git  = child_file_backed.find(gkey);

                                if (git != child_file_backed.end())
                                {
                                    // File-backed grandchild: save to its own file
                                    saveToFile(grandchild,
                                               CURRENT_CONF_DIRECTORY + git->second->filename,
                                               git->second->children);

                                    if (!inline_entry.contains(ConfigJSON::SubConfigFileSection))
                                        inline_entry[ConfigJSON::SubConfigFileSection] = nlohmann::json::array();

                                    inline_entry[ConfigJSON::SubConfigFileSection].push_back(
                                        git->second->filename);
                                }
                                else
                                {
                                    // Inline grandchild
                                    if (!inline_entry.contains(Configuration::SubConfigSection))
                                        inline_entry[Configuration::SubConfigSection] = nlohmann::json::array();
                                    inline_entry[Configuration::SubConfigSection].push_back(grandchild);
                                }
                            }
                        }

                        if (!output.contains(Configuration::SubConfigSection))
                            output[Configuration::SubConfigSection] = nlohmann::json::array();
                        output[Configuration::SubConfigSection].push_back(std::move(inline_entry));
                    }
                    else
                    {
                        bool child_has_sc = child_json.contains(Configuration::SubConfigSection);
                        size_t child_sc_count = 0;
                        if (child_has_sc && child_json[Configuration::SubConfigSection].is_array())
                            child_sc_count = child_json[Configuration::SubConfigSection].size();

                        logdbg << "'" << file_path << "' saving file-backed child: class '"
                               << cid << "' instance '" << iid
                               << "' -> '" << it->second->filename << "'"
                               << " child_has_sub_configs " << child_has_sc
                               << " child_sub_configs_count " << child_sc_count
                               << " child_file_info_children " << it->second->children.size()
                               << " child_keys [";
                        for (auto& [k, v] : child_json.items())
                            logdbg << " " << k;
                        logdbg << " ]";

                        // File-backed child: save to its own file recursively
                        saveToFile(child_json,
                                   CURRENT_CONF_DIRECTORY + it->second->filename,
                                   it->second->children);

                        // Add sub_config_files entry in output (new format: plain path string)
                        if (!output.contains(ConfigJSON::SubConfigFileSection))
                            output[ConfigJSON::SubConfigFileSection] = nlohmann::json::array();

                        output[ConfigJSON::SubConfigFileSection].push_back(it->second->filename);
                    }
                }
                else
                {
                    logdbg << "'" << file_path << "' inline child: class '"
                           << cid << "' instance '" << iid << "'";

                    // Inline child: include in sub_configs array
                    if (!output.contains(Configuration::SubConfigSection))
                        output[Configuration::SubConfigSection] = nlohmann::json::array();

                    output[Configuration::SubConfigSection].push_back(child_json);
                }
            }
            catch (const std::exception& e)
            {
                logerr << "'" << file_path << "' failed to process sub_config entry: " << e.what()
                       << " entry " << child_json.dump(2).substr(0, 200);
                throw;
            }
        }
    }
    else
    {
        if (!json.contains(Configuration::SubConfigSection))
            logdbg << "'" << file_path << "' no sub_configs key in json";
        else
            logdbg << "'" << file_path << "' sub_configs is not an array, type "
                   << json[Configuration::SubConfigSection].type_name();
    }

    // Log output summary - dump truncated content
    {
        std::string dumped = output.dump(2);
        const size_t max_dump = 2000;
        if (dumped.size() > max_dump)
            dumped = dumped.substr(0, max_dump) + "... [truncated, total " + std::to_string(dumped.size()) + " chars]";
        logdbg << "saving '" << file_path << "' content:\n" << dumped;
    }

    // Write to file
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        logerr << "failed to open file for writing: '" << file_path << "'";
        throw std::runtime_error("ConfigJSON::saveToFile: failed to open '" + file_path + "'");
    }
    file << output.dump(4);
}
