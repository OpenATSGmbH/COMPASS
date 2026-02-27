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

#pragma once

#include <string>
#include <vector>
#include <map>

#include "json.hpp"

/**
 * @brief Wraps a JSON config tree, optionally loaded from a file.
 *
 * Three construction modes:
 * - ConfigJSON(conf_filename): loads from CURRENT_CONF_DIRECTORY + conf_filename,
 *   resolves sub_config_files recursively, save() writes back to the same location.
 * - ConfigJSON(json&): wraps an existing json reference (non-owning), save() is a no-op.
 * - ConfigJSON::fromFile(abs_path): loads from an absolute file path,
 *   resolves sub_config_files recursively, save() writes back to that path.
 *
 * After loading, the json() accessor provides a fully-merged tree where all
 * sub_config_files have been inlined into sub_configs.
 */
class ConfigJSON
{
public:
    /// Metadata about a sub-config that was loaded from a separate file
    struct SubConfigFile
    {
        std::string class_name;
        std::string instance_name;
        std::string filename;
        std::vector<SubConfigFile> children;  ///< nested sub_config_files within this child
    };

    /// JSON keys for file-based sub-config references
    static const std::string SubConfigFileSection;
    static const std::string SubConfigFilePath;

    /// Load json from CURRENT_CONF_DIRECTORY + conf_filename, resolving sub_config_files recursively
    explicit ConfigJSON(const std::string& conf_filename);

    /// Wrap an existing json reference (non-owning, save() is a no-op)
    explicit ConfigJSON(nlohmann::json& json);

    /// Load from an absolute file path
    static ConfigJSON fromFile(const std::string& abs_path);

    ConfigJSON(const ConfigJSON&) = delete;
    ConfigJSON& operator=(const ConfigJSON&) = delete;
    ConfigJSON(ConfigJSON&& other) noexcept;
    ConfigJSON& operator=(ConfigJSON&&) = delete;

    /// Access the json tree
    nlohmann::json& json() { return *json_ptr_; }
    const nlohmann::json& json() const { return *json_ptr_; }

    /// The conf filename (empty for non-conf-dir sources)
    const std::string& filename() const { return filename_; }

    /// Whether this config is backed by a file
    bool isFileBacked() const { return file_backed_; }

    /// The tracked sub_config_files at this level
    const std::vector<SubConfigFile>& subConfigFiles() const { return sub_config_files_; }

    /// Save the json tree back to file(s), reconstructing sub_config_files.
    /// No-op if not file-backed.
    void save() const;

private:
    struct FromFileTag {};
    ConfigJSON(const std::string& abs_path, FromFileTag);

    /// Load json from a full file path and resolve sub_config_files
    void loadFromPath(const std::string& path);

    nlohmann::json owned_json_;
    nlohmann::json* json_ptr_{&owned_json_};
    std::string filename_;
    std::string save_path_;
    bool file_backed_{false};
    std::vector<SubConfigFile> sub_config_files_;

    /// Recursively resolve sub_config_files into sub_configs, tracking file origins
    static void resolveSubConfigFiles(nlohmann::json& json,
                                      std::vector<SubConfigFile>& file_info);

    /// Recursively save json to file, splitting out file-backed sub_configs
    static void saveToFile(const nlohmann::json& json,
                           const std::string& file_path,
                           const std::vector<SubConfigFile>& file_info);
};
