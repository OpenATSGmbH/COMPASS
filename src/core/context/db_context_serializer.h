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

namespace context
{

class DBContext;

/**
 * Handles file I/O and schema versioning for DBContext persistence.
 *
 * Storage layout:
 *   ~/.compass/data_contexts/<name>/
 *       context_meta.json
 *       data_sources.json
 *       ffts.json
 *       asterix_decoding.json
 *       sectors.json
 *
 * Each section file has a "version" string (e.g. "1.0") for backwards-compatible upgrades.
 */
class DBContextSerializer
{
public:
    static constexpr const char* META_FILENAME            = "context_meta.json";
    static constexpr const char* SENSORS_FILENAME         = "data_sources.json";
    static constexpr const char* FFTS_FILENAME            = "ffts.json";
    static constexpr const char* ASTERIX_DECODING_FILENAME = "asterix_decoding.json";
    static constexpr const char* SECTORS_FILENAME         = "sectors.json";
    static constexpr const char* COLORS_FILENAME          = "colors.json";

    static constexpr const char* CURRENT_VERSION            = "1.0";
    static constexpr const char* DATA_SOURCES_VERSION       = "1.1"; // bumped for base_color/line_colors

    /// Save a complete context to directory_path/<name>/
    static void save(const DBContext& ctx, const std::string& base_path);

    /// Load a complete context from directory_path/
    static DBContext load(const std::string& context_dir);

    /// List all context names found under base_path
    static std::vector<std::string> listContexts(const std::string& base_path);

    /// Check if a context directory exists
    static bool contextExists(const std::string& base_path, const std::string& name);

    /// Delete a context directory
    static void deleteContext(const std::string& base_path, const std::string& name);

    /// Rename a context directory
    static void renameContext(const std::string& base_path,
                              const std::string& old_name,
                              const std::string& new_name);

    /// Export a context directory as a zip archive
    static void exportContextZip(const std::string& base_path,
                                 const std::string& name,
                                 const std::string& zip_filepath);

    /// Import a context from a zip archive, returns the context name found inside.
    /// Overwrites the context folder if it already exists.
    static std::string importContextZip(const std::string& base_path,
                                        const std::string& zip_filepath);

private:
    /// Get the directory path for a named context
    static std::string contextDir(const std::string& base_path, const std::string& name);
};

} // namespace context
