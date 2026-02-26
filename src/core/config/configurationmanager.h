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

#include <map>
#include <memory>
#include <vector>

#include "configuration.h"
#include "configjson.h"

class Configurable;

/**
 * @brief Main class for configuration loading, generating and writing.
 *
 * Normal class (no singleton) owned by Client and passed by reference to COMPASS.
 * Reads the main config file (client.json) and creates ConfigJSON objects for each
 * root configurable. Root configurables obtain their json& from ConfigJSON and pass
 * subtrees to their children.
 */
class ConfigurationManager
{
  public:
    using Key = std::pair<std::string, std::string>;

    ConfigurationManager();
    ~ConfigurationManager();
    ConfigurationManager(const ConfigurationManager&) = delete;
    ConfigurationManager& operator=(const ConfigurationManager&) = delete;

    /// Parses the main config file (e.g. client.json) and creates a ConfigJSON
    /// for each root configurable listed in its "sub_config_files" array.
    void init(const std::string& main_config_filename);

    bool hasRootConfigJSON(const std::string& class_id, const std::string& instance_id) const;
    ConfigJSON& getRootConfigJSON(const std::string& class_id, const std::string& instance_id);
    const ConfigJSON& getRootConfigJSON(const std::string& class_id, const std::string& instance_id) const;

    /// Registers a root configurable so its parameters are written back before save.
    void registerJsonRootConfigurable(Configurable& configurable);
    void unregisterJsonRootConfigurable(Configurable& configurable);

    /// Writes back all registered root configurables (bottom-up) and saves their ConfigJSON files.
    void saveConfiguration();

  private:
    bool initialized_{false};
    std::string main_config_filename_;

    /// One ConfigJSON per root configurable, loaded from separate files at init().
    std::map<Key, std::unique_ptr<ConfigJSON>> root_config_jsons_;

    /// Root configurables that have been constructed and registered for save-time writeback.
    std::map<Key, Configurable*> json_root_configurables_;
};
