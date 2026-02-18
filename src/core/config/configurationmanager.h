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
#include <vector>

#include "configuration.h"
#include "singleton.h"

class Configurable;

/**
 * @brief Main class for configuration loading, generating and writing.
 *
 * @details Singleton, parses main configuration file using parseConfigurationFile. Each XML
 * configuration file has to consist of a file section (where additional configuration files are
 * defined) and configuration section where all configurations and their sub-configurations are
 * defined.
 *
 * Holds Configurations for all root configurables (no parent). When such a root registers itself,
 * it receives its configuration reference, which is iteratively passed on to its children. When all
 * configurations should be saved, all root configurables pass on their configuration, and unused
 * (root) configurations are saved unmodified.
 *
 */
class ConfigurationManager : public Singleton
{
  public:
    void init(const std::string& main_config_filename);

    /// @brief Destructor
    virtual ~ConfigurationManager();

    /// @brief Registers a configurable as root (no parent)
    Configuration& registerRootConfigurable(Configurable& configurable);
    /// @brief Unregisters a configurable as root
    void unregisterRootConfigurable(Configurable& configurable);

    /// @brief Saves the current configuration
    void saveConfiguration();

    /// @brief Returns singleton instance
    static ConfigurationManager& getInstance()
    {
        static ConfigurationManager instance;
        return instance;
    }

    /// @brief Returns a dummy configuration which is discarded
    // Configuration &getDummyConfiguration () { return dummy_configuration_; }

    bool hasRootConfiguration(const std::string& class_id, const std::string& instance_id);
    Configuration& getRootConfiguration(const std::string& class_id, const std::string& instance_id);

  protected:
    bool initialized_;
    std::string main_config_filename_;
    /// Container with all root configurables (class id, instance id) -> Configurable
    std::map<std::pair<std::string, std::string>, Configurable&> root_configurables_;
    /// Container with all root configurations (class id, instance id) -> Configuration
    std::map<std::pair<std::string, std::string>, std::unique_ptr<Configuration>> root_configurations_;
    // Configuration dummy_configuration_;

    /// @brief Constructor
    ConfigurationManager();

    /// @brief Parses a configuration file
    void parseJSONConfigurationFile(const std::string& filename);

    void saveJSONConfiguration();
};
