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

#include "configurationmanager.h"
#include "configurable.h"
#include "configjson.h"
#include "files.h"
#include "json.hpp"
#include "logger.h"
#include "traced_assert.h"

#include <fstream>

using namespace nlohmann;
using namespace Utils;

ConfigurationManager::ConfigurationManager()
{
}

void ConfigurationManager::init(const std::string& main_config_filename)
{
    traced_assert(!initialized_);
    traced_assert(main_config_filename.size() > 0);

    main_config_filename_ = main_config_filename;
    initialized_ = true;

    std::string path_filename = CURRENT_CONF_DIRECTORY + main_config_filename;
    Files::verifyFileExists(path_filename);

    loginf << "opening main configuration file '" << path_filename << "'";

    // Parse the main config file (e.g. client.json)
    std::ifstream config_file(path_filename, std::ifstream::in);

    try
    {
        json config = json::parse(config_file);
        traced_assert(config.is_object());

        for (auto& it : config.items())
        {
            if (it.key() == ConfigJSON::SubConfigFileSection)
            {
                traced_assert(it.value().is_array());

                for (auto& file_cfg_it : it.value().get<json::array_t>())
                {
                    // New format: plain path string. Legacy format: object with a "path" key.
                    std::string path;
                    if (file_cfg_it.is_string())
                    {
                        path = file_cfg_it.get<std::string>();
                    }
                    else if (file_cfg_it.is_object())
                    {
                        static const std::string path_key = "path";
                        traced_assert(file_cfg_it.contains(path_key));
                        path = file_cfg_it.at(path_key).get<std::string>();
                    }
                    else
                    {
                        throw std::runtime_error(
                            "ConfigurationManager: init: sub_config_files entry must be a string or object");
                    }

                    traced_assert(!path.empty());

                    auto config_json = std::make_unique<ConfigJSON>(path);

                    auto class_name    = Configuration::getClassName(config_json->json());
                    auto instance_name = Configuration::getInstanceName(config_json->json());

                    traced_assert(!class_name.empty() && !instance_name.empty());

                    Key key{class_name, instance_name};
                    traced_assert(root_config_jsons_.find(key) == root_config_jsons_.end());

                    loginf << "creating ConfigJSON for class '" << class_name
                           << "' instance '" << instance_name
                           << "' from '" << path << "'";

                    root_config_jsons_[key] = std::move(config_json);
                }
            }
            else
            {
                throw std::runtime_error(
                    "ConfigurationManager: init: unknown key '" + it.key() + "'");
            }
        }
    }
    catch (json::exception& e)
    {
        logerr << "could not load file '" << path_filename << "'";
        throw;
    }
}

ConfigurationManager::~ConfigurationManager()
{
    logdbg;
    initialized_ = false;
}

bool ConfigurationManager::hasRootConfigJSON(const std::string& class_name,
                                              const std::string& instance_name) const
{
    return root_config_jsons_.count({class_name, instance_name}) > 0;
}

ConfigJSON& ConfigurationManager::getRootConfigJSON(const std::string& class_name,
                                                     const std::string& instance_name)
{
    traced_assert(hasRootConfigJSON(class_name, instance_name));
    return *root_config_jsons_.at({class_name, instance_name});
}

const ConfigJSON& ConfigurationManager::getRootConfigJSON(const std::string& class_name,
                                                           const std::string& instance_name) const
{
    traced_assert(hasRootConfigJSON(class_name, instance_name));
    return *root_config_jsons_.at({class_name, instance_name});
}

void ConfigurationManager::registerJsonRootConfigurable(Configurable& configurable)
{
    traced_assert(initialized_);

    Key key{configurable.className(), configurable.instanceName()};
    traced_assert(json_root_configurables_.find(key) == json_root_configurables_.end());

    json_root_configurables_[key] = &configurable;
}

void ConfigurationManager::unregisterJsonRootConfigurable(Configurable& configurable)
{
    // Safe to call during shutdown when ConfigurationManager may already be torn down
    if (!initialized_)
        return;

    Key key{configurable.className(), configurable.instanceName()};
    json_root_configurables_.erase(key);
}

void ConfigurationManager::saveConfiguration()
{
    loginf << "saving " << root_config_jsons_.size() << " root config JSONs"
           << ", " << json_root_configurables_.size() << " registered root configurables";

    // Save json-backed root configurables via ConfigJSON
    for (auto& [key, config_json] : root_config_jsons_)
    {
        loginf << "processing root config: class '" << key.first
               << "' instance '" << key.second << "'";

        // If there's a registered json-backed configurable, write back its params first
        auto cit = json_root_configurables_.find(key);
        if (cit != json_root_configurables_.end())
        {
            loginf << "writing back config for class '" << key.first
                   << "' instance '" << key.second << "'";
            try
            {
                cit->second->writeBackConfigRecursive();
            }
            catch (const std::exception& e)
            {
                logerr << "writeBackConfigRecursive failed for class '" << key.first
                       << "' instance '" << key.second << "': " << e.what();
                throw;
            }
        }
        else
        {
            loginf << "no registered configurable for class '" << key.first
                   << "' instance '" << key.second << "' — saving json as-is";
        }

        try
        {
            config_json->save();
        }
        catch (const std::exception& e)
        {
            logerr << "save failed for class '" << key.first
                   << "' instance '" << key.second << "': " << e.what();
            throw;
        }

        loginf << "saved root config: class '" << key.first
               << "' instance '" << key.second << "'";
    }

    loginf << "saveConfiguration complete";
}

