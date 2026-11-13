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

#include "plugin/plugin.h"

#include <string>
#include <vector>

/**
 * Loads the plugin modules.
 * - called by main before the application and the logger exist => messages go to std::cerr / std::cout
 * - plugins are all or none: every expected plugin must load, any failure asserts
 * - modules are never unloaded
 */
namespace plugin_manager
{

/// file name of the plugin module with the given plugin name, e.g. "ProbIMM" => "compass_plugin_probimm.so"
std::string fileName(const std::string& name);

/// loads one plugin from dir and checks its name, returns nullptr and sets error on failure
Plugin* loadPlugin(const std::string& dir,
                   const std::string& name,
                   std::string& error);

/// loads the expected plugins from dir in list order, asserts on any failure, prints a summary to stdout
Plugins loadPlugins(const std::string& dir,
                    const std::vector<std::string>& names,
                    bool must_exist = true);

}  // namespace plugin_manager
