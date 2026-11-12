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

#include "pluginmanager.h"

#include "files.h"
#include "traced_assert.h"

#include <algorithm>
#include <cctype>
#include <iostream>

#include <QLibrary>

namespace plugin_manager
{

/**
 * Notice: plugin targets are named compass_plugin_<lowercase name>, without the "lib" prefix.
*/
std::string fileName(const std::string& name)
{
    std::string base = name;
    std::transform(base.begin(), base.end(), base.begin(), [ ] (unsigned char c) { return (char)std::tolower(c); });

#ifdef _WIN32
    return "compass_plugin_" + base + ".dll";
#else
    return "compass_plugin_" + base + ".so";
#endif
}

/**
 * Notice: the module stays loaded when the QLibrary object goes, it is never unloaded.
*/
Plugin* loadPlugin(const std::string& dir,
                   const std::string& name,
                   std::string& error)
{
    const std::string file = dir + "/" + fileName(name);

    if (!Utils::Files::fileExists(file))
    {
        error = "plugin '" + name + "': file '" + file + "' not found";
        return nullptr;
    }

    QLibrary library(QString::fromStdString(file));
    library.setLoadHints(QLibrary::ResolveAllSymbolsHint); // missing symbols fail here, not on first use

    if (!library.load())
    {
        error = "plugin '" + name + "': loading '" + file + "' failed: " + library.errorString().toStdString();
        return nullptr;
    }

    auto create = reinterpret_cast<PluginCreateFunc>(library.resolve("compass_plugin_create"));
    if (!create)
    {
        error = "plugin '" + name + "': entry symbol compass_plugin_create not found in '" + file + "'";
        return nullptr;
    }

    Plugin* plugin = create();

    if (plugin->name() != name)
    {
        error = "plugin '" + name + "': '" + file + "' contains plugin '" + plugin->name() + "'";
        return nullptr;
    }

    return plugin;
}

/**
*/
Plugins loadPlugins(const std::string& dir,
                    const std::vector<std::string>& names)
{
    if (names.empty())
    {
        std::cout << "plugin_manager: no plugins expected" << std::endl;
        return {};
    }

    Plugins plugins;
    plugins.reserve(names.size());

    for (const auto& name : names)
    {
        std::string error;
        Plugin* plugin = loadPlugin(dir, name, error);

        if (!plugin)
        {
            // no logger yet => the cause goes to stderr before the assert
            std::cerr << "plugin_manager: " << error << std::endl;
            traced_assert_msg(plugin, error.c_str());
        }

        std::cout << "plugin_manager: loaded plugin '" << name << "' from '" << dir << "/" << fileName(name) << "'"
                  << std::endl;

        plugins.push_back(plugin);
    }

    return plugins;
}

}  // namespace plugin_manager
