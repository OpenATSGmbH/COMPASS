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

#include "catch.hpp"

#include "plugin/pluginmanager.h"

#include <string>

// the test plugin modules (testplugin.cpp) are built into COMPASS_TEST_PLUGIN_DIR:
// compass_plugin_testplugin contains TestPlugin, compass_plugin_wrongname contains TestPlugin too

TEST_CASE("plugin_manager loads plugin modules", "[plugin]")
{
    const std::string dir = COMPASS_TEST_PLUGIN_DIR;

    SECTION("file name")
    {
        const std::string file = plugin_manager::fileName("TestPlugin");

        REQUIRE(file.rfind("compass_plugin_testplugin.", 0) == 0);
    }

    SECTION("plugin loads, same instance on every load")
    {
        std::string error;
        Plugin* plugin = plugin_manager::loadPlugin(dir, "TestPlugin", error);

        REQUIRE(plugin);
        REQUIRE(error.empty());
        REQUIRE(plugin->name() == "TestPlugin");

        REQUIRE(plugin_manager::loadPlugin(dir, "TestPlugin", error) == plugin);
    }

    SECTION("missing file fails")
    {
        std::string error;

        REQUIRE(plugin_manager::loadPlugin(dir, "Missing", error) == nullptr);
        REQUIRE(error.find("not found") != std::string::npos);
    }

    SECTION("module with another plugin fails")
    {
        std::string error;

        REQUIRE(plugin_manager::loadPlugin(dir, "WrongName", error) == nullptr);
        REQUIRE(error.find("contains plugin 'TestPlugin'") != std::string::npos);
    }

    SECTION("expected plugins load in list order")
    {
        REQUIRE(plugin_manager::loadPlugins(dir, {}).empty());

        const Plugins plugins = plugin_manager::loadPlugins(dir, { "TestPlugin" });

        REQUIRE(plugins.size() == 1);
        REQUIRE(plugins[ 0 ]->name() == "TestPlugin");
    }
}
