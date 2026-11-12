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

#include <QtGlobal>

class CompassRoot;
class QMenu;

namespace rtcommand
{
    class RTCommandRegistry;
}

/**
 * Plugin of the application, extending it at fixed points of the startup and shutdown.
 * - the plugin objects are collected by the executable (main.cpp) and handed via AppHooks to the composition root
 * - each step is called once per plugin, in list order
 * - plugin objects live for the whole process (static instances)
 */
class Plugin
{
public:
    virtual ~Plugin() = default;

    virtual std::string name() const = 0;

    /// before the managers are created: factory entries, no widgets
    virtual void initCore() {}
    /// after the managers are started: runtime commands
    virtual void initCommands(rtcommand::RTCommandRegistry& registry, CompassRoot& root) {}
    /// when the UI is set up, before the view containers are created: views, widgets, menus
    virtual void initUI(CompassRoot& root) {}
    /// when the Debug menu is created: debug entries, e.g. the plugin's test lab
    virtual void addDebugMenuEntries(QMenu* menu) {}
    /// during the shutdown sequence, while the managers still exist
    virtual void shutdown() {}
};

typedef std::vector<Plugin*> Plugins; // non-owning, plugins are static instances

typedef Plugin* (*PluginCreateFunc)(); // type of the entry symbol of a plugin module

/// Defines the entry symbol of a plugin module (compass_plugin_create()), used once per plugin, in a source compiled
/// only into its MODULE target (the plugin's objects may be linked together with other plugins, e.g. in the tests).
/// It returns the plugin's static instance, which lives until the process ends (modules are never unloaded).
#define COMPASS_DECLARE_PLUGIN(PluginClass)                     \
    extern "C" Q_DECL_EXPORT Plugin* compass_plugin_create()    \
    {                                                           \
        static PluginClass plugin;                              \
        return &plugin;                                         \
    }
