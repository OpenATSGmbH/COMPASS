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

class CompassRoot;

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
    /// during the shutdown sequence, while the managers still exist
    virtual void shutdown() {}
};

typedef std::vector<Plugin*> Plugins; // non-owning, plugins are static instances
