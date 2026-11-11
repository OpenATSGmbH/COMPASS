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

#include <functional>

class CompassRoot;
class QMenu;

namespace rtcommand
{
    class RTCommandRegistry;
}

/**
 * Hooks from above compass_app, set by the executable (compass_client's main.cpp).
 * - lets libraries that depend on compass_app (extra commands, test labs, plugins) extend the application
 *   without compass_app naming them
 * - unset hooks are skipped
 */
struct AppHooks
{
    std::function<void(rtcommand::RTCommandRegistry&, CompassRoot&)> register_commands;      // extra runtime commands
    std::function<void(QMenu*, CompassRoot&)>                          add_debug_menu_entries; // debug menu entries (test labs)

    Plugins plugins; // plugins, their init steps are called by CompassRoot and MainWindow
};
