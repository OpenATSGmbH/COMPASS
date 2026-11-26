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

#include "find_error_code.h"

#include <string>
#include <utility>

class AppState;
class Configurable;
class QWidget;

namespace rtcommand
{

/**
 * The application as seen by the runtime command execution (runner, wait conditions, object lookup).
 * - object lookup roots of the command receivers (view containers, views, configuration tree)
 * - whether commands have to wait for the views
 * - implemented by COMPASS
 */
class IRTCommandHost
{
public:
    virtual ~IRTCommandHost() = default;

    virtual AppState& appState() = 0;

    // object lookup roots
    virtual QWidget* viewContainerWidget(const std::string& name) = 0;
    virtual QWidget* latestViewContainerWidget() = 0;
    virtual QWidget* latestViewWidget() = 0;
    virtual std::pair<FindObjectErrCode, Configurable*> findConfigurable(const std::string& path) = 0; // below the root configurable

    // true while the views process data => commands are deferred
    virtual bool viewsBusy() = 0;
};

} // namespace rtcommand
