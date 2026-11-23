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

#include <functional>
#include <string>

/**
 * Functions which must run before the process ends, on every path: the normal return from main,
 * a failed assertion, std::terminate and termination signals.
 *
 * Example: a library which joins its worker threads in a static destructor. On Windows the process
 * exit ends all other threads before the static destructors of the DLLs run, so such a destructor
 * waits forever. The hook stops the threads while they still run.
 */
namespace exit_hooks
{
    /// registers the function, the name is used in the log
    void add(const std::string& name, const std::function<void()>& hook);

    /// runs the hooks in reverse order of registration, only once: later calls do nothing.
    /// An exception of a hook is caught, the other hooks still run.
    void run();

    /**
     * Runs the hooks when it goes out of scope, e.g. in main before the objects whose
     * destructors may end the process.
     */
    struct Scope
    {
        ~Scope() { run(); }
    };
}
