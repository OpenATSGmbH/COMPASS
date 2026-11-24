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

#include <boost/stacktrace.hpp>

// Populated at every throw site (cxa_throw_hook.cpp): on Linux by the __cxa_throw hook, on Windows
// by a vectored exception handler on GCC's throw exception. The terminate handler and the catch blocks
// in main.cpp read it to report where the exception originated, even after the stack has been unwound.
extern thread_local boost::stacktrace::stacktrace last_throw_trace;

// the thrown object of last_throw_trace (most-derived address)
extern thread_local const void* last_throw_object;

// installs the throw hook where it needs installing (Windows: the vectored exception handler) and
// registers throwSiteOf as stacktrace_format::ThrowSiteProvider
void installThrowHook();

// the formatted last_throw_trace if exception_object is last_throw_object, else empty
std::string throwSiteOf(const void* exception_object);
