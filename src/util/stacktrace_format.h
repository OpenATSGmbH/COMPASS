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

#include <boost/stacktrace/stacktrace_fwd.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/**
 * Readable stack traces on Linux and Windows, one line per frame:
 *
 *   0# probimm::Associator::scoreTarget(int) at .../associator.cpp:120 in compass_plugin_probimm.so+0x1a2b3c
 *        inlined in probimm::associateAll(int) at .../associator.cpp:310
 *
 * A frame whose code was inlined into other functions lists them below, innermost first.
 * Function, file and line come from the debug information of the module (-g1, read with libbacktrace).
 * Without it the function name comes from the symbol table (Linux) or the export table (Windows), with
 * the distance to the symbol start ("name+0x1f"). Module and offset are always written, for a lookup
 * in the debug information of the release.
 *
 * The function names of Boost's own formatting are wrong on Windows (it looks them up in the symbol
 * table, which libbacktrace reads wrongly in DLLs), so COMPASS formats the frames here.
 */
namespace stacktrace_format
{
    struct Location
    {
        std::string function; // demangled, empty if unknown
        std::string file;     // empty if unknown
        int         line = 0;
    };

    struct Frame
    {
        std::string function; // demangled, empty if unknown
        std::string file;     // empty if unknown
        int         line = 0;
        std::string module;   // file name of the module, empty if unknown
        uintptr_t   offset = 0; // address relative to the load address of the module

        std::vector<Location> inlined_in; // functions the code was inlined into, innermost first
    };

    /// resolves a code address (a return address of a stack frame)
    Frame resolve(const void* address);

    /// one frame as a line (plus one line per function it was inlined into), without the index
    std::string toString(const Frame& frame, const void* address);

    /// the frames of the trace, one line each
    std::string toString(const boost::stacktrace::stacktrace& trace);

    /// the current call stack, the caller of this function is frame 0
    std::string current(std::size_t max_depth = 64);
}
