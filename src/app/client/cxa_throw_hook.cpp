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

// ---------------------------------------------------------------------------
// __cxa_throw interception - captures a stacktrace at every throw site so
// that terminate/signal handlers can report where the exception originated,
// even after the stack has been unwound.
//
// IMPORTANT - the second parameter type differs between compilers.
//
// The Itanium C++ ABI specifies std::type_info* as the second parameter of
// __cxa_throw, and that is what <cxxabi.h> declares.  However, GCC 15 has a
// quirk: every `throw` expression in a non-system header causes the compiler
// to emit an implicit __cxa_throw declaration with void* parameters.  If our
// definition uses std::type_info*, GCC rejects the translation unit as soon
// as it encounters any throw in a non-system header (boost, etc.).
//
// Clang (and clangd) use the canonical std::type_info* and reject void*.
//
// We resolve this with an #ifdef: void* for GCC, std::type_info* for Clang.
// At the ABI level the two are interchangeable (both are pointer-width),
// so the generated code is identical regardless of which branch is compiled.
// ---------------------------------------------------------------------------

#include <dlfcn.h>

// forward-declare the capture helper (defined below, after boost includes)
static void capture_throw_trace();

#ifdef __clang__
#include <cxxabi.h>
using cxa_throw_type_info = std::type_info*;
#else
using cxa_throw_type_info = void*;
#endif

using cxa_throw_fn = void (*)(void*, cxa_throw_type_info, void (*)(void*));

static cxa_throw_fn get_real_cxa_throw()
{
    static cxa_throw_fn fn =
        reinterpret_cast<cxa_throw_fn>(dlsym(RTLD_NEXT, "__cxa_throw"));
    return fn;
}

extern "C" void __cxa_throw(void* thrown_exception,
                             cxa_throw_type_info tinfo,
                             void (*dest)(void*))
{
    capture_throw_trace();
    get_real_cxa_throw()(thrown_exception, tinfo, dest);
    __builtin_unreachable();
}

// --- now safe to include headers that contain throw expressions ------------

#include "cxa_throw_hook.h"

thread_local boost::stacktrace::stacktrace last_throw_trace;

// guard against recursion (boost::stacktrace itself may throw internally)
static thread_local bool in_cxa_throw_hook = false;

static void capture_throw_trace()
{
    if (!in_cxa_throw_hook)
    {
        in_cxa_throw_hook = true;
        last_throw_trace = boost::stacktrace::stacktrace();
        in_cxa_throw_hook = false;
    }
}
