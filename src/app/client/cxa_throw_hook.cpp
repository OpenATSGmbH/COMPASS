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

#ifdef _WIN32

// Windows: there is no dlsym(RTLD_NEXT) to reach the real __cxa_throw, but with SEH the GCC unwinder
// raises every throw as a Windows exception with the code STATUS_GCC_THROW (libgcc unwind-seh.c),
// with the _Unwind_Exception of the thrown object as first parameter. A vectored exception handler
// sees it first, in the throwing thread, before any unwinding.

#include "cxa_throw_hook.h"
#include "stacktrace_format.h"

#include <unwind.h>
#include <windows.h>

// empty until a throw: the default constructor of boost::stacktrace captures the current stack,
// which the crash handlers would report as the throw site
thread_local boost::stacktrace::stacktrace last_throw_trace(0, 0);
thread_local const void* last_throw_object = nullptr;

static thread_local bool in_throw_hook = false;

// GCC_EXCEPTION(0) in libgcc unwind-seh.c: STATUS_USER_DEFINED | ('G' << 16 | 'C' << 8 | 'C')
static const DWORD STATUS_GCC_THROW = 0x20474343;

static LONG WINAPI throwHookHandler(EXCEPTION_POINTERS* info)
{
    const EXCEPTION_RECORD* record = info ? info->ExceptionRecord : nullptr;

    if (!record || record->ExceptionCode != STATUS_GCC_THROW || record->NumberParameters < 1 || in_throw_hook)
        return EXCEPTION_CONTINUE_SEARCH;

    in_throw_hook = true;

    //the thrown object follows the _Unwind_Exception header of libstdc++ (__get_object_from_ue in
    //unwind-cxx.h). Not __cxa_get_exception_ptr: it returns the pointer adjusted for the catch, which
    //is only set when the unwinder has found the handler, after this point.
    auto ue = reinterpret_cast<const _Unwind_Exception*>(record->ExceptionInformation[ 0 ]);

    //std::rethrow_exception throws a dependent exception (class ends with 0x01), whose header is
    //followed by no object: not recorded, the original throw site is in the thread that threw it
    if (!ue || (ue->exception_class & 0xff) == 0x01)
    {
        in_throw_hook = false;
        return EXCEPTION_CONTINUE_SEARCH;
    }

    const void* object = ue + 1;

    //a rethrow (throw;) raises the same object again: keep the original throw site
    if (object != last_throw_object || last_throw_trace.empty())
    {
        last_throw_trace  = boost::stacktrace::stacktrace();
        last_throw_object = object;
    }

    in_throw_hook = false;

    return EXCEPTION_CONTINUE_SEARCH;
}

void installThrowHook()
{
    AddVectoredExceptionHandler(1, throwHookHandler);

    stacktrace_format::setThrowSiteProvider(&throwSiteOf);
}

std::string throwSiteOf(const void* exception_object)
{
    if (!exception_object || exception_object != last_throw_object || last_throw_trace.empty())
        return {};

    return stacktrace_format::toString(last_throw_trace);
}

#else

#include <dlfcn.h>

// forward-declare the capture helper (defined below, after boost includes)
static void capture_throw_trace(const void* thrown_exception);

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
    capture_throw_trace(thrown_exception);
    get_real_cxa_throw()(thrown_exception, tinfo, dest);
    __builtin_unreachable();
}

// --- now safe to include headers that contain throw expressions ------------

#include "cxa_throw_hook.h"

#include "stacktrace_format.h"

// empty until a throw: the default constructor of boost::stacktrace captures the current stack,
// which the crash handlers would report as the throw site
thread_local boost::stacktrace::stacktrace last_throw_trace(0, 0);
thread_local const void* last_throw_object = nullptr;

// guard against recursion (boost::stacktrace itself may throw internally)
static thread_local bool in_cxa_throw_hook = false;

static void capture_throw_trace(const void* thrown_exception)
{
    if (!in_cxa_throw_hook)
    {
        in_cxa_throw_hook = true;
        last_throw_trace  = boost::stacktrace::stacktrace();
        last_throw_object = thrown_exception;
        in_cxa_throw_hook = false;
    }
}

void installThrowHook()
{
    //the __cxa_throw hook needs no installation
    stacktrace_format::setThrowSiteProvider(&throwSiteOf);
}

std::string throwSiteOf(const void* exception_object)
{
    if (!exception_object || exception_object != last_throw_object || last_throw_trace.empty())
        return {};

    return stacktrace_format::toString(last_throw_trace);
}

#endif
