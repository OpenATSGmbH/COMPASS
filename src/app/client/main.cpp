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

#include "client.h"
#include "compass.h"
#include "logger.h"
#include "msghandler.h"
#include "util/system.h"

#include <QThread>
#include <QTimer>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <boost/stacktrace.hpp>

#include <dlfcn.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <execinfo.h>

using namespace std;

// ---------------------------------------------------------------------------
// __cxa_throw interception — captures a stacktrace at every throw site so
// that terminate/signal handlers can report where the exception originated,
// even after the stack has been unwound.
// ---------------------------------------------------------------------------

thread_local boost::stacktrace::stacktrace last_throw_trace;

// guard against recursion (boost::stacktrace itself may throw internally)
thread_local bool in_cxa_throw_hook = false;

using cxa_throw_fn = void (*)(void*, void*, void (*)(void*));

static cxa_throw_fn real_cxa_throw()
{
    static cxa_throw_fn fn =
        reinterpret_cast<cxa_throw_fn>(dlsym(RTLD_NEXT, "__cxa_throw"));
    return fn;
}

extern "C" void __cxa_throw(void* thrown_exception,
                             void* tinfo,
                             void (*dest)(void*))
{
    if (!in_cxa_throw_hook)
    {
        in_cxa_throw_hook = true;
        last_throw_trace = boost::stacktrace::stacktrace();
        in_cxa_throw_hook = false;
    }

    real_cxa_throw()(thrown_exception, tinfo, dest);
    __builtin_unreachable();
}

// ---------------------------------------------------------------------------
// Terminate handler — called by the runtime when an exception is uncaught
// (e.g. escaping a noexcept boundary or a worker thread).  Prints the
// throw-site stacktrace captured by the __cxa_throw hook above, plus the
// exception message if available.
// ---------------------------------------------------------------------------

void terminateHandler()
{
    std::cerr << "\n=== std::terminate called ===" << std::endl;

    // print the throw-site stacktrace (captured by __cxa_throw hook)
    if (!last_throw_trace.empty())
    {
        std::cerr << "\nStacktrace at throw site:\n"
                  << last_throw_trace << std::endl;
    }

    // try to print the exception message
    if (auto eptr = std::current_exception())
    {
        try
        {
            std::rethrow_exception(eptr);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown exception type" << std::endl;
        }
    }

    std::cerr << "\nStacktrace at terminate:\n"
              << boost::stacktrace::stacktrace() << std::endl;

    // reset SIGABRT to default so abort() doesn't trigger our signal handler
    // (we already printed everything useful above)
    signal(SIGABRT, SIG_DFL);
    std::abort();
}

// ---------------------------------------------------------------------------
// Signal handlers
// ---------------------------------------------------------------------------

// async-signal-safe stacktrace for signals where the stack may be corrupted
void safeSignalHandler(int signum)
{
    const char msg[] = "\nCaught signal: ";
    const char nl[] = "\n";
    write(STDERR_FILENO, msg, sizeof(msg) - 1);

    // print signum as decimal digits (async-signal-safe)
    char buf[16];
    int pos = sizeof(buf);
    int val = signum < 0 ? -signum : signum;
    do {
        buf[--pos] = '0' + (val % 10);
        val /= 10;
    } while (val > 0);
    if (signum < 0)
        buf[--pos] = '-';
    write(STDERR_FILENO, buf + pos, sizeof(buf) - pos);
    write(STDERR_FILENO, nl, 1);

    void* frames[128];
    int count = backtrace(frames, 128);
    backtrace_symbols_fd(frames, count, STDERR_FILENO);

    signal(signum, SIG_DFL);
    raise(signum);
}

// boost::stacktrace handler for signals where the stack is likely intact
void signalHandler(int signum)
{
    std::cerr << "\nCaught signal: " << signum << std::endl;

    if (!last_throw_trace.empty())
    {
        std::cerr << "\nStacktrace at throw site:\n"
                  << last_throw_trace << std::endl;
    }
    else
        std::cerr << "\nStacktrace at signal:\n"
                << boost::stacktrace::stacktrace() << std::endl;

    signal(signum, SIG_DFL);
    raise(signum);
}

int main(int argc, char** argv)
{
    try
    {
        std::set_terminate(terminateHandler);

        signal(SIGSEGV, safeSignalHandler);  // stack likely corrupted
        signal(SIGABRT, signalHandler);      // stack likely intact
        signal(SIGTERM, signalHandler);      // stack likely intact
        
        const bool is_app_image = Utils::System::appDir() != nullptr;

        if (!is_app_image)
        {
            //localbuild => switch to xcb if on wayland (and not specified otherwise)
            if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) 
            {
                auto var = qgetenv("XDG_SESSION_TYPE");
                const char *session = var.constData();
                if (session && QString::fromLocal8Bit(session) == "wayland")
                {
                    std::cout << "setting platform to xcb" << std::endl; 
                    qputenv("QT_QPA_PLATFORM", "xcb");
                }
            }
        }

        // Enable Qt high-DPI scaling
        QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

        Client client(argc, argv);

        if (client.quitRequested())
            return 0;
            // Alternative: _exit(0) to skip static destructors entirely

        // note: do not use COMPASS::instance functions here

        if (!client.run())
        {
            COMPASS::instance().shutdown();
            return -1;
            // Alternative: _exit(1) to skip static destructors entirely
        }

        return client.exec();
        // Alternative: _exit(ret) to skip static destructors entirely
    }
    catch (std::exception& ex)
    {
        cerr << "main: caught exception '" << ex.what() << "'" << endl;

        return -1;
    }
    catch (...)
    {
        cerr << "main: caught exception" << endl;

        return -1;
    }
}
