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
#include "cxa_throw_hook.h"
#include "logger.h"
#include "msghandler.h"
#include "util/system.h"

#include <QThread>
#include <QTimer>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <execinfo.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <limits.h>

using namespace std;

// Path where safeSignalHandler writes a crash backtrace on fatal signals.
// Populated at startup (initCrashLogPath) from the APPIMAGE env var; empty
// when not running from an AppImage, in which case file dump is skipped and
// stderr stays the only channel. Accessed from a signal handler, so mutated
// once at startup and read-only afterwards.
static char crash_log_path[PATH_MAX] = {0};

// Compute "<dirname(APPIMAGE)>/compass_crash_<pid>.log". Called once from
// main() before any signal could fire — uses getenv/snprintf which are NOT
// async-signal-safe and must not be reached from a handler.
static void initCrashLogPath()
{
    const char* appimage = getenv("APPIMAGE");
    if (!appimage) return;

    const char* last_slash = strrchr(appimage, '/');
    if (!last_slash) return;

    size_t dir_len = static_cast<size_t>(last_slash - appimage);
    //reserve room for "/compass_crash_<pid>.log\0"
    if (dir_len + 32 >= sizeof(crash_log_path)) return;

    memcpy(crash_log_path, appimage, dir_len);
    snprintf(crash_log_path + dir_len,
             sizeof(crash_log_path) - dir_len,
             "/compass_crash_%d.log", static_cast<int>(getpid()));
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

// async-signal-safe stacktrace for signals where allocation cannot be trusted
// (e.g. SIGABRT from a glibc heap-corruption check — the allocator is exactly
// the subsystem that just failed). Writes to stderr and, when running as an
// AppImage, to <dirname(APPIMAGE)>/compass_crash_<pid>.log so the trace is
// preserved even if the stdout/stderr pipe is lost on process teardown.
//
// Everything inside this function must use async-signal-safe primitives:
// write/open/close/backtrace_symbols_fd are safe; printf/malloc/std::cerr
// are not.
void safeSignalHandler(int signum)
{
    const char msg[] = "\nCaught signal: ";
    const char nl[] = "\n";

    // format signum as decimal digits (async-signal-safe — no printf)
    char buf[16];
    int pos = sizeof(buf);
    int val = signum < 0 ? -signum : signum;
    do {
        buf[--pos] = '0' + (val % 10);
        val /= 10;
    } while (val > 0);
    if (signum < 0)
        buf[--pos] = '-';

    void* frames[128];
    int count = backtrace(frames, 128);

    // --- stderr (existing behaviour) ---
    write(STDERR_FILENO, msg, sizeof(msg) - 1);
    write(STDERR_FILENO, buf + pos, sizeof(buf) - pos);
    write(STDERR_FILENO, nl, 1);
    backtrace_symbols_fd(frames, count, STDERR_FILENO);

    // --- crash file (only if initCrashLogPath found an AppImage at startup) ---
    if (crash_log_path[0] != '\0')
    {
        int fd = open(crash_log_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (fd >= 0)
        {
            write(fd, msg, sizeof(msg) - 1);
            write(fd, buf + pos, sizeof(buf) - pos);
            write(fd, nl, 1);
            backtrace_symbols_fd(frames, count, fd);
            close(fd);
        }
    }

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

        //resolve crash-log path before installing handlers so a very early
        //signal can still be written to disk
        initCrashLogPath();
        if (crash_log_path[0] != '\0')
            std::cout << "COMPASSClient: crash log path " << crash_log_path << std::endl;

        signal(SIGSEGV, safeSignalHandler);  // stack likely corrupted
        signal(SIGBUS,  safeSignalHandler);  // stack likely corrupted
        signal(SIGFPE,  safeSignalHandler);  // arithmetic fault
        signal(SIGABRT, safeSignalHandler);  // heap may be corrupted — allocator untrusted
        signal(SIGTERM, signalHandler);      // external term — stack intact
        signal(SIGPIPE, SIG_IGN);           // prevent crash on broken socket
        
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

        // Enable Qt high-DPI scaling unless --no_highdpi is given.
        // Some systems (e.g. AlmaLinux 9.7 with GNOME) report inflated DPI,
        // causing double-sized UI. Override with:
        //   --no_highdpi                        (CLI flag)
        //   QT_AUTO_SCREEN_SCALE_FACTOR=0       (env var)
        //   QT_SCALE_FACTOR=1                   (env var, forces 1:1)
        {
            bool disable_highdpi = false;

            for (int i = 1; i < argc; ++i)
            {
                if (std::strcmp(argv[i], "--no_highdpi") == 0)
                {
                    disable_highdpi = true;
                    break;
                }
            }

            if (!qEnvironmentVariableIsEmpty("QT_AUTO_SCREEN_SCALE_FACTOR") &&
                qgetenv("QT_AUTO_SCREEN_SCALE_FACTOR") == "0")
                disable_highdpi = true;

            if (disable_highdpi)
            {
                std::cout << "COMPASSClient: high-DPI scaling disabled" << std::endl;
                QCoreApplication::setAttribute(Qt::AA_DisableHighDpiScaling);
            }
            else
            {
                QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
            }
        }

        Client client(argc, argv);

        if (client.quitRequested())
            return 0;
            // Alternative: _exit(0) to skip static destructors entirely

        if (!client.run())
        {
            return -1;
            // compass_ is destroyed in Client's destructor which calls shutdown()
        }

        return client.exec();
        // Alternative: _exit(ret) to skip static destructors entirely
    }
    catch (std::exception& ex)
    {
        cerr << "main: caught exception '" << ex.what() << "'" << endl;

        if (!last_throw_trace.empty())
            cerr << "\nStacktrace at throw site:\n" << last_throw_trace << endl;

        return -1;
    }
    catch (...)
    {
        cerr << "main: caught exception" << endl;

        if (!last_throw_trace.empty())
            cerr << "\nStacktrace at throw site:\n" << last_throw_trace << endl;

        return -1;
    }
}
