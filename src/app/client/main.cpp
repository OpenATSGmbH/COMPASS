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
#include "crashbreadcrumbs.h"
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
#include <sys/resource.h>
#include <sys/stat.h>
#include <limits.h>
#include <time.h>
#include <ucontext.h>

using namespace std;

// Path where safeSignalHandler writes a crash backtrace on fatal signals.
// Populated at startup (initCrashLogPath) from the APPIMAGE env var; empty
// when not running from an AppImage, in which case file dump is skipped and
// stderr stays the only channel. Accessed from a signal handler, so mutated
// once at startup and read-only afterwards.
static char crash_log_path[PATH_MAX] = {0};

// Compute "<dirname(APPIMAGE)>/compass_crash_<pid>.log". Called once from
// main() before any signal could fire - uses getenv/snprintf which are NOT
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
// Terminate handler - called by the runtime when an exception is uncaught
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

// async-signal-safe crash report for signals where allocation cannot be
// trusted (e.g. SIGABRT from a glibc heap-corruption check - the allocator
// is exactly the subsystem that just failed). Writes to stderr and, when
// running as an AppImage, to <dirname(APPIMAGE)>/compass_crash_<pid>.log so
// the report is preserved even if the stdout/stderr pipe is lost on process
// teardown. Besides the backtrace, the report contains the fault address and
// registers (to identify e.g. a dangling receiver pointer), the recent event
// breadcrumbs recorded by Client::notify, and a copy of /proc/self/maps (to
// classify addresses as heap / mapped file / unmapped).
//
// Everything reached from the handler must use async-signal-safe primitives:
// write/open/close/read/backtrace_symbols_fd are safe; printf/malloc/
// std::cerr are not.

// copy /proc/self/maps to fd, capped so a degenerate mapping list cannot
// bloat the crash log indefinitely
static void writeMemoryMaps(int fd)
{
    using crash_breadcrumbs::writeStr;

    writeStr(fd, "\nMemory mappings (/proc/self/maps):\n");

    int maps_fd = open("/proc/self/maps", O_RDONLY);
    if (maps_fd < 0)
    {
        writeStr(fd, "<unavailable>\n");
        return;
    }

    constexpr size_t max_bytes = 1024 * 1024;
    size_t total = 0;

    char buf[4096];
    ssize_t n;
    while (total < max_bytes && (n = read(maps_fd, buf, sizeof(buf))) > 0)
    {
        ssize_t unused = write(fd, buf, static_cast<size_t>(n));
        (void)unused;
        total += static_cast<size_t>(n);
    }

    if (total >= max_bytes)
        writeStr(fd, "\n<truncated>\n");

    close(maps_fd);
}

static void writeCrashInfo(int fd, int signum, const siginfo_t* info, const ucontext_t* uc,
                           void* const* frames, int frame_count)
{
    using crash_breadcrumbs::writeDec;
    using crash_breadcrumbs::writeHex;
    using crash_breadcrumbs::writeStr;

    writeStr(fd, "\nCaught signal: ");
    writeDec(fd, static_cast<unsigned long long>(signum));
    writeStr(fd, "\n");

    // crash wall-clock time, same unit as the breadcrumb timestamps
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0)
    {
        writeStr(fd, "Crash time_ms: ");
        writeDec(fd, static_cast<unsigned long long>(ts.tv_sec) * 1000
                     + static_cast<unsigned long long>(ts.tv_nsec) / 1000000);
        writeStr(fd, "\n");
    }

    if (info)
    {
        writeStr(fd, "si_code: ");
        writeDec(fd, static_cast<unsigned long long>(info->si_code < 0 ? 0 : info->si_code));
        writeStr(fd, "\n");

        // fault address is only meaningful for memory/arithmetic faults
        if (signum == SIGSEGV || signum == SIGBUS || signum == SIGFPE)
        {
            writeStr(fd, "Fault address: ");
            writeHex(fd, reinterpret_cast<unsigned long long>(info->si_addr));
            writeStr(fd, "\n");
        }
    }

#if defined(__x86_64__)
    if (uc)
    {
        static const struct { const char* name; int reg; } regs[] = {
            {"RIP", REG_RIP}, {"RSP", REG_RSP}, {"RBP", REG_RBP},
            {"RAX", REG_RAX}, {"RBX", REG_RBX}, {"RCX", REG_RCX},
            {"RDX", REG_RDX}, {"RSI", REG_RSI}, {"RDI", REG_RDI},
            {"R8 ", REG_R8},  {"R9 ", REG_R9},  {"R10", REG_R10},
            {"R11", REG_R11}, {"R12", REG_R12}, {"R13", REG_R13},
            {"R14", REG_R14}, {"R15", REG_R15}};

        writeStr(fd, "Registers:\n");
        for (const auto& r : regs)
        {
            writeStr(fd, r.name);
            writeStr(fd, " ");
            writeHex(fd, static_cast<unsigned long long>(uc->uc_mcontext.gregs[r.reg]));
            writeStr(fd, "\n");
        }
    }
#endif

    writeStr(fd, "Backtrace:\n");
    backtrace_symbols_fd(frames, frame_count, fd);

    crash_breadcrumbs::dumpTo(fd);

    writeMemoryMaps(fd);
}

void safeSignalHandler(int signum, siginfo_t* info, void* context)
{
    const ucontext_t* uc = static_cast<const ucontext_t*>(context);

    void* frames[128];
    int count = backtrace(frames, 128);

    // --- stderr (captured in log.txt when redirected) ---
    writeCrashInfo(STDERR_FILENO, signum, info, uc, frames, count);

    // --- crash file (only if initCrashLogPath found an AppImage at startup) ---
    if (crash_log_path[0] != '\0')
    {
        int fd = open(crash_log_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (fd >= 0)
        {
            writeCrashInfo(fd, signum, info, uc, frames, count);
            close(fd);
        }
    }

    // restore the default action and re-raise: the process terminates by the
    // original signal, producing a kernel core dump where system settings
    // permit (see logCoreDumpLimit - COMPASS itself does not enable cores)
    signal(signum, SIG_DFL);
    raise(signum);
}

// run fatal-signal handlers on a dedicated stack, so a stack-overflow SIGSEGV
// can still be reported
static void installAlternateSignalStack()
{
    // fixed size: SIGSTKSZ is no longer a compile-time constant in newer glibc
    static char alt_stack[64 * 1024];

    stack_t ss;
    memset(&ss, 0, sizeof(ss));
    ss.ss_sp = alt_stack;
    ss.ss_size = sizeof(alt_stack);

    sigaltstack(&ss, nullptr);
}

// install safeSignalHandler with SA_SIGINFO, so it receives the fault address
// (siginfo_t) and register state (ucontext) in addition to the signal number
static void installCrashSignalHandler(int signum)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = safeSignalHandler;
    sa.sa_flags = SA_SIGINFO | SA_ONSTACK;

    sigaction(signum, &sa, nullptr);
}

// log the core-dump size limit, so crash reports from customer systems show
// whether a kernel core could have been written at all. Deliberately does
// NOT raise the limit: COMPASS processes can exceed 10 GB RSS, and a core
// of that size written to a customer disk (or worse, a tmpfs working
// directory) is not acceptable as a default. For a targeted debug session,
// cores can be enabled externally with 'ulimit -c unlimited' before launch.
static void logCoreDumpLimit()
{
    struct rlimit rl;
    if (getrlimit(RLIMIT_CORE, &rl) != 0)
        return;

    if (rl.rlim_cur == RLIM_INFINITY)
        std::cout << "COMPASSClient: core dump size limit unlimited" << std::endl;
    else
        std::cout << "COMPASSClient: core dump size limit " << rl.rlim_cur << " bytes"
                  << std::endl;
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

        logCoreDumpLimit();
        installAlternateSignalStack();

        installCrashSignalHandler(SIGSEGV);  // stack likely corrupted
        installCrashSignalHandler(SIGBUS);   // stack likely corrupted
        installCrashSignalHandler(SIGFPE);   // arithmetic fault
        installCrashSignalHandler(SIGABRT);  // heap may be corrupted - allocator untrusted
        signal(SIGTERM, signalHandler);      // external term - stack intact
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
