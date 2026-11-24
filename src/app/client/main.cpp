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
#include "compassroot.h"
#include "crashbreadcrumbs.h"
#include "cxa_throw_hook.h"
#include "exit_hooks.h"
#include "stacktrace_format.h"
#include "files.h"
#include "global.h"
#include "logger.h"
#include "msghandler.h"
#include "plugin/pluginmanager.h"
#include "stringconv.h"
#include "system.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "extra_commands.h"
#include "test_lab.h"
#endif

#include <QThread>
#include <QTimer>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <sstream>
#include <thread>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#include <pthread.h>
#include <execinfo.h>
#include <fcntl.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <ucontext.h>
#endif

using namespace std;

// Path of the log file of this start (Logger::logFilePath), the crash handlers append their reports to it, directly and
// not through the logger: the crashing thread may hold the lock of the logger. Set once at startup
// (initCrashLogPath) and read-only afterwards, as it is read from signal handlers.
static char crash_log_path[PATH_MAX] = {0};

// Called once from main() before any signal could fire - not async-signal-safe.
static void initCrashLogPath()
{
    const std::string& path = Logger::logFilePath(HOME_LOG_DIRECTORY);

    if (path.size() + 1 > sizeof(crash_log_path))
        return;

    memcpy(crash_log_path, path.c_str(), path.size() + 1);
}

// Writes a crash report to stderr and appends it to the log file. Allocates, so not for the
// async-signal-safe handler (writeCrashInfo).
static void writeCrashText(const std::string& text)
{
    std::cerr << text << std::flush;

    if (crash_log_path[ 0 ] != '\0')
    {
        std::ofstream file(crash_log_path, std::ios::app);
        if (file)
            file << text << std::flush;

        // where the report is kept, for the user (the console is gone with the process)
        std::cerr << "\nLog file: " << crash_log_path << std::endl;
    }
}

// ---------------------------------------------------------------------------
// Terminate handler - called by the runtime when an exception is uncaught
// (e.g. escaping a noexcept boundary or a worker thread).  Prints the
// throw-site stacktrace captured by the __cxa_throw hook above, plus the
// exception message if available.
// ---------------------------------------------------------------------------

void terminateHandler()
{
    std::ostringstream ss;

    ss << "\n=== std::terminate called ===" << std::endl;

    // the exception message and the throw site (captured by the throw hook, if it is this exception)
    if (auto eptr = std::current_exception())
    {
        try
        {
            std::rethrow_exception(eptr);
        }
        catch (const std::exception& e)
        {
            ss << "Exception: " << e.what() << std::endl;

            const std::string site = throwSiteOf(dynamic_cast<const void*>(&e));
            if (!site.empty())
                ss << "\nStacktrace at throw site:\n" << site << std::endl;
        }
        catch (...)
        {
            ss << "Unknown exception type" << std::endl;
        }
    }

    ss << "\nStacktrace at terminate:\n"
       << stacktrace_format::current() << std::endl;

    writeCrashText(ss.str());

#ifdef _WIN32
    // abort() unloads the DLLs on Windows (exit_hooks.h), not on Linux
    exit_hooks::run();
#endif

    // reset SIGABRT to default so abort() doesn't trigger our signal handler
    // (we already printed everything useful above)
    signal(SIGABRT, SIG_DFL);
    std::abort();
}

// ---------------------------------------------------------------------------
// Signal handlers
// ---------------------------------------------------------------------------

#ifdef _WIN32

// Windows: unhandled structured exceptions (access violation etc.): exception code, address,
// breadcrumbs and a stack trace to stderr and the log file. The stack trace allocates, so it is
// a best effort after heap corruption. No register dump yet.
static LONG WINAPI unhandledExceptionHandler(EXCEPTION_POINTERS* info)
{
    using crash_breadcrumbs::writeHex;
    using crash_breadcrumbs::writeStr;

    auto writeHeader = [ & ] (int fd)
    {
        writeStr(fd, "\nUnhandled exception, code: ");
        writeHex(fd, info && info->ExceptionRecord ? info->ExceptionRecord->ExceptionCode : 0);
        writeStr(fd, "\nException address: ");
        writeHex(fd, info && info->ExceptionRecord
                     ? reinterpret_cast<unsigned long long>(info->ExceptionRecord->ExceptionAddress) : 0);
        writeStr(fd, "\n");

        crash_breadcrumbs::dumpTo(fd);
    };

    writeHeader(2); // stderr

    if (crash_log_path[ 0 ] != '\0')
    {
        int fd = open(crash_log_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (fd >= 0)
        {
            writeHeader(fd);
            close(fd);
        }
    }

    writeCrashText("\nStacktrace at exception:\n" + stacktrace_format::current() + "\n");

    return EXCEPTION_CONTINUE_SEARCH; // default handling: process terminates
}

// Windows: Ctrl+C, Ctrl+Break and closing the console window end the process with ExitProcess,
// which unloads the DLLs => run the exit hooks first (exit_hooks.h). Runs in its own thread.
static BOOL WINAPI consoleCtrlHandler(DWORD ctrl_type)
{
    (void)ctrl_type;

    exit_hooks::run();

    return FALSE; // default handling: process terminates
}

#else

// async-signal-safe crash report for signals where allocation cannot be
// trusted (e.g. SIGABRT from a glibc heap-corruption check - the allocator
// is exactly the subsystem that just failed). Writes to stderr and appends to
// the log file (crash_log_path), so the report is preserved even if the
// stdout/stderr pipe is lost on process teardown. Besides the backtrace, the report contains the fault address and
// registers (to identify e.g. a dangling receiver pointer), the recent event
// breadcrumbs recorded by Client::notify, and a copy of /proc/self/maps (to
// classify addresses as heap / mapped file / unmapped).
//
// Everything reached from the handler must use async-signal-safe primitives:
// write/open/close/read/backtrace_symbols_fd are safe; printf/malloc/
// std::cerr are not.

// copy /proc/self/maps to fd, capped so a degenerate mapping list cannot
// bloat the crash log indefinitely
// parses a hex number at p (up to the first non-hex character), returns the position after it
static const char* parseHex(const char* p, const char* end, unsigned long long& value)
{
    value = 0;
    for (; p < end; ++p)
    {
        char c = *p;
        int digit = (c >= '0' && c <= '9') ? c - '0' :
                    (c >= 'a' && c <= 'f') ? c - 'a' + 10 :
                    (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
        if (digit < 0)
            break;
        value = value * 16 + static_cast<unsigned long long>(digit);
    }
    return p;
}

// Writes the mappings of /proc/self/maps which contain one of the addresses (fault address, registers,
// backtrace frames), at most MaxMappingLines: the complete map has about 2000 lines, the report needs
// only the classification of its addresses (heap, mapped file, unmapped). An address in no listed
// mapping is unmapped. Async-signal-safe: read/write only, no allocation.
static void writeMemoryMaps(int fd, const unsigned long long* addresses, int num_addresses)
{
    using crash_breadcrumbs::writeDec;
    using crash_breadcrumbs::writeStr;

    constexpr int MaxMappingLines = 200;

    writeStr(fd, "\nMemory mappings containing a reported address (/proc/self/maps):\n");

    int maps_fd = open("/proc/self/maps", O_RDONLY);
    if (maps_fd < 0)
    {
        writeStr(fd, "<unavailable>\n");
        return;
    }

    char line[1024];
    size_t line_len = 0;
    int written = 0;
    unsigned long long total = 0;

    auto processLine = [ & ] ()
    {
        ++total;

        unsigned long long start = 0, end = 0;
        const char* p = parseHex(line, line + line_len, start);
        if (p < line + line_len && *p == '-')
            parseHex(p + 1, line + line_len, end);

        bool match = false;
        for (int i = 0; i < num_addresses && !match; ++i)
            match = addresses[ i ] >= start && addresses[ i ] < end;

        if (match && written < MaxMappingLines)
        {
            ssize_t unused = write(fd, line, line_len);
            (void)unused;
            writeStr(fd, "\n");
            ++written;
        }
    };

    char buf[4096];
    ssize_t n;
    while ((n = read(maps_fd, buf, sizeof(buf))) > 0)
    {
        for (ssize_t i = 0; i < n; ++i)
        {
            if (buf[ i ] == '\n')
            {
                processLine();
                line_len = 0;
            }
            else if (line_len < sizeof(line))
            {
                line[ line_len++ ] = buf[ i ];
            }
        }
    }
    if (line_len > 0)
        processLine();

    close(maps_fd);

    writeDec(fd, static_cast<unsigned long long>(written));
    writeStr(fd, " of ");
    writeDec(fd, total);
    writeStr(fd, " mappings\n");
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

    // the addresses of the report, whose mappings are listed
    unsigned long long addresses[ 160 ];
    int num_addresses = 0;

    if (info && (signum == SIGSEGV || signum == SIGBUS || signum == SIGFPE))
        addresses[ num_addresses++ ] = reinterpret_cast<unsigned long long>(info->si_addr);

#if defined(__x86_64__)
    if (uc)
        for (int r = 0; r < NGREG && num_addresses < 40; ++r)
            addresses[ num_addresses++ ] = static_cast<unsigned long long>(uc->uc_mcontext.gregs[ r ]);
#endif

    for (int i = 0; i < frame_count && num_addresses < 160; ++i)
        addresses[ num_addresses++ ] = reinterpret_cast<unsigned long long>(frames[ i ]);

    writeMemoryMaps(fd, addresses, num_addresses);
}

void safeSignalHandler(int signum, siginfo_t* info, void* context)
{
    const ucontext_t* uc = static_cast<const ucontext_t*>(context);

    void* frames[128];
    int count = backtrace(frames, 128);

    // --- stderr (captured in log.txt when redirected) ---
    writeCrashInfo(STDERR_FILENO, signum, info, uc, frames, count);

    if (crash_log_path[0] != '\0')
    {
        crash_breadcrumbs::writeStr(STDERR_FILENO, "\nLog file: ");
        crash_breadcrumbs::writeStr(STDERR_FILENO, crash_log_path);
        crash_breadcrumbs::writeStr(STDERR_FILENO, "\n");
    }

    // --- log file ---
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

#endif

// ---------------------------------------------------------------------------
// Main-thread stall watchdog: a heartbeat timer on the main thread bumps an atomic
// timestamp; a watchdog thread signals the main thread (SIGUSR2) when the heartbeat
// stops for longer than the threshold, and the async-signal-safe handler dumps the
// main thread's LIVE backtrace to stderr. A "not responding" episode thereby
// documents itself, with a stack, in whatever captures stderr (the test framework's
// per-test logs included); the episode's end is logged with its duration. Long
// stalls re-dump periodically, so a moving stall (several long stretches) shows its
// progression.

namespace main_thread_stall
{

static std::atomic<bool>      watchdog_run { false };
static std::thread            watchdog;

#ifdef _WIN32

// the live backtrace of the stalled main thread needs a signal to the main thread,
// not available on Windows yet
static void start(QObject*)
{
    logwrn << "stall watchdog not supported on Windows";
}

#else

static std::atomic<long long> last_beat_ms { 0 };
static pthread_t              main_thread;

static long long nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// runs on the main thread, interrupting whatever it is stuck in - only
// async-signal-safe calls allowed
static void stallSignalHandler(int, siginfo_t*, void*)
{
    using crash_breadcrumbs::writeStr;

    void* frames[128];
    int count = backtrace(frames, 128);

    writeStr(STDERR_FILENO, "\n=== MAIN THREAD STALL: heartbeat stopped, live backtrace ===\n");
    backtrace_symbols_fd(frames, count, STDERR_FILENO);
    writeStr(STDERR_FILENO, "=== MAIN THREAD STALL: end of backtrace ===\n");
}

static void start(QObject* heartbeat_parent)
{
    const long long threshold_ms = 3000;  // below the window manager's patience
    const long long redump_ms    = 10000; // progression dumps during long stalls

    main_thread  = pthread_self();
    last_beat_ms = nowMs();

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = stallSignalHandler;
    sa.sa_flags = SA_SIGINFO | SA_ONSTACK | SA_RESTART;
    sigaction(SIGUSR2, &sa, nullptr);

    auto* timer = new QTimer(heartbeat_parent);
    QObject::connect(timer, &QTimer::timeout, [] () { last_beat_ms = nowMs(); });
    timer->start(100);

    watchdog_run = true;
    watchdog = std::thread([ threshold_ms, redump_ms ] ()
    {
        long long stall_begin = 0;
        long long last_dump   = 0;

        while (watchdog_run)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            long long now = nowMs();
            long long gap = now - last_beat_ms;

            if (gap > threshold_ms)
            {
                if (!stall_begin)
                {
                    stall_begin = now - gap;
                    last_dump   = 0;
                }

                if (!last_dump || now - last_dump > redump_ms)
                {
                    last_dump = now;
                    pthread_kill(main_thread, SIGUSR2);
                }
            }
            else if (stall_begin)
            {
                logwrn << "main thread was stalled for "
                       << (now - stall_begin) / 1000.0 << " s";

                stall_begin = 0;
            }
        }
    });
}

#endif

static void stop()
{
    watchdog_run = false;

    if (watchdog.joinable())
        watchdog.join();
}

// scope guard, so every return path out of main stops the watchdog thread
struct Scope
{
    ~Scope() { stop(); }
};

} // namespace main_thread_stall

// boost::stacktrace handler for signals where the stack is likely intact
void signalHandler(int signum)
{
    std::ostringstream ss;

    ss << "\nCaught signal: " << signum << std::endl;

    // not the last throw site: it belongs to an exception which was probably caught long ago
    ss << "\nStacktrace at signal:\n"
       << stacktrace_format::current() << std::endl;

    writeCrashText(ss.str());

#ifdef _WIN32
    // the default action ends the process with ExitProcess, which unloads the DLLs (exit_hooks.h)
    exit_hooks::run();
#endif

    signal(signum, SIG_DFL);
    raise(signum);
}

/**
 * Directory of the plugin modules, derived from the executable.
 */
static std::string pluginDirectory()
{
#ifdef _WIN32
    return Utils::System::executableDir() + "/plugins";
#else
    return Utils::System::executableDir() + "/../lib/compass_plugins";
#endif
}

/**
 * Plugins expected as modules (all or none), declared with compass_add_plugin()
 * in experimental_src/CMakeLists.txt and passed as COMPASS_EXPECTED_PLUGINS (comma-separated, empty without
 * experimental source).
 */
static std::vector<std::string> expectedPlugins()
{
    const std::string names = COMPASS_EXPECTED_PLUGINS;
    if (names.empty())
        return {};

    return Utils::String::split(names, ',');
}

/**
 * Hooks for the libraries that depend on compass_app (extra commands, test labs), which compass_app cannot name.
 * Notice: loads the plugin modules, asserts if one of the expected plugins fails to load.
 */
static AppHooks createAppHooks()
{
    AppHooks hooks;

    hooks.plugins = plugin_manager::loadPlugins(pluginDirectory(), expectedPlugins(), true);

#if USE_EXPERIMENTAL_SOURCE == true
    hooks.register_commands = &extra::init_extra_commands;

    hooks.add_debug_menu_entries = [ ] (QMenu* menu, CompassRoot& root)
    {
        TestLabCollection().appendTestLabs(menu, root.compass(), root.taskManager(), root.viewManager());
    };
#endif

    return hooks;
}

int main(int argc, char** argv)
{
    // line buffer stdout before the first output: the logger writes through std::cout, which
    // is fully buffered on a pipe or file, so the last log lines before a crash were lost
    setvbuf(stdout, nullptr, _IOLBF, 0);

    try
    {
        // hooks which must run before the process ends (exit_hooks.h), on every return path out of
        // main: declared first, so they run after the destruction of the client
        exit_hooks::Scope exit_hooks_scope;

        std::set_terminate(terminateHandler);

        // throw-site stack traces for the crash reports and Client::notify
        installThrowHook();

        //resolve crash-log path before installing handlers so a very early
        //signal can still be written to disk
        initCrashLogPath();
        if (crash_log_path[0] != '\0')
            std::cout << "COMPASSClient: crash reports go to " << crash_log_path << std::endl;

#ifdef _WIN32
        SetUnhandledExceptionFilter(unhandledExceptionHandler);
        SetConsoleCtrlHandler(consoleCtrlHandler, TRUE);
        signal(SIGTERM, signalHandler);      // external term - stack intact
#else
        logCoreDumpLimit();
        installAlternateSignalStack();

        installCrashSignalHandler(SIGSEGV);  // stack likely corrupted
        installCrashSignalHandler(SIGBUS);   // stack likely corrupted
        installCrashSignalHandler(SIGFPE);   // arithmetic fault
        installCrashSignalHandler(SIGABRT);  // heap may be corrupted - allocator untrusted
        signal(SIGTERM, signalHandler);      // external term - stack intact
        signal(SIGPIPE, SIG_IGN);           // prevent crash on broken socket
#endif
        
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

        Client client(argc, argv, createAppHooks());

        if (client.quitRequested())
            return 0;
            // Alternative: _exit(0) to skip static destructors entirely

        //stall self-reporting (hidden option --stall_watchdog): any main thread
        //stretch above the threshold dumps its own live backtrace to stderr (see
        //main_thread_stall)
        main_thread_stall::Scope stall_watchdog_scope;

        if (client.stallWatchdogRequested())
            main_thread_stall::start(&client);

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
        std::ostringstream ss;
        ss << "main: caught exception '" << ex.what() << "'" << endl;

        const std::string site = throwSiteOf(dynamic_cast<const void*>(&ex));
        if (!site.empty())
            ss << "\nStacktrace at throw site:\n" << site << endl;

        writeCrashText(ss.str());

        return -1;
    }
    catch (...)
    {
        std::ostringstream ss;
        ss << "main: caught exception of unknown type (no throw site: the thrown object is unknown)" << endl;

        writeCrashText(ss.str());

        return -1;
    }
}
