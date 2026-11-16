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

#include "system.h"

#include "tbbhack.h"

#include <fstream>
#include <limits>
#include <string>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>
#include <cstdlib>

#ifdef _WIN32
#include <malloc.h>
#include <windows.h>
#include <psapi.h>
#include <boost/stacktrace.hpp>
#else
#include <execinfo.h>
#include <malloc.h>
#include <cxxabi.h>
#include <pwd.h>
#include <unistd.h>
#include <sys/types.h>
#endif

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/date_time/microsec_time_clock.hpp>

#include "logger.h"

const double megabyte = 1024 * 1024;
const double gigabyte = 1024 * 1024 * 1024;

namespace Utils
{
namespace System
{
float getTotalRAMinGB()
{
#ifdef _WIN32
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    if (!GlobalMemoryStatusEx(&status))
        return 0;

    return status.ullTotalPhys / gigabyte;
#else
    std::string token;
    std::ifstream file("/proc/meminfo");
    while (file >> token)
    {
        if (token == "MemTotal:")
        {
            std::uint64_t mem;

            if (file >> mem)  // returns in kB
            {
                return mem / megabyte;
            }
            else
            {
                return 0;
            }
        }
        // ignore rest of the line
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return 0;  // nothing found
#endif
}

float getFreeRAMinGB()
{
#ifdef _WIN32
    MEMORYSTATUSEX status;
    status.dwLength = sizeof(status);
    if (!GlobalMemoryStatusEx(&status))
        return 0;

    return status.ullAvailPhys / gigabyte;
#else
    //    struct sysinfo info;
    //    sysinfo (&info);

    //    return ((uint64_t) (info.freeram + info.bufferram) * info.mem_unit)/gigabyte;

    std::string token;
    std::ifstream file("/proc/meminfo");
    while (file >> token)
    {
        if (token == "MemAvailable:")
        {
            std::uint64_t mem;

            if (file >> mem)  // returns in kB
            {
                return mem / megabyte;
            }
            else
            {
                return 0;
            }
        }
        // ignore rest of the line
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return 0;  // nothing found
#endif
}

float getProcessRAMinGB()
{
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS info;
    if (!GetProcessMemoryInfo(GetCurrentProcess(), &info, sizeof(info)))
        return 0;

    // current working set, as VmRSS
    return info.WorkingSetSize / gigabyte;
#else
    std::string token;
    std::ifstream file("/proc/self/status");
    while (file >> token)
    {
        if (token == "VmRSS:")
        {
            std::uint64_t mem;

            if (file >> mem)  // returns in kB
            {
                return mem / megabyte;
            }
            else
            {
                return 0;
            }
        }
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return 0;
#endif
}

std::string exec(const std::string& cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    //std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);

    FILE* pipe = popen(cmd.c_str(), "r");

    if (!pipe)
    {
        logerr << "command '" << cmd << "' popen failed";
        throw std::runtime_error("Utils: exec: popen failed");
    }
    
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
    {
        result += buffer.data();
    }

    pclose(pipe);
    return result;
}

std::string getUserName()
{
#ifdef _WIN32
    char name[256];
    DWORD size = sizeof(name);
    if (GetUserNameA(name, &size))
        return std::string(name);
    return {};
#else
    uid_t uid = geteuid ();
    struct passwd *pw = getpwuid (uid);
    if (pw)
    {
        return std::string(pw->pw_name);
    }
    return {};
#endif
}

double secondsSinceMidnightUTC ()
{
    using namespace boost::posix_time;

    auto p_time = microsec_clock::universal_time (); // UTC.
    return (p_time.time_of_day().total_milliseconds() / 1000.0);
}

int tbbCurrentThreadID()
{
#if TBB_VERSION_MAJOR <= 2018
    int thread_id = pthread_self(); // TODO PHIL
#else
    int thread_id = tbb::this_task_arena::current_thread_index();
#endif

    if (thread_id < 0)
        thread_id = 0; // can be task_arena_base::not_initialized = -1

    return thread_id;
}

void printBacktrace() {
#ifdef _WIN32
    std::cerr << "Stack trace:\n" << boost::stacktrace::stacktrace(0, 25);
#else
    const int MAX_FRAMES = 25;
    void* buffer[MAX_FRAMES];
    int frames = backtrace(buffer, MAX_FRAMES);
    char** symbols = backtrace_symbols(buffer, frames);

    std::cerr << "Stack trace (" << frames << " frames):\n";

    for (int i = 0; i < frames; ++i) {
        std::cerr << symbols[i] << '\n';
    }

    free(symbols);
#endif
}

void trimMemory()
{
#ifdef _WIN32
    _heapmin();
#else
    malloc_trim(0);
#endif
}

const char* appDir()
{
#ifdef _WIN32
    // Windows package: the folder of the executable, with the same layout as the AppImage
    // (compass/conf, compass/data, lib/osgPlugins-*). APPDIR overrides it.
    if (const char* appdir = getenv("APPDIR"))
        return appdir;

    static const std::string exe_dir = [] ()
    {
        char path[MAX_PATH];
        DWORD len = GetModuleFileNameA(nullptr, path, MAX_PATH);
        std::string dir(path, len);
        for (char& c : dir)
            if (c == '\\')
                c = '/';
        return dir.substr(0, dir.find_last_of('/'));
    } ();

    return exe_dir.c_str();
#else
    return getenv("APPDIR");
#endif

    //return nullptr;
}

/**
 * Notice: usable before a QCoreApplication exists (unlike QCoreApplication::applicationDirPath()).
 */
std::string executableDir()
{
#ifdef _WIN32
    char path[ MAX_PATH ];
    DWORD len = GetModuleFileNameA(nullptr, path, MAX_PATH);
    if (len == 0 || len == MAX_PATH)
        return {};

    std::string dir(path, len);
    for (char& c : dir)
        if (c == '\\')
            c = '/';
#else
    char path[ 4096 ];
    ssize_t len = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (len <= 0)
        return {};

    std::string dir(path, (size_t)len);
#endif

    return dir.substr(0, dir.find_last_of('/'));
}

}
}
