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

#include "jobdefs.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif

namespace job
{

int currentCPU()
{
#ifdef _WIN32
    return static_cast<int>(GetCurrentProcessorNumber());
#else
    return sched_getcpu();
#endif
}

bool setCurrentThreadCPUs(const std::vector<int>& cpus)
{
#ifdef _WIN32
    // affinity mask of the current processor group, max. 64 cpus
    DWORD_PTR mask = 0;
    for (int cpu : cpus)
        if (cpu >= 0 && cpu < static_cast<int>(sizeof(DWORD_PTR) * 8))
            mask |= static_cast<DWORD_PTR>(1) << cpu;

    return mask != 0 && SetThreadAffinityMask(GetCurrentThread(), mask) != 0;
#else
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int cpu : cpus)
        CPU_SET(cpu, &cpuset);

    return pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) == 0;
#endif
}

}
