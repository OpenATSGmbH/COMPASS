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

#include "logger.h"

#include <random>

#include <boost/optional.hpp>

#include <QThread>

namespace job
{
    enum class ThreadAffinityCondition
    {
        Always = 0, // apply specified affinity mode in any case
        CPU0        // apply specified affinity mode in case the job is currently executed on cpu0
    };

    enum class ThreadAffinityMode
    {
        Auto = 0,      // do not force affinity
        CPUModulo,     // cpu = job_id % max_threads
        CPURandom,     // cpu = random cpu
        CPUModuloNot0, // cpu = 1 + job_id % (max_threads - 1)
        CPURandomNot0, // cpu = random non-zero cpu
        CPUIndex,      // cpu = specified index
        NotCPU0,       // affinity for all cpus except cpu0
        AllCPUs        // affinity for all cpus
    };

    /**
     */
    struct ThreadAffinity
    {
        ThreadAffinity() = default;
        ThreadAffinity(ThreadAffinityMode mode,
                       ThreadAffinityCondition condition = ThreadAffinityCondition::Always,
                       int cpu_index = -1)
        :   ta_mode     (mode     )
        ,   ta_condition(condition)
        ,   ta_cpu_index(cpu_index) {}

        ThreadAffinity& mode(ThreadAffinityMode mode) { ta_mode = mode; return *this; }
        ThreadAffinity& condition(ThreadAffinityCondition condition) { ta_condition = condition; return *this; }
        ThreadAffinity& cpuIndex(int cpu_index) { ta_cpu_index = cpu_index; return *this; }

        ThreadAffinityMode      ta_mode      = ThreadAffinityMode::Auto;
        ThreadAffinityCondition ta_condition = ThreadAffinityCondition::Always;
        int                     ta_cpu_index = -1;
    };

    /**
     */
    inline void setThreadAffinity(const ThreadAffinity& thread_affinity,
                                  const boost::optional<size_t>& job_id = boost::optional<size_t>())
    {
        //auto always returns and leaves affinity as is
        if (thread_affinity.ta_mode == ThreadAffinityMode::Auto)
            return;

        //only set if currently on cpu0? => return if on different cpu
        if (thread_affinity.ta_condition == ThreadAffinityCondition::CPU0)
        {
            int cpu_cur = sched_getcpu();
            if (cpu_cur != 0)
                return;
        }

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);

        if (thread_affinity.ta_mode == ThreadAffinityMode::NotCPU0)
        {
            //set affinity for all cpus except cpu0
            const int n = QThread::idealThreadCount();
            for (int cpu = 1; cpu < n; ++cpu)
                CPU_SET(cpu, &cpuset);
        }
        else if (thread_affinity.ta_mode == ThreadAffinityMode::AllCPUs)
        {
            //set affinity for all cpus
            const int n = QThread::idealThreadCount();
            for (int cpu = 0; cpu < n; ++cpu)
                CPU_SET(cpu, &cpuset);
        }
        else
        {
            //all modes setting affinity to a specific cpu
            int cpu = -1;

            if (thread_affinity.ta_mode == ThreadAffinityMode::CPURandom)
            {
                //set cpu to random number
                static thread_local std::mt19937 generator(std::random_device{}());
                std::uniform_int_distribution<int> distribution(0, QThread::idealThreadCount());
                cpu = distribution(generator);
            }
            else if (thread_affinity.ta_mode == ThreadAffinityMode::CPUModulo)
            {
                //set cpu to modulo
                if (job_id.has_value())
                    cpu = (int)(job_id.value() % (size_t)QThread::idealThreadCount());
            }
            else if (thread_affinity.ta_mode == ThreadAffinityMode::CPURandomNot0)
            {
                //set cpu to random number, skip 0
                static thread_local std::mt19937 generator(std::random_device{}());
                std::uniform_int_distribution<int> distribution(1, QThread::idealThreadCount());
                cpu = distribution(generator);
            }
            else if (thread_affinity.ta_mode == ThreadAffinityMode::CPUModuloNot0)
            {
                //set cpu to modulo, skip 0
                if (job_id.has_value())
                {
                    const int n = QThread::idealThreadCount() - 1;
                    cpu = 1 + (int)(job_id.value() % (size_t)n);
                }
            }
            else if (thread_affinity.ta_mode == ThreadAffinityMode::CPUIndex)
            {
                //set cpu to specified index
                cpu = thread_affinity.ta_cpu_index;
            }

            if (cpu < 0)
            {
                logerr << "failed to determine target cpu";
                return;
            }

            CPU_SET(cpu, &cpuset);
        }

        //set thread affinity
        pthread_t nativeThread = pthread_self();
        if (pthread_setaffinity_np(nativeThread, sizeof(cpu_set_t), &cpuset) != 0)
        {
            logerr << "failed to set thread affinity";
        }
    }
}
