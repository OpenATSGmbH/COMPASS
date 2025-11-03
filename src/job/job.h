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

#include "jobdefs.h"

#include "logger.h"

#include <memory>

//use std::async instead of QThreadPool
//#define USE_ASYNC_JOBS

#include <QObject>

#ifndef USE_ASYNC_JOBS
#include <QRunnable>
#endif

#include <boost/optional.hpp>

/**
 * @brief Encapsulates a work-package
 *
 * When created, is automatically added to the JobOrderer. Callbacks are performed either when the
 * Job was canceled (set as obsolete) or was completed (done). The work itself is defined in the
 * execute function, which must be overridden (and MUST set the done_ flag to true).
 *
 * Important: The Job and the contained data must be deleted in the callback functions.
 */
#ifdef USE_ASYNC_JOBS
class Job : public QObject
#else
class Job : public QObject, public QRunnable
#endif
{
    Q_OBJECT
signals:
    void doneSignal();
    void obsoleteSignal();

public:
    enum class ThreadAffinityCondition
    {
        Always = 0,
        CPU0
    };

    enum class ThreadAffinityMode
    {
        Auto = 0,
        Modulo,
        Random
    };

#ifdef USE_ASYNC_JOBS
    /// @brief Constructor
    Job(const std::string& name) : name_(name) {}
#else
    /// @brief Constructor
    Job(const std::string& name) : name_(name) { setAutoDelete(false); }
#endif
    /// @brief Destructor
    virtual ~Job() {}
  
    // @brief Main operation function
#ifdef USE_ASYNC_JOBS
    void run()
#else
    void run() override final
#endif
    {
        //set thread affinity
        job::setThreadAffinity(thread_affinity_, job_id_);

        //invoke derived
        run_impl();
    }

    void setJobID(size_t id)
    {
        job_id_ = id;
    }

    void setThreadAffinity(const job::ThreadAffinity& thread_affinity)
    {
        thread_affinity_ = thread_affinity;
    }

    bool started() { return started_; }
    // @brief Returns done flag
    bool done() { return done_; }
    void emitDone() { emit doneSignal(); }
    // @brief Sets obsolete flag
    virtual void setObsolete() 
    {
        logdbg << "Job: " << name_ << ": setObsolete";
        obsolete_ = true;
    }
    // @brief Returns obsolete flag
    bool obsolete() { return obsolete_; }
    //void emitObsolete() { emit doneSignal(); }

    const std::string& name() { return name_; }

protected:
    virtual void run_impl() = 0;

    void setThreadAffinity(ThreadAffinityMode mode, 
                           ThreadAffinityCondition condition)
    {
        //auto always returns and leaves config to whatever scheduler
        if (mode == ThreadAffinityMode::Auto)
            return;

        //only set if currently on cpu0? => return if on different cpu
        bool skip_cpu0 = false;
        if (condition == ThreadAffinityCondition::CPU0)
        {
            int cpu_cur = sched_getcpu();
            if (cpu_cur != 0)
                return;
            else
                skip_cpu0 = true;
        }

        int cpu = -1;
        if (mode == ThreadAffinityMode::Random)
        {
            static thread_local std::mt19937 generator(std::random_device{}());
            std::uniform_int_distribution<int> distribution(skip_cpu0 ? 1 : 0, QThread::idealThreadCount());
            cpu = distribution(generator);
        }
        else if (mode == ThreadAffinityMode::Modulo)
        {
            //use job id to evenly distribute over cpus via modulo
            if (job_id_.has_value())
            {
                int offs = skip_cpu0 ? 1 : 0;
                int n    = skip_cpu0 ? QThread::idealThreadCount() - 1 : QThread::idealThreadCount();
                cpu = offs + (int)(job_id_.value() % (size_t)n);
            }
        }

        if (cpu < 0)
        {
            logerr << "failed to set thread affinity of job " 
                   << job_id_.value() << ": cpu could not be determined";
            return;
        }

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu, &cpuset);

        pthread_t nativeThread = pthread_self();
        if (pthread_setaffinity_np(nativeThread, sizeof(cpu_set_t), &cpuset) != 0)
        {
            logerr << "failed to set thread affinity of job " 
                   << job_id_.value() << " to cpu" << cpu;
        }
    }

    std::string name_;
    ///
    bool started_{false};
    /// Done flag
    bool done_{false};
    /// Obsolete flag
    volatile bool obsolete_{false};

    //virtual void setDone() { done_ = true; }

    boost::optional<size_t> job_id_;
    job::ThreadAffinity     thread_affinity_;
};
