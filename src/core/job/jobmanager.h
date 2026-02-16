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

#include "configurable.h"
#include "singleton.h"
#include "job.h"

#ifndef Q_MOC_RUN
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#endif

#include <QMutex>
#include <QThread>

#include "util/tbbhack.h"

#include <list>
#include <memory>
#include <future>

class WorkerThread;

/**
 */
class JobManagerBase : public QThread
{
public:
    JobManagerBase();
    virtual ~JobManagerBase();

    // blocks started of later ones
    void addBlockingJob(std::shared_ptr<Job> job, 
                        const boost::optional<job::ThreadAffinity>& thread_affinity = boost::optional<job::ThreadAffinity>());
    // does not block start of later ones
    void addNonBlockingJob(std::shared_ptr<Job> job,
                           const boost::optional<job::ThreadAffinity>& thread_affinity = boost::optional<job::ThreadAffinity>());
    // only one db job can be active
    void addDBJob(std::shared_ptr<Job> job,
                  const boost::optional<job::ThreadAffinity>& thread_affinity = boost::optional<job::ThreadAffinity>());

    void cancelJob(std::shared_ptr<Job> job);

    bool hasAnyJobs() const;

    virtual bool hasBlockingJobs() const = 0;
    virtual bool hasNonBlockingJobs() const = 0;
    virtual bool hasDBJobs() const = 0;

    unsigned int numJobs() const;

    virtual unsigned int numBlockingJobs() const = 0;
    virtual unsigned int numNonBlockingJobs() const = 0;
    virtual unsigned int numDBJobs() const = 0;

    virtual int numThreads() const = 0;

    void shutdown();

protected:
    virtual void addBlockingJob_impl(std::shared_ptr<Job> job) = 0;
    virtual void addNonBlockingJob_impl(std::shared_ptr<Job> job) = 0;
    virtual void addDBJob_impl(std::shared_ptr<Job> job) = 0;

    virtual void handleBlockingJobs(bool debug) = 0;
    virtual void handleNonBlockingJobs(bool debug) = 0;
    virtual void handleDBJobs(bool debug) = 0;

    virtual void setJobsObsolete() = 0;

    void run();

    void setDefaultThreadAffinity(const job::ThreadAffinity& thread_affinity);
    void setDefaultThreadAffinityBlocking(const job::ThreadAffinity& thread_affinity);
    void setDefaultThreadAffinityNonBlocking(const job::ThreadAffinity& thread_affinity);
    void setDefaultThreadAffinityDB(const job::ThreadAffinity& thread_affinity);

    volatile bool stop_requested_;
    volatile bool stopped_;

    boost::posix_time::ptime last_update_time_;

private:
    size_t non_blocking_ids_ = 0;
    size_t blocking_ids_     = 0;
    size_t db_ids_           = 0;

    job::ThreadAffinity thread_affinity_default_blocking_;
    job::ThreadAffinity thread_affinity_default_nonblocking_;
    job::ThreadAffinity thread_affinity_default_db_;
};

/**
 */
class JobManagerAsync : public JobManagerBase
{
public:
    /**
     */
    struct AsyncJob
    {
        void exec();
        bool done() const;

        std::shared_ptr<Job> job_;
        std::future<bool>    future_;
        bool                 is_running_ = false;
    };

    typedef std::shared_ptr<AsyncJob> AsyncJobPtr;

    JobManagerAsync();
    virtual ~JobManagerAsync();

    bool hasBlockingJobs() const override;
    bool hasNonBlockingJobs() const override;
    bool hasDBJobs() const override;

    unsigned int numBlockingJobs() const override;
    unsigned int numNonBlockingJobs() const override;
    unsigned int numDBJobs() const override;

    int numThreads() const override;

protected:
    void addBlockingJob_impl(std::shared_ptr<Job> job) override;
    void addNonBlockingJob_impl(std::shared_ptr<Job> job) override;
    void addDBJob_impl(std::shared_ptr<Job> job) override;

    void handleBlockingJobs(bool debug) override;
    void handleNonBlockingJobs(bool debug) override;
    void handleDBJobs(bool debug) override;

    void setJobsObsolete() override;

private:
    AsyncJobPtr active_blocking_job_;
    tbb::concurrent_queue<AsyncJobPtr> blocking_jobs_;

    //AsyncJobPtr active_non_blocking_job_;
    tbb::concurrent_unordered_map<std::string,tbb::concurrent_queue<AsyncJobPtr>> non_blocking_jobs_;

    AsyncJobPtr active_db_job_;
    tbb::concurrent_queue<AsyncJobPtr> queued_db_jobs_;
};

/**
 */
class JobManagerThreadPool : public JobManagerBase
{
public:
    /**
     */
    struct AsyncJob
    {
        void exec();
        bool tryExec();
        bool done() const;

        std::shared_ptr<Job> job_;
        bool                 is_running_ = false;
    };

    typedef std::shared_ptr<AsyncJob> AsyncJobPtr;

    JobManagerThreadPool();
    virtual ~JobManagerThreadPool();

    bool hasBlockingJobs() const override;
    bool hasNonBlockingJobs() const override;
    bool hasDBJobs() const override;

    unsigned int numBlockingJobs() const override;
    unsigned int numNonBlockingJobs() const override;
    unsigned int numDBJobs() const override;

    int numThreads() const override;

protected:
    void addBlockingJob_impl(std::shared_ptr<Job> job) override;
    void addNonBlockingJob_impl(std::shared_ptr<Job> job) override;
    void addDBJob_impl(std::shared_ptr<Job> job) override;

    void handleBlockingJobs(bool debug) override;
    void handleNonBlockingJobs(bool debug) override;
    void handleDBJobs(bool debug) override;

    void setJobsObsolete() override;

private:
    AsyncJobPtr active_blocking_job_;
    tbb::concurrent_queue<AsyncJobPtr> blocking_jobs_;

    boost::mutex non_blocking_queue_mutex_;
    AsyncJobPtr active_non_blocking_job_;
    tbb::concurrent_queue<AsyncJobPtr> non_blocking_jobs_;

    AsyncJobPtr active_db_job_;
    tbb::concurrent_queue<AsyncJobPtr> queued_db_jobs_;

    // if true non-blocking jobs will be executed immediately on add,
    // otherwise execution will be postponed until a thread is free (recommended)
    bool exec_nb_jobs_immediately_ = false; 
};

/**
 */
#ifdef USE_ASYNC_JOBS
class JobManager : public JobManagerAsync, public Singleton, public Configurable
#else 
class JobManager : public JobManagerThreadPool, public Singleton, public Configurable
{
//    Q_OBJECT
//  signals:
//    void databaseBusy();
//    void databaseIdle();

public:
    virtual ~JobManager();

    static JobManager& instance()
    {
        static JobManager instance;
        return instance;
    }

protected:
    JobManager();
};

#endif
