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

#include "jobmanager.h"
#include "logger.h"
#include "stringconv.h"

#include <QTimer>
#include <QCoreApplication>
#include <QThreadPool>

using namespace Utils;

/*************************************************************************************
 * JobManagerBase
 *************************************************************************************/

/**
 */
JobManagerBase::JobManagerBase()
:   stop_requested_(false)
,   stopped_       (false)
{
}

/**
 */
JobManagerBase::~JobManagerBase() = default;

/**
 */
void JobManagerBase::setDefaultThreadAffinity(const job::ThreadAffinity& thread_affinity)
{
    setDefaultThreadAffinityBlocking(thread_affinity);
    setDefaultThreadAffinityNonBlocking(thread_affinity);
    setDefaultThreadAffinityDB(thread_affinity);
}

/**
 */
void JobManagerBase::setDefaultThreadAffinityBlocking(const job::ThreadAffinity& thread_affinity)
{
    thread_affinity_default_blocking_ = thread_affinity;
}

/**
 */
void JobManagerBase::setDefaultThreadAffinityNonBlocking(const job::ThreadAffinity& thread_affinity)
{
    thread_affinity_default_nonblocking_ = thread_affinity;
}

/**
 */
void JobManagerBase::setDefaultThreadAffinityDB(const job::ThreadAffinity& thread_affinity)
{
    thread_affinity_default_db_ = thread_affinity;
}

/**
 */
void JobManagerBase::addBlockingJob(std::shared_ptr<Job> job,
                                    const boost::optional<job::ThreadAffinity>& thread_affinity)
{
    logdbg << "start" << job->name() << " num " << numBlockingJobs();

    job->setJobID(blocking_ids_++);
    job->setThreadAffinity(thread_affinity.has_value() ? thread_affinity.value() : thread_affinity_default_blocking_);

    addBlockingJob_impl(job);
}

/**
 */
void JobManagerBase::addNonBlockingJob(std::shared_ptr<Job> job,
                                       const boost::optional<job::ThreadAffinity>& thread_affinity)
{
    logdbg << "start" << job->name() << " num " << numNonBlockingJobs();

    job->setJobID(non_blocking_ids_++);
    job->setThreadAffinity(thread_affinity.has_value() ? thread_affinity.value() : thread_affinity_default_nonblocking_);

    addNonBlockingJob_impl(job);
}

/**
 */
void JobManagerBase::addDBJob(std::shared_ptr<Job> job,
                              const boost::optional<job::ThreadAffinity>& thread_affinity)
{
    logdbg << "start" << job->name() << " num " << numDBJobs();

    job->setJobID(db_ids_++);
    job->setThreadAffinity(thread_affinity.has_value() ? thread_affinity.value() : thread_affinity_default_db_);

    addDBJob_impl(job);

    //emit databaseBusy();
}

/**
 */
void JobManagerBase::cancelJob(std::shared_ptr<Job> job) 
{ 
    job->setObsolete(); 
}

/**
 */
bool JobManagerBase::hasAnyJobs() const
{ 
    return hasBlockingJobs() || hasNonBlockingJobs() || hasDBJobs(); 
}

/**
 */
unsigned int JobManagerBase::numJobs() const
{ 
    return numBlockingJobs() + numNonBlockingJobs() + numDBJobs(); 
}

/**
 * Creates thread if possible.
 *
 * \exception std::runtime_error if thread already existed
 */
void JobManagerBase::run()
{
    logdbg << "start";

    //boost::posix_time::ptime log_time_ = boost::posix_time::microsec_clock::local_time();

    while (1)
    {
        if (stop_requested_ && !hasAnyJobs())
            break;

//        changed_ = false;
//        really_update_widget_ = false;

        bool debug = !last_update_time_.is_not_a_date_time()
                     && (boost::posix_time::microsec_clock::local_time() - last_update_time_).total_seconds() > 10;

        if (hasBlockingJobs())
            handleBlockingJobs(debug);

        if (hasNonBlockingJobs())
            handleNonBlockingJobs(debug);

        if (hasDBJobs())
            handleDBJobs(debug);

        if (last_update_time_.is_not_a_date_time() || debug)
            last_update_time_ = boost::posix_time::microsec_clock::local_time();

        if (debug && numJobs() > 0)
        {
            loginf << "start" 
                   << " blocking jobs " << numBlockingJobs() 
                   << " non-blocking jobs " << numNonBlockingJobs() 
                   << " db jobs " << numDBJobs();  
        }

//        if (!stop_requested_ && changed_ && !hasDBJobs())
//            emit databaseIdle();

        msleep(1);

//        if ((boost::posix_time::microsec_clock::local_time() - log_time_).total_seconds() > 1)
//        {
//            loginf << "jobs " << numJobs() << " db " << numDBJobs();
//            log_time_ = boost::posix_time::microsec_clock::local_time();
//        }
    }

    traced_assert(!hasBlockingJobs());
    traced_assert(!hasNonBlockingJobs());
    traced_assert(!hasDBJobs());

    stopped_ = true;

    loginf << "stopped";
}

/**
 */
void JobManagerBase::shutdown()
{
    logdbg << "setting jobs obsolete";

    stop_requested_ = true;

    setJobsObsolete();

    logdbg << "waiting on jobs to quit";

    while (hasAnyJobs())
    {
        logdbg << "waiting on jobs to finish: db " << hasDBJobs()
               << " blocking " << hasBlockingJobs() << " non-locking " << hasNonBlockingJobs();

        msleep(1000);
    }

    while (!stopped_)
    {
        logdbg << "waiting on run stop";
        msleep(1000);
    }

    traced_assert(numJobs() == 0);

    logdbg << "done";
}

/*************************************************************************************
 * JobManagerAsync
 *************************************************************************************/

/**
 */
void JobManagerAsync::AsyncJob::exec()
{
    traced_assert(job_);
    traced_assert(!is_running_);

    future_ = std::async(std::launch::async, [ this ] 
    { 
        this->job_->run();
        return true; 
    });

    is_running_ = true;
}

/**
 */
bool JobManagerAsync::AsyncJob::done() const
{
    traced_assert(job_);
    return job_->done(); //future_.valid()
}

/**
 */
JobManagerAsync::JobManagerAsync() 
:   JobManagerBase()
{
    setDefaultThreadAffinity(job::ThreadAffinity().mode(job::ThreadAffinityMode::AllCPUs)
                                                  .condition(job::ThreadAffinityCondition::Always));
}

/**
 */
JobManagerAsync::~JobManagerAsync() = default;

/**
 */
void JobManagerAsync::addBlockingJob_impl(std::shared_ptr<Job> job)
{
    std::shared_ptr<AsyncJob> j(new AsyncJob);
    j->job_ = job;

    blocking_jobs_.push(j);  // only add, do not start
}

/**
 */
void JobManagerAsync::addNonBlockingJob_impl(std::shared_ptr<Job> job)
{
    std::shared_ptr<AsyncJob> j(new AsyncJob);
    j->job_ = job;

    non_blocking_jobs_[job->name()].push(j);  // add and start
    j->exec();
}

/**
 */
void JobManagerAsync::addDBJob_impl(std::shared_ptr<Job> job)
{
    std::shared_ptr<AsyncJob> j(new AsyncJob);
    j->job_ = job;

    queued_db_jobs_.push(j);

    //emit databaseBusy();
}

/**
 */
bool JobManagerAsync::hasBlockingJobs() const
{ 
    return active_blocking_job_ || !blocking_jobs_.empty(); 
}

/**
 */
bool JobManagerAsync::hasNonBlockingJobs() const
{
    return !non_blocking_jobs_.empty();
}

/**
 */
bool JobManagerAsync::hasDBJobs() const
{ 
    return active_db_job_ || !queued_db_jobs_.empty(); 
}

/**
 */
unsigned int JobManagerAsync::numBlockingJobs() const
{
    return active_blocking_job_ ? blocking_jobs_.unsafe_size() + 1 : blocking_jobs_.unsafe_size();
}

/**
 */
unsigned int JobManagerAsync::numNonBlockingJobs() const
{
    unsigned int count{0};

    for (const auto& job_vec_it : non_blocking_jobs_)
        count += job_vec_it.second.unsafe_size();

    return count;
}

/**
 */
unsigned int JobManagerAsync::numDBJobs() const
{
    return active_db_job_ ? queued_db_jobs_.unsafe_size() + 1 : queued_db_jobs_.unsafe_size();
}

/**
 */
int JobManagerAsync::numThreads() const
{ 
    return QThreadPool::globalInstance()->activeThreadCount(); 
}

/**
 */
void JobManagerAsync::handleBlockingJobs(bool debug)
{
    if (active_blocking_job_)  // see if active one exists, finalize if possible
    {
        if (active_blocking_job_->done())
        {
            loginf << "flushing blocking done job";

            if (!stop_requested_)
            {
                active_blocking_job_->job_->emitDone();
                loginf << "blocking job " << active_blocking_job_->job_->name() << " emitted done ";
            }

            active_blocking_job_ = nullptr;
        }
    }

    if (!active_blocking_job_ && !blocking_jobs_.empty())
    {
        if (blocking_jobs_.try_pop(active_blocking_job_))
        {
            loginf << "running blocking job";

            traced_assert(!active_blocking_job_->is_running_ && !active_blocking_job_->job_->started());
            active_blocking_job_->exec();

            //changed_ = true;
            //really_update_widget_ = !hasBlockingJobs();
        }
    }
}

/**
 */
void JobManagerAsync::handleNonBlockingJobs(bool debug)
{
    // while (1)
    // {
    //     if (active_non_blocking_job_)  // see if active one exists
    //     {
    //         if (active_non_blocking_job_->done())  // can be finalized
    //         {
    //             logdbg << "flushing non-blocking done job";

    //             if (!stop_requested_)
    //             {
    //                 active_non_blocking_job_->job_->emitDone();
    //                 logdbg << "done non-blocking job emitted " + active_non_blocking_job_->job_->name();
    //             }

    //             active_non_blocking_job_ = nullptr;
    //         }
    //         else
    //             break;  // active not done, quit loop
    //     }

    //     traced_assert(!active_non_blocking_job_);

    //     if (!non_blocking_jobs_.empty())
    //     {
    //         if (non_blocking_jobs_.try_pop(active_non_blocking_job_))
    //         {
    //             // QThreadPool::globalInstance()->start(active_non_blocking_job_.get());

    //             //changed_ = true;
    //             //really_update_widget_ = !hasNonBlockingJobs();
    //         }
    //     }
    //     else  // no jobs left
    //         break;
    // }

    for (auto it = non_blocking_jobs_.begin(); it != non_blocking_jobs_.end();/*no increment here*/)
    {
        auto &queue = it->second;
        AsyncJobPtr front_job;

        // try to pop one element
        if (queue.try_pop(front_job))
        {
            // if it was *not* done, push it right back
            if (!front_job->done())
            {
                queue.push(front_job);
                continue;
            }

            // otherwise (front_job->done()==true) -> we've removed it permanently
        }

        if (queue.empty())
            it = non_blocking_jobs_.unsafe_erase(it); // erase returns the next iterator
        else
            ++it;
    }
}

/**
 */
void JobManagerAsync::handleDBJobs(bool debug)
{
    if (active_db_job_)  // see if active one exists
    {
        if (active_db_job_->done())
        {
//            if (active_db_job_->obsolete())
//            {
//                logdbg << "flushing obsolete non-blocking job";

//                if (!stop_requested_)
//                    active_db_job_->emitObsolete();
//            }

            logdbg << "flushing non-blocking done job";

            if (!stop_requested_)
            {
                active_db_job_->job_->emitDone();
                logdbg << "done non-blocking job emitted " + active_db_job_->job_->name();
            }

            active_db_job_ = nullptr;
        }
    }

    if (!active_db_job_ && !queued_db_jobs_.empty())
    {
        if (queued_db_jobs_.try_pop(active_db_job_))
        {
            active_db_job_->exec();

            //changed_ = true;
            //really_update_widget_ = !hasDBJobs();
        }
    }
}

/**
 */
void JobManagerAsync::setJobsObsolete()
{
    if (active_db_job_)
        active_db_job_->job_->setObsolete();

    for (auto job_it = queued_db_jobs_.unsafe_begin(); job_it != queued_db_jobs_.unsafe_end();
         ++job_it)
        (*job_it)->job_->setObsolete();

    if (active_blocking_job_)
        active_blocking_job_->job_->setObsolete();

    for (auto job_it = blocking_jobs_.unsafe_begin(); job_it != blocking_jobs_.unsafe_end();
         ++job_it)
        (*job_it)->job_->setObsolete();

    for (const auto& job_vec_it : non_blocking_jobs_)
    {
        for (auto job_it = job_vec_it.second.unsafe_begin(); job_it != job_vec_it.second.unsafe_end(); ++job_it)
            (*job_it)->job_->setObsolete();
    }
}

/*************************************************************************************
 * JobManagerThreadPool
 *************************************************************************************/

/**
 */
void JobManagerThreadPool::AsyncJob::exec()
{
    traced_assert(job_);
    traced_assert(!is_running_);

    QThreadPool::globalInstance()->start(job_.get());

    is_running_ = true;
}

/**
 */
bool JobManagerThreadPool::AsyncJob::tryExec()
{
    traced_assert(job_);
    traced_assert(!is_running_);

    bool ok = QThreadPool::globalInstance()->tryStart(job_.get());

    is_running_ = ok;

    return ok;
}

/**
 */
bool JobManagerThreadPool::AsyncJob::done() const
{
    traced_assert(job_);
    return is_running_ && job_->done(); //future_.valid()
}

/**
 */
JobManagerThreadPool::JobManagerThreadPool()
:   JobManagerBase()
,   exec_nb_jobs_immediately_(false)
{
    setDefaultThreadAffinityBlocking(job::ThreadAffinity().mode(job::ThreadAffinityMode::AllCPUs)
                                                          .condition(job::ThreadAffinityCondition::Always));
    setDefaultThreadAffinityNonBlocking(job::ThreadAffinity().mode(job::ThreadAffinityMode::AllCPUs)
                                                             .condition(job::ThreadAffinityCondition::Always));
    setDefaultThreadAffinityDB(job::ThreadAffinity().mode(job::ThreadAffinityMode::AllCPUs)
                                                    .condition(job::ThreadAffinityCondition::Always));
}

/**
 */
JobManagerThreadPool::~JobManagerThreadPool() = default;

/**
 */
void JobManagerThreadPool::addBlockingJob_impl(std::shared_ptr<Job> job)
{
    std::shared_ptr<AsyncJob> j(new AsyncJob);
    j->job_ = job;

    blocking_jobs_.push(j);  // only add, do not start
}

/**
 */
void JobManagerThreadPool::addNonBlockingJob_impl(std::shared_ptr<Job> job)
{
    std::shared_ptr<AsyncJob> j(new AsyncJob);
    j->job_ = job;

    //queue iterated regularly? => needs further protection
    if (!exec_nb_jobs_immediately_)
        non_blocking_queue_mutex_.lock();

    non_blocking_jobs_.push(j); 

    //queue iterated regularly? => needs further protection
    if (!exec_nb_jobs_immediately_)
        non_blocking_queue_mutex_.unlock();

    //immediately start job?
    if (exec_nb_jobs_immediately_)
        j->exec();
}

/**
 */
void JobManagerThreadPool::addDBJob_impl(std::shared_ptr<Job> job)
{
    std::shared_ptr<AsyncJob> j(new AsyncJob);
    j->job_ = job;

    queued_db_jobs_.push(j);
}

/**
 */
bool JobManagerThreadPool::hasBlockingJobs() const
{ 
    return active_blocking_job_ || !blocking_jobs_.empty(); 
}

/**
 */
bool JobManagerThreadPool::hasNonBlockingJobs() const
{
    return active_non_blocking_job_ || !non_blocking_jobs_.empty();
}

/**
 */
bool JobManagerThreadPool::hasDBJobs() const
{ 
    return active_db_job_ || !queued_db_jobs_.empty(); 
}

/**
 */
unsigned int JobManagerThreadPool::numBlockingJobs() const
{
    return active_blocking_job_ ? blocking_jobs_.unsafe_size() + 1 : blocking_jobs_.unsafe_size();
}

/**
 */
unsigned int JobManagerThreadPool::numNonBlockingJobs() const
{
    return active_non_blocking_job_ ? non_blocking_jobs_.unsafe_size() + 1
                                    : non_blocking_jobs_.unsafe_size();
}

/**
 */
unsigned int JobManagerThreadPool::numDBJobs() const
{
    return active_db_job_ ? queued_db_jobs_.unsafe_size() + 1 : queued_db_jobs_.unsafe_size();
}

/**
 */
int JobManagerThreadPool::numThreads() const 
{ 
    return QThreadPool::globalInstance()->activeThreadCount(); 
}

/**
 */
void JobManagerThreadPool::handleBlockingJobs(bool debug)
{
    if (active_blocking_job_)  // see if active one exists, finalize if possible
    {
        if (active_blocking_job_->done())
        {
//            if (active_blocking_job_->obsolete())
//            {
//                logdbg << "flushing obsolete blocking job";

//                if (!stop_requested_)
//                    active_blocking_job_->emitObsolete();
//            }

            logdbg << "flushing blocking done job";

            if (!stop_requested_)
            {
                active_blocking_job_->job_->emitDone();
                logdbg << "done blocking job emitted " +
                              active_blocking_job_->job_->name();
            }

            active_blocking_job_ = nullptr;
        }
    }

    if (!active_blocking_job_ && !blocking_jobs_.empty())
    {
        if (blocking_jobs_.try_pop(active_blocking_job_))
        {
            traced_assert(!active_blocking_job_->job_->started());
            active_blocking_job_->exec();

            //changed_ = true;
            //really_update_widget_ = !hasBlockingJobs();
        }
    }
}

/**
 */
void JobManagerThreadPool::handleNonBlockingJobs(bool debug)
{
    //try to start all non-running jobs
    size_t num_started = 0;
    size_t num_waiting = 0;
    size_t num_running = 0;
    size_t num_total   = 0;

    if (!exec_nb_jobs_immediately_)
    {
        //try to start postponed jobs 
        auto checkJob = [ & ] (AsyncJobPtr& job)
        {
            if (!job)
                return;

            if (!job->is_running_)
            {
                //try to start on a free thread
                if (job->tryExec())
                    ++num_started; //thread available, could be started
                else
                    ++num_waiting; //no free thread, still waiting
            }
            else
            {
                ++num_running; //already running
            }
        };

        //check active job
        checkJob(active_non_blocking_job_);

        //check queued jobs (uses mutex for safe queue iteration)
        non_blocking_queue_mutex_.lock();
        if (!non_blocking_jobs_.empty())
        {
            auto it    = non_blocking_jobs_.unsafe_begin();
            auto itend = non_blocking_jobs_.unsafe_end();
            for (;it != itend; ++it)
                checkJob(*it);
        }
        non_blocking_queue_mutex_.unlock();

        num_total = num_started + num_waiting + num_running;
    }

    if (debug && num_total > 0)
    {
        logdbg << "start"
               << " started: " << num_started
               << " waiting: " << num_waiting
               << " running: " << num_running;
    }

    while (1)
    {
        if (active_non_blocking_job_)  // see if active one exists
        {
            if (active_non_blocking_job_->done())  // can be finalized
            {
//                if (active_non_blocking_job_->obsolete())
//                {
//                    logdbg << "flushing obsolete non-blocking job";

//                    if (!stop_requested_)
//                        active_non_blocking_job_->emitObsolete();
//                }

                logdbg << "flushing non-blocking done job";

                if (!stop_requested_)
                {
                    active_non_blocking_job_->job_->emitDone();
                    logdbg << "done non-blocking job emitted " +
                                  active_non_blocking_job_->job_->name();
                }

                active_non_blocking_job_ = nullptr;
            }
            else
            {
                break;  // active not done, quit loop
            }
        }

        traced_assert(!active_non_blocking_job_);

        if (!non_blocking_jobs_.empty())
        {
            if (non_blocking_jobs_.try_pop(active_non_blocking_job_))
            {
                // QThreadPool::globalInstance()->start(active_non_blocking_job_.get(), QThread::HighPriority);

                //changed_ = true;
                //really_update_widget_ = !hasNonBlockingJobs();
            }
        }
        else  // no jobs left
        {
            break;
        }
    }
}

/**
 */
void JobManagerThreadPool::handleDBJobs(bool debug)
{
    if (active_db_job_)  // see if active one exists
    {
        if (active_db_job_->done())
        {
//            if (active_db_job_->obsolete())
//            {
//                logdbg << "flushing obsolete non-blocking job";

//                if (!stop_requested_)
//                    active_db_job_->emitObsolete();
//            }

            logdbg << "flushing non-blocking done job";

            if (!stop_requested_)
            {
                active_db_job_->job_->emitDone();
                logdbg << "done non-blocking job emitted " +
                              active_db_job_->job_->name();
            }

            active_db_job_ = nullptr;
        }
    }

    if (!active_db_job_ && !queued_db_jobs_.empty())
    {
        if (queued_db_jobs_.try_pop(active_db_job_))
        {
            active_db_job_->exec();

            //changed_ = true;
            //really_update_widget_ = !hasDBJobs();
        }
    }
}

/**
 */
void JobManagerThreadPool::setJobsObsolete()
{
    if (active_db_job_)
        active_db_job_->job_->setObsolete();

    for (auto job_it = queued_db_jobs_.unsafe_begin(); job_it != queued_db_jobs_.unsafe_end();
         ++job_it)
        (*job_it)->job_->setObsolete();

    if (active_blocking_job_)
        active_blocking_job_->job_->setObsolete();

    for (auto job_it = blocking_jobs_.unsafe_begin(); job_it != blocking_jobs_.unsafe_end();
         ++job_it)
        (*job_it)->job_->setObsolete();

    if (active_non_blocking_job_)
        active_non_blocking_job_->job_->setObsolete();

    for (auto job_it = non_blocking_jobs_.unsafe_begin(); job_it != non_blocking_jobs_.unsafe_end();
         ++job_it)
        (*job_it)->job_->setObsolete();
}

/*************************************************************************************
 * JobManager
 *************************************************************************************/

/**
 */
JobManager::JobManager()
:   Configurable("JobManager", "JobManager0", 0, "threads.json")
{
    logdbg << "start";
}

/**
 */
JobManager::~JobManager() 
{ 
    logdbg << "start"; 
}
