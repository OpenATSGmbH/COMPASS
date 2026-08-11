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

#include "viewasyncprocessor.h"

#include "logger.h"

/**
 */
ViewAsyncProcessor::ViewAsyncProcessor(QObject* parent)
    : QObject(parent)
{
}

/**
 */
ViewAsyncProcessor::~ViewAsyncProcessor()
{
    waitForPending();
}

/**
 */
void ViewAsyncProcessor::launch(const std::string& name,
                                std::function<void()> work,
                                std::function<void()> commit)
{
    unsigned int task_id = next_task_id_++;

    Task& task = pending_[ task_id ];
    task.generation = generation_;
    task.name       = name;
    task.commit     = std::move(commit);

    loginf << "starting " << name;

    //tasks run one at a time: each worker waits for the previously launched task first
    std::shared_future<void> previous_task = chain_;

    task.future = std::async(std::launch::async,
        [ this, task_id, previous_task, work = std::move(work) ] ()
    {
        if (previous_task.valid())
            previous_task.wait();

        bool ok = true;

        try
        {
            work();
        }
        catch (const std::exception& ex)
        {
            logerr << "view processing failed: " << ex.what();
            ok = false;
        }
        catch (...)
        {
            logerr << "view processing failed";
            ok = false;
        }

        //completion on the main thread; Qt drops the call if the processor is destroyed first
        QMetaObject::invokeMethod(this,
                                  [ this, task_id, ok ] () { completeTask(task_id, ok); },
                                  Qt::QueuedConnection);
    });

    chain_ = task.future;
}

/**
 */
void ViewAsyncProcessor::invalidate()
{
    ++generation_;
}

/**
 * Completion of one task, invoked queued on the main thread. Runs the commit, or
 * discards the result when the task is stale (invalidate() ran while it worked) or
 * its work failed.
 */
void ViewAsyncProcessor::completeTask(unsigned int task_id, bool ok)
{
    auto it = pending_.find(task_id);
    if (it == pending_.end())
        return;

    Task task = std::move(it->second);
    pending_.erase(it);

    //the worker posts this completion as its last action, but join anyway so the
    //thread handle is released deterministically
    if (task.future.valid())
        task.future.wait();

    bool stale = task.generation != generation_;

    if (!ok || stale)
    {
        //the commit never runs; the state captured in it dies here, on the main thread
        loginf << "discarding " << task.name << (stale ? ", superseded" : ", failed");
    }
    else
    {
        task.commit();

        loginf << "done with " << task.name;
    }

    if (pending_.empty())
        emit finishedSignal();
}

/**
 * Joins all outstanding workers. Their commits never run; queued completions find no
 * pending record (or die with the object in the destructor).
 */
void ViewAsyncProcessor::waitForPending()
{
    for (auto& p : pending_)
        if (p.second.future.valid())
            p.second.future.wait();

    pending_.clear();
}
