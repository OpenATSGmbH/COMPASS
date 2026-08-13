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

#include <QTimer>

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

    bool was_idle = pending_.empty();

    Task& task = pending_[ task_id ];
    task.generation = generation_;
    task.name       = name;
    task.commit     = std::move(commit);

    loginf << "starting " << name;

    if (was_idle)
        emit startedSignal();

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
 * Completion of one task, invoked queued on the main thread. The completion is NOT
 * run here but drained via a zero timer, one commit per firing: several views'
 * commits arrive as posted events and Qt dispatches a posted-event batch back to
 * back, so seconds of apply work would run without a single native event in between
 * - unanswered window manager pings ("not responding") and no repaints. One commit
 * per timer firing lets the native loop breathe between commits. Deliberately NOT a
 * pump inside the commit: that dispatches other posted events too, and a dispatched
 * waiter (e.g. a runtime command gating on quiet views) then waits inside the pump
 * for a finishedSignal that can only be emitted once the pump returns - a deadlock.
 */
void ViewAsyncProcessor::completeTask(unsigned int task_id, bool ok)
{
    if (!pending_.count(task_id))
        return;

    completed_.emplace_back(task_id, ok);

    armDrain();
}

/**
 */
void ViewAsyncProcessor::armDrain()
{
    if (drain_armed_)
        return;

    drain_armed_ = true;

    QTimer::singleShot(0, this, &ViewAsyncProcessor::drainOneCompletion);
}

/**
 * Runs one queued completion: the commit, or a discard when the task is stale
 * (invalidate() ran while it worked) or its work failed. Re-arms itself while
 * completions remain, and reports all-finished once nothing is queued or pending.
 */
void ViewAsyncProcessor::drainOneCompletion()
{
    drain_armed_ = false;

    bool drained = false;

    if (!completed_.empty())
    {
        drained = true;

        auto [ task_id, ok ] = completed_.front();
        completed_.pop_front();

        auto it = pending_.find(task_id);
        if (it != pending_.end())
        {
            Task task = std::move(it->second);
            pending_.erase(it);

            //the worker posts this completion as its last action, but join anyway so
            //the thread handle is released deterministically
            if (task.future.valid())
                task.future.wait();

            bool stale = task.generation != generation_;

            if (!ok || stale)
            {
                //the commit never runs; the state captured in it dies here, on the
                //main thread
                loginf << "discarding " << task.name << (stale ? ", superseded" : ", failed");
            }
            else
            {
                task.commit();

                loginf << "done with " << task.name;
            }
        }
    }

    if (!completed_.empty())
    {
        armDrain();
        return;
    }

    //only a firing that actually ran a completion may report all-finished: a stale
    //firing after waitForPending() cleared the queues must not emit spuriously
    if (drained && pending_.empty())
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

    //queued but undrained completions die with the pending tasks - their commits
    //must not run (the owner may be shutting down)
    completed_.clear();
}
