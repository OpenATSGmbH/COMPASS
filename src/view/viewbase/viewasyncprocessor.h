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

#include <QObject>

#include <functional>
#include <future>
#include <map>
#include <string>

/**
 * Runs view data processing asynchronously: a work function on a std::async worker
 * thread, then - if the result is still wanted - a commit function queued on the main
 * thread. Used by views to keep heavy per-load computations (geometry builds, row index
 * builds, layer scans) off the main thread, with only the Qt/model/scene side effects
 * committed on it.
 *
 * Semantics (the ones verified for the geographic view geometry build):
 * - Tasks run one at a time: each worker waits for the previously launched task first.
 * - invalidate() bumps a generation counter; results of tasks launched before it are
 *   discarded on completion, their commit never runs (the captured state is destroyed
 *   on the main thread instead).
 * - A work function that throws discards the task the same way.
 * - finishedSignal() is emitted on the main thread whenever the last outstanding task
 *   completed (committed or discarded).
 * - The destructor joins all outstanding workers; queued completions die with the object.
 *
 * All public methods must be called from the main thread.
 */
class ViewAsyncProcessor : public QObject
{
    Q_OBJECT

signals:
    // all outstanding tasks completed (committed or discarded); main thread
    void finishedSignal();

public:
    explicit ViewAsyncProcessor(QObject* parent = nullptr);
    virtual ~ViewAsyncProcessor();

    // runs work on a worker thread (serialized after previously launched tasks), then
    // commit on the main thread if the generation is still current. name is used for
    // logging only.
    void launch(const std::string& name,
                std::function<void()> work,
                std::function<void()> commit);

    // results of all in-flight tasks are stale: discard them on completion
    void invalidate();

    bool hasPending() const { return !pending_.empty(); }

    // joins all outstanding workers; their commits never run
    void waitForPending();

private:
    struct Task
    {
        unsigned int             generation = 0;
        std::string              name;
        std::function<void()>    commit;
        std::shared_future<void> future;
    };

    void completeTask(unsigned int task_id, bool ok);

    unsigned int                  generation_   = 0;
    unsigned int                  next_task_id_ = 0;
    std::map<unsigned int, Task>  pending_;
    std::shared_future<void>      chain_; // previously launched task - one at a time
};
