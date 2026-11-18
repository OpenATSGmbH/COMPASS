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

#include "compass_task_export.h"

#include <QObject>

class TaskManager;
class TaskWidget;

class COMPASS_TASK_API Task : public QObject
{
    Q_OBJECT

  signals:
    void doneSignal();           // emitted when task is done

  public:
    Task(TaskManager& task_manager)
        : task_manager_(task_manager)
    {
    }
    virtual ~Task() {}

    bool done() const { return done_; }

    virtual void initTask() {} 
    virtual bool canRun() { return true; };
    virtual void run() = 0;  // to be overriden by tasks that can run
    virtual void stop() { stopped_= true; } // should also set done_, stopped_ only to be set after shutdown

    virtual void updateFeatures() {}

    TaskManager& manager() const { return task_manager_; }

    std::string tooltip() const { return tooltip_; }

    bool allowUserInteractions() const { return allow_user_interactions_; }
    void allowUserInteractions(bool value) { allow_user_interactions_ = value; }

  protected:
    bool stopped_ {false};
    bool done_{false};
    bool allow_user_interactions_{true};

    std::string tooltip_;

    TaskManager& task_manager_;
};

