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

#include "async.h"
#include "stringconv.h"
#include "logger.h"
#include "traced_assert.h"

#include <future>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "util/tbbhack.h"

#include <QProgressDialog>
#include <QLabel>
#include <QApplication>
#include <QTimer>
#include <QThread>

namespace Utils
{
namespace Async
{

/**
 * Executes the given task asynchronously and shows a dialog with progress information
 * until the task has finished.
 * @param task Task to execute, should return success as bool
 * @param done_cb Callback yielding the number of subtasks finished
 * @param steps Number of subtasks to finish inside 'task'
 * @param task_name Display name of the executed task
 * @param wait_msg Optional message to display in the progress dialog
 * @return True if the task has finished successfully, false otherwise
*/
bool waitDialogAsync(const std::function<bool()>& task,
                     const std::function<int()>& done_cb, 
                     int steps,
                     const std::string& task_name,
                     const std::string& wait_msg)
{
    traced_assert(task);
    traced_assert(done_cb);
    traced_assert(steps > 0);

    boost::posix_time::ptime start_time;
    boost::posix_time::ptime elapsed_time;

    start_time = boost::posix_time::microsec_clock::local_time();

    QProgressDialog dialog("", "", 0, steps);
    dialog.setWindowTitle(QString::fromStdString(task_name));
    dialog.setCancelButton(nullptr);
    dialog.setWindowModality(Qt::ApplicationModal);

    QLabel* progress_label = new QLabel("", &dialog);
    progress_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    dialog.setLabel(progress_label);

    boost::posix_time::time_duration time_diff;
    double elapsed_time_s;
    double time_per_eval, remaining_time_s;

    std::string remaining_time_str;

    bool all_done = false;

    auto outer_task = [ & ] ()
    {
        try
        {
            bool ok = task();
            all_done = true;
            return ok;
        }
        catch (const std::exception& e)
        {
            logerr << "exception '" << e.what() << "'";
            throw e;
        }
    };

    std::future<bool> pending_future = std::async(std::launch::async, outer_task);

    dialog.setValue(0);

    boost::posix_time::ptime last_elapsed_time = boost::posix_time::microsec_clock::local_time();
    elapsed_time = boost::posix_time::microsec_clock::local_time();
    int last_done = 0;

    boost::posix_time::time_duration tmp_time_diff;
    double tmp_elapsed_time_s;

    QTimer t;
    t.setInterval(200);

    auto update_cb = [ & ] ()
    {
        if (all_done)
        {
            //stop updates and close dialog
            t.stop();
            dialog.accept();
            return;
        }

        int done = done_cb();

        traced_assert(done <= steps);

        if (done && done != last_done)
        {
            elapsed_time = boost::posix_time::microsec_clock::local_time();

            time_diff = elapsed_time - start_time;
            elapsed_time_s = time_diff.total_milliseconds() / 1000.0;

            tmp_time_diff = elapsed_time - last_elapsed_time;
            tmp_elapsed_time_s = tmp_time_diff.total_milliseconds() / 1000.0;

            time_per_eval = 0.95 * time_per_eval + 0.05 * (tmp_elapsed_time_s / (double)(done - last_done));
            // halfnhalf
            remaining_time_s = (double)(steps - done) * time_per_eval;

            std::string msg;

            if (steps == 1)
            {
                msg = wait_msg.empty() ? "Computation in progress..." : wait_msg;
            }
            else
            {
                msg  = wait_msg.empty() ? "" : (wait_msg + "\n\n");
                msg += "Elapsed: " + String::timeStringFromDouble(elapsed_time_s, false) + "\n" +
                       "Remaining: "+String::timeStringFromDouble(remaining_time_s, false) + " (estimated)";
            }

            dialog.setLabelText(msg.c_str());
            dialog.setValue(done);

            last_done         = done;
            last_elapsed_time = elapsed_time;
        }
    };

    QObject::connect(&t, &QTimer::timeout, update_cb);

    t.start();     //start update callbacks
    dialog.exec(); //show dialog blocking

    //wait for future to end (just for safety)
    pending_future.wait();

    return pending_future.get();
}

/**
 * Executes the given task asynchronously and shows a dialog with progress information
 * until the task has finished.
 * @param task Task to execute, should return success as bool
 * @param task_name Display name of the executed task
 * @param wait_msg Optional message to display in the progress dialog
 * @return True if the task has finished successfully, false otherwise
*/
bool waitDialogAsync(const std::function<bool()>& task,
                     const std::string& task_name,
                     const std::string& wait_msg)
{
    traced_assert(task);

    bool done = false;

    auto outer_task = [ & ] ()
    {
        bool ok = task();
        done = true;
        return ok;
    };

    auto done_cb = [&] () 
    {
        return (done ? 1 : 0);
    };

    return waitDialogAsync(outer_task, done_cb, 1, task_name, wait_msg);
}

/**
 * Executes the given task asynchronously for various indices of an array 
 * and shows a dialog with progress information until all subtasks have finished.
 * @param task Task to execute for a specific index, should return success as bool
 * @param steps Number of subtasks to finish
 * @param task_name Display name of the executed task
 * @param wait_msg Optional message to display in the progress dialog
 * @return True if the task has finished successfully, false otherwise
*/
bool waitDialogAsyncArray(const std::function<bool(int)>& task,
                          int steps,
                          const std::string& task_name,
                          const std::string& wait_msg)
{
    traced_assert(task);
    traced_assert(steps > 0);

    std::vector<bool> done_flags(steps, false);

    auto mt_task = [&] () 
    {
        tbb::parallel_for(uint(0), (unsigned int)steps, [&] (unsigned int cnt)
        {
            task(cnt);
            done_flags[ cnt ] = true;
        });
        return true;
    };

    auto done_cb = [ & ] ()
    {
        int done = 0;

        for (auto done_it : done_flags)
        {
            if (done_it)
                ++done;
        }

        return done;
    };

    return waitDialogAsync(mt_task, done_cb, steps, task_name, wait_msg);
}

void waitAndProcessEventsFor (unsigned int milliseconds)
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    while ((boost::posix_time::microsec_clock::local_time()-start_time).total_milliseconds() < milliseconds)
    {
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents); //
        QThread::msleep(1);
    }
}

}
}
