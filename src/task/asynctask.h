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

#include "result.h"

#include <QObject>
#include <QDialog>

#include "traced_assert.h"
#include <functional>

class QProgressBar;
class QLabel;
class QPushButton;

/**
 * Progress struct holding various types of progress and a status message.
*/
struct AsyncTaskProgress
{
    enum Flags
    {
        Message = 1 << 0,
        Steps   = 1 << 1,
        Percent = 1 << 2
    };

    void reset()
    {
        flags = 0;
    }
    void setMessage(const std::string& m)
    {
        message = m;
        flags |= Flags::Message;
    }
    void setSteps(int s, int stotal)
    {
        step  = s;
        steps = stotal;
        flags |= Flags::Steps;
    }
    void setPercent(float p)
    {
        percent = p;
        flags |= Flags::Percent;
    }

    float stepsPercent() const
    {
        if (step < 0 || steps <= 0)
            return 0.0f;
        return (float)step / (float)steps;
    };

    std::string message;  // status message
    int         step;     // current step
    int         steps;    // total steps
    float       percent;  // progress in percent

    int flags = 0;
};

/**
 * Wraps an AsyncTaskProgress and informs about progress changes.
*/
class AsyncTaskProgressWrapper : public QObject
{
    Q_OBJECT
public:
    AsyncTaskProgressWrapper(AsyncTaskProgress* p) : progress_(p) { traced_assert(progress_); }
    ~AsyncTaskProgressWrapper() = default;

    AsyncTaskProgress& progress() { return *progress_; }

    void reset(bool submit = false)
    {
        progress_->reset();
        if (submit)
            submitChanges();
    }
    
    void setMessage(const std::string& m,
                    bool reset_progress = false,
                    bool submit = false)
    {
        if (reset_progress)
            reset(false);

        progress_->setMessage(m);

        if (submit)
            submitChanges();
    }

    void setSteps(int s, 
                  int stotal,
                  bool reset_progress = false,
                  bool submit = false)
    {
        if (reset_progress)
            reset(false);

        progress_->setSteps(s, stotal);

        if (submit)
            submitChanges();
    }

    void setPercent(float p,
                    bool reset_progress = false,
                    bool submit = false)
    {
        if (reset_progress)
            reset(false);

        progress_->setPercent(p);

        if (submit)
            submitChanges();
    }

    void submitChanges()
    {
        emit progressChanged();
    }

signals:
    void progressChanged();

private:
    AsyncTaskProgress* progress_ = nullptr;
};

/**
 * Current state of an AsyncTask.
*/
struct AsyncTaskState 
{
    enum class State
    {
        Fresh = 0,
        Running,
        Done,
        Error,
        Aborting,
        Aborted
    };

    bool isAborting() const { return (state == State::Aborting); }
    bool isAborted() const { return (state == State::Aborted); }
    bool isError() const { return (state == State::Error); }
    bool isRunning() const { return (state == State::Running); }
    bool isDone() const { return (state == State::Done); }
    
    void reset()
    {
        state = State::Fresh;
        error = "";
    }

    void setError(const std::string& err)
    {
        state = State::Error;
        error = err;
    }

    State       state = State::Fresh;
    std::string error;
};

/**
 * An async task.
*/
class AsyncTask : public QObject
{
    Q_OBJECT
public:
    AsyncTask();
    virtual ~AsyncTask();

    void run();
    void abort();

    virtual bool canAbort() const { return false; }
    virtual QString title() const { return "Task"; }
    virtual QString defaultMessage() const { return "Working on task"; }

    const AsyncTaskState& taskState() const { return task_state_; }
    const AsyncTaskProgress& taskProgress() const { return task_progress_; }

    bool runAsyncDialog(bool auto_close = true,
                        QWidget* parent = nullptr);
    bool runAsync();

signals:
    void stateChanged();

protected:
    virtual Result run_impl(const AsyncTaskState& state,
                            AsyncTaskProgressWrapper& progress) = 0;
private:
    void setState(AsyncTaskState::State state);
    void setError(const std::string& error);

    AsyncTaskState           task_state_;
    AsyncTaskProgress        task_progress_;
    AsyncTaskProgressWrapper task_progress_wrapper_;
};

/**
 * Async task execturing a passed functional.
*/
class AsyncFuncTask : public AsyncTask
{
public:
    typedef std::function<Result(const AsyncTaskState&, AsyncTaskProgressWrapper&)> Func;

    AsyncFuncTask(const Func& func,
                  const QString& title,
                  const QString& default_message,
                  bool can_abort);
    virtual ~AsyncFuncTask();

    virtual bool canAbort() const override;
    virtual QString title() const override;
    virtual QString defaultMessage() const override;

protected:
    virtual Result run_impl(const AsyncTaskState& state,
                            AsyncTaskProgressWrapper& progress) override;
private:
    Func    func_;
    QString title_;
    QString default_message_;
    bool    can_abort_       = true;
};

/**
 * A dialog showing task progress during the execution of an AsyncTask.
*/
class AsyncTaskDialog : public QDialog
{
public:
    AsyncTaskDialog(AsyncTask* task, 
                    bool auto_close = true,
                    QWidget* parent = nullptr);
    virtual ~AsyncTaskDialog();

private:
    void updateState();
    void updateProgress();
    void createUI();

    void setProgressMin();
    void setProgressMax();
    void setProgressBusy();

    void setButtonCancel();
    void setButtonClose();
    void setButtonCancelling();

    void cancelButtonPressed();

    AsyncTask*    task_          = nullptr;
    bool          auto_close_    = true;

    QProgressBar* progress_bar_  = nullptr;
    QLabel*       msg_label_     = nullptr;
    QPushButton*  cancel_button_ = nullptr;
};
