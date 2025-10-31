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

#include "asynctask.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QLabel>
#include <QPushButton>
#include <QApplication>

#include <future>
#include "traced_assert.h"

/************************************************************************************
 * AsyncTask
 ************************************************************************************/

/**
*/
AsyncTask::AsyncTask()
:   task_progress_wrapper_(&task_progress_)
{
    connect(&task_progress_wrapper_, &AsyncTaskProgressWrapper::progressChanged, this, &AsyncTask::stateChanged, Qt::ConnectionType::QueuedConnection);
}

/**
*/
AsyncTask::~AsyncTask() = default;

/**
*/
void AsyncTask::setState(AsyncTaskState::State state)
{
    task_state_.state = state;
    emit stateChanged();
}

/**
*/
void AsyncTask::setError(const std::string& error)
{
    task_state_.error = error;
    setState(AsyncTaskState::State::Error);
}

/**
*/
void AsyncTask::run()
{
    task_state_.reset();

    setState(AsyncTaskState::State::Running);

    Result result;
    
    try
    {
        result = run_impl(task_state_, task_progress_wrapper_);
    }
    catch(const std::exception& e)
    {
        result = Result::failed(e.what());
    }
    catch(...)
    {
        result = Result::failed("Unknown error");
    }
    
    //task has been aborted?
    if (task_state_.isAborting())
    {
        setState(AsyncTaskState::State::Aborted);
        return;
    }

    //task failed?
    if (!result.ok())
    {
        setError(result.error());
        return;
    }

    //finish any unfinished progress (for optics)
    if (task_progress_.progress() > 0 && !task_progress_.isFinished())
        task_progress_wrapper_.setFinished(true);

    setState(AsyncTaskState::State::Done);
}

/**
*/
void AsyncTask::abort()
{
    setState(AsyncTaskState::State::Aborting);
}

/**
*/
bool AsyncTask::runAsyncDialog(bool auto_close,
                               QWidget* parent)
{
    //generate the managing dialog
    AsyncTaskDialog dlg(this, auto_close, parent);

    //start async task
    auto result = std::async(std::launch::async, 
        [ this ] ()
        {
            try
            {
                this->run();
                return this->taskState().isDone();
            }
            catch (const std::exception& e)
            {
                logerr << "exception '" << e.what() << "'";
                throw e;
            }
        });

    //run dialog
    dlg.exec();

    //wait for result (should not be needed)
    result.wait();

    return result.get();
}

/**
*/
bool AsyncTask::runAsync()
{
    //start async task
    auto result = std::async(std::launch::async, 
        [ this ] ()
        {
            try
            {
                this->run();
                return this->taskState().isDone();
            }
            catch (const std::exception& e)
            {
                logerr << "exception '" << e.what() << "'";
                throw e;
            }
        });

    while (result.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
        QApplication::processEvents();

    return result.get();
}

/************************************************************************************
 * AsyncFuncTask
 ************************************************************************************/

/**
*/
AsyncFuncTask::AsyncFuncTask(const Func& func,
                             const QString& title,
                             const QString& default_message,
                             bool can_abort) 
:   func_           (func           )
,   title_          (title          )
,   default_message_(default_message)
,   can_abort_      (can_abort      )
{
}

/**
*/
AsyncFuncTask::~AsyncFuncTask() = default;

/**
*/
bool AsyncFuncTask::canAbort() const
{
    return can_abort_;
}

/**
*/
QString AsyncFuncTask::title() const
{
    return title_;
}

/**
*/
QString AsyncFuncTask::defaultMessage() const
{
    return default_message_;
}

/**
*/
Result AsyncFuncTask::run_impl(const AsyncTaskState& state,
                               AsyncTaskProgressWrapper& progress)
{
    if (!func_)
        throw std::runtime_error("No func to execute in async task");

    return func_(state, progress);
}

/************************************************************************************
 * AsyncTaskDialog
 ************************************************************************************/

/**
*/
AsyncTaskDialog::AsyncTaskDialog(AsyncTask* task, 
                                 bool auto_close,
                                 QWidget* parent)
:   QDialog    (parent    )
,   task_      (task      )
,   auto_close_(auto_close)
{
    traced_assert(task);

    setWindowTitle(task_->title());
    setMinimumWidth(500);

    createUI();
    updateState();

    connect(task, &AsyncTask::stateChanged, this, &AsyncTaskDialog::updateState, Qt::ConnectionType::QueuedConnection);

    updateProgress();
}

/**
*/
AsyncTaskDialog::~AsyncTaskDialog() = default;

/**
*/
void AsyncTaskDialog::createUI()
{
    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    progress_bar_ = new QProgressBar;
    layout->addWidget(progress_bar_);

    msg_label_ = new QLabel;
    msg_label_->setWordWrap(true);
    layout->addWidget(msg_label_);

    layout->addStretch(1);

    QHBoxLayout* button_layout = new QHBoxLayout;
    button_layout->addStretch(1);

    cancel_button_ = new QPushButton("");
    button_layout->addWidget(cancel_button_);

    setButtonCancel();

    layout->addLayout(button_layout);

    connect(cancel_button_, &QPushButton::pressed, this, &AsyncTaskDialog::cancelButtonPressed);
}

/**
*/
void AsyncTaskDialog::setProgressMin()
{
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
}

/**
*/
void AsyncTaskDialog::setProgressMax()
{
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(100);
}

/**
*/
void AsyncTaskDialog::setProgressBusy()
{
    progress_bar_->setRange(0, 0);
}

/**
*/
void AsyncTaskDialog::setButtonCancel()
{
    cancel_button_->setText("Cancel");
    cancel_button_->setVisible(task_->canAbort());
}

/**
*/
void AsyncTaskDialog::setButtonClose()
{
    cancel_button_->setText("Close");
    cancel_button_->setEnabled(true);
    cancel_button_->setVisible(true);
}

/**
*/
void AsyncTaskDialog::setButtonCancelling()
{
    cancel_button_->setEnabled(false);
}

/**
*/
void AsyncTaskDialog::cancelButtonPressed()
{
    const auto& state = task_->taskState();

    if (state.isError())
    {
        //error => reject dialog and close
        reject();
    }
    else if (state.isDone())
    {
        //done (and no auto close) => accept dialog and close
        accept();
    }
    else if (state.isRunning())
    {
        //still running => cancel now (will trigger state update)
        task_->abort();
    }
}

/**
*/
void AsyncTaskDialog::updateProgress()
{
    const auto& progress = task_->taskProgress();

    //retrieve progress info
    int p     = -1;
    int step  = -1;
    int steps = -1;

    std::string msg;

    if (progress.flags & AsyncTaskProgress::Flags::Percent)
    {
        p = 100 * progress.percent;
    }
    if (progress.flags & AsyncTaskProgress::Flags::Steps)
    {
        if (p < 0)
            p = 100 * progress.stepsPercent();
        
        step  = progress.step;
        steps = progress.steps;
    }
    if (progress.flags & AsyncTaskProgress::Flags::Message)
    {
        msg = progress.message;
    }

    //config progress bar
    if (p >= 0)
    {
        if (progress_bar_->maximum() == 0)
            progress_bar_->setMaximum(100);
        progress_bar_->setValue(p);
    }
    else
    {
        setProgressBusy();
    }

    //config label
    QString txt = msg.empty() ? task_->defaultMessage() : QString::fromStdString(msg);

    if (steps > 0 && step >= 0)
        txt += " (Step " + QString::number(step) +  "/" + QString::number(steps) + ")";

    msg_label_->setText(txt);
}

/**
*/
void AsyncTaskDialog::updateState()
{
    const auto& state = task_->taskState();
    
    if (state.isAborting())
    {
        //set cancelling and update ui
        msg_label_->setText("Cancelling task...");
        setProgressBusy();
        setButtonCancelling();
    }
    else if (state.isAborted())
    {
        msg_label_->setText("Task cancelled");
        setProgressMax();

        //always close dialog
        reject();
    }
    else if (state.isError())
    {
        //set error message and update ui
        msg_label_->setText("Task failed: " + (state.error.empty() ? QString("Unknown error") : QString::fromStdString(state.error)));
        setProgressMax();
        setButtonClose();
    }
    else if (state.isRunning())
    {
        //normal progress update during run
        updateProgress();
    }
    else if (state.isDone())
    {
        //final progress update
        updateProgress();
        setProgressMax();

        //either directly accept or show close button
        if (auto_close_)
            accept();
        else
            setButtonClose();
    }
}
