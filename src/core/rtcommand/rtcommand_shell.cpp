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

#include "rtcommand_shell.h"
#include "rtcommand_registry.h"
#include "rtcommand_manager.h"
#include "rtcommand_response.h"
#include "files.h"

#include <QTextEdit>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QShortcut>
#include <QListWidget>
#include <QPushButton>
#include <QToolButton>
#include <QApplication>
#include <QClipboard>

namespace rtcommand
{

namespace
{
    QIcon icon(const std::string& fn) 
    {
        return Utils::Files::IconProvider::getIcon(fn);
    };
}

/**************************************************************************************
 * RTCommandBacklogWidget
 **************************************************************************************/

/** 
 */
RTCommandBacklogWidget::RTCommandBacklogWidget(QWidget* parent)
:   QWidget(parent)
{
    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    command_list_ = new QListWidget(this);
    command_list_->setSelectionMode(QListWidget::SelectionMode::ExtendedSelection);
    layout->addWidget(command_list_);

    //QHBoxLayout* layout_h = new QHBoxLayout;
    //layout->addLayout(layout_h);
    //layout_h->addStretch(1);
    
    connect(command_list_, &QListWidget::itemDoubleClicked, this, &RTCommandBacklogWidget::acceptCommandItem, Qt::ConnectionType::QueuedConnection);

    updateList();
}

/**
 */
void RTCommandBacklogWidget::updateList()
{
    auto cmds = RTCommandManager::instance().commandBacklog();

    command_list_->clear();

    for (const auto& cmd : cmds)
        command_list_->addItem(QString::fromStdString(cmd));

    if (command_list_->count() > 0)
        command_list_->setCurrentRow(command_list_->count() - 1);
}

/** 
 */
void RTCommandBacklogWidget::acceptCommandItem(QListWidgetItem* item)
{
    emit commandSelected(item->text());
}

/**************************************************************************************
 * RTCommandBacklogDialog
 **************************************************************************************/

/** 
 */
RTCommandBacklogDialog::RTCommandBacklogDialog(QWidget* parent)
:   QDialog(parent)
{
    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    RTCommandBacklogWidget* widget = new RTCommandBacklogWidget(this);
    layout->addWidget(widget);

    auto cb = [ = ] (const QString& cmd)
    {
        this->selected_command_ = cmd;
        this->accept();
    };

    connect(widget, &RTCommandBacklogWidget::commandSelected, cb);
}

/**************************************************************************************
 * RTCommandShell
 **************************************************************************************/

/** 
 */
RTCommandShell::RTCommandShell(QWidget* parent)
:   QWidget(parent)
{
    QVBoxLayout* layout = new QVBoxLayout;
    layout->setSpacing(0);
    setLayout(layout);

    cmd_shell_ = new QTextEdit;
    cmd_shell_->setReadOnly(true);
    cmd_shell_->setFrameStyle(QTextEdit::NoFrame);
    layout->addWidget(cmd_shell_);

    QHBoxLayout* layout_h = new QHBoxLayout;
    layout_h->setSpacing(0);
    layout->addLayout(layout_h);

    cmd_edit_ = new QLineEdit;
    cmd_edit_->setFocusPolicy(Qt::FocusPolicy::StrongFocus);
    cmd_edit_->setCursorPosition(0);
    layout_h->addWidget(cmd_edit_);

    backlog_button_ = new QToolButton;
    backlog_button_->setIcon(icon("edit_old.png"));
    backlog_button_->setToolTip("Show command backlog");
    layout_h->addWidget(backlog_button_);

    copy_button_ = new QToolButton;
    copy_button_->setIcon(icon("save.png"));
    copy_button_->setToolTip("Copy last reply");
    layout_h->addWidget(copy_button_);

    connect(cmd_edit_, &QLineEdit::returnPressed, this, &RTCommandShell::processCommand);
    connect(backlog_button_, &QToolButton::pressed, this, &RTCommandShell::showBacklog);
    connect(copy_button_, &QToolButton::pressed, this, &RTCommandShell::copyLastReply);

    auto s_last = new QShortcut(cmd_edit_);
    s_last->setKey(Qt::Key_Up);
    connect(s_last, &QShortcut::activated, this, &RTCommandShell::lastCmd);

    auto s_next = new QShortcut(cmd_edit_);
    s_next->setKey(Qt::Key_Down);
    connect(s_next, &QShortcut::activated, this, &RTCommandShell::nextCmd);

    connect(&RTCommandManager::instance(), &RTCommandManager::shellCommandProcessed, this, &RTCommandShell::receiveResult);

    cmd_edit_->setFocus();
}

/**
*/
void RTCommandShell::enableCmdLine(bool enable)
{
    cmd_edit_->setEnabled(enable);
    backlog_button_->setEnabled(enable);
}

/**
*/
void RTCommandShell::resetLocalBacklog()
{
    local_backlog_.clear();
    local_backlog_index_ = -1;
}

/**
*/
void RTCommandShell::updateCommandFromLocalBacklog()
{
    cmd_edit_->setText("");
    cmd_edit_->setCursorPosition(0);

    if (local_backlog_.empty() || local_backlog_index_ < 0)
        return;

    cmd_edit_->setText(QString::fromStdString(local_backlog_.at(local_backlog_index_)));
    cmd_edit_->setCursorPosition(cmd_edit_->text().count());
}

/**
*/
void RTCommandShell::showBacklog()
{
    RTCommandBacklogDialog dlg(this);
    if (dlg.exec() == QDialog::Accepted)
    {
        const QString& cmd = dlg.selectedCommand();
        cmd_edit_->setText(cmd);
        cmd_edit_->setCursorPosition(cmd.count());
    }
}

/**
*/
void RTCommandShell::copyLastReply() const
{
    if (last_reply_.isEmpty())
        return;

    qApp->clipboard()->setText(last_reply_);
}

/**
*/
void RTCommandShell::processCommand()
{
    QString line = cmd_edit_->text().trimmed();
    
    cmd_edit_->setText("");
    last_reply_ = "";
    copy_button_->setEnabled(false);
    
    resetLocalBacklog();

    if (line.isEmpty())
        return;

    auto issue_info = RTCommandManager::instance().addCommand(line.toStdString(), RTCommandManager::Source::Shell);

    log(line, LogType::Plain);

    if (issue_info.issued)
    {
        enableCmdLine(false);
    }
    else
    {
        std::string msg = RTCommandResponse(issue_info).errorToString();
        logResult(msg, true);
    }

    cmd_edit_->setFocus();
}

/**
*/
void RTCommandShell::receiveResult(const QString& msg, const QString& data, bool error)
{
    logResult(error ? msg.toStdString() : data.toStdString(), error);

    enableCmdLine(true);
    cmd_edit_->setFocus();

    last_reply_ = data;

    copy_button_->setEnabled(!last_reply_.isEmpty());
}

/**
*/
void RTCommandShell::logResult(std::string msg, bool error)
{
    if (!error)
    {
        if (msg.empty())
        {
            log("success", LogType::Success, true);
            //log("", LogType::Success);
        }
        else
        {
            log(QString::fromStdString(msg), LogType::Success, true);
            //log("", LogType::Success);
        }
        
    }   
    else
    {
        if (msg.empty())
        {
            log("unknown error", LogType::Error, true);
            //log("", LogType::Error);
        }
        else
        {
            log(QString::fromStdString(msg), LogType::Error, true);
            //log("", LogType::Error);
        }
    }
}

/**
*/
void RTCommandShell::log(const QString& txt, LogType log_type, bool indent)
{
    QColor color = Qt::black; //LogType::Plain
    if (log_type == LogType::Error)
        color = Qt::red;
    else if (log_type == LogType::Warning)
        color = Qt::darkYellow;
    else if (log_type == LogType::Success)
        color = Qt::darkGreen;

    cmd_shell_->setTextColor(color);

    QStringList lines = txt.split("\n");

    for (const auto& l : lines)
        cmd_shell_->append(indent ? QString::fromStdString(RTCommand::ReplyStringIndentation) + l : l);
}

/**
*/
void RTCommandShell::lastCmd()
{
    if (local_backlog_.empty())
    {
        //init local backlog
        local_backlog_ = RTCommandManager::instance().commandBacklog();
        if (local_backlog_.empty())
            return;

        local_backlog_index_ = (int)local_backlog_.size() - 1;
    }
    else 
    {
        local_backlog_index_ = std::max(0, local_backlog_index_ - 1);
    }

    updateCommandFromLocalBacklog();
}

/**
*/
void RTCommandShell::nextCmd()
{
    if (local_backlog_.empty())
        return;

    int n = (int)local_backlog_.size();

    ++local_backlog_index_;

    if (local_backlog_index_ == n)
    {
        resetLocalBacklog();
    }

    updateCommandFromLocalBacklog();
}

} // namespace rtcommand
