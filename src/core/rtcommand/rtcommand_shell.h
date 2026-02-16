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

#include <vector>

#include <QWidget>
#include <QDialog>

class QTextEdit;
class QLineEdit;
class QToolButton;
class QListWidgetItem;
class QListWidget;

namespace rtcommand
{

/**
 * 
 */
class RTCommandBacklogWidget : public QWidget
{
    Q_OBJECT
public:
    RTCommandBacklogWidget(QWidget* parent = nullptr);
    virtual ~RTCommandBacklogWidget() = default;

signals:
    void commandSelected(const QString&);

private:
    void updateList();
    void acceptCommandItem(QListWidgetItem* item);

    QListWidget* command_list_ = nullptr;
};

/**
 * 
 */
class RTCommandBacklogDialog : public QDialog
{
public:
    RTCommandBacklogDialog(QWidget* parent = nullptr);
    virtual ~RTCommandBacklogDialog() = default;

    const QString& selectedCommand() const { return selected_command_; }

private:
    QString selected_command_;
};

/**
 * 
 */
class RTCommandShell : public QWidget 
{
public:
    RTCommandShell(QWidget* parent = nullptr);
    virtual ~RTCommandShell() = default;

private:
    enum class LogType 
    {
        Plain = 0,
        Warning,
        Error,
        Success
    };

    void enableCmdLine(bool enable);
    void resetLocalBacklog();
    void updateCommandFromLocalBacklog();
    void showBacklog();
    void copyLastReply() const;
    void processCommand();
    void log(const QString& txt, LogType log_type = LogType::Plain, bool indent = false);
    void logResult(std::string msg, bool error);

    void receiveResult(const QString& msg, const QString& data, bool error);

    void lastCmd();
    void nextCmd();

    QTextEdit*   cmd_shell_;
    QLineEdit*   cmd_edit_;
    QToolButton* backlog_button_;
    QToolButton* copy_button_;

    std::vector<std::string> local_backlog_;
    int                      local_backlog_index_ = -1;

    QString last_reply_;
};

} // namespace rtcommand
