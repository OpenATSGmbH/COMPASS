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

#include "logwidget.h"
//#include "reportwidget.h"
#include "files.h"
#include "logger.h"
//#include "taskmanager.h"

#include <QVBoxLayout>
#include <QDateTime>
#include <QScrollBar>
#include <QToolBar>
#include <QTableView>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QSortFilterProxyModel>
#include <QResizeEvent>

#include <algorithm>

using namespace Utils;

LogWidget::LogWidget(LogStore& log_store)
    : ToolBoxWidget(nullptr), log_store_(log_store)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    // table
    proxy_model_ = new QSortFilterProxyModel();
    proxy_model_->setSourceModel(&log_store_);

    table_view_ = new QTableView();
    table_view_->setModel(proxy_model_);
    table_view_->setSortingEnabled(true);
    table_view_->sortByColumn(1, Qt::AscendingOrder);
    table_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_view_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    table_view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
    table_view_->horizontalHeader()->setMaximumSectionSize(600);
    //table_view_->setIconSize(QSize(24, 24));
    table_view_->setContextMenuPolicy(Qt::CustomContextMenu);
    table_view_->setWordWrap(true);
    table_view_->reset();
    // update done later

    // connect(table_view_, &QTableView::customContextMenuRequested,
    //         this, &TargetListWidget::customContextMenuSlot);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
    main_layout->addWidget(table_view_);

    setLayout(main_layout);

    default_icon_ = QIcon(Utils::Files::getIconFilepath("log.png").c_str());
    error_icon_ = QIcon(Utils::Files::getIconFilepath("log_err.png").c_str());

    connect (&log_store_, &LogStore::messagesChangedSignal,
            this, &LogWidget::messagesChangedSlot, Qt::QueuedConnection);

    //updateDisplay();
    checkIcon();
}

LogWidget::~LogWidget(){}

void LogWidget::messagesChangedSlot()
{
    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();

    checkIcon();
}

void LogWidget::acceptMessagesSlot()
{
    log_store_.acceptMessages();
}

void LogWidget::checkIcon()
{
    auto old_val = has_unaccepted_errors_;

    has_unaccepted_errors_ = std::any_of(
        log_store_.logEntries().begin(), log_store_.logEntries().end(),
        [](const LogStore::LogEntry& entry) {
            return entry.type_ == LogStreamType::Error && !entry.accepted_;
        });

    if (old_val != has_unaccepted_errors_)
        emit iconChangedSignal();
}

/**
 */
QIcon LogWidget::toolIcon() const
{
    if (has_unaccepted_errors_)
        return error_icon_;
    else
        return default_icon_;
}

/**
 */
std::string LogWidget::toolName() const
{
    return "Log";
}

/**
 */
std::string LogWidget::toolInfo() const
{
    return "Log";
}

/**
 */
std::vector<std::string> LogWidget::toolLabels() const
{
    return { "Log" };
}

/**
 */
toolbox::ScreenRatio LogWidget::defaultScreenRatio() const
{
    return ToolBoxWidget::defaultScreenRatio();
}

/**
 */
void LogWidget::addToConfigMenu(QMenu* menu)
{
}

/**
 */
void LogWidget::addToToolBar(QToolBar* tool_bar)
{
    auto action_accept = tool_bar->addAction("Accept");
    action_accept->setIcon(Utils::Files::IconProvider::getIcon("done.png"));
    connect(action_accept, &QAction::triggered, this, &LogWidget::acceptMessagesSlot);
}

/**
 */
void LogWidget::loadingStarted()
{
}

/**
 */
void LogWidget::loadingDone()
{
}

void LogWidget::resizeEvent(QResizeEvent* event)
{
    const QSize new_size = event->size();

    if (table_view_)
    {
        table_view_->horizontalHeader()->setMaximumSectionSize(2 * new_size.width() / 3);

        table_view_->resizeColumnsToContents();
        table_view_->resizeRowsToContents();
    }

    QWidget::resizeEvent(event);
}

