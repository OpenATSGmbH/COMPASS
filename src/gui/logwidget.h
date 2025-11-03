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

#include "toolboxwidget.h"
#include "util/logmodel.h"

#include <QTextEdit>
#include <QIcon>

#include <string>
#include <vector>

class LogStore;

class QTableView;
class QSortFilterProxyModel;

class LogWidget : public ToolBoxWidget
{
    Q_OBJECT

public slots:
    void messagesChangedSlot();
    void acceptMessagesSlot();

public:
    explicit LogWidget(LogStore& log_store);
    virtual ~LogWidget();

    //ToolBoxWidget
    QIcon toolIcon() const override final;
    std::string toolName() const override final;
    std::string toolInfo() const override final;
    std::vector<std::string> toolLabels() const override final;
    toolbox::ScreenRatio defaultScreenRatio() const override final;
    void addToConfigMenu(QMenu* menu) override final;
    void addToToolBar(QToolBar* tool_bar) override final;
    void loadingStarted() override final;
    void loadingDone() override final;

protected:
    LogStore& log_store_;

    void checkIcon();

    QTableView* table_view_{nullptr};
    QSortFilterProxyModel* proxy_model_{nullptr};

    QIcon default_icon_;
    QIcon error_icon_;

    bool has_unaccepted_errors_{false};

    void resizeEvent(QResizeEvent* event) override;
};
