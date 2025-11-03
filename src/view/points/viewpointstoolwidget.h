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

#include <QCursor>
#include <QMenu>
#include <QWidget>
#include <map>

//Select first open View Point (from current ordering)
//Select next open View Point (from current View Point)
//Mark current Viewpoint as closed and select next open View Point (from current View Point)
//Mark current Viewpoint as open and select next open View Point (from current View Point)
//Clear All View Points and import from File
//Export All View Points as file

enum ViewPointsTool
{
    SELECT_NEXT_TOOL = 0,
    SELECT_NEXT_OPEN_TOOL,
    OPEN_CURRENT_SELECT_NEXT_TOOL,
    CLOSE_CURRENT_SELECT_NEXT_TOOL
};

class QToolButton;
class QToolBar;

class ViewPointsWidget;

class ViewPointsToolWidget : public QWidget
{
    Q_OBJECT

signals:

public slots:
    void actionTriggeredSlot(QAction* action);
    void typeFilteredSlot ();
    void statusFilteredSlot ();

    void columnFilteredSlot ();

public:
    ViewPointsToolWidget(ViewPointsWidget* vp_widget, QWidget* parent = nullptr);

private:
    ViewPointsWidget* vp_widget_ {nullptr};

    QToolBar* toolbar_ {nullptr};

    QToolButton* select_next_button_{nullptr};
    QToolButton* select_next_open_button_{nullptr};

    QToolButton* step_next_open_button_{nullptr};
    QToolButton* open_current_step_next_button_{nullptr};
    QToolButton* close_current_step_next_button_{nullptr};

    void showColumnsMenu ();
    void showTypesMenu ();
    void showStatusesMenu ();
};
