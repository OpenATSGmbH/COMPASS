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

#include <QString>
#include <QKeySequence>
#include <QIcon>
#include <QCursor>

#include <functional>

class QAction;

/**
 * Description of a view tool, registered with ViewRenderWidget::registerTool().
 * The description holds nothing about the mouse, the mouse handling is a service of the render widget.
 * A toolbar shows the tools that have an icon.
 */
struct ViewTool
{
    /// ids from here on are reserved for the host of the render widget (e.g. ViewWidget)
    static const int HostIdBase = 1000;

    int          id = -1;
    QString      name;
    QKeySequence key;
    QIcon        icon;
    QCursor      cursor;

    enum class Kind
    {
        Trigger, // not checkable, runs once (Zoom to Home, Invert Selection)
        Toggle,  // checkable, independent (configuration panel)
        Mode     // checkable, exclusive, one is always active (Navigate, Select, Zoom)
    };
    Kind kind = Kind::Trigger;

    std::function<void(bool)>     callback; // Trigger: called once. Toggle / Mode: on and off.
    std::function<void(QAction*)> update;   // enabled / checked state of the toolbar action
};
