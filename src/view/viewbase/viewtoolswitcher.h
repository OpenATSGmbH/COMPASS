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

#include "compass_view_export.h"

#include <QString>
#include <QIcon>
#include <QCursor>
#include <QKeySequence>

#include <functional>
#include <map>

class QWidget;
class QShortcut;

/**
 * Class for registering and switching view 'tools'.
 * A tool consists of various attributes e.g. an id, an icon, a keyboard shortcut, a mouse cursor etc.
 * A default tool can be set, which is used when the currend tool is ended via endCurrentTool().
 */
class COMPASS_VIEW_API ViewToolSwitcher : public QObject
{
    Q_OBJECT
public:
    struct Tool
    {
        int          id = -1;
        QString      name;
        QKeySequence key_combination;
        QIcon        icon;
        QCursor      cursor;
        QShortcut*   shortcut = nullptr;
    };

    ViewToolSwitcher() = default;
    virtual ~ViewToolSwitcher() = default;

    void setDataWidget(QWidget* data_widget);
    void setDefaultTool(int default_id, bool send_update = true);

    void addTool(int id, 
                 const QString& name, 
                 const QKeySequence& key_combination = QKeySequence(), 
                 const QIcon& icon = QIcon(), 
                 const QCursor& cursor = QCursor());
    
    void setCurrentTool(int id, bool force_update = false);
    void endCurrentTool();

    int currentTool() const;
    QCursor currentCursor() const;
    QString currentName() const;
    const Tool* getTool(int id) const;
    bool hasTool(int id) const; 

    void update();

    /// Called on the Escape shortcut before the current tool is ended. Returns true if the
    /// active tool consumed the Escape, e.g. to cancel a running step, the tool then stays.
    void setEscapeHandler(std::function<bool()> handler) { escape_handler_ = handler; }

signals:
    void toolChanged(int id, const QCursor& cursor);

private:
    std::map<int, Tool> tools_;

    int current_tool_ = -1;
    int default_tool_ = -1;

    QWidget* data_widget_ = nullptr;

    std::function<bool()> escape_handler_;
};
