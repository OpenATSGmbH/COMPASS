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

#include "viewtoolswitcher.h"

#include <QShortcut>
#include <QWidget>

#include <iostream>

/**
 * Sets the data widget. This is needed for correct shortcut scope.
 */
void ViewToolSwitcher::setDataWidget(QWidget* data_widget)
{
    data_widget_ = data_widget;

    if (data_widget_)
    {
        //register shortcut for ending a tool
        auto end_tool_shortcut = new QShortcut(Qt::Key_Escape, data_widget_);
        connect(end_tool_shortcut, &QShortcut::activated, [ = ] () { this->endCurrentTool(); });
    }
}

/**
 * Sets a default tool. This tool will be set if the current tool is ended.
 */
void ViewToolSwitcher::setDefaultTool(int default_id, bool send_update)
{
    if (default_id < 0)
    {
        default_tool_ = -1;
    }
    else
    {
        if (!hasTool(default_id))
            throw std::runtime_error("ViewToolSwitcher::addTool: unknown default id");

        default_tool_ = default_id;
    }

    current_tool_ = default_tool_;

    if (send_update)
        update();      
}

/**
 * Register a new tool.
 */
void ViewToolSwitcher::addTool(int id, 
                               const QString& name, 
                               const QKeySequence& key_combination, 
                               const QIcon& icon, 
                               const QCursor& cursor)
{
    if (tools_.find(id) != tools_.end())
        throw std::runtime_error("ViewToolSwitcher::addTool: duplicate id");

    Tool t;
    t.id              = id;
    t.name            = name;
    t.key_combination = key_combination;
    t.icon            = icon;
    t.cursor          = cursor;

    if (!key_combination.isEmpty())
    {
        t.name += " [" + key_combination.toString() + "]";

        t.shortcut = new QShortcut(key_combination, data_widget_);
        connect(t.shortcut, &QShortcut::activated, [ = ] () { this->setCurrentTool(id); });
    }

    tools_[ id ] = t;
}

/**
 * Sets the current tool.
 */
void ViewToolSwitcher::setCurrentTool(int id, bool force_update)
{
    //return if the new tool is the same as the old one?
    if (!force_update && current_tool_ == id)
        return;

    if (id < 0)
    {
        current_tool_ = -1;
    }
    else
    {
        if (tools_.find(id) == tools_.end())
            throw std::runtime_error("ViewToolSwitcher::setCurrentTool: unknown id");

        current_tool_ = id;
    }

    auto c = currentCursor();

    //inform receivers on tool change
    emit toolChanged(id, c);
}

/**
 * Invoke receivers using the currently set tool.
 */
void ViewToolSwitcher::update()
{
    setCurrentTool(current_tool_, true);
}

/**
 * Ends the current tool and switches back to the default tool.
 */
void ViewToolSwitcher::endCurrentTool()
{
    setCurrentTool(default_tool_);
}

/**
 * Returns the current tool id.
 */
int ViewToolSwitcher::currentTool() const
{
    return current_tool_;
}

/**
 * Returns the current tool's cursor.
 */
QCursor ViewToolSwitcher::currentCursor() const
{
    auto tool = getTool(current_tool_);
    if (!tool)
        return QCursor();

    return tool->cursor;
}

/**
 * Returns the current tool's name.
 */
QString ViewToolSwitcher::currentName() const
{
    auto tool = getTool(current_tool_);
    if (!tool)
        return QString();

    return tool->name;
}

/**
 * Returns the tool of the given id, or nullptr if the id wasn't found.
 */
const ViewToolSwitcher::Tool* ViewToolSwitcher::getTool(int id) const
{
    auto it = tools_.find(id);
    if (it == tools_.end())
        return nullptr;

    return &it->second;
}

/**
 * Checks if a tool of the given id was added to the switcher.
 */
bool ViewToolSwitcher::hasTool(int id) const
{
    return (tools_.find(id) != tools_.end());
}

