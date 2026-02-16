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

#include "ui_test_find.h"
#include "logger.h"
#include "compass.h"

#include <QWidget>
#include <QAction>

using namespace rtcommand;

namespace ui_test
{

/**
 */
std::string objectName(const QString& obj_name)
{
    if (obj_name.isEmpty())
        return "";
    return "'" + obj_name.toStdString() + "'";
}

/**
 */
void logObjectError(const QString& prefix, 
                    const QString& obj_name,
                    FindObjectErrCode code)
{
    if (code == FindObjectErrCode::NoError)
        return;

    QString err = "yielded unknown error";
    if (code == FindObjectErrCode::Invalid)
        err = "is invalid";
    else if (code == FindObjectErrCode::NotFound)
        err = "not found";
    else if (code == FindObjectErrCode::WrongType)
        err = "has wrong type";
    
    logerr << prefix.toStdString() << ": Object " << objectName(obj_name) + " " + err.toStdString();
}

/**
 */
std::pair<FindObjectErrCode, QObject*> findObjectNoPath(QObject* parent, const QString& obj_name)
{
    if (!parent)
        return std::make_pair(FindObjectErrCode::Invalid, nullptr);

    if (obj_name.isEmpty() || parent->objectName() == obj_name)
        return std::make_pair(FindObjectErrCode::NoError, parent);

    //look for child actions
    // if (parent->isWidgetType())
    // {
    //     QWidget* w = dynamic_cast<QWidget*>(parent);

    //     //try to find in actions
    //     for (auto action : w->actions())
    //     {
    //         if (action->objectName() == obj_name)
    //             return std::make_pair(FindObjectErrCode::NoError, action);
    //     }
        
    //     //not found in actions, try to find as child
    // }

    QObject* obj = parent->findChild<QObject*>(obj_name, Qt::FindChildrenRecursively);
    if (!obj)
        return std::make_pair(FindObjectErrCode::NotFound, nullptr);

    return std::make_pair(FindObjectErrCode::NoError, obj);
}

/**
 */
std::pair<FindObjectErrCode, QObject*> findObject(QObject* parent, const QString& obj_name)
{
    if (!parent)
        return std::make_pair(FindObjectErrCode::Invalid, nullptr);

    if (obj_name.isEmpty() || parent->objectName() == obj_name)
        return std::make_pair(FindObjectErrCode::NoError, parent);

    int idx = obj_name.indexOf(".");

    //no path? use that version
    if (idx < 0)
        return findObjectNoPath(parent, obj_name);

    //split into path
    QStringList path = obj_name.split(".");

    //traverse the given object path and jump from child to child
    QObject* last_obj = parent;
    for (const QString& sub_obj : path)
    {
        loginf << "looking for object '" << sub_obj.toStdString() << "'"; 

        auto obj = findObjectNoPath(last_obj, sub_obj.trimmed());
        if (obj.first != FindObjectErrCode::NoError)
            return std::make_pair(obj.first, nullptr);

        loginf << "   FOUND";

        last_obj = obj.second;
    }

    return std::make_pair(FindObjectErrCode::NoError, last_obj);
}

/**
 */
std::pair<QObject*, int> findSignalRecursive(QObject* obj, const QString& signal_name)
{
    if (!obj || !obj->metaObject())
        return std::make_pair(nullptr, -1);

    //found signal in current object?
    int idx = obj->metaObject()->indexOfSignal(signal_name.toStdString().c_str());
    if (idx >= 0)
        return std::make_pair(obj, idx);

    //if not find in children
    for (auto child : obj->children())
    {
        auto child_signal = findSignalRecursive(child, signal_name);
        if (child_signal.first != nullptr && child_signal.second >= 0)
            return child_signal;
    }

    //not found in subtree
    return std::make_pair(nullptr, -1);
}
std::pair<QObject*, int> findSignal(QObject* parent, const QString& signal_name)
{
    return findSignalRecursive(parent, signal_name);
}

} // namespace ui_test
