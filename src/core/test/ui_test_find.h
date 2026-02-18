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

#include "rtcommand_defs.h"

#include <QWidget>
#include <QWindow>
#include <QString>

namespace ui_test
{

std::string objectName(const QString& obj_name);
void logObjectError(const QString& prefix, const QString& obj_name, rtcommand::FindObjectErrCode code);
std::pair<rtcommand::FindObjectErrCode, QObject*> findObjectNoPath(QObject* parent, const QString& obj_name = "");
std::pair<rtcommand::FindObjectErrCode, QObject*> findObject(QObject* parent, const QString& obj_name = "");
std::pair<QObject*, int> findSignal(QObject* parent, const QString& signal_name);

/**
 */
template<class T>
inline std::pair<rtcommand::FindObjectErrCode, T*> findObjectAs(QObject* parent, const QString& obj_name = "")
{
    auto obj = findObject(parent, obj_name);
    if (obj.first != rtcommand::FindObjectErrCode::NoError)
        return std::make_pair(obj.first, nullptr);

    T* obj_cast = dynamic_cast<T*>(obj.second);
    if (!obj_cast)
        return std::make_pair(rtcommand::FindObjectErrCode::WrongType, nullptr);

    return std::make_pair(rtcommand::FindObjectErrCode::NoError, obj_cast);
}
template<>
inline std::pair<rtcommand::FindObjectErrCode, QObject*> findObjectAs(QObject* parent, const QString& obj_name)
{
    return findObject(parent, obj_name);
}
template<>
inline std::pair<rtcommand::FindObjectErrCode, QWidget*> findObjectAs(QObject* parent, const QString& obj_name)
{
    auto obj = findObject(parent, obj_name);
    if (obj.first != rtcommand::FindObjectErrCode::NoError)
        return std::make_pair(obj.first, nullptr);

    if (!obj.second->isWidgetType())
        return std::make_pair(rtcommand::FindObjectErrCode::WrongType, nullptr);

    return std::make_pair(rtcommand::FindObjectErrCode::NoError, dynamic_cast<QWidget*>(obj.second));
}
template<>
inline std::pair<rtcommand::FindObjectErrCode, QWindow*> findObjectAs(QObject* parent, const QString& obj_name)
{
    auto obj = findObject(parent, obj_name);
    if (obj.first != rtcommand::FindObjectErrCode::NoError)
        return std::make_pair(obj.first, nullptr);

    if (!obj.second->isWindowType())
        return std::make_pair(rtcommand::FindObjectErrCode::WrongType, nullptr);

    return std::make_pair(rtcommand::FindObjectErrCode::NoError, dynamic_cast<QWindow*>(obj.second));
}

} // namespace ui_test
