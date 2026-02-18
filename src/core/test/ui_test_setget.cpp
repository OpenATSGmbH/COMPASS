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

#include "ui_test_setget.h"
#include "ui_test_setget_native.h"
#include "ui_test_find.h"
#include "ui_test_testable.h"
#include "rtcommand_defs.h"

#include <QWidget>

namespace ui_test
{

/**
 * Widget type agnostic value setter.
 * Includes checks on complex (non-qt-native) widgets.
 */
bool setUIElement(QWidget* parent, 
                  const QString& obj_name, 
                  const QString& value, 
                  int delay)
{
    auto w = findObjectAs<QWidget>(parent, obj_name);
    if (w.first != rtcommand::FindObjectErrCode::NoError)
        return false;

    UITestable* w_test = dynamic_cast<UITestable*>(w.second);

    //not ui testable? try native Qt types
    if (!w_test)
    {
        return setUIElementNative(parent, obj_name, value, delay);
    }
    
    //try to set via string first
    if (w_test->uiSet(value))
        return true;

    //not supported or failed? try rerouting to native ui element
    auto w_child = w_test->uiRerouteToNative();
    if (w_child)
    {
        return setUIElementNative(w_child, "", value, delay);
    }
    
    //not supported or failed? fail
    return false;
}

/**
 * Widget type agnostic value getter.
 * Includes checks on complex (non-qt-native) widgets.
 */
boost::optional<QString> getUIElement(QWidget* parent, 
                                      const QString& obj_name,
                                      const QString& what)
{
    auto w = findObjectAs<QWidget>(parent, obj_name);
    if (w.first != rtcommand::FindObjectErrCode::NoError)
        return {};

    UITestable* w_test = dynamic_cast<UITestable*>(w.second);

    //not ui testable? try native Qt types
    if (!w_test)
    {
        return getUIElementNative(parent, obj_name, what);
    }

    return w_test->uiGet(what);
}

/**
 * Widget type agnostic json value getter.
 * Includes checks on complex (non-qt-native) widgets.
 */
nlohmann::json getUIElementJSON(QWidget* parent, 
                                    const QString& obj_name,
                                    const QString& what)
{
    auto w = findObjectAs<QWidget>(parent, obj_name);
    if (w.first != rtcommand::FindObjectErrCode::NoError)
        return {};

    UITestable* w_test = dynamic_cast<UITestable*>(w.second);

    //not ui testable? try native Qt types
    if (!w_test)
    {
        return getUIElementNativeJSON(parent, obj_name, what);
    }

    return w_test->uiGetJSON(what);
}

} // namespace ui_test
