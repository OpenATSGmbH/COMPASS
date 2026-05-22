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

#include "ui_test_setget_native.h"
#include "ui_test_find.h"
#include "ui_test_widget_setters.h"
#include "ui_test_widget_getters.h"
#include "rtcommand_defs.h"

#include <QWidget>

/**
 * Attempts a cast of the given widget to the given type and invokes
 * the templated value setter if the cast succeeds.
 */
#define TRY_INVOKE_UI_SETTER(WidgetType, widget, value, delay, hint)                            \
    if (dynamic_cast<WidgetType*>(widget))                                                      \
        return setUIElement<WidgetType>(dynamic_cast<WidgetType*>(widget), value, delay, hint);

/**
 * Attempts a cast of the given widget to the given type and invokes
 * the templated value getter if the cast succeeds.
 */
#define TRY_INVOKE_UI_GETTER(WidgetType, widget, what)                                 \
    if (dynamic_cast<WidgetType*>(widget))                                             \
        return getUIElementValue<WidgetType>(dynamic_cast<WidgetType*>(widget), what);

/**
 * Attempts a cast of the given widget to the given type and invokes
 * the templated json value getter if the cast succeeds.
 */
#define TRY_INVOKE_UI_GETTER_JSON(WidgetType, widget, what)                                    \
    if (dynamic_cast<WidgetType*>(widget))                                                \
        return getUIElementValueJSON<WidgetType>(dynamic_cast<WidgetType*>(widget), what);

namespace ui_test
{

/**
 * Widget type agnostic value setter.
 * Checks on native Qt widgets.
 */
bool setUIElementNative(QWidget* parent, 
                        const QString& obj_name, 
                        const QString& value, 
                        int delay,
                        const SetUIHint& hint)
{
    auto w = findObjectAs<QWidget>(parent, obj_name);
    if (w.first != rtcommand::FindObjectErrCode::NoError)
        return false;

    //declare setters for each type of widget here
    //corresponding template specializations need to be defined in 
    //ui_test_widget_setters.h
    TRY_INVOKE_UI_SETTER(QLabel, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QMenu, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QMenuBar, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QComboBox, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QTabWidget, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QToolBar, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QLineEdit, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QTextEdit, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QSpinBox, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QDoubleSpinBox, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QScrollBar, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QAbstractSlider, w.second, value, delay, hint) //for all other sliders which were not handled before
    TRY_INVOKE_UI_SETTER(QCheckBox, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QPushButton, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QRadioButton, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QToolButton, w.second, value, delay, hint)
    TRY_INVOKE_UI_SETTER(QAbstractButton, w.second, value, delay, hint) //for all other buttons which were not handled before

    //try to process some specific commands if the widget is a dialog
    TRY_INVOKE_UI_SETTER(QDialog, w.second, value, delay, hint)

    //try to invoke a slot in the widget which has the name passed by value
    TRY_INVOKE_UI_SETTER(QWidget, w.second, value, delay, hint)

    return false;
}

/**
 * Widget type agnostic value getter.
 * Checks on native Qt widgets.
 */
boost::optional<QString> getUIElementNative(QWidget* parent, 
                                            const QString& obj_name,
                                            const QString& what)
{
    auto w = findObjectAs<QObject>(parent, obj_name);
    if (w.first != rtcommand::FindObjectErrCode::NoError)
        return {};

    TRY_INVOKE_UI_GETTER(QLabel, w.second, what)
    TRY_INVOKE_UI_GETTER(QMenuBar, w.second, what)
    TRY_INVOKE_UI_GETTER(QComboBox, w.second, what)
    TRY_INVOKE_UI_GETTER(QTabWidget, w.second, what)
    TRY_INVOKE_UI_GETTER(QToolBar, w.second, what)
    TRY_INVOKE_UI_GETTER(QLineEdit, w.second, what)
    TRY_INVOKE_UI_GETTER(QTextEdit, w.second, what)
    TRY_INVOKE_UI_GETTER(QSpinBox, w.second, what)
    TRY_INVOKE_UI_GETTER(QDoubleSpinBox, w.second, what)
    TRY_INVOKE_UI_GETTER(QAbstractSlider, w.second, what)
    //...maybe add special button getters
    TRY_INVOKE_UI_GETTER(QAbstractButton, w.second, what) //for all other buttons which were not handled before

    return {};
}

/**
 * Widget type agnostic json value getter.
 * Checks on native Qt widgets.
 */
nlohmann::json getUIElementNativeJSON(QWidget* parent, 
                                      const QString& obj_name,
                                      const QString& what)
{
    auto w = findObjectAs<QObject>(parent, obj_name);
    if (w.first != rtcommand::FindObjectErrCode::NoError)
        return {};

    TRY_INVOKE_UI_GETTER_JSON(QLabel, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QMenuBar, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QComboBox, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QTabWidget, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QToolBar, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QLineEdit, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QTextEdit, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QSpinBox, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QDoubleSpinBox, w.second, what)
    TRY_INVOKE_UI_GETTER_JSON(QAbstractSlider, w.second, what)
    //...maybe add special button getters
    TRY_INVOKE_UI_GETTER_JSON(QAbstractButton, w.second, what) //for all other buttons which were not handled before

    return {};
}

} // namespace ui_test
