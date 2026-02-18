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

#include "ui_test_common.h"
#include "ui_test_conversions.h"

#include "json.hpp"

#include <QMenu>
#include <QMenuBar>
#include <QComboBox>
#include <QTabWidget>
#include <QToolBar>
#include <QLineEdit>
#include <QTextEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QAbstractSlider>
#include <QPushButton>
#include <QToolButton>
#include <QCheckBox>
#include <QRadioButton>
#include <QLabel>
#include <QAction>

#include <QWidget>
#include <QString>
#include <QStringList>

#include <cmath>

#include <boost/optional.hpp>

namespace ui_test
{

    /****************************************************************************************
     * string getters
     ****************************************************************************************/

    /**
     */
    template <class T>
    inline boost::optional<QString> getUIElementValue(T* widget, const QString& what = "")
    {
        return {};
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QLabel* widget, const QString& what)
    {
        return widget->text();
    }
    namespace 
    {
        QAction* findAction(QWidget* menu, const QString& text, bool is_menu)
        {
            for (auto a : menu->actions())
            {
                if (a->isSeparator() || (is_menu && !a->menu()))
                    continue;

                QString txt = is_menu ? normalizedMenuName(a->text()) :
                                        normalizedActionName(a->text());
                if (txt == text)
                    return a;
            }
            return nullptr;
        }
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QMenuBar* widget, const QString& what)
    {
        if (what.isEmpty())
            return {};

        auto path = conversions::valueFromString<QStringList>(what);
        if (!path.has_value() || path.value().empty())
            return {};

        int n = path.value().count();

        QWidget* last_menu   = widget;
        QAction* last_action = nullptr;
        for (int i = 0; i < n; ++i)
        {
            last_action = findAction(last_menu, path.value()[ i ], i < n - 1);
            if (!last_action)
                return {};

            if (last_action->menu())
                last_menu = last_action->menu();
        }

        if (!last_action)
            return {};

        return conversions::stringFromValue<bool>(last_action->isChecked());
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QComboBox* widget, const QString& what)
    {
        return widget->currentText();
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QTabWidget* widget, const QString& what)
    {
        return widget->tabBar()->tabText(widget->currentIndex());
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QToolBar* widget, const QString& what)
    {
        if (what.isEmpty())
            return {};

        for (auto a : widget->actions())
            if (!a->isSeparator() && what == normalizedToolName(a->text()))
                return conversions::stringFromValue<bool>(a->isChecked());

        return {};
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QLineEdit* widget, const QString& what)
    {
        return widget->text();
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QTextEdit* widget, const QString& what)
    {
        return widget->toPlainText();
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QSpinBox* widget, const QString& what)
    {
        return conversions::stringFromValue<int>(widget->value());
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QDoubleSpinBox* widget, const QString& what)
    {
        return conversions::stringFromValue<double>(widget->value(), widget->decimals());
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QAbstractSlider* widget, const QString& what)
    {
        const double t = (double)(widget->value() - widget->minimum()) / (double)(widget->maximum() - widget->minimum());
        const double p = t * 100.0;

        const double PercDec = 2;

        return conversions::stringFromValue<double>(p, PercDec);
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QAbstractButton* widget, const QString& what)
    {
        //standard behaviour for buttons: return check state
        return conversions::stringFromValue<bool>(widget->isChecked());
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QPushButton* widget, const QString& what)
    {
        return getUIElementValue<QAbstractButton>(widget, what);
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QToolButton* widget, const QString& what)
    {
        return getUIElementValue<QAbstractButton>(widget, what);
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QRadioButton* widget, const QString& what)
    {
        return getUIElementValue<QAbstractButton>(widget, what); 
    }
    template<>
    inline boost::optional<QString> getUIElementValue(QCheckBox* widget, const QString& what)
    {
        return getUIElementValue<QAbstractButton>(widget, what);
    }

    /****************************************************************************************
     * json getters
     ****************************************************************************************/

    template <class T>
    inline nlohmann::json value2JSON(const T& v)
    {
        nlohmann::json json_val;
        json_val[ "value" ] = v;
        return json_val;
    }

    /**
     */
    template <class T>
    inline nlohmann::json getUIElementValueJSON(T* widget, const QString& what = "")
    {
        return {};
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QLabel* widget, const QString& what)
    {
        return value2JSON(widget->text().toStdString());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QMenuBar* widget, const QString& what)
    {
        if (what.isEmpty())
            return {};

        auto path = conversions::valueFromString<QStringList>(what);
        if (!path.has_value() || path.value().empty())
            return {};

        int n = path.value().count();

        QWidget* last_menu   = widget;
        QAction* last_action = nullptr;
        for (int i = 0; i < n; ++i)
        {
            last_action = findAction(last_menu, path.value()[ i ], i < n - 1);
            if (!last_action)
                return {};

            if (last_action->menu())
                last_menu = last_action->menu();
        }

        if (!last_action)
            return {};

        return value2JSON(last_action->isChecked());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QComboBox* widget, const QString& what)
    {
        return value2JSON(widget->currentText().toStdString());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QTabWidget* widget, const QString& what)
    {
        return value2JSON(widget->tabBar()->tabText(widget->currentIndex()).toStdString());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QToolBar* widget, const QString& what)
    {
        if (what.isEmpty())
            return {};

        for (auto a : widget->actions())
            if (!a->isSeparator() && what == normalizedToolName(a->text()))
                return value2JSON(a->isChecked());

        return {};
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QLineEdit* widget, const QString& what)
    {
        return value2JSON(widget->text().toStdString());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QTextEdit* widget, const QString& what)
    {
        return value2JSON(widget->toPlainText().toStdString());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QSpinBox* widget, const QString& what)
    {
        return value2JSON(widget->value());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QDoubleSpinBox* widget, const QString& what)
    {
        //@TODO: decimals?
        return value2JSON(widget->value());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QAbstractSlider* widget, const QString& what)
    {
        const double t = (double)(widget->value() - widget->minimum()) / (double)(widget->maximum() - widget->minimum());
        const double p = t * 100.0;

        //@TODO: decimals?
        return value2JSON(p);
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QAbstractButton* widget, const QString& what)
    {
        //standard behaviour for buttons: return check state
        return value2JSON(widget->isChecked());
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QPushButton* widget, const QString& what)
    {
        return getUIElementValueJSON<QAbstractButton>(widget, what);
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QToolButton* widget, const QString& what)
    {
        return getUIElementValueJSON<QAbstractButton>(widget, what);
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QRadioButton* widget, const QString& what)
    {
        return getUIElementValueJSON<QAbstractButton>(widget, what); 
    }
    template<>
    inline nlohmann::json getUIElementValueJSON(QCheckBox* widget, const QString& what)
    {
        return getUIElementValueJSON<QAbstractButton>(widget, what);
    }
    
} // namespace ui_test
