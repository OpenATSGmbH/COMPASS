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

#include "json.hpp"

#include <boost/optional.hpp>

#include <QString>

class QWidget;

namespace ui_test
{
    /**
     * Base class for testable ui elements.
     * Can be used to test complex non-native Qt widgets.
    */
    class UITestable
    {
    public:
        UITestable() = default;
        virtual ~UITestable() = default;

        /**
         * Return the value of the widget as string.
         * Override to make available for testing.
        */
        virtual boost::optional<QString> uiGet(const QString& what = QString()) const { return {}; }
        virtual nlohmann::json uiGetJSON(const QString& what = QString()) const { return {}; }

        /**
         * Set the widgets contents from the given string using injected ui events.
         * This method is not meant as a simple value setter like e.g. line_edit->setText(), 
         * but rather should simulate ui events leading to the ui element to be set.
         * (see ui_test_event_injections.h/.cpp)
         * Override to make available for testing.
        */
        virtual bool uiSet(const QString& str) { return false; }

        /**
         * Return a native Qt subwidget which can be processed by default ui injections handled in ui_test_setget_native.h/.cpp.
         * Override to make available for testing.
        */
        virtual QWidget* uiRerouteToNative() const { return nullptr; }

        /**
         * "Refreshes" the ui element, the meaning of it being specific to the implementation. 
        */
        virtual void uiRefresh() {}
    };
}
