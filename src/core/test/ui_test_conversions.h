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

#include "rtcommand/rtcommand_helpers.h"

#include <QString>
#include <QStringList>

#include <boost/optional.hpp>

namespace ui_test
{
    /**
     * Value conversions from string values to specific value types.
     */
    namespace conversions
    {
        /**
         */
        template<typename T>
        inline boost::optional<T> valueFromString(const QString& value)
        {
            return {};
        }
        template<>
        inline boost::optional<int> valueFromString(const QString& value)
        {
            if (value.isEmpty())
                return {};

            bool ok;
            int ret = value.toInt(&ok);

            if (!ok)
                return {};

            return ret;
        }
        template<>
        inline boost::optional<double> valueFromString(const QString& value)
        {
            if (value.isEmpty())
                return {};

            bool ok;
            double ret = value.toDouble(&ok);

            if (!ok)
                return {};

            return ret;
        }
        template<>
        inline boost::optional<QStringList> valueFromString(const QString& value)
        {
            if (value.isEmpty())
                return {};
            
            QStringList strings = rtcommand::parameterToStrings(value);
            if (strings.empty())
                return {};

            return strings;
        }
        template<>
        inline boost::optional<bool> valueFromString(const QString& value)
        {
            if (value.isEmpty())
                return {};

            if (value == "true")
                return true;
            else if (value == "false")
                return false;

            //unknown value
            return {};
        }

        /**
         */
        template<typename T>
        inline boost::optional<QString> stringFromValue(const T& value, int prec = 2)
        {
            return {};
        }
        template<>
        inline boost::optional<QString> stringFromValue(const int& value, int prec)
        {
            return QString::number(value);
        }
        template<>
        inline boost::optional<QString> stringFromValue(const double& value, int prec)
        {
            return QString::number(value, 'f', prec);
        }
        template<>
        inline boost::optional<QString> stringFromValue(const QStringList& value, int prec)
        {
            return rtcommand::parameterFromStrings(value);
        }
        template<>
        inline boost::optional<QString> stringFromValue(const bool& value, int prec)
        {
            return (value ? QString("true") : QString("false"));
        }
    }
} // namespace ui_test
