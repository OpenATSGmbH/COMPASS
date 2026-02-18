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

#include <files.h>

#include <QString>
#include <QRegularExpression>

#define UI_TEST_OBJ_NAME(widget_ptr, display_name) widget_ptr->setObjectName(ui_test::normalizedObjectName(display_name));

namespace ui_test
{

/**
 * Event hint, e.g. where an injected click should happen.
 */
struct SetUIHint
{
    bool valid() const
    {
        return (x >= 0 ||
                y >= 0);
    }

    int x = -1; //x-pos of event
    int y = -1; //y-pos of event
};

/**
 */
inline QString normalizedObjectName(const QString& text)
{
    return QString::fromStdString(Utils::Files::normalizeFilename(text.toStdString(), true));
}

/**
 */
inline QString normalizedToolName(const QString& text)
{
    int idx = text.lastIndexOf("[");
    if (idx < 0)
        return text;

    QString ret = text;
    ret.truncate(idx);

    return ret.trimmed();
}

/**
 */
inline QString normalizedMenuName(const QString& text)
{
    QString txt = text;
    if (txt.startsWith("&"))
        txt = txt.mid(1);

    return txt.trimmed();
}

/**
 */
inline QString normalizedActionName(const QString& text)
{
    QString txt = text.trimmed();
    txt.remove(QRegularExpression("[.]+$"));

    return txt;
}

} // namespace ui_test
