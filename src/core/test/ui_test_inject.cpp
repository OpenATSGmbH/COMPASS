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

#include "ui_test_inject.h"
#include "ui_test_find.h"
#include "ui_test_event_injections.h"

#include "rtcommand_defs.h"

#include <QWidget>

namespace ui_test
{

/**
*/
bool injectUIEvent(QWidget* parent, 
                   const QString& obj_name, 
                   const QString& event, 
                   int delay)
{
    auto w = findObjectAs<QWidget>(parent, obj_name);
    if (w.first != rtcommand::FindObjectErrCode::NoError)
        return false;

    auto splitEvent = [ & ] (QString& evt_type, QStringList& params, const QString& str)
    {
        int idx = str.indexOf("(", 0);
        if (idx < 1)
            return false;

        evt_type = str.mid(0, idx);

        auto param_str = str.mid(idx);
        if (!param_str.startsWith("(") || !param_str.endsWith(")"))
            return false;

        param_str = param_str.mid(1, param_str.count() - 2);

        params = param_str.split(",");

        return true;
    };

    auto mouseButtonFromString = [ & ] (const QString& str)
    {
        if (str == "left")
            return Qt::LeftButton;
        else if (str == "middle")
            return Qt::MiddleButton;
        else if (str == "right")
            return Qt::RightButton;
        
        return Qt::NoButton;
    };

    enum MouseAction
    {
        NoAction = 0,
        Click,
        Rect
    };

    auto mouseActionFromString = [ & ] (const QString& str)
    {
        if (str == "click")
            return MouseAction::Click;
        else if (str == "rect")
            return MouseAction::Rect;
        
        return MouseAction::NoAction;
    };

    auto coordFromString = [ & ] (const QString& str)
    {
        Coord c;
        if (str.isEmpty())
            return c;

        QString s = str;

        bool percent = false;
        if (s.endsWith("%"))
        {
            percent = true;
            s.chop(1);
        }

        if (s.isEmpty())
            return c;

        bool ok;
        double v = s.toDouble(&ok);

        if (!ok)
            return c;

        c.coord_system = percent ? Coord::CoordSystem::Percent : Coord::CoordSystem::Pixels;
        c.value        = percent ? v / 100.0 : v; 

        return c;
    };

    QString evt_type;
    QStringList params;
    if (!splitEvent(evt_type, params, event))
        return false;

    if (evt_type == "mouse")
    {
        if (params.size() < 2)
            return false;

        auto button = mouseButtonFromString(params[ 0 ]);
        auto action = mouseActionFromString(params[ 1 ]);

        if (button == Qt::NoButton || 
            action == MouseAction::NoAction)
            return false;

        if (action == MouseAction::Click)
        {
            if (params.size() != 4)
                return false;

            auto xc = coordFromString(params[ 2 ]);
            auto yc = coordFromString(params[ 3 ]);
            
            return injectMouseClick(w.second, button, xc, yc, delay);
        }
        else if (action == MouseAction::Rect)
        {
            if (params.size() != 6)
                return false;

            auto xc0 = coordFromString(params[ 2 ]);
            auto yc0 = coordFromString(params[ 3 ]);
            auto xc1 = coordFromString(params[ 4 ]);
            auto yc1 = coordFromString(params[ 5 ]);

            return injectMouseRect(w.second, button, xc0, yc0, xc1, yc1, delay);
        }
        else
        {
            //unknown action
            return false;
        }
    }
    else if (evt_type == "wheel")
    {
        //@TODO
        return false;
    }
    else if (evt_type == "keys")
    {
        if (params.size() != 1)
            return false;

        return injectKeySequenceEvent(w.second, "", QKeySequence(params[ 0 ]));
    }

    //unknown event
    return false;
}

/**
*/
bool injectMouseClick(QWidget* w,
                      Qt::MouseButton button,
                      const Coord& xcoord,
                      const Coord& ycoord,
                      int delay)
{
    if (!xcoord.valid() || !ycoord.valid())
        return false;

    QRect r = w->rect();

    return injectClickEvent(w, 
                            "", 
                            xcoord.getValue(0.0, r.width()), 
                            ycoord.getValue(0.0, r.height()), 
                            button, 
                            delay);
}

/**
*/
bool injectMouseRect(QWidget* w,
                     Qt::MouseButton button,
                     const Coord& xcoord0,
                     const Coord& ycoord0,
                     const Coord& xcoord1,
                     const Coord& ycoord1,
                     int delay)
{
    if (!xcoord0.valid() || !ycoord0.valid() || !xcoord1.valid() || !ycoord1.valid())
        return false;

    QRect r = w->rect();

    return injectRectEvent(w, 
                           "", 
                           xcoord0.getValue(0.0, r.width()), 
                           ycoord0.getValue(0.0, r.height()), 
                           xcoord1.getValue(0.0, r.width()), 
                           ycoord1.getValue(0.0, r.height()), 
                           button, 
                           delay);
}

} // namespace ui_test
