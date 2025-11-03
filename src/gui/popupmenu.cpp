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

#include "popupmenu.h"

#include "logger.h"
#include "traced_assert.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QWidgetAction>
#include <QPushButton>
#include <QToolButton>

/**
 */
PopupMenu::PopupMenu(QAbstractButton* host, 
                     QWidget* content)
:   host_   (host   )
,   content_(content)
{
    traced_assert(host_);

    if (content_)
    {
        auto w_action = new QWidgetAction(this);
        w_action->setDefaultWidget(content_);

        addAction(w_action);
    }

    bool configured = true;
    if (dynamic_cast<QPushButton*>(host_))
        dynamic_cast<QPushButton*>(host_)->setMenu(this);
    else if (dynamic_cast<QToolButton*>(host_))
        dynamic_cast<QToolButton*>(host_)->setMenu(this);
    else
        configured = false;
    
    traced_assert(configured);
}

/**
 */
void PopupMenu::setPreShowCallback(const std::function<void()>& cb)
{
    cb_pre_ = cb;
}

/**
 */
void PopupMenu::setPostShowCallback(const std::function<void()>& cb)
{
    cb_post_ = cb;
}

/**
 */
void PopupMenu::showEvent(QShowEvent* event)
{
    if (cb_pre_)
        cb_pre_();

    QMenu::showEvent(event);

    if (cb_post_)
        cb_post_();

    if (content_)
    {
        //remember original height chosen by qt
        if (initial_height_ < 0)
            initial_height_ = height();

        const int ExtraSpace = 5;

        auto pos_upper   = host_->mapToGlobal(QPoint(0, 0));
        auto pos_lower   = host_->mapToGlobal(QPoint(0, host_->height()));
        auto screen      = QApplication::desktop()->screenGeometry(host_);
        int  space_upper = std::max(0, pos_upper.y() - ExtraSpace);
        int  space_lower = std::max(0, screen.height() - pos_lower.y() - ExtraSpace);

        //show either above or below, depending on screen space
        bool show_above = space_upper > screen.height() / 2;

        //determine y-position and height
        int y, h;
        if (show_above)
        {
            //show above button
            h = initial_height_ > space_upper ? space_upper : initial_height_;
            y = pos_upper.y() - h;
        }
        else
        {
            //show below button
            h = initial_height_ > space_lower ? space_lower : initial_height_;
            y = pos_lower.y();
        }

        //position and resize
        move(x(), y);
        setFixedHeight(h);
        content_->setFixedHeight(h);
    }
}
