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

#include <functional>

#include <QMenu>

class QAbstractButton;

/**
 */
class PopupMenu : public QMenu 
{
public:
    PopupMenu(QAbstractButton* host, 
              QWidget* content = nullptr);
    virtual ~PopupMenu() = default;

    void setPreShowCallback(const std::function<void()>& cb);
    void setPostShowCallback(const std::function<void()>& cb);

protected:
    void showEvent(QShowEvent* event);

private:
    QAbstractButton* host_    = nullptr;
    QWidget*         content_ = nullptr;

    int initial_height_ = -1;

    std::function<void()> cb_pre_;
    std::function<void()> cb_post_;
};
