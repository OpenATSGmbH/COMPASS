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

#include <QDialog>

class QListWidget;

namespace context
{

class DBContextManager;

/**
 * Dialog shown when contexts exist but none is active.
 * Forces the user to select one before proceeding.
 */
class DBContextSelectDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DBContextSelectDialog(DBContextManager& manager, QWidget* parent = nullptr);

    std::string selectedContextName() const { return selected_name_; }

private slots:
    void selectSlot();
    void itemDoubleClickedSlot();

private:
    DBContextManager& manager_;
    QListWidget* list_widget_{nullptr};
    std::string selected_name_;
};

} // namespace context
