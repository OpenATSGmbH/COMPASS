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

#include <map>
#include <string>

class QCheckBox;
class QPushButton;

namespace context
{

class DBContextManager;

/**
 * Dialog for deleting one or more contexts. At least one must remain.
 */
class DBContextDeleteDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DBContextDeleteDialog(DBContextManager& manager, QWidget* parent = nullptr);

private slots:
    void selectAllSlot();
    void selectNoneSlot();
    void updateDeleteButton();
    void deleteSlot();

private:
    DBContextManager& manager_;
    std::map<std::string, QCheckBox*> checkboxes_;
    QPushButton* delete_button_{nullptr};
};

} // namespace context
