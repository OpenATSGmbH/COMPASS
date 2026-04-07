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

class QLineEdit;

namespace context
{

class DBContextManager;

/**
 * Dialog shown when no contexts exist. Forces the user to create one.
 */
class DBContextCreateDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DBContextCreateDialog(DBContextManager& manager, QWidget* parent = nullptr);

    std::string createdContextName() const { return created_name_; }

private slots:
    void createSlot();

private:
    DBContextManager& manager_;
    QLineEdit* name_edit_{nullptr};
    std::string created_name_;
};

} // namespace context
