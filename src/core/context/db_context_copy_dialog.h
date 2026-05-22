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

class QComboBox;
class QLineEdit;
class QPushButton;

namespace context
{

class DBContextManager;

/**
 * Dialog for copying (duplicating) a context under a new name.
 */
class DBContextCopyDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DBContextCopyDialog(DBContextManager& manager, QWidget* parent = nullptr);

private slots:
    void updateCopyButton();
    void copySlot();

private:
    DBContextManager& manager_;
    QComboBox* source_combo_{nullptr};
    QLineEdit* name_edit_{nullptr};
    QPushButton* copy_button_{nullptr};
};

} // namespace context
