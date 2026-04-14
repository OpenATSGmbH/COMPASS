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

#include <QWidget>

#include <functional>
#include <string>

class QLineEdit;

namespace context
{

class DBContextManager;

class FFTEditWidget : public QWidget
{
    Q_OBJECT

public:
    FFTEditWidget(DBContextManager& manager,
                  std::function<void()> on_changed,
                  QWidget* parent = nullptr);

    void showFFT(const std::string& name);
    void clear();

private slots:
    void nameEditedSlot();
    void positionEditedSlot();
    void modeSEditedSlot();
    void mode3AEditedSlot();
    void modeCEditedSlot();

private:
    void saveCurrent();

    DBContextManager& manager_;
    std::function<void()> on_changed_;

    std::string current_name_;
    bool has_current_{false};

    QLineEdit* name_edit_{nullptr};
    QLineEdit* latitude_edit_{nullptr};
    QLineEdit* longitude_edit_{nullptr};
    QLineEdit* altitude_edit_{nullptr};
    QLineEdit* mode_s_edit_{nullptr};
    QLineEdit* mode_3a_edit_{nullptr};
    QLineEdit* mode_c_edit_{nullptr};
};

} // namespace context
