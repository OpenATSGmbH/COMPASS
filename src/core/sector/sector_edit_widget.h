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

class QLineEdit;
class QComboBox;
class QCheckBox;
class QPushButton;
class QLabel;

namespace context
{

class DBContextManager;

class SectorEditWidget : public QWidget
{
    Q_OBJECT

public:
    SectorEditWidget(DBContextManager& manager,
                     std::function<void()> on_changed,
                     QWidget* parent = nullptr);

    void showSector(unsigned int sector_id);
    void clear();

private slots:
    void nameEditedSlot();
    void layerChangedSlot();
    void excludeChangedSlot();
    void colorClickedSlot();
    void altMinEditedSlot();
    void altMaxEditedSlot();

private:
    void saveCurrent();

    DBContextManager& manager_;
    std::function<void()> on_changed_;

    unsigned int current_sector_id_{0};
    bool has_current_{false};

    QLineEdit* name_edit_{nullptr};
    QComboBox* layer_combo_{nullptr};
    QCheckBox* exclude_check_{nullptr};
    QPushButton* color_button_{nullptr};
    QLineEdit* alt_min_edit_{nullptr};
    QLineEdit* alt_max_edit_{nullptr};
    QLabel* points_label_{nullptr};
};

} // namespace context
