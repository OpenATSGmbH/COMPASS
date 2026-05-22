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
#include <vector>

class QLineEdit;
class QComboBox;
class QCheckBox;
class QPushButton;
class QLabel;
class Sector;

namespace context
{

class SectorEditWidget : public QWidget
{
    Q_OBJECT

public:
    using LayerNamesFunc = std::function<std::vector<std::string>()>;

    SectorEditWidget(std::function<void()> on_changed,
                     LayerNamesFunc layer_names_func,
                     QWidget* parent = nullptr);

    void show(Sector& sector);
    void clear();
    void setReadOnly(bool read_only);

    bool hasCurrentSector() const { return current_sector_ != nullptr; }
    unsigned int currentSectorId() const;

private slots:
    void nameEditedSlot();
    void layerChangedSlot();
    void excludeChangedSlot();
    void colorClickedSlot();
    void altMinEditedSlot();
    void altMaxEditedSlot();

private:
    std::function<void()> on_changed_;
    LayerNamesFunc layer_names_func_;

    Sector* current_sector_{nullptr};
    bool read_only_{false};

    QLineEdit* name_edit_{nullptr};
    QComboBox* layer_combo_{nullptr};
    QCheckBox* exclude_check_{nullptr};
    QPushButton* color_button_{nullptr};
    QLineEdit* alt_min_edit_{nullptr};
    QLineEdit* alt_max_edit_{nullptr};
    QLabel* points_label_{nullptr};
};

} // namespace context
