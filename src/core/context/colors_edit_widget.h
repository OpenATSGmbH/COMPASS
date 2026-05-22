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
#include <map>
#include <string>

class QRadioButton;
class QPushButton;
class QVBoxLayout;
class QGridLayout;

namespace context
{

class DBContextManager;

/**
 * Edits the `ContextColors` attached to the active context:
 *  - Preference: Light / Dark (used when auto-generating new DS base colors)
 *  - DSType palette: one color button per DSType + reset-to-defaults
 *  - DBContent palette: one color button per DBContent + reset-to-defaults
 *  - Data Source Colors: per-DS base color + Auto Light/Dark/Reset, four line
 *    color buttons with per-line reset.
 *
 * on_changed is invoked after any edit so the dialog can persist the context.
 */
class ColorsEditWidget : public QWidget
{
    Q_OBJECT
public:
    ColorsEditWidget(DBContextManager& manager,
                     std::function<void()> on_changed,
                     QWidget* parent = nullptr);

    /// rebuild the color-button rows for the current active context
    void refresh();

private slots:
    void preferenceChangedSlot();
    void dsTypeColorClickedSlot();
    void dbContentColorClickedSlot();
    void resetDSTypeDefaultsSlot();
    void resetDBContentDefaultsSlot();

    // per-data-source editing
    void dsBaseColorClickedSlot();
    void dsLineColorClickedSlot();

    // bulk actions applied to every data source at once
    void autoLightAllClickedSlot();
    void autoDarkAllClickedSlot();
    void resetAllClickedSlot();

private:
    void buildUI();
    void applySwatch(QPushButton* button, const QColor& color);
    void clearLayout(QVBoxLayout* layout);

    DBContextManager& manager_;
    std::function<void()> on_changed_;

    QRadioButton* light_radio_ {nullptr};
    QRadioButton* dark_radio_ {nullptr};

    QGridLayout* ds_type_grid_ {nullptr};
    QGridLayout* dbcontent_grid_ {nullptr};
    QGridLayout* data_source_grid_ {nullptr};

    std::map<std::string, QPushButton*> ds_type_buttons_;
    std::map<std::string, QPushButton*> dbcontent_buttons_;
};

} // namespace context
