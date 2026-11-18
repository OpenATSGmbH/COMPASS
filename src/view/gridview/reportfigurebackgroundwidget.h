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

#include "gridbackgroundsettings.h"

#include <QGroupBox>

#include <functional>
#include <set>
#include <string>

class QCheckBox;
class QComboBox;
class QLabel;
class QSpinBox;
class QTreeWidget;
class QTreeWidgetItem;

namespace context
{
    class DBContextManager;
}

/**
 * Group "Report Figure Background" of the report generation tasks (evaluation, data source analysis): edits the
 * background set of their geographic report figures, with the fields of the Grid View layer panel.
 *
 * The sector tree shows the task default selection until the user changes a sector checkbox. Then the selection
 * is stored as hidden layers and hidden sectors.
 */
class ReportFigureBackgroundWidget : public QGroupBox
{
    Q_OBJECT

signals:
    void backgroundChangedSignal();

public:
    typedef std::function<std::set<std::string>()> DefaultHiddenLayersFunc;

    /// default_hidden_layers gives the hidden sector layers of the task default
    ReportFigureBackgroundWidget(const context::DBContextManager& ctx_man,
                                 const DefaultHiddenLayersFunc& default_hidden_layers,
                                 QWidget* parent = nullptr);
    ~ReportFigureBackgroundWidget() override;

    void setBackground(const ReportFigureBackground& background);
    const ReportFigureBackground& background() const { return background_; }

protected:
    /// the task default can change while the widget is hidden (e.g. the evaluated sector layers)
    void showEvent(QShowEvent* event) override;

private:
    void updateFromBackground();
    void updateMapFiles();
    void updateSectorTree();
    void updateEnabledStates();

    void selectColor();
    void sectorItemChanged(QTreeWidgetItem* item, int column);
    void setAllSectorLayers(bool shown);
    /// the shown selection becomes the own selection of the user
    void takeOverSectorSelection();
    void changed();

    const context::DBContextManager& ctx_man_;
    DefaultHiddenLayersFunc          default_hidden_layers_;

    ReportFigureBackground background_;

    QLabel*      color_label_        {nullptr};
    QCheckBox*   map_show_check_     {nullptr};
    QComboBox*   map_file_combo_     {nullptr};
    QSpinBox*    map_opacity_box_    {nullptr};
    QCheckBox*   borders_show_check_ {nullptr};
    QCheckBox*   sectors_show_check_ {nullptr};
    QTreeWidget* sector_tree_        {nullptr};
    QWidget*     sector_buttons_     {nullptr};

    bool updating_ = false;
};
