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

#include "reportfigurebackgroundwidget.h"
#include "gridbackground_defs.h"
#include "gridbackgrounditems.h"

#include "db_context_manager.h"
#include "sector.h"
#include "sectorlayer.h"

#include "ui_test_common.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

/**
 */
ReportFigureBackgroundWidget::ReportFigureBackgroundWidget(const context::DBContextManager& ctx_man,
                                                           const DefaultHiddenLayersFunc& default_hidden_layers,
                                                           QWidget* parent)
:   QGroupBox("Report Figure Background", parent)
,   ctx_man_              (ctx_man)
,   default_hidden_layers_(default_hidden_layers)
{
    UI_TEST_OBJ_NAME(this, "report_figure_background");

    auto form = new QFormLayout(this);

    // background color
    {
        auto row = new QHBoxLayout;

        color_label_ = new QLabel;
        color_label_->setFixedSize(40, 20);
        UI_TEST_OBJ_NAME(color_label_, "background_color");
        row->addWidget(color_label_);

        auto color_button = new QPushButton("Select Color");
        color_button->setIcon(QIcon());
        color_button->setToolTip("Select the color below the map and the grid");
        UI_TEST_OBJ_NAME(color_button, "background_color_button");
        connect(color_button, &QPushButton::clicked, this, &ReportFigureBackgroundWidget::selectColor);
        row->addWidget(color_button);
        row->addStretch(1);

        form->addRow("Background Color", row);
    }

    map_show_check_ = new QCheckBox;
    map_show_check_->setToolTip("Draw the map below the grid. Map tiles that are not in the cache need an internet connection.");
    UI_TEST_OBJ_NAME(map_show_check_, "show_map");
    connect(map_show_check_, &QCheckBox::toggled, this, [ this ] (bool on)
    {
        if (updating_)
            return;
        background_.settings.map_show = on;
        changed();
    });
    form->addRow("Show Map", map_show_check_);

    map_file_combo_ = new QComboBox;
    map_file_combo_->setToolTip("Map file of the maps folder of the data directory");
    UI_TEST_OBJ_NAME(map_file_combo_, "map_file");
    connect(map_file_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [ this ] (int idx)
    {
        if (updating_ || idx < 0)
            return;
        const QVariant file_name = map_file_combo_->itemData(idx);
        if (!file_name.isValid())
            return;
        background_.settings.map_file = file_name.toString().toStdString();
        changed();
    });
    form->addRow("Map", map_file_combo_);

    map_opacity_box_ = new QSpinBox;
    map_opacity_box_->setRange(0, 100);
    map_opacity_box_->setSingleStep(10);
    map_opacity_box_->setSuffix(" %");
    map_opacity_box_->setToolTip("Opacity of the map above the background color");
    UI_TEST_OBJ_NAME(map_opacity_box_, "map_opacity");
    connect(map_opacity_box_, QOverload<int>::of(&QSpinBox::valueChanged), this, [ this ] (int v)
    {
        if (updating_)
            return;
        background_.settings.map_opacity = static_cast<unsigned int>(v);
        changed();
    });
    form->addRow("Map Opacity", map_opacity_box_);

    borders_show_check_ = new QCheckBox;
    borders_show_check_->setToolTip("Draw the national borders above the map and the grid");
    UI_TEST_OBJ_NAME(borders_show_check_, "show_national_borders");
    connect(borders_show_check_, &QCheckBox::toggled, this, [ this ] (bool on)
    {
        if (updating_)
            return;
        background_.settings.borders_show = on;
        changed();
    });
    form->addRow("Show National Borders", borders_show_check_);

    sectors_show_check_ = new QCheckBox;
    sectors_show_check_->setToolTip("Draw the outlines of the selected sectors of the Data Context");
    UI_TEST_OBJ_NAME(sectors_show_check_, "show_sectors");
    connect(sectors_show_check_, &QCheckBox::toggled, this, [ this ] (bool on)
    {
        if (updating_)
            return;
        background_.settings.sectors_show = on;
        changed();
    });
    form->addRow("Show Sectors", sectors_show_check_);

    // sectors: one checkbox per sector layer and per sector
    {
        auto sectors_widget = new QWidget;
        auto sectors_layout = new QVBoxLayout(sectors_widget);
        sectors_layout->setContentsMargins(0, 0, 0, 0);

        sector_tree_ = new QTreeWidget;
        sector_tree_->setHeaderHidden(true);
        sector_tree_->setColumnCount(1);
        sector_tree_->setMinimumHeight(150);
        sector_tree_->setToolTip("Sector layers and sectors drawn in the figures");
        UI_TEST_OBJ_NAME(sector_tree_, "sector_tree");
        connect(sector_tree_, &QTreeWidget::itemChanged, this, &ReportFigureBackgroundWidget::sectorItemChanged);
        sectors_layout->addWidget(sector_tree_);

        sector_buttons_ = new QWidget;
        auto button_row = new QHBoxLayout(sector_buttons_);
        button_row->setContentsMargins(0, 0, 0, 0);

        auto select_all = new QPushButton("Select All");
        select_all->setIcon(QIcon());
        select_all->setToolTip("Draw all sector layers and all sectors");
        UI_TEST_OBJ_NAME(select_all, "select_all_button");
        connect(select_all, &QPushButton::clicked, this, [ this ] () { setAllSectorLayers(true); });

        auto select_none = new QPushButton("Select None");
        select_none->setIcon(QIcon());
        select_none->setToolTip("Draw no sector layer");
        UI_TEST_OBJ_NAME(select_none, "select_none_button");
        connect(select_none, &QPushButton::clicked, this, [ this ] () { setAllSectorLayers(false); });

        button_row->addWidget(select_all);
        button_row->addStretch(1);
        button_row->addWidget(select_none);

        sectors_layout->addWidget(sector_buttons_);

        form->addRow("Sectors", sectors_widget);
    }

    updateFromBackground();
}

/**
 */
ReportFigureBackgroundWidget::~ReportFigureBackgroundWidget() = default;

/**
 */
void ReportFigureBackgroundWidget::setBackground(const ReportFigureBackground& background)
{
    background_ = background;

    updateFromBackground();
}

/**
 */
void ReportFigureBackgroundWidget::showEvent(QShowEvent* event)
{
    updateSectorTree();

    QGroupBox::showEvent(event);
}

/**
 */
void ReportFigureBackgroundWidget::updateFromBackground()
{
    updating_ = true;

    const auto& settings = background_.settings;

    const QString color = QString::fromStdString(settings.color);
    color_label_->setStyleSheet("QLabel { background-color: " + color + "; border: 1px solid gray; }");
    color_label_->setToolTip(color);

    map_show_check_->setChecked(settings.map_show);
    map_opacity_box_->setValue(static_cast<int>(settings.map_opacity));
    borders_show_check_->setChecked(settings.borders_show);
    sectors_show_check_->setChecked(settings.sectors_show);

    updating_ = false;

    updateMapFiles();
    updateSectorTree();
    updateEnabledStates();
}

/**
 */
void ReportFigureBackgroundWidget::updateMapFiles()
{
    updating_ = true;

    map_file_combo_->clear();

    const std::string& current = background_.settings.map_file;

    int current_idx = -1;

    for (const auto& file : gridbackground::eligibleMapFiles(gridbackground::mapDirectory()))
    {
        if (file.file_name == current)
            current_idx = map_file_combo_->count();

        map_file_combo_->addItem(QString::fromStdString(file.title), QString::fromStdString(file.file_name));
    }

    // a stored map file that is not in the maps folder stays selected, it is not replaced silently
    if (current_idx < 0 && !current.empty())
    {
        current_idx = map_file_combo_->count();
        map_file_combo_->addItem(QString::fromStdString(gridbackground::mapTitle(current) + " (Not Available)"),
                                 QString::fromStdString(current));
    }

    if (map_file_combo_->count() == 0)
        map_file_combo_->addItem("No Map Files Available");

    map_file_combo_->setCurrentIndex(std::max(current_idx, 0));

    updating_ = false;
}

/**
 */
void ReportFigureBackgroundWidget::updateSectorTree()
{
    updating_ = true;

    sector_tree_->clear();

    const GridBackgroundSettings shown = background_.resolve(default_hidden_layers_());

    if (ctx_man_.hasActiveContext() && ctx_man_.sectorsLoaded())
    {
        for (const auto& layer : ctx_man_.sectorLayers())
        {
            const bool layer_shown = !shown.hidden_sector_layers.count(layer->name());

            auto layer_item = new QTreeWidgetItem(sector_tree_);
            layer_item->setText(0, QString::fromStdString(layer->name()));
            layer_item->setData(0, Qt::UserRole, QString::fromStdString(layer->name()));
            layer_item->setFlags(layer_item->flags() | Qt::ItemIsUserCheckable);
            layer_item->setCheckState(0, layer_shown ? Qt::Checked : Qt::Unchecked);

            for (const auto& sector : layer->sectors())
            {
                auto sector_item = new QTreeWidgetItem(layer_item);
                sector_item->setText(0, QString::fromStdString(sector->name()));
                sector_item->setIcon(0, gridBackgroundSectorIcon(sector->colorStr()));
                sector_item->setData(0, Qt::UserRole, sector->id());
                sector_item->setFlags(sector_item->flags() | Qt::ItemIsUserCheckable);
                sector_item->setCheckState(0, shown.hidden_sectors.count(sector->id()) ? Qt::Unchecked : Qt::Checked);
                sector_item->setDisabled(!layer_shown);
            }
        }
    }

    if (sector_tree_->topLevelItemCount() == 0)
    {
        auto placeholder = new QTreeWidgetItem(sector_tree_);
        placeholder->setText(0, "No sectors defined in the Data Context.");
        placeholder->setFlags(Qt::NoItemFlags);
    }

    updating_ = false;
}

/**
 */
void ReportFigureBackgroundWidget::updateEnabledStates()
{
    const bool has_map_file = map_file_combo_->itemData(map_file_combo_->currentIndex()).isValid();

    map_file_combo_->setEnabled(background_.settings.map_show && has_map_file);
    map_opacity_box_->setEnabled(background_.settings.map_show);
    sector_tree_->setEnabled(background_.settings.sectors_show);
    sector_buttons_->setEnabled(background_.settings.sectors_show);
}

/**
 */
void ReportFigureBackgroundWidget::selectColor()
{
    const QColor color = QColorDialog::getColor(QColor(QString::fromStdString(background_.settings.color)),
                                                this, "Background Color");
    if (!color.isValid())
        return;

    background_.settings.color = color.name().toStdString();

    updateFromBackground();
    emit backgroundChangedSignal();
}

/**
 */
void ReportFigureBackgroundWidget::sectorItemChanged(QTreeWidgetItem* item, int column)
{
    if (updating_ || column != 0 || !item)
        return;

    takeOverSectorSelection();

    const bool shown = item->checkState(0) == Qt::Checked;

    if (!item->parent())
    {
        const std::string layer_name = item->data(0, Qt::UserRole).toString().toStdString();

        if (shown)
            background_.hidden_sector_layers->erase(layer_name);
        else
            background_.hidden_sector_layers->insert(layer_name);

        updating_ = true;
        for (int i = 0; i < item->childCount(); ++i)
            item->child(i)->setDisabled(!shown);
        updating_ = false;
    }
    else
    {
        const unsigned int sector_id = item->data(0, Qt::UserRole).toUInt();

        if (shown)
            background_.settings.hidden_sectors.erase(sector_id);
        else
            background_.settings.hidden_sectors.insert(sector_id);
    }

    emit backgroundChangedSignal();
}

/**
 */
void ReportFigureBackgroundWidget::setAllSectorLayers(bool shown)
{
    takeOverSectorSelection();

    if (shown)
    {
        background_.hidden_sector_layers->clear();
        background_.settings.hidden_sectors.clear();
    }
    else
    {
        for (const auto& layer : ctx_man_.sectorLayers())
            background_.hidden_sector_layers->insert(layer->name());
    }

    updateSectorTree();
    emit backgroundChangedSignal();
}

/**
 */
void ReportFigureBackgroundWidget::takeOverSectorSelection()
{
    if (!background_.hidden_sector_layers.has_value())
        background_.hidden_sector_layers = default_hidden_layers_();
}

/**
 */
void ReportFigureBackgroundWidget::changed()
{
    updateEnabledStates();
    emit backgroundChangedSignal();
}
