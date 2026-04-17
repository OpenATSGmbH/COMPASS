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

#include "db_context_edit_dialog.h"
#include "colors_edit_widget.h"
#include "db_context_edit_tree_item.h"
#include "db_context_edit_tree_model.h"
#include "db_context_manager.h"
#include "db_context_serializer.h"
#include "db_context_copy_dialog.h"
#include "db_context_delete_dialog.h"
#include "db_context_rename_dialog.h"
#include "datasourceeditwidget.h"
#include "datasourcecreatedialog.h"
#include "asterixconfigwidget.h"
#include "sector_edit_widget.h"
#include "sector_import_utils.h"
#include "fft_edit_widget.h"
#include "questiondialog.h"
#include "importsectordialog.h"
#include "airspace.h"
#include "compass.h"
#include "sector.h"
#include "sectorlayer.h"
#include "logger.h"

#include <boost/filesystem.hpp>

#include <QApplication>
#include <QComboBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QMenu>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStackedWidget>
#include <QStyledItemDelegate>
#include <QHeaderView>
#include <QTreeView>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace context
{

/**
 * Custom delegate that right-aligns the "(sac/sic)" suffix in DataSource items.
 */
class TreeItemDelegate : public QStyledItemDelegate
{
public:
    using QStyledItemDelegate::QStyledItemDelegate;

    void paint(QPainter* painter, const QStyleOptionViewItem& option,
               const QModelIndex& index) const override
    {
        QStyleOptionViewItem opt = option;
        initStyleOption(&opt, index);

        QString text = opt.text;
        int paren = text.lastIndexOf('(');

        if (paren < 0)
        {
            QStyledItemDelegate::paint(painter, opt, index);
            return;
        }

        // draw selection/hover background
        opt.text.clear();
        QApplication::style()->drawControl(QStyle::CE_ItemViewItem, &opt, painter, opt.widget);

        QString name_part = text.left(paren).trimmed();
        QString suffix_part = text.mid(paren);

        painter->save();

        if (opt.state & QStyle::State_Selected)
            painter->setPen(opt.palette.color(QPalette::HighlightedText));
        else
            painter->setPen(opt.palette.color(QPalette::Text));

        QRect rect = opt.rect.adjusted(4, 0, -4, 0);
        painter->drawText(rect, Qt::AlignLeft | Qt::AlignVCenter, name_part);
        painter->drawText(rect, Qt::AlignRight | Qt::AlignVCenter, suffix_part);

        painter->restore();
    }
};

DBContextEditDialog::DBContextEditDialog(DBContextManager& manager, QWidget* parent)
    : QDialog(parent)
    , manager_(manager)
{
    loginf << "opening edit dialog";

    setWindowTitle("Edit Data Contexts");
    setMinimumSize(900, 600);
    setModal(true);

    auto* main_layout = new QVBoxLayout();

    // top bar: context selector + buttons
    {
        auto* top_layout = new QHBoxLayout();

        top_layout->addWidget(new QLabel("Context:"));

        context_combo_ = new QComboBox();
        context_combo_->setMinimumWidth(200);
        connect(context_combo_, &QComboBox::currentTextChanged,
                this, &DBContextEditDialog::contextComboChangedSlot);
        top_layout->addWidget(context_combo_);

        top_layout->addStretch();

        copy_button_ = new QPushButton("Copy");
        copy_button_->setIcon(QIcon());
        copy_button_->setToolTip("Copy this context under a new name");
        connect(copy_button_, &QPushButton::clicked, this, &DBContextEditDialog::copySlot);
        top_layout->addWidget(copy_button_);

        rename_button_ = new QPushButton("Rename");
        rename_button_->setIcon(QIcon());
        rename_button_->setToolTip("Rename the current context");
        connect(rename_button_, &QPushButton::clicked, this, &DBContextEditDialog::renameSlot);
        top_layout->addWidget(rename_button_);

        delete_button_ = new QPushButton("Delete");
        delete_button_->setIcon(QIcon());
        delete_button_->setToolTip("Delete one or more contexts");
        connect(delete_button_, &QPushButton::clicked, this, &DBContextEditDialog::deleteSlot);
        top_layout->addWidget(delete_button_);

        auto* export_button = new QPushButton("Export as ZIP");
        export_button->setIcon(QIcon());
        export_button->setToolTip("Export the current context as a zip archive");
        connect(export_button, &QPushButton::clicked, this, &DBContextEditDialog::exportZipSlot);
        top_layout->addWidget(export_button);

        auto* import_button = new QPushButton("Import from ZIP");
        import_button->setIcon(QIcon());
        import_button->setToolTip("Import a context from a zip archive");
        connect(import_button, &QPushButton::clicked, this, &DBContextEditDialog::importZipSlot);
        top_layout->addWidget(import_button);

        main_layout->addLayout(top_layout);
    }

    // splitter: tree (left) + detail (right)
    {
        auto* splitter = new QSplitter(Qt::Horizontal);

        // tree view
        tree_view_ = new QTreeView();
        tree_view_->setHeaderHidden(true);
        tree_view_->setItemDelegate(new TreeItemDelegate(tree_view_));
        tree_view_->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(tree_view_, &QTreeView::customContextMenuRequested,
                this, &DBContextEditDialog::showContextMenuSlot);
        connect(tree_view_, &QTreeView::clicked,
                this, &DBContextEditDialog::itemClickedSlot);
        splitter->addWidget(tree_view_);

        // detail area
        auto* scroll = new QScrollArea();
        scroll->setWidgetResizable(true);

        detail_stack_ = new QStackedWidget();
        scroll->setWidget(detail_stack_);

        splitter->addWidget(scroll);

        splitter->setSizes({320, 580});

        main_layout->addWidget(splitter, 1);
    }

    // close button
    {
        auto* button_layout = new QHBoxLayout();
        button_layout->addStretch();

        auto* close_button = new QPushButton("Close");
        close_button->setIcon(QIcon());
        connect(close_button, &QPushButton::clicked, this, &QDialog::accept);
        button_layout->addWidget(close_button);

        main_layout->addLayout(button_layout);
    }

    setLayout(main_layout);

    // build tree model
    tree_model_ = new DBContextEditTreeModel(manager_, this);
    tree_view_->setModel(tree_model_);

    // create data source edit widget
    ds_edit_widget_ = new DataSourceEditWidget(
        false,
        [this](unsigned int) { manager_.saveContext(manager_.activeContextName()); rebuildTree(); },
        [this](unsigned int id) { manager_.deleteDataSource(id); rebuildTree(); },
        &manager_);
    detail_stack_->addWidget(ds_edit_widget_);

    // create ASTERIX config widget
    asterix_widget_ = new ASTERIXConfigWidget(manager_, {}, {}, this);
    detail_stack_->addWidget(asterix_widget_);

    // create sector edit widget
    sector_edit_widget_ = new SectorEditWidget(
        [this]() {
            manager_.saveContext(manager_.activeContextName());
            manager_.rebuildSectorLayers();
            rebuildTree();
        },
        [this]() -> std::vector<std::string> {
            std::vector<std::string> names;
            for (const auto& layer : manager_.sectorLayers())
                names.push_back(layer->name());
            return names;
        },
        this);
    detail_stack_->addWidget(sector_edit_widget_);

    // create FFT edit widget
    fft_edit_widget_ = new FFTEditWidget([this]() {
        manager_.saveContext(manager_.activeContextName());
        rebuildTree();
    }, this);
    detail_stack_->addWidget(fft_edit_widget_);

    // create colors edit widget
    colors_edit_widget_ = new ColorsEditWidget(manager_, [this]() {
        manager_.saveContext(manager_.activeContextName());
    }, this);
    detail_stack_->addWidget(colors_edit_widget_);

    // initial empty placeholder
    auto* empty_label = new QLabel("Select an item to view details.");
    empty_label->setAlignment(Qt::AlignCenter);
    detail_stack_->addWidget(empty_label);
    detail_stack_->setCurrentWidget(empty_label);

    loginf << "dialog created, populating";

    // populate combo and expand tree
    rebuildContextCombo();
    tree_view_->expandAll();

    // connect to context changes
    connect(&manager_, &DBContextManager::activeContextChangedSignal,
            this, [this]() { rebuildContextCombo(); rebuildTree(); });
    connect(&manager_, &DBContextManager::contextsChangedSignal,
            this, [this]() { rebuildContextCombo(); rebuildTree(); });
}

DBContextEditDialog::~DBContextEditDialog() = default;

void DBContextEditDialog::rebuildContextCombo()
{
    loginf << "rebuilding context combo";

    QSignalBlocker blocker(context_combo_);
    context_combo_->clear();

    for (const auto& name : manager_.contextNames())
        context_combo_->addItem(QString::fromStdString(name));

    if (manager_.hasActiveContext())
        context_combo_->setCurrentText(QString::fromStdString(manager_.activeContextName()));

    bool db_has_data = manager_.compass().dbOpened() && manager_.hasInsertedData();

    // disable context switching when a DB with imported data is open
    context_combo_->setEnabled(!db_has_data);

    bool can_delete = manager_.contextNames().size() > 1 && !db_has_data;
    delete_button_->setEnabled(can_delete);
    delete_button_->setToolTip(db_has_data ? "Cannot delete contexts while a database with imported data is open"
                             : can_delete  ? "Delete one or more contexts"
                                           : "Cannot delete the last remaining context");
}

void DBContextEditDialog::rebuildTree()
{
    loginf << "rebuilding tree";

    tree_model_->rebuild();
    tree_view_->expandAll();
    ds_edit_widget_->clear();
    sector_edit_widget_->clear();
    fft_edit_widget_->clear();

    // refresh ASTERIX widget
    asterix_widget_->updateSlot();
}

void DBContextEditDialog::contextComboChangedSlot(const QString& name)
{
    std::string ctx_name = name.toStdString();
    if (ctx_name.empty() || !manager_.hasContext(ctx_name))
        return;

    if (manager_.hasActiveContext() && manager_.activeContextName() == ctx_name)
        return;

    loginf << "switching to context '" << ctx_name << "'";

    manager_.setActiveContext(ctx_name);
}

void DBContextEditDialog::itemClickedSlot(const QModelIndex& index)
{
    if (!index.isValid())
        return;

    auto* item = static_cast<DBContextEditTreeItem*>(index.internalPointer());

    if (auto* ds_item = dynamic_cast<DataSourceItem*>(item))
    {
        loginf << "clicked data source id " << ds_item->dsId();

        auto* ds = manager_.dataSource(ds_item->dsId());
        if (ds)
            ds_edit_widget_->show(*ds, manager_.compass().lastUsedPath());
        showDetailWidget(ds_edit_widget_);
    }
    else if (dynamic_cast<ASTERIXConfigLeafItem*>(item))
    {
        loginf << "clicked ASTERIX configuration";

        showDetailWidget(asterix_widget_);
    }
    else if (auto* group = dynamic_cast<GroupItem*>(item))
    {
        auto& ctx = manager_.activeContext();
        QString summary;
        switch (group->groupType())
        {
        case GroupItem::DataSources:
            summary = QString::number(ctx.dataSources().size()) + " data source(s)";
            break;
        case GroupItem::SectorLayers:
            summary = QString::number(manager_.sectorLayers().size()) + " sector layer(s), " +
                      QString::number(ctx.sectors().size()) + " sector(s)";
            break;
        case GroupItem::FFTs:
            summary = QString::number(ctx.ffts().size()) + " FFT(s)";
            break;
        case GroupItem::Colors:
            loginf << "clicked Colors group";
            colors_edit_widget_->refresh();
            showDetailWidget(colors_edit_widget_);
            return;
        }
        loginf << "clicked group: " << summary.toStdString();

        showDetailWidget(createPlaceholderLabel(summary));
    }
    else if (dynamic_cast<ColorsSubItem*>(item))
    {
        loginf << "clicked colors sub-item";
        colors_edit_widget_->refresh();
        showDetailWidget(colors_edit_widget_);
    }
    else if (auto* sl_item = dynamic_cast<SectorLayerItem*>(item))
    {
        if (manager_.sectorsLoaded() && manager_.hasSectorLayer(sl_item->layerName()))
        {
            auto layer = manager_.sectorLayer(sl_item->layerName());
            QString summary = QString::number(layer->size()) + " sector(s) in layer '" +
                              QString::fromStdString(sl_item->layerName()) + "'";
            showDetailWidget(createPlaceholderLabel(summary));
        }
    }
    else if (auto* sec_item = dynamic_cast<SectorItem*>(item))
    {
        loginf << "clicked sector id " << sec_item->sectorId();

        auto sec = manager_.sector(sec_item->sectorId());
        if (sec)
            sector_edit_widget_->show(*sec);
        showDetailWidget(sector_edit_widget_);
    }
    else if (auto* fft_item = dynamic_cast<FFTItem*>(item))
    {
        loginf << "clicked FFT '" << fft_item->fftName() << "'";

        auto* fft = manager_.fft(fft_item->fftName());
        if (fft)
            fft_edit_widget_->show(*fft);
        showDetailWidget(fft_edit_widget_);
    }
}

void DBContextEditDialog::showDetailWidget(QWidget* widget)
{
    if (!widget)
        return;

    logdbg << "showing detail widget";

    if (detail_stack_->indexOf(widget) < 0)
        detail_stack_->addWidget(widget);

    detail_stack_->setCurrentWidget(widget);
}

QWidget* DBContextEditDialog::createPlaceholderLabel(const QString& text)
{
    auto* label = new QLabel(text);
    label->setAlignment(Qt::AlignCenter);
    label->setWordWrap(true);
    return label;
}

void DBContextEditDialog::showContextMenuSlot(const QPoint& pos)
{
    QModelIndex index = tree_view_->indexAt(pos);
    if (!index.isValid())
        return;

    auto* item = static_cast<DBContextEditTreeItem*>(index.internalPointer());

    loginf << "context menu on item '" << item->data(0).toString().toStdString() << "'";

    if (auto* group = dynamic_cast<GroupItem*>(item))
    {
        switch (group->groupType())
        {
        case GroupItem::DataSources:  showDataSourcesGroupMenu(); break;
        case GroupItem::SectorLayers: showSectorLayersGroupMenu(); break;
        case GroupItem::FFTs:         showFFTsGroupMenu(); break;
        }
    }
    else if (auto* ds_item = dynamic_cast<DataSourceItem*>(item))
    {
        showDataSourceItemMenu(ds_item->dsId());
    }
    else if (auto* sl_item = dynamic_cast<SectorLayerItem*>(item))
    {
        showSectorLayerMenu(sl_item->layerName());
    }
    else if (auto* sec_item = dynamic_cast<SectorItem*>(item))
    {
        showSectorItemMenu(sec_item->sectorId());
    }
    else if (auto* fft_item = dynamic_cast<FFTItem*>(item))
    {
        showFFTItemMenu(fft_item->fftName());
    }
}

// ============================================================
// Context menu: Data Sources
// ============================================================

void DBContextEditDialog::showDataSourcesGroupMenu()
{
    loginf << "showing data sources group menu";

    QMenu menu;

    menu.addAction("Add Data Source...", [this]()
    {
        DataSourceCreateDialog dialog(manager_, this);
        connect(&dialog, &DataSourceCreateDialog::doneSignal, &dialog, &QDialog::close);
        dialog.exec();

        if (dialog.cancelled())
            return;

        auto& ds = manager_.createDataSource(dialog.sac(), dialog.sic(), "", dialog.dsType());
        rebuildTree();
    });

    menu.addSeparator();

    menu.addAction("Import...", [this]()
    {
        QString path = QFileDialog::getOpenFileName(this, "Import Sensors", "", "JSON Files (*.json)");
        if (path.isEmpty()) return;
        manager_.importSensors(path.toStdString());
        rebuildTree();
    });

    menu.addAction("Export...", [this]()
    {
        QString path = QFileDialog::getSaveFileName(this, "Export Sensors", "", "JSON Files (*.json)");
        if (path.isEmpty()) return;
        manager_.exportSensors(path.toStdString());
    });

    menu.addSeparator();

    auto* del_all = menu.addAction("Delete All", [this]()
    {
        if (!QuestionDialog::ask(this, "Delete All Data Sources",
                "Delete all data sources from this context?"))
            return;

        auto& ctx = manager_.activeContext();
        ctx.dataSources().clear();
        manager_.saveContext(manager_.activeContextName());
        rebuildTree();
    });

    // disable if empty or if any data source has data in the database
    bool any_has_data = false;
    for (const auto& ds : manager_.activeContext().dataSources())
    {
        if (manager_.hasNumInserted(ds.id()))
        {
            any_has_data = true;
            break;
        }
    }
    del_all->setEnabled(!manager_.activeContext().dataSources().empty() && !any_has_data);
    if (any_has_data)
        del_all->setToolTip("Cannot delete data sources that have data in the database");

    menu.exec(QCursor::pos());
}

void DBContextEditDialog::showDataSourceItemMenu(unsigned int ds_id)
{
    loginf << "showing menu for data source id " << ds_id;

    QMenu menu;

    bool has_data = manager_.hasNumInserted(ds_id);

    auto* del_action = menu.addAction("Delete", [this, ds_id]()
    {
        if (!QuestionDialog::ask(this, "Delete Data Source",
                "Delete this data source?"))
            return;

        manager_.deleteDataSource(ds_id);
        manager_.saveContext(manager_.activeContextName());
        ds_edit_widget_->clear();
        rebuildTree();
    });

    if (has_data)
    {
        del_action->setEnabled(false);
        del_action->setToolTip("Cannot delete a data source that has data in the database");
    }

    menu.exec(QCursor::pos());
}

// ============================================================
// Context menu: Sector Layers
// ============================================================

void DBContextEditDialog::showSectorLayersGroupMenu()
{
    loginf << "showing sector layers group menu";

    QMenu menu;

    menu.addAction("Import from File...", [this]()
    {
        QString path = QFileDialog::getOpenFileName(this, "Import Sectors",  "",
            "Vector Files (*.shp *.geojson *.json *.gml);;All Files (*)");
        if (path.isEmpty()) return;

        auto sectors = sector_utils::parseGDALFile(path.toStdString());
        if (sectors.empty())
        {
            QMessageBox::warning(this, "Import Sectors",
                "No sectors found in file or file could not be opened.");
            return;
        }

        loginf << "parsed " << sectors.size() << " sectors from '" << path.toStdString() << "'";

        // extract filename without extension as default layer name
        QFileInfo fi(path);
        std::string default_layer = fi.baseName().toStdString();

        ImportSectorDialog dialog(default_layer, this);
        if (dialog.exec() != QDialog::Accepted)
            return;

        std::string layer_name = dialog.layerName();
        bool exclude = dialog.exclude();
        QColor color = dialog.color();

        for (const auto& sec : sectors)
            manager_.createSector(sec.name, layer_name, exclude, color, sec.points);

        loginf << "imported " << sectors.size() << " sectors into layer '" << layer_name << "'";

        rebuildTree();
    });

    if (!manager_.compass().isAppImage())
    {
        menu.addAction("Import Air Space...", [this]()
        {
            importAirSpace();
            rebuildTree();
        });
    }

    menu.addSeparator();

    menu.addAction("Import JSON...", [this]()
    {
        QString path = QFileDialog::getOpenFileName(this, "Import Sectors (JSON)", "", "JSON Files (*.json)");
        if (path.isEmpty()) return;
        manager_.importSectors(path.toStdString());
        rebuildTree();
    });

    menu.addAction("Export JSON...", [this]()
    {
        QString path = QFileDialog::getSaveFileName(this, "Export Sectors", "", "JSON Files (*.json)");
        if (path.isEmpty()) return;
        manager_.exportSectors(path.toStdString());
    });

    menu.addSeparator();

    auto* del_all = menu.addAction("Delete All", [this]()
    {
        if (!QuestionDialog::ask(this, "Delete All Sectors",
                "Delete all sectors from this context?"))
            return;

        manager_.deleteAllSectors();
        rebuildTree();
    });
    del_all->setEnabled(!manager_.activeContext().sectors().empty());

    menu.exec(QCursor::pos());
}

void DBContextEditDialog::showSectorLayerMenu(const std::string& layer_name)
{
    loginf << "showing menu for sector layer '" << layer_name << "'";

    QMenu menu;

    menu.addAction("Delete Layer", [this, layer_name]()
    {
        if (!QuestionDialog::ask(this, "Delete Sector Layer",
                "Delete all sectors in layer '" + QString::fromStdString(layer_name) + "'?"))
            return;

        auto layer = manager_.sectorLayer(layer_name);
        if (!layer)
            return;

        // delete all sectors in this layer (copy vector since deletion modifies it)
        auto sectors_copy = layer->sectors();
        for (const auto& sector : sectors_copy)
            manager_.deleteSector(sector);

        rebuildTree();
    });

    menu.exec(QCursor::pos());
}

void DBContextEditDialog::showSectorItemMenu(unsigned int sector_id)
{
    loginf << "showing menu for sector id " << sector_id;

    QMenu menu;

    menu.addAction("Move to Layer...", [this, sector_id]()
    {
        auto sector = manager_.sector(sector_id);
        if (!sector)
            return;

        bool ok = false;
        QString new_layer = QInputDialog::getText(this, "Move Sector",
            "New layer name:", QLineEdit::Normal,
            QString::fromStdString(sector->layerName()), &ok);
        if (!ok || new_layer.isEmpty()) return;

        manager_.moveSector(sector_id, sector->layerName(), new_layer.toStdString());
        rebuildTree();
    });

    menu.addAction("Delete", [this, sector_id]()
    {
        if (!QuestionDialog::ask(this, "Delete Sector",
                "Delete this sector?"))
            return;

        auto sector = manager_.sector(sector_id);
        if (sector)
            manager_.deleteSector(sector);
        rebuildTree();
    });

    menu.exec(QCursor::pos());
}

// ============================================================
// Air Space import
// ============================================================

void DBContextEditDialog::importAirSpace()
{
    loginf;

    QString filename = QFileDialog::getOpenFileName(
        this, "Import Air Space Sectors from JSON",
        QString::fromStdString(manager_.compass().lastUsedPath()), "*.json");

    if (filename.isEmpty())
        return;

    auto max_sector_id = manager_.maxSectorId();

    AirSpace air_space;

    if (!air_space.readJSON(filename.toStdString(), max_sector_id))
    {
        QMessageBox::critical(this, "Error", "Air space file could not be read.");
        return;
    }

    // extract filename without extension as default layer name
    QFileInfo fi(filename);
    std::string default_layer = fi.baseName().toStdString();

    QDialog dlg(this);
    dlg.setWindowTitle("Import Air Space Sectors");

    auto* layout = new QVBoxLayout;
    dlg.setLayout(layout);

    // layer name input
    auto* layer_layout = new QHBoxLayout;
    layer_layout->addWidget(new QLabel("Layer name:"));
    auto* layer_edit = new QLineEdit(QString::fromStdString(default_layer));
    layer_edit->setToolTip("All imported sectors will be placed into this layer");
    layer_layout->addWidget(layer_edit);
    layout->addLayout(layer_layout);

    // sector selection tree
    auto* list = new QTreeWidget(&dlg);
    list->setHeaderLabels({ "", "Sector", "#Points", "Altitude min", "Altitude max" });
    list->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    list->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    list->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    list->header()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    list->header()->setSectionResizeMode(4, QHeaderView::ResizeToContents);

    layout->addWidget(list);

    auto* button_layout = new QHBoxLayout;
    auto* button_import = new QPushButton("Import");
    button_import->setIcon(QIcon());
    button_import->setToolTip("Import selected sectors");
    auto* button_cancel = new QPushButton("Cancel");
    button_cancel->setIcon(QIcon());
    button_cancel->setToolTip("Cancel import");

    connect(button_import, &QPushButton::pressed, &dlg, &QDialog::accept);
    connect(button_cancel, &QPushButton::pressed, &dlg, &QDialog::reject);

    button_layout->addStretch(1);
    button_layout->addWidget(button_import);
    button_layout->addWidget(button_cancel);

    layout->addLayout(button_layout);

    auto layers = air_space.layers();

    for (const auto& l : layers)
    {
        for (const auto& s : l->sectors())
        {
            auto* item = new QTreeWidgetItem;
            item->setCheckState(0, Qt::Checked);
            item->setText(1, QString::fromStdString(s->name()));
            item->setText(2, QString::number(s->points().size()));
            item->setText(3, s->hasMinimumAltitude() ? QString::number(s->minimumAltitude()) : "-");
            item->setText(4, s->hasMaximumAltitude() ? QString::number(s->maximumAltitude()) : "-");

            list->addTopLevelItem(item);
        }
    }

    dlg.resize(500, 800);

    if (dlg.exec() == QDialog::Rejected)
        return;

    std::string layer_name = layer_edit->text().trimmed().toStdString();
    if (layer_name.empty())
    {
        QMessageBox::warning(this, "Import Air Space", "Layer name must not be empty.");
        return;
    }

    std::map<std::string, bool> sectors_to_import;
    std::vector<std::string> duplicates;

    for (int i = 0; i < list->topLevelItemCount(); ++i)
    {
        auto* item = list->topLevelItem(i);
        if (item->checkState(0) != Qt::Checked)
            continue;

        std::string sec_name = item->text(1).toStdString();

        if (manager_.hasSector(sec_name, layer_name))
            duplicates.push_back(sec_name);
        else
            sectors_to_import[sec_name] = true;
    }

    if (!duplicates.empty())
    {
        QString msg = "The following sectors already exist in layer '" +
                      QString::fromStdString(layer_name) + "' and will be skipped:\n\n";
        for (const auto& d : duplicates)
            msg += "  - " + QString::fromStdString(d) + "\n";

        QMessageBox::information(this, "Duplicate Sectors", msg);
    }

    if (sectors_to_import.empty())
        return;

    manager_.importAirSpace(air_space, sectors_to_import, layer_name);
}

// ============================================================
// Context menu: FFTs
// ============================================================

void DBContextEditDialog::showFFTsGroupMenu()
{
    loginf << "showing FFTs group menu";

    QMenu menu;

    menu.addAction("Add FFT...", [this]()
    {
        bool ok = false;
        QString name = QInputDialog::getText(this, "Add FFT", "FFT name:",
                                             QLineEdit::Normal, "", &ok);
        if (!ok || name.isEmpty()) return;

        std::string fft_name = name.toStdString();
        if (manager_.hasFFT(fft_name))
        {
            QMessageBox::warning(this, "Add FFT", "An FFT with this name already exists.");
            return;
        }

        manager_.createFFT(fft_name);
        manager_.saveContext(manager_.activeContextName());
        rebuildTree();
    });

    menu.addSeparator();

    menu.addAction("Import...", [this]()
    {
        QString path = QFileDialog::getOpenFileName(this, "Import FFTs", "", "JSON Files (*.json)");
        if (path.isEmpty()) return;
        manager_.importFFTs(path.toStdString());
        rebuildTree();
    });

    menu.addAction("Export...", [this]()
    {
        QString path = QFileDialog::getSaveFileName(this, "Export FFTs", "", "JSON Files (*.json)");
        if (path.isEmpty()) return;
        manager_.exportFFTs(path.toStdString());
    });

    menu.addSeparator();

    auto* del_all = menu.addAction("Delete All", [this]()
    {
        if (!QuestionDialog::ask(this, "Delete All FFTs",
                "Delete all FFTs from this context?"))
            return;

        manager_.deleteAllFFTs();
        rebuildTree();
    });
    del_all->setEnabled(!manager_.activeContext().ffts().empty());

    menu.exec(QCursor::pos());
}

void DBContextEditDialog::showFFTItemMenu(const std::string& fft_name)
{
    loginf << "showing menu for FFT '" << fft_name << "'";

    QMenu menu;

    menu.addAction("Delete", [this, fft_name]()
    {
        if (!QuestionDialog::ask(this, "Delete FFT",
                "Delete FFT '" + QString::fromStdString(fft_name) + "'?"))
            return;

        manager_.deleteFFT(fft_name);
        rebuildTree();
    });

    menu.exec(QCursor::pos());
}

// ============================================================
// Top-bar buttons
// ============================================================

void DBContextEditDialog::copySlot()
{
    loginf << "copy button clicked";

    DBContextCopyDialog dialog(manager_, this);
    dialog.exec();
}

void DBContextEditDialog::renameSlot()
{
    if (!manager_.hasActiveContext())
        return;

    loginf << "rename button clicked";

    DBContextRenameDialog dialog(manager_, this);
    dialog.exec();
}

void DBContextEditDialog::deleteSlot()
{
    loginf << "delete button clicked";

    DBContextDeleteDialog dialog(manager_, this);
    dialog.exec();
}

void DBContextEditDialog::exportZipSlot()
{
    if (!manager_.hasActiveContext())
        return;

    const auto& name = manager_.activeContext().name();

    QString filepath = QFileDialog::getSaveFileName(
        this, "Export Context as Zip", QString::fromStdString(name + ".zip"),
        "Zip Archives (*.zip)");

    if (filepath.isEmpty())
        return;

    loginf << "exporting context '" << name << "' to " << filepath.toStdString();

    manager_.exportContextZip(name, filepath.toStdString());
}

void DBContextEditDialog::importZipSlot()
{
    QString filepath = QFileDialog::getOpenFileName(
        this, "Import Context from Zip", "",
        "Zip Archives (*.zip)");

    if (filepath.isEmpty())
        return;

    // peek at the zip to find the context name (first directory component)
    // by attempting the import — but first check if a context of that name exists
    // We need to read the name from the zip before extracting.
    // Use a temporary extraction to read the name, then confirm overwrite.

    // read context name from the zip meta file
    std::string zip_path = filepath.toStdString();

    // extract to a temp dir to read the name
    std::string temp_base = "/tmp/compass_ctx_import_" +
        std::to_string(QApplication::applicationPid());
    boost::filesystem::create_directories(temp_base);

    std::string ctx_name = DBContextSerializer::importContextZip(temp_base, zip_path);
    boost::filesystem::remove_all(temp_base);

    if (ctx_name.empty())
    {
        logerr << "could not determine context name from zip";
        return;
    }

    // check if context already exists
    if (manager_.hasContext(ctx_name))
    {
        if (!QuestionDialog::ask(this, "Overwrite Context",
                "A context named '" + QString::fromStdString(ctx_name) +
                "' already exists.\n\nOverwrite it with the imported version?"))
            return;

        // delete the existing context folder on disk
        DBContextSerializer::deleteContext(manager_.basePath(), ctx_name);
    }

    loginf << "importing context from " << zip_path;

    manager_.importContextZip(zip_path);
}

} // namespace context
