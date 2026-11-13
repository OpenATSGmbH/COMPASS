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

#include "gridbackgrounditems.h"
#include "gridbackgroundrenderer.h"
#include "gridbackground_defs.h"
#include "gridview.h"

#include "viewmanager.h"
#include "appstate.h"
#include "appmode.h"
#include "db_context_manager.h"
#include "sector.h"
#include "sectorlayer.h"
#include "questiondialog.h"

#include "files.h"
#include "logger.h"

#include <QAction>
#include <QApplication>
#include <QBitmap>
#include <QColorDialog>
#include <QInputDialog>
#include <QMenu>
#include <QPixmap>
#include <QPixmapCache>
#include <QTimer>

#include <functional>

namespace
{
    const std::string SectorKeyPrefix = "S";

    const QString LiveReason = "Not available in Live Mode: Running";

    /// Delete All: deletes all sectors of the active Data Context after a confirmation
    void addDeleteAllAction(QMenu& menu, GridView& grid_view, bool loaded, bool enabled)
    {
        auto& ctx_man = grid_view.viewManager().dbContextManager();

        GridView* view = &grid_view;

        QAction* delete_all_action = menu.addAction("Delete All");
        delete_all_action->setToolTip(!loaded ? "No active Data Context"
                                      : !enabled ? LiveReason
                                      : "Delete all sectors of the active Data Context");
        delete_all_action->setEnabled(loaded && enabled && !ctx_man.sectorLayers().empty());

        QObject::connect(delete_all_action, &QAction::triggered, view, [ view ] ()
        {
            QTimer::singleShot(0, view, [ view ] ()
            {
                if (!QuestionDialog::ask(QApplication::activeWindow(), "Delete All Sectors",
                                         "Delete all sectors of the active Data Context?"))
                    return;

                view->viewManager().dbContextManager().deleteAllSectors();
            });
        });
    }

    /// the polygon icon in the sector color (as in the Geographic View)
    QIcon sectorIcon(const std::string& color)
    {
        QPixmap     pixmap;
        std::string filename = Utils::Files::getIconFilepath("polygon.png");

        if (!QPixmapCache::find(filename.c_str(), &pixmap))
        {
            pixmap.load(filename.c_str());
            QPixmapCache::insert(filename.c_str(), pixmap);
        }

        QBitmap mask = pixmap.createMaskFromColor(Qt::black, Qt::MaskOutColor);
        pixmap.fill(QColor(color.c_str()));
        pixmap.setMask(mask);

        return QIcon(pixmap);
    }

    /**
     * Asks for a name until it is not empty and does not exist, false on cancel.
     */
    bool askName(const QString& title,
                 const QString& label,
                 std::string& name,
                 const std::function<bool(const std::string&)>& exists,
                 const QString& exists_text)
    {
        QString text = QString::fromStdString(name);
        QString hint;

        while (true)
        {
            bool ok = false;

            text = QInputDialog::getText(QApplication::activeWindow(), title, hint + label, QLineEdit::Normal, text, &ok);

            if (!ok)
                return false;

            text = text.trimmed();

            if (text.isEmpty())
            {
                hint = "The name must not be empty.\n";
                continue;
            }

            if (exists(text.toStdString()))
            {
                hint = exists_text.arg(text) + "\n";
                continue;
            }

            name = text.toStdString();
            return true;
        }
    }
}

/********************************************************************************************
 * GridBackgroundItem
 ********************************************************************************************/

/**
 */
GridBackgroundItem::GridBackgroundItem(const std::string& name, GridView& view, const GridBackgroundRenderer& renderer)
:   LayerTreeItem(name)
,   view_        (view)
,   renderer_    (renderer)
{
}

/**
 */
GridBackgroundItem::~GridBackgroundItem() = default;

/**
 */
bool GridBackgroundItem::isEnabled() const
{
    return renderer_.available();
}

/**
 */
bool GridBackgroundItem::sectorEditsEnabled() const
{
    return view_.viewManager().appState().appMode() != AppMode::LiveRunning;
}

/**
 * Called for the item and all items below on every visibility change, only a change of the item's
 * own checked state is stored (not the cascade of an ancestor or a sync from the settings).
 */
void GridBackgroundItem::onEffectiveHiddenChanged()
{
    if (hidden_ == stored_hidden_)
        return;

    stored_hidden_ = hidden_;
    storeShown(!hidden_);
}

/**
 */
void GridBackgroundItem::initFromSettings()
{
    hidden_        = canHide() ? !shownInSettings() : false;
    stored_hidden_ = hidden_;

    updateDisplay();
}

/**
 */
void GridBackgroundItem::syncFromSettings()
{
    const bool hidden = canHide() ? !shownInSettings() : false;

    stored_hidden_ = hidden;

    if (hidden != hidden_)
        LayerTreeItem::setHidden(hidden, false);

    updateDisplay();
    notifyChanged();

    for (int i = 0; i < childCount(); ++i)
        if (auto item = dynamic_cast<GridBackgroundItem*>(child(i)))
            item->syncFromSettings();
}

/**
 */
void GridBackgroundItem::notifySubtreeChanged()
{
    notifyChanged();

    for (int i = 0; i < childCount(); ++i)
        if (auto item = dynamic_cast<GridBackgroundItem*>(child(i)))
            item->notifySubtreeChanged();
}

/********************************************************************************************
 * GridBackgroundRootItem
 ********************************************************************************************/

/**
 */
GridBackgroundRootItem::GridBackgroundRootItem(GridView& view, const GridBackgroundRenderer& renderer)
:   GridBackgroundItem("Background Map", view, renderer)
{
    initFromSettings();

    //drawing order from top to bottom: borders above the map
    appendChild(std::make_unique<GridBackgroundBordersItem>(view, renderer));
    appendChild(std::make_unique<GridBackgroundMapItem>(view, renderer));
    appendChild(std::make_unique<GridBackgroundColorItem>(view, renderer));
}

/**
 */
GridBackgroundRootItem::~GridBackgroundRootItem() = default;

/**
 * Map icon as in the Geographic View.
 */
QVariant GridBackgroundRootItem::icon() const
{
    return Utils::Files::IconProvider::getIcon("globe.png");
}

/**
 */
bool GridBackgroundRootItem::shownInSettings() const
{
    return view_.settings().background_show;
}

/**
 */
void GridBackgroundRootItem::storeShown(bool shown)
{
    view_.setBackgroundShow(shown);
}

/********************************************************************************************
 * GridBackgroundMapItem
 ********************************************************************************************/

/**
 */
GridBackgroundMapItem::GridBackgroundMapItem(GridView& view, const GridBackgroundRenderer& renderer)
:   GridBackgroundItem("Map", view, renderer)
{
    initFromSettings();
}

/**
 */
GridBackgroundMapItem::~GridBackgroundMapItem() = default;

/**
 */
bool GridBackgroundMapItem::shownInSettings() const
{
    return view_.settings().background_map_show;
}

/**
 */
void GridBackgroundMapItem::storeShown(bool shown)
{
    view_.setBackgroundMapShow(shown);
}

/**
 */
void GridBackgroundMapItem::updateDisplay()
{
    std::string name = "Map: " + gridbackground::mapTitle(view_.settings().background_map_file);

    if (renderer_.mapUnavailable())
        name += " (Not Available)";

    setName(name);
}

/**
 */
QVariant GridBackgroundMapItem::tooltip() const
{
    if (renderer_.mapUnavailable())
        return QString("The map could not be loaded. Switch the map off and on to try again.");

    return QString::fromStdString(gridbackground::mapDirectory() + view_.settings().background_map_file);
}

/**
 */
void GridBackgroundMapItem::buildContextMenu(QMenu& menu)
{
    GridView* view = &view_;

    QMenu* file_menu = menu.addMenu("Map File");

    const auto files = gridbackground::eligibleMapFiles(gridbackground::mapDirectory());

    if (files.empty())
    {
        QAction* none = file_menu->addAction("No Map Files Available");
        none->setEnabled(false);
    }

    for (const auto& file : files)
    {
        QAction* act = file_menu->addAction(QString::fromStdString(file.title));
        act->setCheckable(true);
        act->setChecked(file.file_name == view_.settings().background_map_file);

        const std::string file_name = file.file_name;

        QObject::connect(act, &QAction::triggered, view, [ view, file_name ] () { view->setBackgroundMapFile(file_name); });
    }

    QMenu* opacity_menu = menu.addMenu("Opacity");

    for (unsigned int i = 0; i <= 100; i += 10)
    {
        QAction* act = opacity_menu->addAction(QString::number(i) + "%");
        act->setCheckable(true);
        act->setChecked(view_.settings().background_map_opacity == i);

        QObject::connect(act, &QAction::triggered, view, [ view, i ] () { view->setBackgroundMapOpacity(i); });
    }
}

/********************************************************************************************
 * GridBackgroundBordersItem
 ********************************************************************************************/

/**
 */
GridBackgroundBordersItem::GridBackgroundBordersItem(GridView& view, const GridBackgroundRenderer& renderer)
:   GridBackgroundItem("National Borders", view, renderer)
{
    initFromSettings();
}

/**
 */
GridBackgroundBordersItem::~GridBackgroundBordersItem() = default;

/**
 */
bool GridBackgroundBordersItem::shownInSettings() const
{
    return view_.settings().background_borders_show;
}

/**
 */
void GridBackgroundBordersItem::storeShown(bool shown)
{
    view_.setBackgroundBordersShow(shown);
}

/********************************************************************************************
 * GridBackgroundColorItem
 ********************************************************************************************/

/**
 */
GridBackgroundColorItem::GridBackgroundColorItem(GridView& view, const GridBackgroundRenderer& renderer)
:   GridBackgroundItem("Background Color", view, renderer)
{
    initFromSettings();
}

/**
 */
GridBackgroundColorItem::~GridBackgroundColorItem() = default;

/**
 */
void GridBackgroundColorItem::updateDisplay()
{
    QColor color(QString::fromStdString(view_.settings().background_color));
    setColor(color.isValid() ? color : QColor(Qt::white));
}

/**
 * Deferred: the click is handled inside the tree view's event processing.
 */
bool GridBackgroundColorItem::iconClicked()
{
    GridView* view = &view_;

    QTimer::singleShot(0, view, [ view ] ()
    {
        QColor current(QString::fromStdString(view->settings().background_color));

        QColor color = QColorDialog::getColor(current, QApplication::activeWindow(), "Background Color");
        if (color.isValid())
            view->setBackgroundColor(color.name().toStdString());
    });

    return true;
}

/********************************************************************************************
 * GridBackgroundSectorsItem
 ********************************************************************************************/

/**
 */
GridBackgroundSectorsItem::GridBackgroundSectorsItem(GridView& view, const GridBackgroundRenderer& renderer)
:   GridBackgroundItem("Sectors", view, renderer)
{
    initFromSettings();
}

/**
 */
GridBackgroundSectorsItem::~GridBackgroundSectorsItem() = default;

/**
 */
QVariant GridBackgroundSectorsItem::icon() const
{
    return Utils::Files::IconProvider::getIcon("polygon.png");
}

/**
 */
bool GridBackgroundSectorsItem::shownInSettings() const
{
    return view_.settings().background_sectors_show;
}

/**
 */
void GridBackgroundSectorsItem::storeShown(bool shown)
{
    view_.setBackgroundSectorsShow(shown);
}

/**
 */
std::optional<std::vector<std::string>> GridBackgroundSectorsItem::childKeys() const
{
    std::vector<std::string> keys;

    auto& ctx_man = view_.viewManager().dbContextManager();

    if (ctx_man.hasActiveContext() && ctx_man.sectorsLoaded())
        for (const auto& layer : ctx_man.sectorLayers())
            keys.push_back(layer->name());

    return keys;
}

/**
 */
std::unique_ptr<LayerTreeItem> GridBackgroundSectorsItem::createChild(const std::string& key)
{
    return std::make_unique<GridBackgroundSectorLayerItem>(key, view_, renderer_);
}

/**
 * Delete All: all sectors of the active Data Context (Add New needs the sector edit tool of the Geographic View).
 */
void GridBackgroundSectorsItem::buildContextMenu(QMenu& menu)
{
    auto& ctx_man = view_.viewManager().dbContextManager();

    const bool loaded = ctx_man.hasActiveContext() && ctx_man.sectorsLoaded();

    addDeleteAllAction(menu, view_, loaded, sectorEditsEnabled());
}

/********************************************************************************************
 * GridBackgroundSectorLayerItem
 ********************************************************************************************/

/**
 */
GridBackgroundSectorLayerItem::GridBackgroundSectorLayerItem(const std::string& layer_name,
                                                             GridView& view,
                                                             const GridBackgroundRenderer& renderer)
:   GridBackgroundItem(layer_name, view, renderer)
{
    initFromSettings();
}

/**
 */
GridBackgroundSectorLayerItem::~GridBackgroundSectorLayerItem() = default;

/**
 */
QVariant GridBackgroundSectorLayerItem::icon() const
{
    return Utils::Files::IconProvider::getIcon("polygon.png");
}

/**
 */
bool GridBackgroundSectorLayerItem::shownInSettings() const
{
    return view_.backgroundSectorLayerShown(name_);
}

/**
 */
void GridBackgroundSectorLayerItem::storeShown(bool shown)
{
    view_.setBackgroundSectorLayerShow(name_, shown);
}

/**
 */
std::optional<std::vector<std::string>> GridBackgroundSectorLayerItem::childKeys() const
{
    std::vector<std::string> keys;

    auto& ctx_man = view_.viewManager().dbContextManager();

    if (!ctx_man.hasActiveContext() || !ctx_man.sectorsLoaded() || !ctx_man.hasSectorLayer(name_))
        return keys;

    for (const auto& sector : ctx_man.sectorLayer(name_)->sectors())
        keys.push_back(GridBackgroundSectorItem::key(sector->id()));

    return keys;
}

/**
 */
std::unique_ptr<LayerTreeItem> GridBackgroundSectorLayerItem::createChild(const std::string& key)
{
    const unsigned int sector_id = (unsigned int)std::stoul(key.substr(SectorKeyPrefix.size()));

    return std::make_unique<GridBackgroundSectorItem>(sector_id, view_, renderer_);
}

/**
 * Name or color of a kept sector may have changed.
 */
bool GridBackgroundSectorLayerItem::updateChild(LayerTreeItem& child)
{
    auto sector_item = dynamic_cast<GridBackgroundSectorItem*>(&child);

    return sector_item ? sector_item->refresh() : false;
}

/**
 * Rename Layer, Set Color, Delete (this layer with its sectors), Delete All (all sectors), Add New needs
 * the sector edit tool of the Geographic View.
 */
void GridBackgroundSectorLayerItem::buildContextMenu(QMenu& menu)
{
    auto& ctx_man = view_.viewManager().dbContextManager();

    const bool        enabled    = sectorEditsEnabled();
    const std::string layer_name = name_;

    auto sector_layer = ctx_man.hasSectorLayer(layer_name) ? ctx_man.sectorLayer(layer_name) : nullptr;

    QColor initial_color = !sector_layer || sector_layer->sectors().empty()
        ? QColor(Sector::defaultColorStr().c_str())
        : QColor(sector_layer->sectors().front()->colorStr().c_str());

    GridView* view = &view_;

    QAction* rename_action = menu.addAction("Rename Layer...");
    rename_action->setToolTip(enabled ? "Rename this sector layer" : LiveReason);
    rename_action->setEnabled(enabled);

    QObject::connect(rename_action, &QAction::triggered, view, [ view, layer_name ] ()
    {
        QTimer::singleShot(0, view, [ view, layer_name ] ()
        {
            auto& ctx_man = view->viewManager().dbContextManager();

            std::string new_name = layer_name;

            auto exists = [ &ctx_man, &layer_name ] (const std::string& name)
            {
                return name != layer_name && ctx_man.hasSectorLayer(name);
            };

            if (!askName("Rename Sector Layer", "Layer name:", new_name, exists, "A layer '%1' exists already."))
                return;

            if (new_name != layer_name && ctx_man.hasSectorLayer(layer_name))
                ctx_man.renameSectorLayer(layer_name, new_name);
        });
    });

    QAction* color_action = menu.addAction("Set Color...");
    color_action->setToolTip(enabled ? "Set the color of all sectors in this layer" : LiveReason);
    color_action->setEnabled(enabled);

    QObject::connect(color_action, &QAction::triggered, view, [ view, layer_name, initial_color ] ()
    {
        QTimer::singleShot(0, view, [ view, layer_name, initial_color ] ()
        {
            QColor color = QColorDialog::getColor(initial_color, QApplication::activeWindow(), "Sector Layer Color");

            if (!color.isValid())
                return;

            view->viewManager().dbContextManager().setSectorLayerColor(layer_name, color);
        });
    });

    menu.addSeparator();

    QAction* delete_action = menu.addAction("Delete");
    delete_action->setToolTip(enabled ? "Delete this sector layer with all its sectors" : LiveReason);
    delete_action->setEnabled(enabled);

    QObject::connect(delete_action, &QAction::triggered, view, [ view, layer_name ] ()
    {
        QTimer::singleShot(0, view, [ view, layer_name ] ()
        {
            if (!QuestionDialog::ask(QApplication::activeWindow(), "Delete Sector Layer",
                                     "Delete the sector layer '" + QString::fromStdString(layer_name)
                                     + "' with all its sectors?"))
                return;

            view->viewManager().dbContextManager().deleteSectorLayer(layer_name);
        });
    });

    addDeleteAllAction(menu, view_, true, enabled);
}

/********************************************************************************************
 * GridBackgroundSectorItem
 ********************************************************************************************/

/**
 */
GridBackgroundSectorItem::GridBackgroundSectorItem(unsigned int sector_id,
                                                   GridView& view,
                                                   const GridBackgroundRenderer& renderer)
:   GridBackgroundItem("", view, renderer)
,   sector_id_        (sector_id)
{
    refresh();
    initFromSettings();
}

/**
 */
GridBackgroundSectorItem::~GridBackgroundSectorItem() = default;

/**
 */
std::string GridBackgroundSectorItem::key(unsigned int sector_id)
{
    return SectorKeyPrefix + std::to_string(sector_id);
}

/**
 */
std::string GridBackgroundSectorItem::stateKeyId() const
{
    return key(sector_id_);
}

/**
 */
bool GridBackgroundSectorItem::refresh()
{
    auto& ctx_man = view_.viewManager().dbContextManager();

    if (!ctx_man.hasActiveContext() || !ctx_man.sectorsLoaded() || !ctx_man.hasSector(sector_id_))
        return false;

    auto sector = ctx_man.sector(sector_id_);

    bool changed = false;

    if (sector->name() != name_)
    {
        setName(sector->name());
        changed = true;
    }

    const std::string color = sector->colorStr();

    if (color != icon_color_)
    {
        icon_       = sectorIcon(color);
        icon_color_ = color;
        changed     = true;
    }

    return changed;
}

/**
 */
bool GridBackgroundSectorItem::shownInSettings() const
{
    return view_.backgroundSectorShown(sector_id_);
}

/**
 */
void GridBackgroundSectorItem::storeShown(bool shown)
{
    view_.setBackgroundSectorShow(sector_id_, shown);
}

/**
 * Rename, Set Color, Delete as in the Geographic View (Edit Polygon needs its sector edit tool).
 * The actions run deferred and look the sector up by id, it may be gone by then.
 */
void GridBackgroundSectorItem::buildContextMenu(QMenu& menu)
{
    const bool         enabled   = sectorEditsEnabled();
    const unsigned int sector_id = sector_id_;

    GridView* view = &view_;

    auto sectorById = [ view, sector_id ] () -> std::shared_ptr<Sector>
    {
        auto& ctx_man = view->viewManager().dbContextManager();
        return ctx_man.hasSector(sector_id) ? ctx_man.sector(sector_id) : nullptr;
    };

    QAction* rename_action = menu.addAction("Rename...");
    rename_action->setToolTip(enabled ? "Rename this sector" : LiveReason);
    rename_action->setEnabled(enabled);

    QObject::connect(rename_action, &QAction::triggered, view, [ view, sectorById ] ()
    {
        QTimer::singleShot(0, view, [ view, sectorById ] ()
        {
            auto sector = sectorById();
            if (!sector)
                return;

            auto& ctx_man = view->viewManager().dbContextManager();

            std::string old_name   = sector->name();
            std::string layer_name = sector->layerName();
            std::string new_name   = old_name;

            auto exists = [ &ctx_man, &old_name, &layer_name ] (const std::string& name)
            {
                return name != old_name && ctx_man.hasSector(name, layer_name);
            };

            if (!askName("Rename Sector", "Sector name:", new_name, exists, "A sector '%1' exists already in this layer."))
                return;

            // the dialog ran an event loop => look it up again
            sector = sectorById();

            if (sector && new_name != old_name)
                ctx_man.renameSector(sector, new_name);
        });
    });

    QAction* color_action = menu.addAction("Set Color...");
    color_action->setToolTip(enabled ? "Set the color of this sector" : LiveReason);
    color_action->setEnabled(enabled);

    QObject::connect(color_action, &QAction::triggered, view, [ view, sectorById ] ()
    {
        QTimer::singleShot(0, view, [ view, sectorById ] ()
        {
            auto sector = sectorById();
            if (!sector)
                return;

            QColor color = QColorDialog::getColor(QColor(sector->colorStr().c_str()), QApplication::activeWindow(), "Sector Color");

            sector = sectorById();

            if (!sector || !color.isValid())
                return;

            view->viewManager().dbContextManager().setSectorColor(sector, color);
        });
    });

    menu.addSeparator();

    QAction* delete_action = menu.addAction("Delete");
    delete_action->setToolTip(enabled ? "Delete this sector" : LiveReason);
    delete_action->setEnabled(enabled);

    QObject::connect(delete_action, &QAction::triggered, view, [ view, sectorById ] ()
    {
        QTimer::singleShot(0, view, [ view, sectorById ] ()
        {
            auto sector = sectorById();
            if (!sector)
                return;

            if (!QuestionDialog::ask(QApplication::activeWindow(), "Delete Sector",
                                     "Delete the sector '" + QString::fromStdString(sector->name())
                                     + "' from layer '" + QString::fromStdString(sector->layerName()) + "'?"))
                return;

            sector = sectorById();

            if (sector)
                view->viewManager().dbContextManager().deleteSector(sector);
        });
    });

    addDeleteAllAction(menu, view_, true, enabled);
}
