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

#include "layertreeitem.h"

#include <QIcon>

#include <memory>
#include <optional>
#include <string>
#include <vector>

class GridView;
class GridBackgroundRenderer;

/**
 * Layer tree items of the Grid View background, list and sector style as in the Geographic View:
 *
 *   Background Map              (map icon)
 *     National Borders          (yellow, above the map and the grid)
 *     Map: <map title>          (context menu: Map File, Opacity)
 *     Background Color          (color icon as the data items, no checkbox, click on the icon: color dialog)
 *   Sectors                     top level, (polygon icon, context menu: Delete All)
 *     <sector layer>            (polygon icon, context menu: Rename Layer..., Set Color..., Delete, Delete All)
 *       <sector>                (polygon icon in the sector color, context menu: Rename..., Set Color..., Delete, Delete All)
 *
 * - the checked states live in the GridView settings (view presets, view points), the items
 *   write them on every change and read them on syncFromSettings()
 * - inactive (greyed out) while the background is not available (no longitude / latitude grid)
 * - Delete deletes the clicked layer or sector, Delete All all sectors, both after a confirmation
 * - sector edits change the Data Context (as in the Geographic View), drawing a new sector or
 *   editing a polygon needs the Geographic View
 */
class GridBackgroundItem : public LayerTreeItem
{
public:
    GridBackgroundItem(const std::string& name, GridView& view, const GridBackgroundRenderer& renderer);
    ~GridBackgroundItem() override;

    bool isEnabled() const override;
    bool isExpandable() const override { return false; }

    void onEffectiveHiddenChanged() override;

    /// takes the checked state (and the display) from the settings, also below
    void syncFromSettings();
    /// repaints the subtree (e.g. after an availability change)
    void notifySubtreeChanged();

protected:
    /// initial checked state and display, called by the constructors of the subclasses
    void initFromSettings();

    virtual bool shownInSettings() const = 0;
    virtual void storeShown(bool shown) = 0;
    virtual void updateDisplay() {}

    /// sector edits are not possible in live running
    bool sectorEditsEnabled() const;

    GridView&                     view_;
    const GridBackgroundRenderer& renderer_;

    bool stored_hidden_ = false; // checked state last written to / read from the settings
};

/**
 */
class GridBackgroundRootItem : public GridBackgroundItem
{
public:
    GridBackgroundRootItem(GridView& view, const GridBackgroundRenderer& renderer);
    ~GridBackgroundRootItem() override;

    bool isExpandable() const override { return true; }
    std::string stateKeyId() const override { return "background"; }
    QVariant icon() const override;

protected:
    bool shownInSettings() const override;
    void storeShown(bool shown) override;
};

/**
 */
class GridBackgroundMapItem : public GridBackgroundItem
{
public:
    GridBackgroundMapItem(GridView& view, const GridBackgroundRenderer& renderer);
    ~GridBackgroundMapItem() override;

    std::string stateKeyId() const override { return "map"; } // fixed, the name follows the map file
    QVariant tooltip() const override;

    void buildContextMenu(QMenu& menu) override;

protected:
    bool shownInSettings() const override;
    void storeShown(bool shown) override;
    void updateDisplay() override;
};

/**
 */
class GridBackgroundBordersItem : public GridBackgroundItem
{
public:
    GridBackgroundBordersItem(GridView& view, const GridBackgroundRenderer& renderer);
    ~GridBackgroundBordersItem() override;

protected:
    bool shownInSettings() const override;
    void storeShown(bool shown) override;
};

/**
 * Color of the plot area below the map, no checkbox.
 */
class GridBackgroundColorItem : public GridBackgroundItem
{
public:
    GridBackgroundColorItem(GridView& view, const GridBackgroundRenderer& renderer);
    ~GridBackgroundColorItem() override;

    bool canHide() const override { return false; }
    bool reservesCheckboxSpace() const override { return true; } // icon aligned with the siblings
    std::string stateKeyId() const override { return "background_color"; }

    /// color icon as the data items, a click on it opens the color dialog (no context menu)
    bool iconClicked() override;

protected:
    bool shownInSettings() const override { return true; }
    void storeShown(bool shown) override { (void)shown; }
    void updateDisplay() override;
};

/**
 * Sector layers of the data context as children (updated in place).
 */
class GridBackgroundSectorsItem : public GridBackgroundItem
{
public:
    GridBackgroundSectorsItem(GridView& view, const GridBackgroundRenderer& renderer);
    ~GridBackgroundSectorsItem() override;

    bool isExpandable() const override { return true; }
    std::string stateKeyId() const override { return "background_sectors"; }
    QVariant icon() const override;

    std::optional<std::vector<std::string>> childKeys() const override;
    std::unique_ptr<LayerTreeItem> createChild(const std::string& key) override;

    void buildContextMenu(QMenu& menu) override;

protected:
    bool shownInSettings() const override;
    void storeShown(bool shown) override;
};

/**
 * A sector layer, its sectors as children (updated in place, keyed by sector id).
 */
class GridBackgroundSectorLayerItem : public GridBackgroundItem
{
public:
    GridBackgroundSectorLayerItem(const std::string& layer_name, GridView& view, const GridBackgroundRenderer& renderer);
    ~GridBackgroundSectorLayerItem() override;

    bool isExpandable() const override { return true; }
    QVariant icon() const override;

    std::optional<std::vector<std::string>> childKeys() const override;
    std::unique_ptr<LayerTreeItem> createChild(const std::string& key) override;
    bool updateChild(LayerTreeItem& child) override;

    void buildContextMenu(QMenu& menu) override;

protected:
    bool shownInSettings() const override;
    void storeShown(bool shown) override;
};

/**
 * A sector, polygon icon in the sector color.
 */
class GridBackgroundSectorItem : public GridBackgroundItem
{
public:
    GridBackgroundSectorItem(unsigned int sector_id, GridView& view, const GridBackgroundRenderer& renderer);
    ~GridBackgroundSectorItem() override;

    std::string stateKeyId() const override; // "S<sector id>", the name may change
    QVariant icon() const override { return QVariant(icon_); }

    void buildContextMenu(QMenu& menu) override;

    /// name and icon from the data context, true if changed
    bool refresh();

    static std::string key(unsigned int sector_id);

protected:
    bool shownInSettings() const override;
    void storeShown(bool shown) override;

private:
    unsigned int sector_id_;
    QIcon        icon_;
    std::string  icon_color_;
};
