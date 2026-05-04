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

#include <QVariant>

#include <string>
#include <vector>

class QTreeView;

/**
 * Abstract view-supplied adapter for the leaf payload of a DBContent row.
 *
 * Owned by the view (typically one-per-series in a vector kept alive for the
 * lifetime of the data widget). The DBContentLeafItem stores a raw pointer
 * back into this and forwards visibility / custom-column queries.
 *
 * Notes:
 *  - color() is read at leaf construction to populate the icon; the payload
 *    is not expected to change color after rebuild — on color mode change
 *    the view rebuilds the tree.
 *  - setVisible is called on every effective-hidden-state change.
 */
class DBContentLeafPayload
{
public:
    virtual ~DBContentLeafPayload() = default;

    virtual std::string name() const = 0;
    virtual unsigned int count() const = 0;
    virtual unsigned int nullCount() const { return 0; }

    virtual QColor color() const = 0;
    /// Push visibility through to the underlying data (e.g. DataSeries::visible).
    virtual void setVisible(bool) = 0;

    /// Stable id across rebuilds so hidden state can be round-tripped via
    /// view points. Empty string = do not persist.
    virtual std::string persistenceId() const = 0;

    /// Optional per-column data. `view_col_idx` is the zero-based index of
    /// the custom column as registered by the view (i.e. model column minus 2).
    virtual QVariant customColumn(int /*view_col_idx*/) const { return {}; }
};

// ---------------------------------------------------------------------------

/**
 * Position in the 5-level DBContent tree. Only group items (DSType/DS/Line)
 * use level to drive context-menu text; DBContentRootItem uses it for default
 * expansion by color mode.
 */
enum class DBContentLayerLevel
{
    Root       = 0,    ///< visible "DBContent" parent
    DSType     = 1,
    DataSource = 2,
    Line       = 3,
    DBContent  = 4     ///< leaf
};

/**
 * Intermediate group (DSType / DS / Line) under the DBContent root.
 *
 * Own color is aggregated bottom-up after the tree is built: when every
 * colored child agrees, the group takes that color; otherwise the group's
 * color is invalid (no icon).
 */
class DBContentGroupItem : public LayerTreeItem
{
public:
    DBContentGroupItem(const std::string& name, DBContentLayerLevel level);

    DBContentLayerLevel level() const { return level_; }

    /// Recompute color_ from direct children: common valid color, or invalid
    /// if children disagree. Assumes children's colors are up-to-date.
    void recomputeColorFromDirectChildren();

    void buildContextMenu(QMenu& menu) override;

protected:
    DBContentLayerLevel level_;
};

// ---------------------------------------------------------------------------

/**
 * Leaf item (DBContent level). Holds a raw pointer to a view-owned payload.
 * Pushes visibility changes through to the payload on every effective-hidden
 * state change.
 */
class DBContentLeafItem : public LayerTreeItem
{
public:
    DBContentLeafItem(const std::string& name, DBContentLeafPayload* payload);

    std::string persistenceId() const override;

    QVariant itemData(int column) const override;

    void onEffectiveHiddenChanged() override;

    void buildContextMenu(QMenu& menu) override;

    DBContentLeafPayload* payload() const { return payload_; }

private:
    DBContentLeafPayload* payload_ {nullptr};
};

// ---------------------------------------------------------------------------

/**
 * "DBContent" top-level group: the base-owned root of the DSType/DS/Line
 * subtree. Always shown (even empty).
 *
 * Use rebuildFrom() to repopulate from a flat list of leaf entries. The 4
 * group levels are materialized on demand; bottom-up color aggregation runs
 * after population.
 */
class DBContentRootItem : public DBContentGroupItem
{
public:
    struct LeafEntry
    {
        std::string           ds_type;
        std::string           ds_name;
        std::string           line;          ///< e.g. "L1"
        std::string           dbcontent;
        DBContentLeafPayload* payload;       ///< lifetime owned by the view
    };

    DBContentRootItem();

    /// Build the DSType/DS/Line/DBContent subtree from `entries` as a list of
    /// DETACHED top-level children (the view hands these to
    /// LayerTreeModel::refreshSubtree, which attaches them inside scoped
    /// begin/endInsertRows — no full model reset needed).
    std::vector<std::unique_ptr<LayerTreeItem>> buildChildrenFrom(
        const std::vector<LeafEntry>& entries) const;

    /// Recompute colors bottom-up. Call after buildChildrenFrom's result has
    /// been reattached so color_ on each group reflects the common color of
    /// its (new) direct children.
    void recomputeColorsRecursive();

    /// Expand the tree so the row whose color distinguishes the current
    /// color mode is visible.
    ///   DSType         -> collapsed
    ///   DataSource     -> depth 0
    ///   DataSourceLine -> depth 1
    ///   DBContent      -> depth 2
    /// `mode` follows COMPASS::colorMode().
    void applyDefaultExpansionForColorMode(QTreeView* tree_view, unsigned int mode) const;
};
