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

#include <optional>
#include <string>
#include <vector>

/**
 * Abstract view-supplied adapter for the leaf payload of a DBContent row.
 *
 * Owned by the view (typically one-per-series in a vector kept alive for the
 * lifetime of the data widget). The DBContentLeafItem stores a raw pointer
 * back into this and forwards visibility / custom-column queries.
 *
 * Notes:
 *  - color() is read when the leaf gets the payload (construction or rebind on
 *    an update); a color mode change updates the tree with new payloads.
 *  - setVisible is called on every effective-hidden-state change and on rebind.
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

    /// Stable id of the layer (leaf key, effective-hidden ids for the view's data
    /// filter). Empty string = no id.
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

class DBContentRootItem;

/**
 * Intermediate group (DSType / DS / Line) under the DBContent root.
 *
 * Own color is aggregated bottom-up after the tree is built: when every
 * colored child agrees, the group takes that color; otherwise the group's
 * color is invalid (no icon).
 *
 * Children are updated in place from the root's entries (DBContentRootItem::updateFrom()),
 * selected by the group's key path.
 */
class DBContentGroupItem : public LayerTreeItem
{
public:
    DBContentGroupItem(const std::string& name,
                       DBContentLayerLevel level,
                       const std::optional<unsigned int>& ds_id,
                       const DBContentRootItem* data_root,
                       const std::vector<std::string>& path);

    DBContentLayerLevel level() const { return level_; }
    /// data source groups only, if the view provides the id
    const std::optional<unsigned int>& dsID() const { return ds_id_; }

    /// Stable path-based id ("group:<DSType>[/<DS>[/<Line>]]") for the
    /// effective-hidden ids. Empty for the root level.
    std::string persistenceId() const override;

    /// by data source id if set, else by name (the key path covers the parents)
    std::string stateKeyId() const override;

    /// Recompute color_ from direct children: common valid color, or invalid
    /// if children disagree. Assumes children's colors are up-to-date.
    void recomputeColorFromDirectChildren();

    void buildContextMenu(QMenu& menu) override;

    std::optional<std::vector<std::string>> childKeys() const override;
    std::unique_ptr<LayerTreeItem> createChild(const std::string& key) override;
    bool updateChild(LayerTreeItem& child) override;

    bool isStale() const override;
    QVariant tooltip() const override;

protected:
    DBContentLayerLevel         level_;
    std::optional<unsigned int> ds_id_;               // data source groups only, see dsID()
    const DBContentRootItem*    data_root_ = nullptr; // holds the entries during an update
    std::vector<std::string>    path_;                // update keys from the DSType level down to this group
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

    /// cached, the payload may be destroyed before the leaf is removed
    std::string persistenceId() const override { return persistence_id_; }

    /// count empty without a payload
    QVariant itemData(int column) const override;

    void onEffectiveHiddenChanged() override;

    void buildContextMenu(QMenu& menu) override;

    bool isStale() const override;
    QVariant tooltip() const override;

    /// nullptr while detached (e.g. reloading)
    DBContentLeafPayload* payload() const { return payload_; }
    /// rebinds the leaf (same persistence id), color and visibility follow; nullptr detaches it
    void setPayload(DBContentLeafPayload* payload);

private:
    DBContentLeafPayload* payload_ {nullptr};
    std::string           persistence_id_;
};

// ---------------------------------------------------------------------------

/**
 * "DBContent" top-level group: the base-owned root of the DSType/DS/Line
 * subtree. Always shown (even empty).
 *
 * - updateFrom() updates the subtree in place from a flat list of leaf entries: groups and leaves
 *   which still exist are kept (leaves rebound to the new payloads), vanished ones dropped, new
 *   ones created; bottom-up color aggregation afterwards
 * - stale while reloading: the leaves are detached from their payloads (detachPayloads()) and
 *   shown as stale until the next update ends it (setStale())
 */
class DBContentRootItem : public DBContentGroupItem
{
public:
    struct LeafEntry
    {
        std::string                 ds_type;
        std::string                 ds_name;
        std::string                 line;          ///< e.g. "L1"
        std::string                 dbcontent;
        DBContentLeafPayload*       payload;       ///< lifetime owned by the view
        std::optional<unsigned int> ds_id;         ///< data source groups by id if set, else by name
    };

    /// leaf directly below the root, in front of the DSType groups (e.g. a selection overlay)
    struct TopLeafEntry
    {
        std::string           name;
        DBContentLeafPayload* payload; ///< lifetime owned by the view
    };

    DBContentRootItem();

    /// Updates the subtree in place, entries without payload are skipped, vanished items are forgotten.
    /// Notice: the entries are only used during the call, the view releases its old payloads afterwards.
    void updateFrom(const std::vector<LeafEntry>& entries,
                    const std::vector<TopLeafEntry>& top_leaves);

    /// all leaves drop their payloads (e.g. before the view releases them)
    void detachPayloads();

    bool isStale() const override { return stale_; }
    void setStale(bool stale) { stale_ = stale; }

    /// Recompute colors bottom-up, so color_ on each group reflects the common color of its direct
    /// children. Runs after every update.
    void recomputeColorsRecursive();

    std::optional<std::vector<std::string>> childKeys() const override;
    std::unique_ptr<LayerTreeItem> createChild(const std::string& key) override;
    bool updateChild(LayerTreeItem& child) override;
    void onSubtreeUpdated() override;

    /// update keys of the children of the group with the key path, nullopt outside updateFrom()
    std::optional<std::vector<std::string>> entryChildKeys(const std::vector<std::string>& path) const;
    /// the first entry below the key path whose next level has the key, nullptr if there is none
    const LeafEntry* findEntry(const std::vector<std::string>& path, const std::string& key) const;

private:
    const TopLeafEntry* findTopLeaf(const std::string& key) const;

    const std::vector<LeafEntry>*         entries_    = nullptr; // during updateFrom() only
    const std::vector<TopLeafEntry>*      top_leaves_ = nullptr; // during updateFrom() only
    std::vector<std::vector<std::string>> entry_keys_;           // update keys per entry, DSType level first
    bool                                  stale_      = false;
};
