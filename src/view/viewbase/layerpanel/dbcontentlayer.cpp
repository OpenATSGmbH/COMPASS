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

#include "dbcontentlayer.h"
#include "layertreemodel.h"
#include "traced_assert.h"

#include <QAction>
#include <QMenu>
#include <QString>

#include <algorithm>
#include <functional>
#include <set>

namespace
{
    const char* StaleTooltip = "Data is being reloaded";

    /// the stale flag of the Data subtree the item belongs to
    bool dataStale(const LayerTreeItem* item)
    {
        for (const LayerTreeItem* it = item; it; it = it->parentItem())
            if (const auto* root = dynamic_cast<const DBContentRootItem*>(it))
                return root->isStale();

        return false;
    }

    /// update key of the entry on the given level (1 = DSType ... 4 = leaf), as updateKey() of the item
    std::string entryKey(const DBContentRootItem::LeafEntry& entry, size_t level)
    {
        switch (level)
        {
            case 1: return entry.ds_type;
            case 2: return entry.ds_id ? "ds:" + std::to_string(*entry.ds_id) : entry.ds_name;
            case 3: return entry.line;
            default:
            {
                const std::string id = entry.payload->persistenceId();
                return id.empty() ? entry.dbcontent : id;
            }
        }
    }

    /// group name on the given level (1 = DSType ... 3 = line), leaf name on level 4
    const std::string& entryName(const DBContentRootItem::LeafEntry& entry, size_t level)
    {
        switch (level)
        {
            case 1:  return entry.ds_type;
            case 2:  return entry.ds_name;
            case 3:  return entry.line;
            default: return entry.dbcontent;
        }
    }

    /// leaf below the root: keyed as the item (persistence id, else the name)
    std::string topLeafKey(const DBContentRootItem::TopLeafEntry& entry)
    {
        const std::string id = entry.payload->persistenceId();
        return id.empty() ? entry.name : id;
    }

    const size_t LeafLevel = 4;
}

// ---- DBContentGroupItem ---------------------------------------------------

DBContentGroupItem::DBContentGroupItem(const std::string& name,
                                       DBContentLayerLevel level,
                                       const std::optional<unsigned int>& ds_id,
                                       const DBContentRootItem* data_root,
                                       const std::vector<std::string>& path)
    : LayerTreeItem(name), level_(level), ds_id_(ds_id), data_root_(data_root), path_(path)
{
}

/**
 * Static (not reconciled) outside an update of the root.
 */
std::optional<std::vector<std::string>> DBContentGroupItem::childKeys() const
{
    if (!data_root_)
        return std::nullopt;

    return data_root_->entryChildKeys(path_);
}

/**
 */
std::unique_ptr<LayerTreeItem> DBContentGroupItem::createChild(const std::string& key)
{
    traced_assert(data_root_);

    const auto* entry = data_root_->findEntry(path_, key);
    if (!entry)
        return nullptr;

    const size_t level = path_.size() + 1;

    if (level == LeafLevel)
        return std::make_unique<DBContentLeafItem>(entry->dbcontent, entry->payload);

    std::vector<std::string> path = path_;
    path.push_back(key);

    return std::make_unique<DBContentGroupItem>(entryName(*entry, level),
                                                (DBContentLayerLevel)level,
                                                level == 2 ? entry->ds_id : std::nullopt,
                                                data_root_,
                                                path);
}

/**
 * Leaves are rebound to the entry's payload, groups follow name changes (e.g. a renamed data source).
 */
bool DBContentGroupItem::updateChild(LayerTreeItem& child)
{
    traced_assert(data_root_);

    const auto* entry = data_root_->findEntry(path_, child.updateKey());
    if (!entry)
        return false;

    const size_t level = path_.size() + 1;

    bool changed = false;

    const std::string& name = entryName(*entry, level);
    if (child.name() != name)
    {
        child.setName(name);
        changed = true;
    }

    if (level == LeafLevel)
    {
        static_cast<DBContentLeafItem&>(child).setPayload(entry->payload);
        changed = true; // count, color
    }

    return changed;
}

/**
 */
bool DBContentGroupItem::isStale() const
{
    return dataStale(this);
}

/**
 */
QVariant DBContentGroupItem::tooltip() const
{
    return isStale() ? QVariant(StaleTooltip) : QVariant();
}

std::string DBContentGroupItem::stateKeyId() const
{
    return ds_id_ ? "ds:" + std::to_string(*ds_id_) : std::string();
}

std::string DBContentGroupItem::persistenceId() const
{
    if (level_ == DBContentLayerLevel::Root)
        return {};

    // Names along the parent chain (DSType / DS / Line) are stable across
    // rebuilds, so the joined path identifies the same group in the new tree.
    std::string path;
    for (const LayerTreeItem* it = this; it; it = it->parentItem())
    {
        const auto* group = dynamic_cast<const DBContentGroupItem*>(it);
        if (!group || group->level() == DBContentLayerLevel::Root)
            break;
        path = group->name() + (path.empty() ? "" : "/") + path;
    }
    return "group:" + path;
}

void DBContentGroupItem::recomputeColorFromDirectChildren()
{
    QColor common;
    bool mismatch = false;
    for (int i = 0; i < childCount(); ++i)
    {
        QColor cc = child(i)->color();
        if (!cc.isValid())
            continue;
        if (!common.isValid())
            common = cc;
        else if (common != cc)
        {
            mismatch = true;
            break;
        }
    }
    setColor(mismatch ? QColor() : common);
}

void DBContentGroupItem::buildContextMenu(QMenu& menu)
{
    const QString nm = QString::fromStdString(name());

    // resolved by key when triggered, the group may be rebuilt while the menu is open
    auto add = [&menu, this](const QString& text, const QString& tip,
                             std::function<void(LayerTreeItem&)> fn)
    {
        QAction* a = menu.addAction(text);
        a->setToolTip(tip);
        connectItemAction(a, fn);
    };

    switch (level_)
    {
        case DBContentLayerLevel::DSType:
            menu.addSection(QString("DSType \"%1\"").arg(nm));
            break;
        case DBContentLayerLevel::DataSource:
            menu.addSection(QString("Data Source \"%1\"").arg(nm));
            break;
        case DBContentLayerLevel::Line:
            menu.addSection(QString("Line \"%1\"").arg(nm));
            break;
        case DBContentLayerLevel::Root:
            menu.addSection("DBContent");
            break;
        default:
            return;
    }

    add("Show All Children",
        "Show every direct child of this entry (this entry and grandchildren untouched)",
        [](LayerTreeItem& group){
            for (int i = 0; i < group.childCount(); ++i)
                group.child(i)->setHidden(false, /*emit_signal=*/false);
            if (group.model())
                emit group.model()->hiddenChangedSignal();
        });
    add("Hide All Children",
        "Hide every direct child of this entry (this entry and grandchildren untouched)",
        [](LayerTreeItem& group){
            for (int i = 0; i < group.childCount(); ++i)
                group.child(i)->setHidden(true, /*emit_signal=*/false);
            if (group.model())
                emit group.model()->hiddenChangedSignal();
        });
}

// ---- DBContentLeafItem ---------------------------------------------------

DBContentLeafItem::DBContentLeafItem(const std::string& name, DBContentLeafPayload* payload)
    : LayerTreeItem(name), payload_(payload)
{
    traced_assert(payload_);
    setColor(payload_->color());

    persistence_id_ = payload_->persistenceId();
}

QVariant DBContentLeafItem::itemData(int column) const
{
    if (column == 0)
        return QString::fromStdString(name_);
    if (!payload_)
        return {};
    if (column == 1)
        return QVariant((qulonglong)payload_->count());
    // custom columns start at 2
    return payload_->customColumn(column - 2);
}

void DBContentLeafItem::onEffectiveHiddenChanged()
{
    if (payload_)
        payload_->setVisible(!effectiveHidden());
}

/**
 * Notice: a new payload starts visible, toggles while detached are pushed here.
 */
void DBContentLeafItem::setPayload(DBContentLeafPayload* payload)
{
    payload_ = payload;

    if (!payload_)
        return;

    traced_assert(payload_->persistenceId() == persistence_id_);

    setColor(payload_->color());
    payload_->setVisible(!effectiveHidden());
}

/**
 */
bool DBContentLeafItem::isStale() const
{
    return dataStale(this);
}

/**
 */
QVariant DBContentLeafItem::tooltip() const
{
    return isStale() ? QVariant(StaleTooltip) : QVariant();
}

void DBContentLeafItem::buildContextMenu(QMenu& menu)
{
    const QString nm = QString::fromStdString(name());
    menu.addSection(QString("DBContent \"%1\"").arg(nm));
    // Leaves have no per-item actions beyond the global "All" section. The
    // checkbox in col 0 toggles visibility; deselect-others-type actions live
    // on group items.
}

// ---- DBContentRootItem ---------------------------------------------------

DBContentRootItem::DBContentRootItem()
    : DBContentGroupItem("Data", DBContentLayerLevel::Root, std::nullopt, this, {})
{
}

/**
 * Notice: entries without payload are skipped.
 */
void DBContentRootItem::updateFrom(const std::vector<LeafEntry>& entries,
                                   const std::vector<TopLeafEntry>& top_leaves)
{
    entries_    = &entries;
    top_leaves_ = &top_leaves;

    entry_keys_.clear();
    entry_keys_.reserve(entries.size());

    for (const auto& entry : entries)
    {
        if (!entry.payload)
        {
            entry_keys_.emplace_back();
            continue;
        }

        std::vector<std::string> keys;
        keys.reserve(LeafLevel);
        for (size_t level = 1; level <= LeafLevel; ++level)
            keys.push_back(entryKey(entry, level));

        entry_keys_.push_back(std::move(keys));
    }

    update();

    // the payloads may be released by the view now
    entries_    = nullptr;
    top_leaves_ = nullptr;
    entry_keys_.clear();
}

/**
 */
void DBContentRootItem::detachPayloads()
{
    std::function<void(LayerTreeItem*)> detach = [ & ] (LayerTreeItem* item)
    {
        if (auto* leaf = dynamic_cast<DBContentLeafItem*>(item))
            leaf->setPayload(nullptr);

        for (int i = 0; i < item->childCount(); ++i)
            detach(item->child(i));
    };
    detach(this);
}

/**
 */
std::optional<std::vector<std::string>> DBContentRootItem::entryChildKeys(const std::vector<std::string>& path) const
{
    if (!entries_)
        return std::nullopt;

    const size_t level = path.size() + 1;
    traced_assert(level <= LeafLevel);

    std::vector<std::string> keys;
    std::set<std::string>    seen;

    for (const auto& entry_keys : entry_keys_)
    {
        if (entry_keys.empty())
            continue;

        if (!std::equal(path.begin(), path.end(), entry_keys.begin()))
            continue;

        const std::string& key = entry_keys[ level - 1 ];
        if (seen.insert(key).second)
            keys.push_back(key);
    }

    return keys;
}

/**
 */
const DBContentRootItem::LeafEntry* DBContentRootItem::findEntry(const std::vector<std::string>& path,
                                                                 const std::string& key) const
{
    if (!entries_)
        return nullptr;

    const size_t level = path.size() + 1;
    traced_assert(level <= LeafLevel);

    for (size_t i = 0; i < entry_keys_.size(); ++i)
    {
        const auto& entry_keys = entry_keys_[ i ];
        if (entry_keys.empty())
            continue;

        if (entry_keys[ level - 1 ] == key && std::equal(path.begin(), path.end(), entry_keys.begin()))
            return &(*entries_)[ i ];
    }

    return nullptr;
}

/**
 */
const DBContentRootItem::TopLeafEntry* DBContentRootItem::findTopLeaf(const std::string& key) const
{
    if (!top_leaves_)
        return nullptr;

    for (const auto& entry : *top_leaves_)
        if (entry.payload && topLeafKey(entry) == key)
            return &entry;

    return nullptr;
}

/**
 * The top leaves first, then the DSType groups.
 */
std::optional<std::vector<std::string>> DBContentRootItem::childKeys() const
{
    auto keys = entryChildKeys({});
    if (!keys)
        return std::nullopt;

    std::vector<std::string> all;
    all.reserve(top_leaves_->size() + keys->size());

    for (const auto& entry : *top_leaves_)
        if (entry.payload)
            all.push_back(topLeafKey(entry));

    all.insert(all.end(), keys->begin(), keys->end());

    return all;
}

/**
 */
std::unique_ptr<LayerTreeItem> DBContentRootItem::createChild(const std::string& key)
{
    if (const auto* top_leaf = findTopLeaf(key))
        return std::make_unique<DBContentLeafItem>(top_leaf->name, top_leaf->payload);

    return DBContentGroupItem::createChild(key);
}

/**
 */
bool DBContentRootItem::updateChild(LayerTreeItem& child)
{
    if (auto* leaf = dynamic_cast<DBContentLeafItem*>(&child))
    {
        const auto* top_leaf = findTopLeaf(child.updateKey());
        if (!top_leaf)
            return false;

        leaf->setName(top_leaf->name);
        leaf->setPayload(top_leaf->payload);
        return true;
    }

    return DBContentGroupItem::updateChild(child);
}

/**
 */
void DBContentRootItem::onSubtreeUpdated()
{
    recomputeColorsRecursive();
}

void DBContentRootItem::recomputeColorsRecursive()
{
    std::function<void(LayerTreeItem*)> walk = [&](LayerTreeItem* it)
    {
        for (int i = 0; i < it->childCount(); ++i)
            walk(it->child(i));

        if (auto* g = dynamic_cast<DBContentGroupItem*>(it))
            g->recomputeColorFromDirectChildren();
    };
    walk(this);
}
