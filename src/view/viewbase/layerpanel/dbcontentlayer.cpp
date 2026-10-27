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

#include <functional>
#include <unordered_map>

// ---- DBContentGroupItem ---------------------------------------------------

DBContentGroupItem::DBContentGroupItem(const std::string& name,
                                       DBContentLayerLevel level,
                                       const std::optional<unsigned int>& ds_id)
    : LayerTreeItem(name), level_(level), ds_id_(ds_id)
{
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
    if (column == 1)
        return QVariant((qulonglong)(payload_ ? payload_->count() : 0u));
    // custom columns start at 2
    if (payload_)
        return payload_->customColumn(column - 2);
    return {};
}

void DBContentLeafItem::onEffectiveHiddenChanged()
{
    if (payload_)
        payload_->setVisible(!effectiveHidden());
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
    : DBContentGroupItem("Data", DBContentLayerLevel::Root)
{
}

namespace
{
    /// matches by ds_id if given (same-named data sources stay apart), else by name
    DBContentGroupItem* findOrCreateGroup(LayerTreeItem* parent,
                                          const std::string& name,
                                          DBContentLayerLevel level,
                                          const std::optional<unsigned int>& ds_id = std::nullopt)
    {
        for (int i = 0; i < parent->childCount(); ++i)
        {
            auto* c = static_cast<DBContentGroupItem*>(parent->child(i));

            if (ds_id ? c->dsID() == ds_id : !c->dsID() && c->name() == name)
                return c;
        }
        auto group = std::make_unique<DBContentGroupItem>(name, level, ds_id);
        auto* raw  = group.get();
        parent->appendChild(std::move(group));
        return raw;
    }
}

std::vector<std::unique_ptr<LayerTreeItem>> DBContentRootItem::buildChildrenFrom(
    const std::vector<LeafEntry>& entries) const
{
    // Assemble the subtree on a detached scratch parent, then move its
    // children out. The caller passes this list to refreshSubtree which
    // reattaches them to the real DBContentRootItem inside scoped
    // begin/endInsertRows - avoids the full modelReset that would make
    // QHeaderView drop its section widths.
    auto scratch = std::make_unique<DBContentGroupItem>("<scratch>", DBContentLayerLevel::Root);

    for (const auto& e : entries)
    {
        if (!e.payload)
            continue;

        auto* ds_type_group = findOrCreateGroup(scratch.get(),    e.ds_type,   DBContentLayerLevel::DSType);
        auto* ds_group      = findOrCreateGroup(ds_type_group,    e.ds_name,   DBContentLayerLevel::DataSource, e.ds_id);
        auto* line_group    = findOrCreateGroup(ds_group,         e.line,      DBContentLayerLevel::Line);

        auto leaf = std::make_unique<DBContentLeafItem>(e.dbcontent, e.payload);
        line_group->appendChild(std::move(leaf));
    }

    return scratch->moveChildrenOut();
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
