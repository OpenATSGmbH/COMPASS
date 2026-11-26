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

#include "viewlayertreemodel.h"

#include "dbcontentlayer.h"
#include "annotationsrootitem.h"

#include <functional>

#include <QTreeView>

/**
 */
ViewLayerTreeModel::ViewLayerTreeModel(const std::vector<LayerColumnSpec>& custom_columns,
                                       bool show_annotations,
                                       QObject* parent)
:   LayerTreeModel(parent)
{
    for (const auto& col : custom_columns)
        addColumn(col);

    // Always-shown "DBContent" root - panel owns it via the model.
    auto root_uptr = std::make_unique<DBContentRootItem>();
    dbcontent_root_item_ = root_uptr.get();
    addRootItem(std::move(root_uptr));

    if (show_annotations)
    {
        auto annot_root_uptr = std::make_unique<AnnotationsRootItem>();
        annotations_root_item_ = annot_root_uptr.get();
        addRootItem(std::move(annot_root_uptr));
    }
}

/**
 */
ViewLayerTreeModel::~ViewLayerTreeModel() = default;

/**
 */
void ViewLayerTreeModel::updateDataSubtree(const std::vector<DBContentRootItem::LeafEntry>& entries,
                                           const std::vector<DBContentRootItem::TopLeafEntry>& top_leaves,
                                           bool content_final)
{
    const bool final = data_complete_ || content_final;

    // partial data would drop the stale rows not loaded yet
    if (dataReloading() && !final)
        return;

    dbcontent_root_item_->updateFrom(entries, top_leaves);

    if (final)
        setDataStale(false);
}

/**
 */
void ViewLayerTreeModel::beginDataReload()
{
    if (dataReloading() || dbcontent_root_item_->childCount() == 0)
        return;

    dbcontent_root_item_->detachPayloads();

    setDataStale(true);
}

/**
 */
void ViewLayerTreeModel::clearDataSubtree()
{
    dbcontent_root_item_->updateFrom({}, {});

    setDataStale(false);
}

/**
 */
bool ViewLayerTreeModel::dataReloading() const
{
    return dbcontent_root_item_->isStale();
}

/**
 */
bool ViewLayerTreeModel::hasStaleItems() const
{
    return dataReloading();
}

/**
 * Notice: repaints the whole data subtree (greyed rows, empty counts).
 */
void ViewLayerTreeModel::setDataStale(bool stale)
{
    if (dbcontent_root_item_->isStale() == stale)
        return;

    dbcontent_root_item_->setStale(stale);

    notifySubtreeChanged(dbcontent_root_item_);

    emit staleChangedSignal();
}

/**
 */
void ViewLayerTreeModel::notifySubtreeChanged(LayerTreeItem* item)
{
    itemChanged(item);

    std::function<void(LayerTreeItem*)> notify = [ & ] (LayerTreeItem* parent)
    {
        childItemsChanged(parent);

        for (int i = 0; i < parent->childCount(); ++i)
            notify(parent->child(i));
    };
    notify(item);
}
