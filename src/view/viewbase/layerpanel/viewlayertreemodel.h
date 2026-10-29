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

#include "layertreemodel.h"
#include "layertreedefs.h"
#include "dbcontentlayer.h"

class AnnotationsRootItem;

/**
 */
class ViewLayerTreeModel : public LayerTreeModel
{
    Q_OBJECT

public:
    explicit ViewLayerTreeModel(const std::vector<LayerColumnSpec>& custom_columns,
                                bool show_annotations,
                                QObject* parent = nullptr);
    virtual ~ViewLayerTreeModel() override;

    DBContentRootItem* dbContentRootItem() { return dbcontent_root_item_; }
    const DBContentRootItem* dbContentRootItem() const { return dbcontent_root_item_; }

    AnnotationsRootItem* annotationsRootItem() { return annotations_root_item_; }
    const AnnotationsRootItem* annotationsRootItem() const { return annotations_root_item_; }

    bool showsAnnotations() const { return annotations_root_item_ != nullptr; }

    /// the view's data set is final (source finalize received, neither cleared nor partially updated since)
    void setDataComplete(bool complete) { data_complete_ = complete; }
    bool dataComplete() const { return data_complete_; }

    /// Updates the data subtree in place from the entries (see DBContentRootItem::updateFrom()), the view
    /// releases its old payloads afterwards; vanished items are forgotten.
    /// - complete data => a reload ends
    /// - while reloading, updates from incomplete data are skipped => the stale rows stay
    /// content_final => display not from the loaded data (e.g. an annotation): ends a reload
    void updateDataSubtree(const std::vector<DBContentRootItem::LeafEntry>& entries,
                           const std::vector<DBContentRootItem::TopLeafEntry>& top_leaves = {},
                           bool content_final = false);

    /// a reload starts: the leaves drop their payloads (released by the view next) and show stale until
    /// the final update; nothing to do if the subtree is empty
    void beginDataReload();
    /// removes the items of the data subtree (forgotten), ends a reload
    void clearDataSubtree();
    bool dataReloading() const;

    bool hasStaleItems() const override;

private:
    void setDataStale(bool stale);
    void notifySubtreeChanged(LayerTreeItem* item);

    DBContentRootItem*   dbcontent_root_item_   = nullptr;
    AnnotationsRootItem* annotations_root_item_ = nullptr;

    bool data_complete_ = false; // see setDataComplete()
};
