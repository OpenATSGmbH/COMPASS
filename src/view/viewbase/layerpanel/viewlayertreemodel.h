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

class DBContentRootItem;
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

    /// rebuilds the data subtree, prunes the states of vanished data items if the data is complete
    void refreshDataSubtree(const std::function<std::vector<std::unique_ptr<LayerTreeItem>>()>& build);

private:
    DBContentRootItem*   dbcontent_root_item_   = nullptr;
    AnnotationsRootItem* annotations_root_item_ = nullptr;

    bool data_complete_ = false; // see setDataComplete()
};
