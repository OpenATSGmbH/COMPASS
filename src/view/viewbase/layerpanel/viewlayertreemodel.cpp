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
