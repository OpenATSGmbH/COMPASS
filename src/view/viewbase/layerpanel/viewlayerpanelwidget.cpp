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

#include "viewlayerpanelwidget.h"
#include "viewlayertreemodel.h"
#include "layertreeitem.h"

#include "dbcontentlayer.h"
#include "annotationsrootitem.h"

#include <QTreeView>

#include <functional>

/**
 */
ViewLayerPanelWidget::ViewLayerPanelWidget(const std::vector<LayerColumnSpec>& custom_columns,
                                           bool show_annotations,
                                           QWidget* parent,
                                           LayerTreeItemDelegate* delegate,
                                           ViewLayerTreeModel* external_model)
    : LayerPanelWidget(parent,
                       delegate,
                       external_model ? external_model : new ViewLayerTreeModel(custom_columns, show_annotations),
                       external_model != nullptr)
{
    model()->applyHeaderSettings(treeView()->header());

    // Auto-expand the Annotations subtree when it's rebuilt so newly added
    // group + leaf rows are visible immediately. Restricted to the
    // AnnotationsRootItem subtree to avoid fighting the DBContent root's
    // color-mode-driven default expansion (handled by the view's config
    // widget on layerTreeRebuiltSignal).
    connect(model(), &LayerTreeModel::subtreeRefreshedSignal,
            this, [this](LayerTreeItem* parent)
    {
        auto* anno_root = model()->annotationsRootItem();
        if (!anno_root || parent != anno_root)
            return;

        QTreeView* view = treeView();
        LayerTreeModel* m = model();

        const QModelIndex anno_idx = m->index(anno_root->row(), 0, QModelIndex());
        if (!anno_idx.isValid())
            return;

        std::function<void(const QModelIndex&)> expand_walk = [&](const QModelIndex& idx)
        {
            view->expand(idx);
            const int n = m->rowCount(idx);
            for (int i = 0; i < n; ++i)
                expand_walk(m->index(i, 0, idx));
        };
        expand_walk(anno_idx);
    });
}
