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

#include <QWidget>

#include <memory>
#include <set>
#include <string>

class LayerTreeItem;
class LayerTreeModel;
class LayerTreeItemDelegate;
class LayerColumnSpec;

class QTreeView;

/**
 * A Qt widget hosting a QTreeView over a LayerTreeModel.
 *
 * Views construct this, optionally register extra columns via the model
 * (model()->addColumn(...)), and add top-level subtrees via
 * addRootItem(std::unique_ptr<LayerTreeItem>). The panel owns the tree view,
 * the model, and the delegate.
 *
 * Context menu: clicked items contribute their own section via
 * LayerTreeItem::buildContextMenu(); the panel appends a global "All" section
 * (Select All, Deselect All, Expand All, Collapse All).
 */
class LayerPanelWidget : public QWidget
{
    Q_OBJECT

public:
    /// If `delegate` is null, a default LayerTreeItemDelegate is used.
    /// The panel takes ownership of a non-null delegate.
    explicit LayerPanelWidget(QWidget* parent = nullptr,
                              LayerTreeItemDelegate* delegate = nullptr);
    explicit LayerPanelWidget(QWidget* parent,
                              LayerTreeItemDelegate* delegate,
                              LayerTreeModel* model,
                              bool external_model);
    ~LayerPanelWidget() override;

    virtual LayerTreeModel* model() const { return model_; }
    QTreeView* treeView() const { return tree_view_; }

    /// Convenience: append a top-level item to the model's invisible root.
    LayerTreeItem* addRootItem(std::unique_ptr<LayerTreeItem> item);

    /// Persistence helpers routed through the model.
    std::set<std::string> persistedHiddenIds() const;
    void applyPersistedHiddenIds(const std::set<std::string>& ids);

    void enableAutoExpand(int max_depth = -1);
    void expandLayers(int max_depth = -1);

private slots:
    void onContextMenuRequested(const QPoint& pos);

private:
    void init(LayerTreeItemDelegate* delegate);

    void autoExpand();

    LayerTreeModel*                 model_    {nullptr};
    LayerTreeItemDelegate*          delegate_ {nullptr};
    QTreeView*                      tree_view_{nullptr};

    bool model_is_external_     = false;
    bool auto_expand_           = false;
    int  auto_expand_max_depth_ = -1;
};
