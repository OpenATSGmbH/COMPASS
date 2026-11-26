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

#include <functional>
#include <memory>
#include <set>
#include <string>

class LayerTreeItem;
class LayerTreeModel;
class LayerTreeItemDelegate;
class LayerColumnSpec;

class QItemSelection;
class QModelIndex;
class QTimer;
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
 *
 * View state: kept rows keep expansion and selection natively (the model updates in place), new rows
 * get the default expansion policy, vanished rows are forgotten. A destroyed panel leaves its view
 * state on the items (LayerTreeItem::ViewHandover) for the next panel on the same model.
 */
class LayerPanelWidget : public QWidget
{
    Q_OBJECT

public:
    /// decides if a new item starts expanded (depth 0 = top level)
    typedef std::function<bool(const LayerTreeItem& item, int depth)> DefaultExpansionFunc;

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

    void expandLayers(int max_depth = -1);

    /// rows following the previous default switch to the new one, rows expanded or collapsed by the
    /// user (and rows handed over from a previous panel) keep their expansion
    virtual void setDefaultExpansion(const DefaultExpansionFunc& func);

    /// display of stale items, combination of LayerTreeItemDelegate::StaleIndication flags (default: all)
    void setStaleIndication(int flags);
    /// the wait icon animates (model has stale items, icon enabled)
    bool isSpinnerActive() const;

private slots:
    void onContextMenuRequested(const QPoint& pos);

protected:
    /// a policy input changed: rows that followed old_default follow the current policy, the others
    /// (user deviations, handed over rows) keep their expansion
    void defaultExpansionChanged(const DefaultExpansionFunc& old_default);
    /// the row and its descendants get the default expansion
    void applyDefaultExpansion(const QModelIndex& index);

private:
    typedef std::function<void(const QModelIndex&, LayerTreeItem*, int)> VisitFunc;

    void init(LayerTreeItemDelegate* delegate);

    void visitRows(const QModelIndex& parent, int first, int last, int depth, const VisitFunc& func) const;
    void visitAllRows(const VisitFunc& func) const;
    int depthOf(const QModelIndex& index) const;
    bool defaultExpanded(const LayerTreeItem& item, int depth, const DefaultExpansionFunc& policy) const;
    void setExpanded(const QModelIndex& index, bool expanded);

    /// new rows (insert, reset) get the default expansion
    void applyDefaultExpansionToRows(const QModelIndex& parent, int first, int last);
    void applyDefaultExpansionToAll();

    /// leaves expansion, selection and current row on the items (destructor)
    void handOverViewState();
    /// applies the view state left on the items, default_others => the rows without one get the default
    void takeOverViewState(bool default_others);
    void clearViewHandover();

    void updateSpinner();
    void spinnerTick();

    LayerTreeModel*                 model_    {nullptr};
    LayerTreeItemDelegate*          delegate_ {nullptr};
    QTreeView*                      tree_view_{nullptr};
    QTimer*                         spinner_timer_{nullptr}; // animates the wait icon of stale items

    bool model_is_external_ = false;

    DefaultExpansionFunc expansion_policy_; // effective default policy, see setDefaultExpansion()
};
