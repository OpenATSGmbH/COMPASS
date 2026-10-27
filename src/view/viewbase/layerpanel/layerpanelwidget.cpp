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

#include "layerpanelwidget.h"
#include "layertreeitem.h"
#include "layertreeitemdelegate.h"
#include "layertreemodel.h"

#include <QAction>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QMenu>
#include <QScopedValueRollback>
#include <QTreeView>
#include <QVBoxLayout>

/**
 * Default ctor.
 */
LayerPanelWidget::LayerPanelWidget(QWidget* parent, 
                                   LayerTreeItemDelegate* delegate)
    : QWidget           (parent)
    , model_            (new LayerTreeModel)
    , model_is_external_(false)
{
    init(delegate);
}

/**
 * Ctor with external model to manage.
 */
LayerPanelWidget::LayerPanelWidget(QWidget* parent, 
                                   LayerTreeItemDelegate* delegate,
                                   LayerTreeModel* model,
                                   bool external_model)
    : QWidget           (parent)
    , model_            (model ? model : new LayerTreeModel)
    , model_is_external_(external_model)
{
    init(delegate);
}

LayerPanelWidget::~LayerPanelWidget()
{
    // an external model may get a new panel (e.g. config widget rebuild) => hand over the view state,
    // the successor may already exist (a destroyed model is replaced by an empty one => pointer compare only)
    if (model_is_external_ && tree_view_->model() == model_)
    {
        model_->disconnect(this);

        captureViewStateAll();

        emit model_->viewStateHandedOverSignal();
    }

    if (!model_is_external_)
    {
        delete model_;
        model_ = nullptr;
    }
}

void LayerPanelWidget::init(LayerTreeItemDelegate* delegate)
{
    auto* layout = new QVBoxLayout(this);
    layout->setMargin(0);

    tree_view_ = new QTreeView(this);
    tree_view_->setObjectName("layer_tree"); // ui tests: item paths and context menus

    // connected before setModel() => runs before the selection model prunes vanishing rows
    connect(model_, &QAbstractItemModel::rowsAboutToBeRemoved, this, &LayerPanelWidget::captureViewState);
    connect(model_, &QAbstractItemModel::modelAboutToBeReset,  this, &LayerPanelWidget::captureViewStateAll);

    tree_view_->setModel(model_);

    // connected after setModel() => runs after the view has registered the new rows
    connect(model_, &QAbstractItemModel::rowsInserted, this, &LayerPanelWidget::restoreViewState);
    connect(model_, &QAbstractItemModel::modelReset,   this, &LayerPanelWidget::restoreViewStateAll);
    connect(tree_view_->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &LayerPanelWidget::updatePendingSelection);
    connect(model_, &LayerTreeModel::viewStateHandedOverSignal, this, &LayerPanelWidget::restoreViewStateAll);

    // checkboxes are painted from the items' hidden state => repaint, no relayout
    connect(model_, &LayerTreeModel::hiddenChangedSignal, tree_view_, [ this ] { tree_view_->viewport()->update(); });

    delegate_ = delegate ? delegate : new LayerTreeItemDelegate(this);
    tree_view_->setItemDelegate(delegate_);

    // Header settings are re-applied after columns are registered; callers
    // should call model->addColumn(...) then model->applyHeaderSettings(header).
    model_->applyHeaderSettings(tree_view_->header());

    tree_view_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(tree_view_, &QTreeView::customContextMenuRequested,
            this, &LayerPanelWidget::onContextMenuRequested);

    layout->addWidget(tree_view_);
    setLayout(layout);

    // an existing model may carry the state of a previous panel
    restoreViewStateAll();
}

LayerTreeItem* LayerPanelWidget::addRootItem(std::unique_ptr<LayerTreeItem> item)
{
    return model_->addRootItem(std::move(item));
}

void LayerPanelWidget::onContextMenuRequested(const QPoint& pos)
{
    const QModelIndex idx = tree_view_->indexAt(pos);
    LayerTreeItem* item = idx.isValid()
        ? static_cast<LayerTreeItem*>(idx.internalPointer())
        : nullptr;

    QMenu menu(this);
    menu.setToolTipsVisible(true);

    if (item)
    {
        item->buildContextMenu(menu);
        model_->buildCustomItemContextMenu(menu, item);
    }

    // --- Global section (always appended) ---
    menu.addSection("All");

    QAction* sel_all = menu.addAction("Show All");
    sel_all->setToolTip("Show every entry");
    connect(sel_all, &QAction::triggered, this, [this]{
        model_->traverse([](const QModelIndex&, LayerTreeItem* it){
            if (it->canHide()) it->setHidden(false, /*emit_signal=*/false);
        });
        emit model_->hiddenChangedSignal();
    });

    QAction* desel_all = menu.addAction("Hide All");
    desel_all->setToolTip("Hide every entry");
    connect(desel_all, &QAction::triggered, this, [this]{
        model_->traverse([](const QModelIndex&, LayerTreeItem* it){
            if (it->canHide()) it->setHidden(true, /*emit_signal=*/false);
        });
        emit model_->hiddenChangedSignal();
    });

    menu.addSeparator();

    QAction* expand_all = menu.addAction("Expand All");
    expand_all->setToolTip("Expand every row");
    connect(expand_all, &QAction::triggered, this, [this]{ tree_view_->expandAll(); });

    QAction* collapse_all = menu.addAction("Collapse All");
    collapse_all->setToolTip("Collapse every row");
    connect(collapse_all, &QAction::triggered, this, [this]{ tree_view_->collapseAll(); });

    menu.exec(tree_view_->viewport()->mapToGlobal(pos));
}

void LayerPanelWidget::expandLayers(int max_depth)
{
    auto expandCB = [ & ] (const QModelIndex& index, LayerTreeItem* item)
    {
        if (item && !item->isExpandable())
            return;

        treeView()->expand(index);
    };

    model()->traverse(expandCB, max_depth);
}

/**
 * Notice: also applied to the current items following the previous default.
 */
void LayerPanelWidget::setDefaultExpansion(const DefaultExpansionFunc& func)
{
    // user deviations from the previous default are kept
    captureViewStateAll();

    expansion_policy_ = func;

    restoreViewStateAll();
}

/**
 * Calls func on the given rows of parent and all their descendants.
 */
void LayerPanelWidget::visitRows(const QModelIndex& parent,
                                 int first,
                                 int last,
                                 int depth,
                                 const VisitFunc& func) const
{
    for (int row = first; row <= last; ++row)
    {
        const QModelIndex index = model_->index(row, 0, parent);
        if (!index.isValid())
            continue;

        func(index, static_cast<LayerTreeItem*>(index.internalPointer()), depth);

        const int n = model_->rowCount(index);
        if (n > 0)
            visitRows(index, 0, n - 1, depth + 1, func);
    }
}

/**
 * Depth of an index, 0 for top-level items, -1 for the invisible root.
 */
int LayerPanelWidget::depthOf(const QModelIndex& index) const
{
    int depth = -1;
    for (QModelIndex idx = index; idx.isValid(); idx = idx.parent())
        ++depth;

    return depth;
}

/**
 */
bool LayerPanelWidget::defaultExpanded(const LayerTreeItem& item, int depth) const
{
    return item.isExpandable() && expansion_policy_ && expansion_policy_(item, depth);
}

/**
 * Remembers expansion and selection of the given rows and their descendants before they vanish.
 * Notice: requires the items to be still alive, i.e. models must notify before mutating.
 */
void LayerPanelWidget::captureViewState(const QModelIndex& parent, int first, int last)
{
    auto& state = model_->state();

    const bool persist_expansion = state.persists(LayerTreeState::PersistExpansion);
    const bool persist_selection = state.persists(LayerTreeState::PersistSelection);

    if (!persist_expansion && !persist_selection)
        return;

    auto* selection = persist_selection ? tree_view_->selectionModel() : nullptr;

    const QModelIndex current     = persist_selection ? tree_view_->currentIndex() : QModelIndex();
    const QModelIndex current_row = current.isValid() ? current.sibling(current.row(), 0) : QModelIndex();

    auto captureItem = [ & ] (const QModelIndex& index, LayerTreeItem* item, int depth)
    {
        const std::string key = item->stateKey();
        if (key.empty())
            return;

        const bool expanded  = item->isExpandable() && tree_view_->isExpanded(index);
        const bool deviation = expanded != defaultExpanded(*item, depth);
        const bool selected  = selection && selection->isSelected(index);

        state.updateItemState(key, [ & ] (LayerTreeState::ItemState& item_state)
        {
            // only deviations from the default policy are remembered
            if (persist_expansion)
                item_state.expanded = deviation ? std::optional<bool>(expanded) : std::nullopt;

            if (selected)
                item_state.selected = true;
        });

        if (current_row.isValid() && index == current_row)
            state.setPendingCurrent(key);
    };

    visitRows(parent, first, last, depthOf(parent) + 1, captureItem);
}

/**
 */
void LayerPanelWidget::captureViewStateAll()
{
    const int n = model_->rowCount();
    if (n > 0)
        captureViewState(QModelIndex(), 0, n - 1);
}

/**
 * Applies remembered expansion (else the default policy) and the pending selection to the given
 * rows and their descendants.
 */
void LayerPanelWidget::restoreViewState(const QModelIndex& parent, int first, int last)
{
    auto* selection = tree_view_->selectionModel();

    QScopedValueRollback<bool> restoring(restoring_view_state_, true);

    auto restoreItem = [ & ] (const QModelIndex& index, LayerTreeItem* item, int depth)
    {
        const std::string key = item->stateKey();

        const LayerTreeState::ItemState* state = model_->state().itemState(key);

        if (item->isExpandable())
        {
            const bool expand = state && state->expanded ? *state->expanded : defaultExpanded(*item, depth);

            if (expand != tree_view_->isExpanded(index))
                tree_view_->setExpanded(index, expand);
        }

        if (!state || !selection || (!state->selected && !state->current))
            return;

        if (state->selected)
            selection->select(index, QItemSelectionModel::Select | QItemSelectionModel::Rows);

        if (state->current)
            selection->setCurrentIndex(index, QItemSelectionModel::NoUpdate);

        // restored => no longer pending
        model_->state().clearItemState(key, LayerTreeState::PersistSelection);
    };

    visitRows(parent, first, last, depthOf(parent) + 1, restoreItem);
}

/**
 */
void LayerPanelWidget::restoreViewStateAll()
{
    const int n = model_->rowCount();
    if (n > 0)
        restoreViewState(QModelIndex(), 0, n - 1);
}

/**
 * A new selection by the user supersedes the pending one.
 */
void LayerPanelWidget::updatePendingSelection(const QItemSelection& selected)
{
    if (restoring_view_state_ || selected.isEmpty())
        return;

    model_->state().clearPendingSelection();
}
