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
#include <QPointer>
#include <QTimer>
#include <QTreeView>
#include <QVBoxLayout>

namespace
{
    const int SpinnerIntervalMS = 100; // wait icon step interval
}

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

        handOverViewState();

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

    tree_view_->setModel(model_);

    // connected after setModel() => runs after the view has registered the new rows
    connect(model_, &QAbstractItemModel::rowsInserted, this, &LayerPanelWidget::applyDefaultExpansionToRows);
    connect(model_, &QAbstractItemModel::modelReset,   this, &LayerPanelWidget::applyDefaultExpansionToAll);

    // the predecessor was destroyed after this panel was created
    connect(model_, &LayerTreeModel::viewStateHandedOverSignal, this, [ this ] ()
    {
        takeOverViewState(false);
        clearViewHandover();
    });

    // checkboxes are painted from the items' hidden state => repaint, no relayout
    connect(model_, &LayerTreeModel::hiddenChangedSignal, tree_view_, [ this ] { tree_view_->viewport()->update(); });

    delegate_ = delegate ? delegate : new LayerTreeItemDelegate(this);
    tree_view_->setItemDelegate(delegate_);

    spinner_timer_ = new QTimer(this);
    spinner_timer_->setInterval(SpinnerIntervalMS);
    connect(spinner_timer_, &QTimer::timeout, this, &LayerPanelWidget::spinnerTick);
    connect(model_, &LayerTreeModel::staleChangedSignal, this, &LayerPanelWidget::updateSpinner);
    updateSpinner();

    // Header settings are re-applied after columns are registered; callers
    // should call model->addColumn(...) then model->applyHeaderSettings(header).
    model_->applyHeaderSettings(tree_view_->header());

    tree_view_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(tree_view_, &QTreeView::customContextMenuRequested,
            this, &LayerPanelWidget::onContextMenuRequested);

    layout->addWidget(tree_view_);
    setLayout(layout);

    // an existing model may carry the view state of a previous panel; kept on the items until the
    // construction is done, so the policies set by derived panels leave the handed over rows alone
    takeOverViewState(true);

    // an external model may be destroyed before
    QPointer<LayerTreeModel> model = model_;
    QTimer::singleShot(0, this, [ this, model ] ()
    {
        if (model)
            clearViewHandover();
    });
}

/**
 */
void LayerPanelWidget::setStaleIndication(int flags)
{
    delegate_->setStaleIndication(flags);

    updateSpinner();
    tree_view_->viewport()->update();
}

/**
 */
bool LayerPanelWidget::isSpinnerActive() const
{
    return spinner_timer_->isActive();
}

/**
 * Runs the wait icon timer only while it has something to show.
 */
void LayerPanelWidget::updateSpinner()
{
    const bool spin = model_->hasStaleItems() &&
                      (delegate_->staleIndication() & LayerTreeItemDelegate::StaleSpinner);

    if (spin && !spinner_timer_->isActive())
        spinner_timer_->start();
    else if (!spin && spinner_timer_->isActive())
        spinner_timer_->stop();
}

/**
 * Notice: a viewport update only repaints the visible rows, nothing while the panel is hidden.
 */
void LayerPanelWidget::spinnerTick()
{
    if (!tree_view_->isVisible())
        return;

    delegate_->advanceSpinner();
    tree_view_->viewport()->update();
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
 */
void LayerPanelWidget::setDefaultExpansion(const DefaultExpansionFunc& func)
{
    const DefaultExpansionFunc old_default = expansion_policy_;

    expansion_policy_ = func;

    defaultExpansionChanged(old_default);
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
void LayerPanelWidget::visitAllRows(const VisitFunc& func) const
{
    const int n = model_->rowCount();
    if (n > 0)
        visitRows(QModelIndex(), 0, n - 1, 0, func);
}

/**
 */
bool LayerPanelWidget::defaultExpanded(const LayerTreeItem& item, int depth, const DefaultExpansionFunc& policy) const
{
    return item.isExpandable() && policy && policy(item, depth);
}

/**
 */
void LayerPanelWidget::setExpanded(const QModelIndex& index, bool expanded)
{
    if (expanded != tree_view_->isExpanded(index))
        tree_view_->setExpanded(index, expanded);
}

/**
 */
void LayerPanelWidget::applyDefaultExpansionToRows(const QModelIndex& parent, int first, int last)
{
    visitRows(parent, first, last, depthOf(parent) + 1, [ & ] (const QModelIndex& index, LayerTreeItem* item, int depth)
    {
        if (item->isExpandable())
            setExpanded(index, defaultExpanded(*item, depth, expansion_policy_));
    });
}

/**
 */
void LayerPanelWidget::applyDefaultExpansionToAll()
{
    const int n = model_->rowCount();
    if (n > 0)
        applyDefaultExpansionToRows(QModelIndex(), 0, n - 1);
}

/**
 */
void LayerPanelWidget::applyDefaultExpansion(const QModelIndex& index)
{
    if (index.isValid())
        applyDefaultExpansionToRows(index.parent(), index.row(), index.row());
}

/**
 * A row's expansion equal to old_default => it followed the default, else the user changed it.
 */
void LayerPanelWidget::defaultExpansionChanged(const DefaultExpansionFunc& old_default)
{
    visitAllRows([ & ] (const QModelIndex& index, LayerTreeItem* item, int depth)
    {
        if (!item->isExpandable() || item->viewHandover())
            return;

        if (tree_view_->isExpanded(index) != defaultExpanded(*item, depth, old_default))
            return;

        setExpanded(index, defaultExpanded(*item, depth, expansion_policy_));
    });
}

/**
 */
void LayerPanelWidget::handOverViewState()
{
    auto* selection = tree_view_->selectionModel();

    const QModelIndex current     = tree_view_->currentIndex();
    const QModelIndex current_row = current.isValid() ? current.sibling(current.row(), 0) : QModelIndex();

    visitAllRows([ & ] (const QModelIndex& index, LayerTreeItem* item, int)
    {
        LayerTreeItem::ViewHandover handover;
        handover.expanded = item->isExpandable() && tree_view_->isExpanded(index);
        handover.selected = selection && selection->isSelected(index);
        handover.current  = current_row.isValid() && index == current_row;

        item->setViewHandover(handover);
    });
}

/**
 */
void LayerPanelWidget::takeOverViewState(bool default_others)
{
    auto* selection = tree_view_->selectionModel();

    visitAllRows([ & ] (const QModelIndex& index, LayerTreeItem* item, int depth)
    {
        const auto& handover = item->viewHandover();

        if (!handover)
        {
            if (default_others && item->isExpandable())
                setExpanded(index, defaultExpanded(*item, depth, expansion_policy_));

            return;
        }

        if (item->isExpandable())
            setExpanded(index, handover->expanded);

        if (handover->selected && selection)
            selection->select(index, QItemSelectionModel::Select | QItemSelectionModel::Rows);

        if (handover->current && selection)
            selection->setCurrentIndex(index, QItemSelectionModel::NoUpdate);
    });
}

/**
 */
void LayerPanelWidget::clearViewHandover()
{
    model_->traverse([] (const QModelIndex&, LayerTreeItem* item) { item->clearViewHandover(); });
}
