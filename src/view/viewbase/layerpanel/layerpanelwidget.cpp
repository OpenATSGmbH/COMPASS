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
#include <QMenu>
#include <QTreeView>
#include <QVBoxLayout>

LayerPanelWidget::LayerPanelWidget(QWidget* parent, 
                                   LayerTreeItemDelegate* delegate)
    : QWidget(parent)
    , model_(std::make_unique<LayerTreeModel>())
{
    auto* layout = new QVBoxLayout(this);
    layout->setMargin(0);

    tree_view_ = new QTreeView(this);
    tree_view_->setModel(model_.get());

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
}

LayerPanelWidget::~LayerPanelWidget() = default;

LayerTreeItem* LayerPanelWidget::addRootItem(std::unique_ptr<LayerTreeItem> item)
{
    return model_->addRootItem(std::move(item));
}

std::set<std::string> LayerPanelWidget::persistedHiddenIds() const
{
    return model_->persistedHiddenIds();
}

void LayerPanelWidget::applyPersistedHiddenIds(const std::set<std::string>& ids)
{
    model_->applyPersistedHiddenIds(ids);
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
        item->buildContextMenu(menu);

    // --- Global section (always appended) ---
    menu.addSection("All");

    QAction* sel_all = menu.addAction("Select All");
    sel_all->setToolTip("Show every entry");
    connect(sel_all, &QAction::triggered, this, [this]{
        model_->traverse([](const QModelIndex&, LayerTreeItem* it){
            if (it->canHide()) it->setHidden(false, /*emit_signal=*/false);
        });
        emit model_->hiddenChangedSignal();
    });

    QAction* desel_all = menu.addAction("Deselect All");
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
