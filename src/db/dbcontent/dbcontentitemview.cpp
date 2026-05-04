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

#include "dbcontentitemview.h"
#include "dbcontentitemmodel.h"
#include "dbcontentitemprovider.h"

#include "traced_assert.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QTreeView>
#include <QVBoxLayout>

/**
 */
DBContentItemView::DBContentItemView(DBContentItemModel& model, QWidget* parent)
    : QWidget(parent)
    , model_(model)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);

    // Grouping combo — populated before connecting to avoid spurious setGrouping() calls
    QHBoxLayout* grouping_layout = new QHBoxLayout();

    QLabel* grouping_label = new QLabel("Item Mode", this);
    grouping_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    grouping_layout->addWidget(grouping_label);

    grouping_box_ = new QComboBox(this);

    for (auto g : DBContentItemProvider::getGroupings())
    {
        grouping_box_->addItem(
            QString::fromStdString(DBContentItemProvider::groupingToString(g)));
    }

    grouping_box_->setCurrentText(
        QString::fromStdString(
            DBContentItemProvider::groupingToString(model_.provider().grouping())));

    connect(grouping_box_, &QComboBox::currentTextChanged,
            this, &DBContentItemView::groupingChangedSlot);

    grouping_layout->addWidget(grouping_box_);

    layout->addLayout(grouping_layout);

    // Item tree view
    tree_view_ = new QTreeView(this);
    tree_view_->setModel(&model_);
    tree_view_->setRootIsDecorated(false);
    tree_view_->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(tree_view_, &QTreeView::customContextMenuRequested,
            [this](const QPoint& pos) {
                model_.showContextMenu(tree_view_->indexAt(pos),
                                       tree_view_->viewport()->mapToGlobal(pos),
                                       tree_view_);
            });

    connect(tree_view_, &QTreeView::doubleClicked,
            &model_, &DBContentItemModel::itemDoubleClickedSlot);

    layout->addWidget(tree_view_);
}

/**
 */
void DBContentItemView::groupingChangedSlot(const QString& text)
{
    model_.provider().setGrouping(
        DBContentItemProvider::groupingFromString(text.toStdString()), true);
}

/**
 */
void DBContentItemView::setActiveGroupings(unsigned int flags, 
                                           const std::optional<dbContent::Grouping>& grouping,
                                           bool run_update)
{
    active_groupings_ = flags;
    auto groupings    = DBContentItemProvider::getGroupings(flags);
    traced_assert(!groupings.empty()); // !at least one grouping!

    auto last_item    = grouping_box_->currentText();
    auto default_item = QString::fromStdString(DBContentItemProvider::groupingToString(DBContentItemProvider::DefaultGrouping));
    auto ext_item     = grouping.has_value() ? QString::fromStdString(DBContentItemProvider::groupingToString(grouping.value())) : QString();

    grouping_box_->blockSignals(true);
    grouping_box_->clear();

    for (auto g : groupings)
    {
        grouping_box_->addItem(QString::fromStdString(DBContentItemProvider::groupingToString(g)));
    }

    if (grouping.has_value() && grouping_box_->findText(ext_item) >= 0)
    {
        // 1) try to set to externally passed item
        grouping_box_->setCurrentText(ext_item);
    }
    else if (grouping_box_->findText(last_item) >= 0)
    {
        // 2) try to set to last item
        grouping_box_->setCurrentText(last_item);
    }
    else if (grouping_box_->findText(default_item) >= 0)
    {
        // 3) try to set to default item
        grouping_box_->setCurrentText(default_item);
    }
    else
    {
        // 4) set to first item
        grouping_box_->setCurrentIndex(0);
    }

    auto current_item = grouping_box_->currentText();
    if (current_item != last_item)
        model_.provider().setGrouping(DBContentItemProvider::groupingFromString(current_item.toStdString()), run_update);
        
    grouping_box_->blockSignals(false);
}
