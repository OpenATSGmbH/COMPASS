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

#include <QComboBox>
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

    layout->addWidget(grouping_box_);

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
        DBContentItemProvider::groupingFromString(text.toStdString()));
}
