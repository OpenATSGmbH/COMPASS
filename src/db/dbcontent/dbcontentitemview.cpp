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
#include <QStandardItemModel>
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
    
    connect(&model_.provider(), &DBContentItemProvider::groupingChangedSignal,
            this, &DBContentItemView::updateGrouping);

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
void DBContentItemView::updateGrouping()
{
    traced_assert(isGroupingActive(model_.provider().grouping()));
    traced_assert(grouping_box_->findText(QString::fromStdString(model_.provider().groupingAsString())) >= 0);

    grouping_box_->blockSignals(true);
    grouping_box_->setCurrentText(QString::fromStdString(model_.provider().groupingAsString()));
    grouping_box_->blockSignals(false);
}

/**
 */
dbContent::Grouping DBContentItemView::currentGrouping() const
{
    return model_.provider().grouping();
}

/**
 */
void DBContentItemView::setActiveGroupings(unsigned int flags)
{
    active_groupings_ = flags;

    auto* model = qobject_cast<QStandardItemModel*>(grouping_box_->model());
    traced_assert(model);

    const auto all_groupings = DBContentItemProvider::getGroupings();
    traced_assert(static_cast<int>(all_groupings.size()) == model->rowCount());

    for (int i = 0; i < model->rowCount(); ++i)
    {
        const unsigned int g_flag = 1u << static_cast<unsigned int>(all_groupings[i]);
        if (auto* item = model->item(i))
            item->setEnabled((flags & g_flag) != 0);
    }

    //!default grouping shall always be active!
    traced_assert(isGroupingActive(DBContentItemProvider::DefaultGrouping));
}

/**
 */
bool DBContentItemView::isGroupingActive(dbContent::Grouping grouping) const
{
    const unsigned int g_flag = 1u << static_cast<unsigned int>(grouping);
    return (active_groupings_ & g_flag) != 0;
}
