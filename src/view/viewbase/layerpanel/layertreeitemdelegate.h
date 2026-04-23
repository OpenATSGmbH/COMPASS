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

#include <QStyledItemDelegate>

/**
 * Delegate for LayerTreeModel.
 *
 * Column 0: paints [checkbox] [color icon] [name] inline.
 *   - Checkbox visible only when item->canHide() is true.
 *   - Color icon only painted when the item has a valid color.
 *   - Clicking the checkbox toggles the item's hidden state.
 *   - Color icon is NOT clickable — colors cannot be edited from the tree.
 *
 * Other columns: delegate to the default painting path, with alignment taken
 * from the model's TextAlignmentRole.
 */
class LayerTreeItemDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    explicit LayerTreeItemDelegate(QObject* parent = nullptr);

    void paint(QPainter* painter, const QStyleOptionViewItem& option,
               const QModelIndex& index) const override;

    bool editorEvent(QEvent* event, QAbstractItemModel* model,
                     const QStyleOptionViewItem& option,
                     const QModelIndex& index) override;
};
