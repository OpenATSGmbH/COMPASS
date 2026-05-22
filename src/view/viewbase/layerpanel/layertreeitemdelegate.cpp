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

#include "layertreeitemdelegate.h"
#include "layertreeitem.h"
#include "layertreemodel.h"
#include "traced_assert.h"

#include <QApplication>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QStyleOptionButton>

namespace
{
    constexpr int kSpace = 2;
    constexpr int kIconW = 14;
    constexpr int kIconH = 14;
}

LayerTreeItemDelegate::LayerTreeItemDelegate(QObject* parent)
    : QStyledItemDelegate(parent)
{
}

void LayerTreeItemDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option,
                                  const QModelIndex& index) const
{
    if (index.column() != 0)
    {
        QStyleOptionViewItem opt = option;
        // honor model-provided alignment
        QVariant align = index.data(Qt::TextAlignmentRole);
        if (align.isValid())
            opt.displayAlignment = Qt::Alignment(align.toInt());
        QStyledItemDelegate::paint(painter, opt, index);
        return;
    }

    auto* item = static_cast<LayerTreeItem*>(index.internalPointer());
    traced_assert(item);

    QFont font = QApplication::font();
    QFontMetrics fm(font);

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);

    const int row_h  = option.rect.height();
    const int cbox_w = fm.height();
    int x = option.rect.left();
    const int y = option.rect.top();

    if (item->canHide())
    {
        QStyleOptionButton cbOpt;
        cbOpt.rect  = QRect(x, y + (row_h - cbox_w) / 2, cbox_w, cbox_w);
        cbOpt.state = item->hidden() ? QStyle::State_Off : QStyle::State_On;
        QApplication::style()->drawControl(QStyle::CE_CheckBox, &cbOpt, painter);
        x += cbox_w + kSpace;
    }

    if (reserve_icon_column_)
    {
        QIcon icon = qvariant_cast<QIcon>(index.data(LayerTreeModel::IconRole));
        if (!icon.isNull())
        {
            const QRect iconRect(x, y + (row_h - kIconH) / 2, kIconW, kIconH);
            icon.paint(painter, iconRect, Qt::AlignCenter, QIcon::Normal, QIcon::Off);
        }
        x += kIconW + kSpace;
    }

    painter->setFont(font);
    const QString text = index.data(Qt::DisplayRole).toString();
    QRect textRect = option.rect;
    textRect.setLeft(x);
    painter->drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter, text);

    painter->restore();
}

bool LayerTreeItemDelegate::editorEvent(QEvent* event, QAbstractItemModel* /*model*/,
                                        const QStyleOptionViewItem& option,
                                        const QModelIndex& index)
{
    if (index.column() != 0)
        return true;
    if (event->type() != QEvent::MouseButtonRelease)
        return true;

    auto* e = static_cast<QMouseEvent*>(event);
    const int clickX = e->x();
    const int clickY = e->y();

    QFontMetrics fm(QApplication::font());
    const int row_h  = option.rect.height();
    const int cbox_w = fm.height();
    int x = option.rect.left();
    const int y = option.rect.top();

    auto* item = static_cast<LayerTreeItem*>(index.internalPointer());
    traced_assert(item);

    if (item->canHide())
    {
        const QRect cbox_rect(x, y + (row_h - cbox_w) / 2, cbox_w, cbox_w);
        if (cbox_rect.contains(clickX, clickY))
        {
            item->setHidden(!item->hidden());
            return true;
        }
    }

    // Color icon is intentionally not clickable - colors are owned by the
    // data context and cannot be edited from the tree.

    return true;
}
