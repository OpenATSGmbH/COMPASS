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

#include "scatterseriestreeitem.h"
#include "scatterseriesmodel.h"
#include "logger.h"

#include <QApplication>

#include "traced_assert.h"

#include <QApplication>
#include <QBrush>
#include <QColorDialog>
#include <QDialog>
#include <QMenu>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QThread>
#include <QtGui>

const unsigned int space = 2;

ScatterSeriesTreeItemDelegate::ScatterSeriesTreeItemDelegate(QObject* parent) : QStyledItemDelegate(parent) {}

void ScatterSeriesTreeItemDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option,
                                const QModelIndex& index) const
{
    logdbg << "r " << index.row() << " c " << index.column();

    if (index.column() == 1)  // only do custom painting in column 0
    {
        QStyledItemDelegate::paint(painter, option, index);
        return;
    }

    QFont font = QApplication::font();
    QFontMetrics fm(font);

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);

    ScatterSeriesTreeItem* item = static_cast<ScatterSeriesTreeItem*>(index.internalPointer());
    traced_assert(item);

    const int row_h  = option.rect.height();
    const int cbox_w = fm.height();
    const int icon_w = 14; // matches makeColorIcon
    const int icon_h = 14;
    int x = option.rect.left();
    const int y = option.rect.top();

    if (item->canHide())
    {
        QStyleOptionButton cbOpt;
        cbOpt.rect  = QRect(x, y + (row_h - cbox_w) / 2, cbox_w, cbox_w);
        cbOpt.state = item->hidden() ? QStyle::State_Off : QStyle::State_On;
        QApplication::style()->drawControl(QStyle::CE_CheckBox, &cbOpt, painter);
        x += cbox_w + space;
    }

    QIcon icon = qvariant_cast<QIcon>(index.data(ScatterSeriesModel::DataRole::IconRole));
    if (!icon.isNull())
    {
        const QRect iconRect(x, y + (row_h - icon_h) / 2, icon_w, icon_h);
        icon.paint(painter, iconRect, Qt::AlignCenter, QIcon::Normal, QIcon::Off);
    }
    x += icon_w + space;

    painter->setFont(font);
    const QString headerText = qvariant_cast<QString>(index.data(0));
    QRect textRect = option.rect;
    textRect.setLeft(x);
    painter->drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter, headerText);

    painter->restore();
}

bool ScatterSeriesTreeItemDelegate::editorEvent(QEvent* event, QAbstractItemModel* model,
                                      const QStyleOptionViewItem& option, const QModelIndex& index)
{
    if (index.column() == 1)
        return true;

    if (event->type() != QEvent::MouseButtonRelease)
        return true;

    QMouseEvent* e = (QMouseEvent*)event;
    const int clickX = e->x();
    const int clickY = e->y();

    QFontMetrics fm(QApplication::font());
    const int row_h  = option.rect.height();
    const int cbox_w = fm.height();
    const int icon_w = 14;
    const int icon_h = 14;
    int x = option.rect.left();
    const int y = option.rect.top();

    ScatterSeriesTreeItem* item = static_cast<ScatterSeriesTreeItem*>(index.internalPointer());
    traced_assert(item);

    if (item->canHide())
    {
        const QRect cbox_rect(x, y + (row_h - cbox_w) / 2, cbox_w, cbox_w);
        if (cbox_rect.contains(clickX, clickY))
        {
            item->hide(!item->hidden());
            return true;
        }
        x += cbox_w + space;
    }

    const QRect icon_rect(x, y + (row_h - icon_h) / 2, icon_w, icon_h);
    if (icon_rect.contains(clickX, clickY))
    {
        if (item->canEditColor())
        {
            QColor color = QColorDialog::getColor(item->color(), nullptr,
                                                  QString::fromStdString(item->name()));
            if (color.isValid())
            {
                item->setColor(color);
                item->emitColorChanged();
            }
        }
        else if (item->hasMenu())
        {
            item->execMenu(QPoint(e->globalX(), e->globalY()));
        }
    }
    return true;
}


QIcon ScatterSeriesTreeItem::makeColorIcon(const QColor& color)
{
    constexpr int w = 14;
    constexpr int h = 14;
    constexpr qreal radius = 3.0;

    QPixmap pixmap(w, h);
    pixmap.fill(Qt::transparent);

    if (color.isValid())
    {
        QPainter p(&pixmap);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(QPen(Qt::darkGray, 1, Qt::SolidLine));
        p.setBrush(QBrush(color));
        QRectF r(0.5, 0.5, w - 1.0, h - 1.0);
        p.drawRoundedRect(r, radius, radius);
        p.end();
    }
    return QIcon(pixmap);
}

void ScatterSeriesTreeItem::rebuildColorIcon()
{
    color_icon_ = makeColorIcon(color_);
}

ScatterSeriesTreeItem::ScatterSeriesTreeItem(
    const std::string& name, const QColor& color, ScatterSeriesModel& model,
    ScatterSeriesCollection::DataSeries* data_series,
    ScatterSeriesTreeItem* parent_item)
    : name_(name), model_(model), data_series_(data_series), parent_item_(parent_item), color_(color)
{
    rebuildColorIcon();
}

ScatterSeriesTreeItem::ScatterSeriesTreeItem(
    const std::string& name, const QColor& color, ScatterSeriesModel& model,
    ScatterSeriesTreeItem* parent_item)
    : name_(name), model_(model), parent_item_(parent_item), color_(color)
{
    rebuildColorIcon();
}

ScatterSeriesTreeItem::~ScatterSeriesTreeItem()
{
    child_items_.clear();
}

void ScatterSeriesTreeItem::appendChild(ScatterSeriesTreeItem* item)
{
    logdbg << item->name();

    traced_assert(!child_items_.count(item->name()));

    child_items_[item->name()].reset(item);
}

// void ScatterSeriesTreeItem::removeChild(ScatterSeriesTreeItem* item)
// {
//     logdbg << item->name();
//     auto it = std::find(child_items_.begin(), child_items_.end(), item);
//     traced_assert(it != child_items_.end());

//     if (it != child_items_.end())
//         child_items_.erase(it);
// }

// void ScatterSeriesTreeItem::updateHiddenImpl()
// {
//     for (auto& child_it : child_items_)
//         child_it.second->updateHidden();
// }

// void ScatterSeriesTreeItem::moveChildUp(ScatterSeriesTreeItem* child)
// {
//     auto it = std::find(child_items_.begin(), child_items_.end(), child);
//     traced_assert(it != child_items_.end());

//     if (it != child_items_.begin())
//         std::iter_swap(it, it - 1);
// }
// void ScatterSeriesTreeItem::moveChildDown(ScatterSeriesTreeItem* child)
// {
//     auto it = std::find(child_items_.begin(), child_items_.end(), child);
//     traced_assert(it != child_items_.end());

//     if (it + 1 != child_items_.end())
//         std::iter_swap(it, it + 1);
// }
// void ScatterSeriesTreeItem::moveChildToBegin(ScatterSeriesTreeItem* child)
// {
//     auto it = std::find(child_items_.begin(), child_items_.end(), child);
//     traced_assert(it != child_items_.end());

//     std::rotate(child_items_.begin(), it, it + 1);
// }
// void ScatterSeriesTreeItem::moveChildToEnd(ScatterSeriesTreeItem* child)
// {
//     auto it = std::find(child_items_.begin(), child_items_.end(), child);
//     traced_assert(it != child_items_.end());

//     if (it == child_items_.end() - 1)
//         return;

//     std::rotate(it, it + 1, child_items_.end());
// }

// void ScatterSeriesTreeItem::moveUp()
// {
//     traced_assert(parent_item_);
//     parent_item_->moveChildUp(this);
// }
// void ScatterSeriesTreeItem::moveDown()
// {
//     traced_assert(parent_item_);
//     parent_item_->moveChildDown(this);
// }
// void ScatterSeriesTreeItem::moveToBegin()
// {
//     traced_assert(parent_item_);

//     parent_item_->moveChildToBegin(this);
// }
// void ScatterSeriesTreeItem::moveToEnd()
// {
//     traced_assert(parent_item_);
//     parent_item_->moveChildToEnd(this);
// }

unsigned int ScatterSeriesTreeItem::getIndexOf(ScatterSeriesTreeItem* child)
{
    // Use std::find_if with a lambda to find the item
    // auto it = std::find_if(
    //     itemMap.begin(),
    //     itemMap.end(),
    //     [targetItem](const std::pair<const std::string, std::unique_ptr<ScatterSeriesTreeItem>>& pair) {
    //         return pair.second.get() == targetItem;
    //     }
    //     );

    auto it = std::find_if(child_items_.begin(), child_items_.end(),
                           [child](const std::pair<const std::string, std::unique_ptr<ScatterSeriesTreeItem>>& x)
                           { return x.second.get() == child;});

    traced_assert(it != child_items_.end());

    return distance(child_items_.begin(), it);
}

void ScatterSeriesTreeItem::clear()
{
    child_items_.clear();
}

ScatterSeriesTreeItem* ScatterSeriesTreeItem::child(int row)
{
    traced_assert(row >= 0);
    traced_assert((unsigned int)row < child_items_.size());

    auto it = std::next(child_items_.begin(), row);
    traced_assert(it != child_items_.end());

    logdbg  << "child: " << it->second->name();
    return it->second.get();
}

int ScatterSeriesTreeItem::childCount() const
{
    logdbg << "count " << child_items_.size();
    return child_items_.size();
}

int ScatterSeriesTreeItem::columnCount() const
{
    return 2;
}

QVariant ScatterSeriesTreeItem::data(int column) const
{
    if (column == 0)
        return name_.c_str();
    else if (column == 1)
    {
        if (data_series_)
            return (unsigned int) data_series_->scatter_series.points.size();
        else
            return QVariant();
    }
    else
        traced_assert(false);
}

QVariant ScatterSeriesTreeItem::icon() const
{
    return color_icon_;
}

ScatterSeriesTreeItem* ScatterSeriesTreeItem::parentItem() { return parent_item_; }

int ScatterSeriesTreeItem::row() const
{
    if (parent_item_)
        return parent_item_->getIndexOf(const_cast<ScatterSeriesTreeItem*>(this));

    return 0;
}

QColor ScatterSeriesTreeItem::color() const
{
    return color_;
}

void ScatterSeriesTreeItem::setColor(const QColor& color)
{
    color_ = color;
    if (data_series_)
        data_series_->color = color; // keep chart in sync until next redraw

    rebuildColorIcon();
}

void ScatterSeriesTreeItem::emitColorChanged()
{
    emit model_.colorChangedSignal(name_, data_series_ ? data_series_->color : QColor());
}

void ScatterSeriesTreeItem::hide(bool value)
{
    loginf << "start" << name_ << " hidden " << value;

    hidden_ = value;

    updateHidden();

    emit model_.visibilityChangedSignal();
}

void ScatterSeriesTreeItem::hideAll(bool emit_signal)
{
    hidden_ = true;

    for (auto& child_it : child_items_)
        child_it.second->hideAll(false);

    updateHidden();

    if (emit_signal)
        emit model_.visibilityChangedSignal();
}

void ScatterSeriesTreeItem::updateHidden()
{
    if (data_series_)
        data_series_->visible = !itemHidden();

    for (auto& child_it : child_items_)
        child_it.second->updateHidden();
}


/**
 * Checks if the item is hidden, either itself or by parent
 */
bool ScatterSeriesTreeItem::itemHidden() const
{
    return hidden() || (parent_item_ && parent_item_->itemHidden());
}

