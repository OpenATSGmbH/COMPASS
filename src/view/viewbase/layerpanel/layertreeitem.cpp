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

#include "layertreeitem.h"
#include "layertreemodel.h"

#include <QBrush>
#include <QPainter>
#include <QPen>
#include <QPixmap>

LayerTreeItem::LayerTreeItem(const std::string& name, LayerTreeItem* parent_item)
    : name_(name), parent_item_(parent_item)
{
}

LayerTreeItem::~LayerTreeItem() = default;

LayerTreeItem* LayerTreeItem::child(int row) const
{
    if (row < 0 || row >= (int)children_.size())
        return nullptr;
    return children_[row].get();
}

int LayerTreeItem::indexOf(const LayerTreeItem* child) const
{
    for (size_t i = 0; i < children_.size(); ++i)
        if (children_[i].get() == child)
            return (int)i;
    return -1;
}

int LayerTreeItem::row() const
{
    if (!parent_item_)
        return 0;
    return parent_item_->indexOf(this);
}

LayerTreeItem* LayerTreeItem::appendChild(std::unique_ptr<LayerTreeItem> child)
{
    child->parent_item_ = this;
    child->setModel(model_);
    LayerTreeItem* raw = child.get();
    children_.push_back(std::move(child));
    return raw;
}

void LayerTreeItem::clearChildren()
{
    children_.clear();
}

void LayerTreeItem::removeChildAt(int row)
{
    if (row < 0 || row >= (int)children_.size())
        return;
    children_.erase(children_.begin() + row);
}

std::vector<std::unique_ptr<LayerTreeItem>> LayerTreeItem::moveChildrenOut()
{
    auto out = std::move(children_);
    children_.clear();
    for (auto& c : out)
        c->parent_item_ = nullptr;
    return out;
}

void LayerTreeItem::setModel(LayerTreeModel* model)
{
    model_ = model;
    for (auto& c : children_)
        c->setModel(model);
}

QVariant LayerTreeItem::itemData(int column) const
{
    if (column == 0)
        return QString::fromStdString(name_);
    return {};
}

QVariant LayerTreeItem::icon() const
{
    if (!color_.isValid())
        return {};
    return color_icon_;
}

void LayerTreeItem::setColor(const QColor& color)
{
    color_ = color;
    rebuildColorIcon();
}

void LayerTreeItem::rebuildColorIcon()
{
    constexpr int w = 14;
    constexpr int h = 14;
    constexpr qreal radius = 3.0;

    QPixmap pixmap(w, h);
    pixmap.fill(Qt::transparent);

    if (color_.isValid())
    {
        QPainter p(&pixmap);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(QPen(Qt::darkGray, 1, Qt::SolidLine));
        p.setBrush(QBrush(color_));
        QRectF r(0.5, 0.5, w - 1.0, h - 1.0);
        p.drawRoundedRect(r, radius, radius);
    }
    color_icon_ = QIcon(pixmap);
}

bool LayerTreeItem::effectiveHidden() const
{
    if (hidden_) return true;
    return parent_item_ ? parent_item_->effectiveHidden() : false;
}

void LayerTreeItem::setHidden(bool value, bool emit_signal)
{
    hidden_ = value;
    cascadeEffectiveHidden();
    if (emit_signal && model_)
        emit model_->hiddenChangedSignal();
}

void LayerTreeItem::hideSubtree(bool emit_signal)
{
    hidden_ = true;
    for (auto& c : children_)
        c->hideSubtree(false);
    cascadeEffectiveHidden();
    if (emit_signal && model_)
        emit model_->hiddenChangedSignal();
}

void LayerTreeItem::showSubtree(bool emit_signal)
{
    hidden_ = false;
    for (auto& c : children_)
        c->showSubtree(false);
    cascadeEffectiveHidden();
    if (emit_signal && model_)
        emit model_->hiddenChangedSignal();
}

void LayerTreeItem::cascadeEffectiveHidden()
{
    onEffectiveHiddenChanged();
    for (auto& c : children_)
        c->cascadeEffectiveHidden();
}
