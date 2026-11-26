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

#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

#include <map>
#include <set>

#include <QAction>
#include <QBrush>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPointer>

namespace
{
    /// escapes the key separators => names and ids may contain '/' and '#'
    std::string escapeKeySegment(const std::string& segment)
    {
        std::string escaped;
        escaped.reserve(segment.size());

        for (char c : segment)
        {
            if (c == '\\' || c == '/' || c == '#')
                escaped += '\\';
            escaped += c;
        }

        return escaped;
    }
}

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
    return insertChild(childCount(), std::move(child));
}

/**
*/
LayerTreeItem* LayerTreeItem::insertChild(int row, std::unique_ptr<LayerTreeItem> child)
{
    traced_assert(child);
    traced_assert(row >= 0 && row <= childCount());

    const bool attach = model_ && !child->model_;

    child->parent_item_ = this;
    child->setModel(model_);
    LayerTreeItem* raw = child.get();
    children_.insert(children_.begin() + row, std::move(child));

    // newly attached => the model applies the config
    if (attach)
        model_->subtreeAttached(raw);

    return raw;
}

/**
*/
void LayerTreeItem::moveChild(int from, int to)
{
    traced_assert(from >= 0 && from < childCount());
    traced_assert(to >= 0 && to < childCount());

    if (from == to)
        return;

    auto child = std::move(children_[ from ]);
    children_.erase(children_.begin() + from);
    children_.insert(children_.begin() + to, std::move(child));
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

/**
*/
void LayerTreeItem::removeChildrenAt(int first, int last)
{
    traced_assert(first >= 0 && first <= last && last < childCount());

    children_.erase(children_.begin() + first, children_.begin() + last + 1);
}

LayerTreeItem* LayerTreeItem::appendChildNotified(std::unique_ptr<LayerTreeItem> child)
{
    if (model_)
        return model_->appendChildItem(this, std::move(child));

    return appendChild(std::move(child));
}

void LayerTreeItem::removeChildNotified(int row)
{
    if (model_)
        model_->removeChildItem(this, row);
    else
        removeChildAt(row);
}

void LayerTreeItem::notifyChanged()
{
    if (model_)
        model_->itemChanged(this);
}

/**
*/
void LayerTreeItem::insertChildrenNotified(int row, std::vector<std::unique_ptr<LayerTreeItem>> children)
{
    if (model_)
    {
        model_->insertChildItems(this, row, std::move(children));
        return;
    }

    for (auto& child : children)
        insertChild(row++, std::move(child));
}

/**
*/
void LayerTreeItem::moveChildNotified(int from, int to)
{
    if (model_)
        model_->moveChildItem(this, from, to);
    else
        moveChild(from, to);
}

/**
*/
void LayerTreeItem::removeChildrenNotified(int first, int last)
{
    if (model_)
        model_->removeChildItems(this, first, last);
    else
        removeChildrenAt(first, last);
}

/**
*/
void LayerTreeItem::update()
{
    if (model_)
        model_->updateSubtree(this);
    else
        updateChildren();
}

/**
*/
std::string LayerTreeItem::updateKey() const
{
    const std::string id = stateKeyId();

    return id.empty() ? name_ : id;
}

/**
 * Reconciles the own children (dynamic items only), recurses into the kept ones (new ones were
 * updated before their insert), then onSubtreeUpdated().
 * Notice: kept rows get one dataChanged if they or their subtrees changed (aggregates).
 */
bool LayerTreeItem::updateChildren()
{
    bool changed      = false;
    bool kept_changed = false;

    std::vector<LayerTreeItem*> kept;

    const auto keys = childKeys();
    if (keys)
    {
        changed = reconcileChildren(*keys, kept, kept_changed);
    }
    else
    {
        kept.reserve(children_.size());
        for (auto& child : children_)
            kept.push_back(child.get());
    }

    for (auto* child : kept)
        if (child->updateChildren())
            kept_changed = true;

    if (kept_changed && model_)
        model_->childItemsChanged(this);

    onSubtreeUpdated();

    return changed || kept_changed;
}

/**
 * First removes the children whose key vanished (in contiguous ranges), then walks the keys by
 * position: keeps a matching child (a later one is moved up), else creates and fills a new one
 * (consecutive new ones are inserted together).
 */
bool LayerTreeItem::reconcileChildren(const std::vector<std::string>& keys,
                                      std::vector<LayerTreeItem*>& kept,
                                      bool& kept_changed)
{
    bool changed = false;

    const std::set<std::string> wanted(keys.begin(), keys.end());

    // vanished children and duplicates (the first one stays)
    {
        std::set<std::string> present;
        std::vector<bool>     drop(children_.size(), false);

        for (size_t i = 0; i < children_.size(); ++i)
        {
            const std::string key = children_[ i ]->updateKey();

            // only children added by other means can repeat a key
            const bool duplicate = !present.insert(key).second;
            if (duplicate)
                logwrn << "duplicate child '" << key << "' below '" << name_ << "', removed";

            drop[ i ] = duplicate || !wanted.count(key);
        }

        for (int last = childCount() - 1; last >= 0; --last)
        {
            if (!drop[ last ])
                continue;

            int first = last;
            while (first > 0 && drop[ first - 1 ])
                --first;

            removeChildrenNotified(first, last);
            changed = true;

            last = first;
        }
    }

    std::map<std::string, LayerTreeItem*> existing;
    for (auto& child : children_)
        existing.emplace(child->updateKey(), child.get());

    kept.reserve(keys.size());

    int row = 0;

    std::set<std::string>                       seen;
    std::vector<std::unique_ptr<LayerTreeItem>> pending;

    auto insertPending = [ & ] ()
    {
        if (pending.empty())
            return;

        const int n = (int)pending.size();

        insertChildrenNotified(row, std::move(pending));
        pending.clear();

        row    += n;
        changed = true;
    };

    for (const auto& key : keys)
    {
        if (!seen.insert(key).second)
        {
            logwrn << "duplicate child key '" << key << "' below '" << name_ << "', skipped";
            continue;
        }

        auto it = existing.find(key);
        if (it == existing.end())
        {
            auto child = createChild(key);
            if (!child)
            {
                logwrn << "no child created for key '" << key << "' below '" << name_ << "'";
                continue;
            }

            traced_assert(child->updateKey() == key);
            traced_assert(!child->model());

            // detached => its subtree is complete when inserted (one insert, one config pass)
            child->update();

            pending.push_back(std::move(child));
            continue;
        }

        insertPending();

        LayerTreeItem* child = it->second;
        const int      from  = indexOf(child);

        traced_assert(from >= row);

        if (from != row)
        {
            moveChildNotified(from, row);
            changed = true;
        }

        if (updateChild(*child))
            kept_changed = true;

        kept.push_back(child);
        ++row;
    }

    insertPending();

    // all remaining children are wanted => matched
    traced_assert(row == childCount());

    return changed;
}

void LayerTreeItem::rename(const std::string& name)
{
    const std::string old_key = stateKey();

    name_ = name;

    if (!model_)
        return;

    // id-keyed items keep their key => nothing moves
    model_->state().renameItemConfigs(old_key, stateKey());

    notifyChanged();
}

void LayerTreeItem::connectItemAction(QAction* action, const std::function<void(LayerTreeItem&)>& func) const
{
    traced_assert(action);
    traced_assert(model_);

    // resolved when triggered: the item may have been rebuilt (same key => its successor) or removed
    QPointer<LayerTreeModel> model = model_;
    const std::string        key   = stateKey();

    QObject::connect(action, &QAction::triggered, [ model, key, func ] ()
    {
        if (!model)
            return;

        LayerTreeItem* item = model->findItem(key);
        if (!item)
        {
            logwrn << "layer tree item '" << key << "' not available (any more)";
            return;
        }

        func(*item);
    });
}

void LayerTreeItem::setModel(LayerTreeModel* model)
{
    model_ = model;
    for (auto& c : children_)
        c->setModel(model);
}

std::string LayerTreeItem::stateKey() const
{
    // the id only replaces the own name segment => every key starts with its parent's key
    std::string key = escapeKeySegment(stateKeyId());

    if (key.empty())
    {
        if (!parent_item_)
            return {}; // invisible root

        // repeated sibling names => append occurrence index (unescaped '#', names have it escaped)
        int repetition = 0;
        for (const auto& sibling : parent_item_->children_)
        {
            if (sibling.get() == this)
                break;
            if (sibling->name_ == name_ && sibling->stateKeyId().empty())
                ++repetition;
        }

        key = escapeKeySegment(name_);
        if (repetition > 0)
            key += "#" + std::to_string(repetition);
    }

    const std::string parent_key = parent_item_ ? parent_item_->stateKey() : std::string();

    return parent_key.empty() ? key : parent_key + "/" + key;
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
    storeHiddenState();
    cascadeEffectiveHidden();
    if (emit_signal && model_)
        emit model_->hiddenChangedSignal();
}

void LayerTreeItem::hideSubtree(bool emit_signal)
{
    hidden_ = true;
    storeHiddenState();
    for (auto& c : children_)
        c->hideSubtree(false);
    cascadeEffectiveHidden();
    if (emit_signal && model_)
        emit model_->hiddenChangedSignal();
}

void LayerTreeItem::showSubtree(bool emit_signal)
{
    hidden_ = false;
    storeHiddenState();
    for (auto& c : children_)
        c->showSubtree(false);
    cascadeEffectiveHidden();
    if (emit_signal && model_)
        emit model_->hiddenChangedSignal();
}

void LayerTreeItem::storeHiddenState()
{
    // not attached => constructed state, nothing to persist
    if (!model_)
        return;

    if (persistHiddenStateToConfig())
        config()[ "hidden" ] = hidden_;
}

bool LayerTreeItem::persistHiddenStateToConfig() const
{
    if (persistsSubtreeToConfig())
        return true;

    return parent_item_ ? parent_item_->persistHiddenStateToConfig() : false;
}

nlohmann::json& LayerTreeItem::config()
{
    traced_assert(model_); // the key depends on the parent chain => only after attach

    return model_->state().configRoot()[ stateKey() ];
}

const nlohmann::json& LayerTreeItem::config() const
{
    static const nlohmann::json empty = nlohmann::json::object();

    if (!model_)
        return empty;

    const nlohmann::json& root = static_cast<const LayerTreeModel*>(model_)->state().configRoot();

    auto it = root.find(stateKey());

    return it != root.end() && it->is_object() ? *it : empty;
}

std::optional<bool> LayerTreeItem::configuredHidden() const
{
    if (!model_ || !persistHiddenStateToConfig())
        return {};

    const nlohmann::json& cfg = config();

    auto it = cfg.find("hidden");
    if (it == cfg.end() || !it->is_boolean())
        return {};

    return it->get<bool>();
}

void LayerTreeItem::cascadeEffectiveHidden()
{
    onEffectiveHiddenChanged();
    for (auto& c : children_)
        c->cascadeEffectiveHidden();
}
