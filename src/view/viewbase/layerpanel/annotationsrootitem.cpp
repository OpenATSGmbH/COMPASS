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

#include "annotationsrootitem.h"
#include "annotationgroupitem.h"
#include "annotationleafitem.h"
#include "layertreemodel.h"

#include "files.h"
#include "traced_assert.h"

#include <memory>
#include <vector>

namespace
{
    const std::string GroupKeyPrefix = "group:";
    const std::string LeafKeyPrefix  = "anno:";
}

AnnotationsRootItem::AnnotationsRootItem()
    : LayerTreeItem("Annotations")
    , icon_(Utils::Files::IconProvider::getIcon("compass.png"))
{
}

QVariant AnnotationsRootItem::icon() const
{
    return QVariant(icon_);
}

/**
 * Keys: groups by name, leaves by title ("#n" for repeated titles below the same parent);
 * leaves of an unnamed group directly below the root, so the user does not see a blank
 * intermediate row for the "no plot_group" case.
 */
void AnnotationsRootItem::updateFrom(const std::vector<VariableView::AnnotationGroup>& groups,
                                     int           current_group_idx,
                                     int           current_anno_idx,
                                     VariableView* view)
{
    Layout layout;

    for (int g = 0; g < (int)groups.size(); ++g)
    {
        const auto& group = groups[ g ];

        std::string parent_key;
        if (!group.name.empty())
        {
            parent_key = GroupKeyPrefix + group.name;

            layout.root_keys.push_back(parent_key);
            layout.group_names[ parent_key ] = group.name;
        }

        auto& leaves = layout.leaves[ parent_key ];

        for (int a = 0; a < (int)group.annotations.size(); ++a)
        {
            const std::string title = group.annotations[ a ].metadata.fullTitle();

            int repetition = 0;
            for (const auto& leaf : leaves)
                if (leaf.title == title)
                    ++repetition;

            LeafSpec spec;
            spec.key            = LeafKeyPrefix + title + (repetition > 0 ? "#" + std::to_string(repetition) : "");
            spec.title          = title;
            spec.group_idx      = g;
            spec.annotation_idx = a;

            if (parent_key.empty())
                layout.root_keys.push_back(spec.key);

            leaves.push_back(spec);
        }
    }

    layout_            = std::move(layout);
    current_group_idx_ = current_group_idx;
    current_anno_idx_  = current_anno_idx;
    view_              = view;

    update();

    layout_.reset();
}

/**
 * Static (not reconciled) outside updateFrom().
 */
std::optional<std::vector<std::string>> AnnotationsRootItem::childKeys() const
{
    if (!layout_)
        return std::nullopt;

    return layout_->root_keys;
}

/**
 */
std::unique_ptr<LayerTreeItem> AnnotationsRootItem::createChild(const std::string& key)
{
    traced_assert(layout_);

    auto it = layout_->group_names.find(key);
    if (it != layout_->group_names.end())
        return std::make_unique<AnnotationGroupItem>(it->second, key, this);

    return createLeaf("", key);
}

/**
 * Groups are kept by name, nothing to update on them.
 */
bool AnnotationsRootItem::updateChild(LayerTreeItem& child)
{
    if (!dynamic_cast<AnnotationLeafItem*>(&child))
        return false;

    return updateLeaf("", child);
}

/**
 */
std::optional<std::vector<std::string>> AnnotationsRootItem::leafKeys(const std::string& parent_key) const
{
    if (!layout_)
        return std::nullopt;

    std::vector<std::string> keys;

    auto it = layout_->leaves.find(parent_key);
    if (it != layout_->leaves.end())
        for (const auto& spec : it->second)
            keys.push_back(spec.key);

    return keys;
}

/**
 * Notice: created detached with the radio state of the view's current annotation.
 */
std::unique_ptr<LayerTreeItem> AnnotationsRootItem::createLeaf(const std::string& parent_key, const std::string& key) const
{
    const LeafSpec* spec = findLeaf(parent_key, key);
    if (!spec)
        return nullptr;

    auto leaf = std::make_unique<AnnotationLeafItem>(spec->title, spec->key, spec->group_idx, spec->annotation_idx, view_);
    leaf->setActive(isCurrent(*spec));

    return leaf;
}

/**
 */
bool AnnotationsRootItem::updateLeaf(const std::string& parent_key, LayerTreeItem& child) const
{
    auto* leaf = dynamic_cast<AnnotationLeafItem*>(&child);
    traced_assert(leaf);

    const LeafSpec* spec = findLeaf(parent_key, child.updateKey());
    if (!spec)
        return false;

    const bool was_hidden = leaf->hidden();

    leaf->setIndices(spec->group_idx, spec->annotation_idx);
    leaf->setActive(isCurrent(*spec));

    return leaf->hidden() != was_hidden;
}

/**
 */
const AnnotationsRootItem::LeafSpec* AnnotationsRootItem::findLeaf(const std::string& parent_key,
                                                                   const std::string& key) const
{
    if (!layout_)
        return nullptr;

    auto it = layout_->leaves.find(parent_key);
    if (it == layout_->leaves.end())
        return nullptr;

    for (const auto& spec : it->second)
        if (spec.key == key)
            return &spec;

    return nullptr;
}

/**
 */
bool AnnotationsRootItem::isCurrent(const LeafSpec& spec) const
{
    return spec.group_idx == current_group_idx_ && spec.annotation_idx == current_anno_idx_;
}
