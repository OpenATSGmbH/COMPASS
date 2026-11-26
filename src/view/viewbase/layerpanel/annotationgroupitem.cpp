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

#include "annotationgroupitem.h"
#include "annotationsrootitem.h"

#include "traced_assert.h"

AnnotationGroupItem::AnnotationGroupItem(const std::string& name,
                                         const std::string& update_key,
                                         const AnnotationsRootItem* root)
    : LayerTreeItem(name)
    , update_key_(update_key)
    , root_(root)
{
    traced_assert(root_);
}

/**
 * Static (not reconciled) outside an update of the root.
 */
std::optional<std::vector<std::string>> AnnotationGroupItem::childKeys() const
{
    return root_->leafKeys(update_key_);
}

/**
 */
std::unique_ptr<LayerTreeItem> AnnotationGroupItem::createChild(const std::string& key)
{
    return root_->createLeaf(update_key_, key);
}

/**
 */
bool AnnotationGroupItem::updateChild(LayerTreeItem& child)
{
    return root_->updateLeaf(update_key_, child);
}
